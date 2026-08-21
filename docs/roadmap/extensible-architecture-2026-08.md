# Extensible OSS architecture roadmap

> Status: proposed, 2026-08-20. This roadmap changes architecture and contributor
> experience, not the default SLAM algorithm. Every migration step must preserve
> the existing deterministic replay, accuracy, runtime, memory, map-quality, and
> Autoware bundle gates.

## 1. Goal

Make `lidarslam_ros2` a platform where an external contributor can add a
registration method, place recognizer, loop verifier, optimizer, map refiner,
or exporter without editing the central ROS components.

Success means:

- a useful extension can live in a separate ROS package and repository;
- the default `RKO-LIO + graph_based_slam` workflow remains unchanged;
- online ROS components and deterministic offline runners use the same plugin;
- invalid or incompatible plugins fail at startup with an actionable error;
- plugin performance and determinism are measurable through a shared contract
  test kit;
- permissive-license and provenance rules remain enforceable.

The first compatibility promise is **source API and configuration stability
within one major plugin API version**. Cross-distribution C++ binary ABI
compatibility is not promised: ROS 2, PCL, Eigen, and compiler ABIs differ
between Humble/Jazzy and Ubuntu releases.

## 2. Current architecture and constraints

The repository already has strong foundations:

- composable ROS 2 nodes through `rclcpp_components`;
- a ROS-free, clock-free `BackendCore` and deterministic offline runners;
- many pure logic headers with focused tests;
- release gates for byte determinism, APE, map quality, runtime, memory, and
  Autoware map verification.

The main extension bottlenecks are:

1. scan registration selection is a string-driven `if/else` chain inside
   `ScanMatcherComponent`;
   NDT-only behavior also appears at later call sites through concrete casts;
2. loop registration has a separate hard-coded factory with a smaller set of
   implementations and different configuration semantics;
3. four place-recognition implementations and their databases are compiled
   directly into `BackendCore`, whose public getters expose their concrete
   database types;
4. loop verification, pose-graph optimization, map refinement, and export are
   callable modules but not external extension boundaries;
5. public headers are installed, but there is no versioned plugin API,
   capability negotiation, plugin manifest, or downstream contract-test kit;
6. configuration is distributed across large YAML parameter sets, so adding an
   algorithm currently adds central parameters and branches.

## 3. Design rules

1. **Functional core, ROS shell, replaceable algorithms.** Plugin interfaces
   remain free of `rclcpp::Node`, publishers, subscriptions, filesystems, and
   wall-clock scheduling. The shell owns ROS and I/O.
2. **One ordered input, one deterministic result.** Offline runners and live
   components call the same implementation and data contract.
3. **Plugin discovery only at startup.** The processing hot path contains no
   dynamic lookup and no parameter reads.
4. **Small role-specific interfaces.** Do not create a universal `SlamPlugin`
   or expose the entire component state.
5. **Capabilities are explicit.** A plugin declares required point fields,
   initial-guess support, covariance support, deterministic mode, thread model,
   and optional sensor requirements before processing starts.
6. **Built-ins migrate before third-party plugins.** Each seam first wraps the
   current implementation and proves byte-identical or tolerance-bound output.
7. **Rule of two.** Extract a new shared interface only when the built-in path
   and at least one second implementation exercise it. Avoid speculative seams.
8. **Fail closed.** Unknown class IDs, API-version mismatches, missing point
   fields, invalid parameters, and unsupported capabilities are startup errors.
9. **No hidden dataset branches.** Plugins receive geometry and declared sensor
   data, never dataset or sequence identity.
10. **Load outside the deterministic core.** ROS shells and offline runners own
    `pluginlib::ClassLoader`, validate manifests, and inject already-created
    interfaces. `BackendCore` never depends on pluginlib or performs discovery.

## 4. Target package and runtime shape

The target dependency direction is:

```text
lidarslam_plugin_interfaces   (C++14 types, abstract roles, API version; no rclcpp)
            ^
            |
lidarslam_default_plugins     (NDT/GICP, built-in descriptors, verifier, g2o,
            |                  map refinement/export adapters)
            |
  scanmatcher / graph_based_slam cores
            ^
            |
      ROS components and offline runners
```

Initially, interfaces may remain inside their existing package to keep PRs
small. Move them to `lidarslam_plugin_interfaces` only after downstream
install-space tests prove the boundary. Default implementations may likewise
remain in `scanmatcher` or `graph_based_slam` until the interface is stable.

Every plugin role shares these base concepts:

- `PluginApiVersion {major, minor}`;
- `PluginMetadata {id, implementation_version, license, capabilities}`;
- immutable configuration created at startup;
- typed request/result structures with explicit validity and diagnostics;
- `reset()` semantics and a declared thread model;
- structured timing, iteration, convergence, and failure diagnostics.

The common interface stays at C++14 because `graph_based_slam` currently uses
C++14, even though `scanmatcher` can use C++17 internally. The host records the
loaded class ID, implementation version, API version, library identity,
capabilities, and configuration hash in each run manifest.

The configuration shape becomes:

```yaml
registration:
  plugin: lidarslam_default_plugins/NdtOmp
  parameters:
    resolution: 1.0
    maximum_iterations: 100
```

Legacy parameters such as `registration_method: NDT` map to the equivalent
built-in plugin for two minor releases and emit one deprecation warning.

## 5. Extension roles and priority

| Priority | Role | Minimal contract | Why this order |
| --- | --- | --- | --- |
| P0 | `RegistrationPlugin` | configure, set target, align source with initial guess, return pose/fitness/convergence/diagnostics/capabilities | Existing duplicate factories and contributor demand make this the safest first seam. |
| P1 | `PlaceRecognitionPlugin` | ingest immutable submap, query ranked candidates, serialize deterministic diagnostics | Four built-ins are directly coupled to `BackendCore`. |
| P1 | `LoopVerifierPlugin` | verify one candidate and return a constraint or typed rejection | Enables 3D-BBS and future verification without central branching. |
| P2 | `GraphOptimizerPlugin` | optimize typed nodes/constraints under explicit gauge and robust-kernel config | Allows g2o alternatives after graph semantics are frozen. |
| P2 | `MapRefinerPlugin` | refine immutable submaps/poses and return poses plus quality diagnostics | Existing refiner is already close to a pure boundary. |
| P2 | map-export strategy | consume a frozen map snapshot and emit a declared artifact manifest | Start as a normal C++ strategy; promote it to pluginlib only after a real external exporter requires it. |
| P3 | `FrontendPlugin` | ordered LiDAR/IMU events to odometry/submap events | High value but too broad until registration and map-update contracts stabilize. |
| P3 | sensor adapters | normalize vendor messages into canonical point/IMU events | Prefer separate adapter nodes first; use in-process plugins only with measured need. |

Candidate aggregation policy, safety gates, loop-edge deduplication, scheduling,
and benchmark scoring remain core policy rather than plugins. Allowing plugins
to replace the judge would weaken reproducibility and safety.

Registration capabilities replace NDT-specific casts and branches. They cover,
at minimum, rotation/translation priors, adaptive correspondence controls,
mean-correspondence diagnostics, target preprocessing, covariance, and
deterministic execution. Requesting an unsupported capability is a startup
configuration error rather than a silently ignored option.

## 6. Compatibility and lifecycle policy

- Version plugin interfaces independently from the application release.
- A major API mismatch is a hard startup failure. A newer minor capability is
  accepted only when the host can ignore it safely.
- Changing an existing pure virtual signature requires a new major plugin API.
- Configuration keys are namespaced by plugin ID. Renames keep aliases and
  migration warnings for at least two minor application releases.
- ROS messages and artifact JSON/YAML retain explicit `schema_version` fields.
- Plugin manifests declare license and upstream source. The default workflow
  loads only plugins allowed by the permissive-license policy.
- The core never catches an arbitrary plugin exception and continues with
  partially mutated state. Each call either returns a valid result or a typed
  failure; fatal exceptions terminate that processing session cleanly.

## 7. Delivery phases and hard gates

### Phase 0 — contracts and characterization

Deliver:

- architecture decision record for plugin boundaries and compatibility;
- inventory of current algorithms, parameters, dependencies, licenses, and
  ROS/offline call sites;
- frozen registration request/result fixtures for NDT and GICP;
- downstream install-space test skeleton and example consumer package.

Gate: no production behavior change; existing CI and release gates pass; the
contract can represent every result currently used by scanmatcher and loop
registration without exposing ROS component state.

### Phase 1 — registration seam

Deliver:

- one shared `RegistrationPlugin` interface and `pluginlib` loader;
- built-in NDT and GICP adapters, then optional FAST_GICP/small_gicp adapters;
- identical selection semantics in scanmatcher, backend, and offline runners;
- legacy parameter adapter and clear startup diagnostics;
- external `example_registration_plugin` built in a separate workspace.

Gate:

- default NDT outputs are byte-identical on deterministic fixtures and replay;
- all existing registration methods pass their current accuracy gates;
- plugin dispatch adds less than 0.5% processing overhead;
- end-to-end RTF remains within 5% of the corresponding baseline;
- missing, incompatible, and throwing plugins pass negative tests;
- Humble and Jazzy install-space consumer builds pass.

### Phase 2 — loop pipeline seams

Deliver:

- `PlaceRecognitionPlugin` and `LoopVerifierPlugin`;
- adapters for Scan Context, BEV, SOLiD, Triangle, distance fallback, and 3D-BBS;
- deterministic candidate ordering independent of plugin discovery order;
- per-plugin budgets and diagnostics, while the core retains aggregation and
  acceptance policy.

Scan Context is the pilot migration. BEV and Triangle move later because their
current cross-checking creates a more complicated dependency boundary. Concrete
descriptor-database getters are deprecated only after equivalent contract
tests cover their behavior.

Gate:

- frozen descriptor databases and accepted loop-edge sets are byte-identical;
- plugin ordering cannot change the result;
- false-loop, timeout, memory, and deterministic replay gates pass;
- a standalone example descriptor can be installed without editing the core.

### Phase 3 — optimization, refinement, and export

Deliver:

- immutable graph, map snapshot, constraint, and artifact-manifest contracts;
- adapters for the existing g2o optimizer, clean-room map refiner, and Autoware
  map-export strategy;
- one small alternative/reference implementation per seam for contract tests.

Gate:

- default trajectory and map hashes remain unchanged where byte identity is
  promised;
- exporters cannot mutate estimator state;
- map-quality, Autoware verification, resource, and license gates pass;
- failed exporters leave no partially valid artifact manifest.

### Phase 4 — frontend and sensor ecosystem

Deliver only after the earlier contracts have survived at least one release:

- typed ordered LiDAR/IMU event contracts;
- frontend lifecycle and state-handoff contract;
- external frontend adapter example;
- documented adapter-node path for new LiDAR vendors and point-time layouts.

Gate:

- RKO-LIO remains the unchanged default;
- frontend determinism and complete map-authoring E2E pass;
- capability checks reject missing per-point time, calibration, or IMU data
  before mapping begins;
- no dataset-specific branch enters the plugin API.

### Phase 5 — contributor SDK and ecosystem release

Deliver:

- `create_lidarslam_plugin` scaffold command or template repository;
- contract-test CMake helper and reusable CI workflow;
- extension cookbook, API reference, compatibility matrix, and migration guide;
- plugin proposal issue form with license, capability, benchmark, and maintainer
  fields;
- curated registry listing compatibility and evidence, not an automatic trust
  store.

Gate: a contributor unfamiliar with the core can generate, build, test, install,
discover, and run an example plugin from a clean workspace using only published
documentation. Target time: under 30 minutes.

## 8. First two-week sprint

Keep the first sprint behavior-preserving and small:

1. Write the registration API decision record and parameter mapping table.
2. Add characterization tests around the current scanmatcher and loop
   registration factories.
3. Define typed registration request/result/diagnostic structures with no ROS
   dependencies.
4. Wrap NDT behind the interface without dynamic loading and prove identical
   output first.
5. Add the `pluginlib` loader only after that proof.
6. Build one separate-workspace identity/example plugin in CI.
7. Publish a short “add a registration plugin” guide.

Sprint exit: the default YAML still works, deterministic replay remains
identical, the example plugin loads from an installed external package, and no
new algorithm-specific branch was added to either central component.

## 9. OSS health metrics

Track architecture and community outcomes together:

| Metric | Initial target |
| --- | ---: |
| Core-file edits required for a new plugin | 0 |
| External example plugin build matrix | Humble + Jazzy |
| Plugin contract-test pass rate | 100% |
| Default deterministic replay regression | 0 bytes |
| Plugin dispatch runtime overhead | < 0.5% |
| Documented plugin setup time | < 30 min |
| Deprecated configuration window | >= 2 minor releases |
| Unversioned public artifact schemas | 0 |

These targets do not replace SLAM quality gates. A plugin can be easy to add
and still be rejected from the default workflow for accuracy, runtime, memory,
map quality, determinism, license, or maintenance reasons.

## 10. Risks and explicit non-goals

- **Over-abstraction:** require a second real implementation before extracting
  each seam and keep policy in the core.
- **C++ ABI fragility:** promise source compatibility, publish a build matrix,
  and reject incompatible API majors at startup.
- **Parameter sprawl:** namespace plugin parameters and validate them once at
  startup; do not copy all plugin knobs into central YAML schemas.
- **Nondeterministic plugin behavior:** provide deterministic fixtures and mark
  capabilities honestly; deterministic offline mapping may reject plugins that
  cannot satisfy the contract.
- **License contamination:** manifests, CI checks, and default-workflow policy
  remain mandatory; loading code dynamically does not remove license duties.
- **Unsafe fallback:** never silently substitute another algorithm after a
  plugin fails to load or fails mid-run.

Non-goals for the first three phases are a distributed plugin marketplace,
runtime hot-swapping during a map, a universal point type, stable C++ binary ABI
across ROS distributions, and replacing ROS 2 composition itself.
