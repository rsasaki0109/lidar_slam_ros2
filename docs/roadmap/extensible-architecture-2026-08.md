# Extensible OSS architecture roadmap

> Status: proposed, 2026-08-20. This roadmap changes architecture and contributor
> experience, not the default SLAM algorithm. Every migration step must preserve
> the existing deterministic replay, accuracy, runtime, memory, map-quality, and
> Autoware bundle gates.

## 0. Current status matrix (2026-08-21)

| Area | Status | Evidence and next gate |
| --- | --- | --- |
| C++14 `RegistrationPlugin` API, typed request/result, capabilities, and failure contract | **Implemented** | [`registration-plugin-api.md`](../architecture/registration-plugin-api.md); installed interface and contract tests pass. The public C++14 interface is consumed by the clean external fixture below. |
| Shell `pluginlib` loader and host/pluginlib hybrid resolver | **Implemented / experimental** | Offline and explicit live-startup discovery, provenance, API/capability/config validation, and failure tests pass. The clean external consumer proof passes on both Jazzy and Humble; external DSO replay/promotion remains pending/No-Go under the independent ODR gate. |
| External author SDK/template and contract guide | **Implemented** | [`registration-plugin-authoring.md`](../registration-plugin-authoring.md), the C++14 template under `examples/`, and isolated template proof passes on both Jazzy and Humble. |
| Built-in NDT same-translation-unit adapter path | **Implemented** | Legacy default remains unchanged; the HILTI exp04 baseline/adapter frontend and map-artifact receipts are byte-identical (the existing indoor absolute profile still has its documented violation). |
| GICP and optional Small GICP/VGICP adapters | **Experimental** | Small HILTI/MID-360 compatibility and the scoped symbol-isolated DSO gate pass for the pinned Jazzy/vendor replay; these are not absolute-accuracy claims, and broader toolchain/live gates remain pending. |
| FAST_GICP / FAST_VGICP | **Pending** | Dependency is absent in the supported host; no class or fallback is advertised. |
| Backend loop-registration/plugin seams | **Implemented / experimental** | Live and offline `graph_based_slam` NDT now resolve the same host-resident `lidarslam_builtin/NdtOmp` `backend_loop` request/session before observable processing; `BackendCore` consumes only the typed interface. R2, the path-independent R4 provenance fixture, the M4a receipt/parser fixture, and the pinned MID-360 three-run artifact comparison pass. That historical receipt **fails only the strict max-RTF gate** (`1.006913460 > 1.0`); the M4b bounded-cache implementation/tests and formal stride-5 MID-360 development-profile gate pass (`max RTF=0.264233831`, wall CV `2.484173052%`, peak RSS `565.222656250 MiB`). M4c HILTI exp04 and exp07 three-run backend regression gates also pass with old optimized artifacts exact; the paired exp04 map check passes at 2%, while the unchanged indoor absolute profile fails on both old/current reports. This closes cache/general regression for these receipts only; official dense-GT/SOTA comparison and broader promotion remain M5 pending. GICP stays an explicit legacy bridge. |
| Competitive SOTA evidence validator | **Implemented / fail-closed; preregistered, not frozen** | Additive schema-v2 mode in `evaluate_competitive_suite_gate.py` separates historical exp02/03/21 regression slots from the primary-fresh partition. Every system must provide every dataset in both partitions with exactly three run records; completion, RTF/RSS, map, and per-sequence regression checks cover both, while aggregate APE and hierarchical CI use fresh only. It requires profile-assigned fresh slots (selection/input/reference/calibration hashes), all rivals, pinned per-system provenance, a common scorer fingerprint, an equal canonical seven-field thread policy, and the remaining safety evidence. Exp14/16/18 are now preregistered as `selected_unopened` in `configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml`; input/GT/calibration hashes are still null, so real evidence is necessarily `INCOMPLETE` until `frozen_unopened`. Exposed exp02/03/21 assets cannot be relabelled fresh. Synthetic boundary/negative tests pass and no README claim is authorized. |
| Live-node plugin preflight | **Implemented / experimental** | Read-only `registration_plugin_enable`, `registration_plugin_class`, and `registration_plugin_allow_external` are validated before pub/sub creation; default constructor behavior is unchanged and runtime hot reload is rejected. External DSO promotion remains **No-Go** until independent ODR, lifecycle, rollback, and Humble/Jazzy replay gates pass. |
| README superiority claims | **No-Go for new claims** | Existing sequence-scoped comparisons remain; no universal or plugin-performance claim is authorized. |

The status table is normative for planning. A replay receipt may establish
offline compatibility without authorizing live default promotion or an accuracy
claim. The live preflight row authorizes only explicit startup selection and
provenance/failure handling; it does not authorize an external DSO accuracy or
production-performance claim.

### M5a competitive evidence-validator slice (2026-08-21)

The first M5 implementation is deliberately a validator, not a benchmark
result. The additive schema-v2 mode of
`scripts/evaluate_competitive_suite_gate.py` preserves the old report-only
`--gate` path while rejecting incomplete victory evidence. It does not treat
the exposed/frozen exp02, exp03, or exp21 slots as fresh. Instead, the profile
has a separate `fresh_holdout_slots` contract. M5b preregisters Exp14, Exp16,
and Exp18 in `configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml`
with status `selected_unopened`; their input-manifest, ground-truth, and
calibration archive hashes are still null, so any real v2 invocation remains
`INCOMPLETE`. Once downloaded and reviewed, each fresh slot must move to
`frozen_unopened` with those identities, and every system/run must repeat
them. The preregistration contains no performance data and does not authorize
a README or SOTA claim.

The contract fixes two explicit partitions: the exposed/frozen historical
`holdout_slots` (`exp02`, `exp03`, `exp21`) as a regression partition, and
profile-assigned `fresh_holdout_slots` as the primary-fresh partition. `ours`
plus every required rival must provide exactly three records for every dataset
in both partitions, including failed records. Historical records are required
for regression checks but are excluded from aggregate APE and CI; fresh records
alone determine the >=10% aggregate and superiority interval. Pinned
revision/container/toolchain/config provenance remains system-specific, while
the scorer fingerprint and input/reference/calibration/machine/hardware/
thread/Release identity are common. The required thread policy is
`cpu_affinity`, `max_threads`, `omp_num_threads`, `openblas_num_threads`,
`mkl_num_threads`, `tbb_num_threads`, and `accelerator_policy`, with a
canonical mapping hash recorded in the receipt. Complete runs must provide
finite APE, processing RTF `<=1`, peak RSS, map metrics, artifact hashes, and
zero catastrophic/verified-false-loop evidence.

Accuracy is an aggregate over datasets, not nine pseudo-independent runs. The
three runs are averaged within each dataset; the fixed seed `20260821`
resamples dataset clusters for 10,000 draws and requires the 95% percentile
lower bound of the best-rival-minus-ours APE difference to be positive. A
minimum aggregate improvement of 10%, per-sequence primary regression at most
2% against that sequence's best rival, per-dataset/per-rival map non-regression,
RSS, fresh-slot completeness, and all-rival completeness are separate required
checks. The 95% CI is a fixed-seed two-stage bootstrap: resample fresh dataset
clusters, then independently resample the three runs within each selected
dataset for ours and each rival; runs are never treated as pseudo-independent
datasets. The validator emits matching JSON/YAML `PASS`, `FAIL`, `INCOMPLETE`,
or `INVALID` receipts and records profile/evidence SHA-256 identities.
Synthetic pass, exact-boundary, false-freshness, pending-slot, hash-mismatch,
failed-run, old-schema, identity, RTF, per-sequence/map, and all-rival CI
negative tests are the current evidence; real fresh-slot competitor evidence
remains pending and cannot support a README or SOTA claim.

### M5b fresh-holdout selection preregistration (2026-08-21)

The read-only exposure audit selected three HILTI 2022 additional sequences
with official dense 6DoF IMU references and the same Phasma sensor/calibration
contract: Exp14 Basement 2, Exp16 Attic to Upper Gallery 2, and Exp18 Corridor
Lower Gallery 2. The machine-readable selection receipt is
`configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml`; its
profile hash is repeated in all three fresh slots. The receipt pins official
revision `e62017f907007fdc5ab8c721842e4ae7359d7f49`, bag sizes and LFS hashes,
GT file blob identities, calibration YAML blob identities, license, exposure
audit commands, and the blind policy. It intentionally leaves downloaded
input-manifest, GT-content, and calibration-archive SHA-256 fields null.

The slots are therefore `selected_unopened`, not `frozen_unopened`, and the
schema-v2 validator must remain `INCOMPLETE`; self-declared evidence cannot
promote them. The required order is selection-receipt commit, download and
hash without opening GT content, update to `frozen_unopened`, then the first
run and only afterward scoring. No performance result, README claim, or SOTA
claim is authorized by this preregistration. Its freshness is bounded to the
recorded repository/workspace/media audit and is not a proof of external
historical use absence.

### M5b execution-identity freeze (2026-08-21)

The next pre-run artifact is
`configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml`.
It records the ours/GLIM/FAST-LIVO2 repository revisions and URLs, runner and
configuration hashes, container recipe/tag/digest state, toolchain source,
scorer fingerprint, machine-fingerprint source, exact `Release`, and the
seven-key thread policy (`cpu_affinity`, `max_threads`, `OMP`, OpenBLAS, MKL,
TBB, and accelerator policy). The receipt now records a verified clean ours
revision, fresh machine fingerprint, and CPUs 0--7 with all thread limits set
to 8; overall status remains `pending` because containers/toolchains and fresh
input hashes are unresolved. The recorded policy must be enforced with
`taskset`/Docker `--cpuset-cpus` plus the explicit OMP/OpenBLAS/MKL/TBB
environment variables before execution. FAST's upstream visual
configuration remains an external-container artifact until its pinned image is
built; GLIM's CPU track disables visual input while preserving the canonical
camera messages for the cross-system input contract.

`scripts/check_competitive_execution_selection.py` is a read-only,
fail-closed preflight. It verifies the profile's receipt path/SHA, file and
tree hashes, 40-hex revisions, `sha256:<64hex>` container digests, exact
`Release`, scorer/machine files, and complete equal thread policy. It emits
machine-readable JSON/YAML and reports missing values as `INCOMPLETE`; it does
not build, download, open GT, or run SLAM. Consequently M5 remains
`INCOMPLETE` until the receipt is refreshed after a clean revision, reproducible
images/toolchains, current machine capture, and frozen fresh-holdout hashes.
The scorer digest is recomputed from sorted canonical JSON entries containing
file name, repository-relative path, measured SHA-256, and policy; a stored
label cannot override a changed scorer. The CLI records profile SHA, execution
receipt SHA, scorer fingerprint, and the canonical thread-policy hash. Pending
status values are not accepted merely because their digest fields are filled.

The profile/receipt hashes use `canonical_profile_sha256_v1`: the parsed full
profile mapping is deep-copied, only
`competitive_slam_profile.evidence_gate_v2.execution_selection_receipt_sha256`
is removed, and the result is encoded as sorted compact UTF-8 JSON before
SHA-256. The receipt stores this canonical profile hash and kind, while the
profile stores the raw full-file receipt SHA. No other profile mutation is
excluded, so wrong-kind or mismatched hashes fail closed without a mutual-hash
cycle.

The capture/finalize workflow is
`scripts/capture_competitive_execution_identity.py`. `capture` emits a
machine-readable observation artifact for the current receipt: git revision,
tracked/untracked/clean provenance, non-secret machine fingerprint, explicit
thread-environment values, and local Docker image IDs. With an explicitly
bound local image, it additionally runs bounded `--pull=never --network none
--read-only` compiler/linker/ROS/PCL/Eigen/OpenMP probes and binds the
toolchain fingerprint to the inspected image digest; source bindings provide
Git provenance only. `finalize` verifies receipt ownership and emits a decision, but never
writes or promotes the reviewed receipt. It also records that no fresh bag or
GT was opened. The current artifact is intentionally `INCOMPLETE` because the
worktree is dirty, rival images/toolchains are not system-container-ready, and
the seven-key policy still contains nulls. A future clean freeze must rerun the
capture in each pinned system container, review the resulting hashes, then
update the receipt explicitly before the existing checker can report ready.
Both commands accept explicit repeatable `--source SYSTEM=PATH` and
`--image SYSTEM=TAG` bindings; absent bindings produce an exact read-only
compiler/linker/ROS/PCL/Eigen/OpenMP probe manifest rather than guessed
readiness. The finite-state verifier compares measured revision, clean
provenance, image digest, toolchain fields/fingerprint, machine identity, and
canonical thread policy. Thus a complete ready/frozen synthetic contract can
be `PASS`, but no pending production receipt can be promoted implicitly.

### M3 live backend NDT migration gate

The live `graph_based_slam` component now constructs the host-resident NDT
adapter in its own translation unit and resolves it through the shell-side
`RegistrationResolver`. The backend role uses an explicit typed request with
the historical loop defaults: `resolution=ndt_resolution`, epsilon `0.01`,
`maximum_iterations=100`, `step_size=0.1`, `outlier_ratio=0.55`,
`num_threads=ndt_num_threads`, and `DIRECT7`. The resolver validates the API,
license, target policy, initial-guess/aligned-source capabilities, and mean
correspondence metric before `initializePubSub()`; failure is actionable and
there is no fallback or hot reload. Startup logs record backend kind, canonical
class, API/license, capability bits, and host provenance markers.

The live `registration_method` parameter is read-only as well, so a runtime
attempt to switch the backend is rejected rather than becoming a silent no-op.
The preflight barrier runs before sensor subscriptions and before the optional
degeneracy CSV is opened; constructor-owned TF objects exist earlier, but no
sensor data or map processing is observable before successful preflight.

`BackendCore::searchLoopForSubmap()` has a typed
`RegistrationPlugin` overload. It now submits the host-prepared source/target,
3D-BBS-derived initial guess, and consumes typed aligned cloud,
convergence/failure, transform, and fitness fields before applying the existing
loop gates and log lines. The offline runner is handled by the M3b section
below: its NDT path uses the same host-resident resolver/session boundary,
while its explicit GICP legacy bridge remains shell-owned. There is no PCL
overload or second registration implementation in `BackendCore`. Live GICP
keeps its legacy construction through the same typed bridge in this slice.

The R2 fixture runs both the direct legacy PCL object wrapped by
`PclRegistrationAdapter` and the host-resident `NdtOmp` typed adapter with the
same preprocessing/defaults; proposal pair/found state, relative transform,
fitness, convergence/failure path, overlap/gate result, and every log line are
bitwise-identical. Deterministic arrival batching, strict gate rejection, and
empty-cloud behavior remain covered. The preflight/bridge tests also cover
exact backend parameter mapping, invalid configuration rejection, empty target
rejection, and reset to `kNotConfigured`. The live component target
embeds the NDT implementation in the same translation unit and does not link
`liblidarslam_default_plugins.so`; only the loader shell DSO appears in
`ldd`. A full live/offline replay gate is intentionally not claimed here; it is
part of the remaining R4 real-bag gate.

### M3b offline backend NDT migration gate

`graph_slam_offline_runner` now uses the same
`backend_registration::BackendRegistrationRequest` as the live component. The
request fixes `role=backend_loop`, class ID, API/license policy, required
capabilities, and sorted typed NDT parameters. The runner resolves
`lidarslam_builtin/NdtOmp` through `RegistrationResolver`, retains the session
and typed plugin for the full run, and injects only `RegistrationPlugin&` into
`BackendCore`. The concrete adapter is included in the runner translation unit
and the default plugin DSO is not linked, preserving the ODR-safe host path.

Before the first bag message, the runner writes
`registration_plugin_receipt.yaml` with backend kind, canonical metadata,
capabilities, requirements, typed parameters, and host/pluginlib provenance.
The shared path-independent canonical identity is covered by an R4 unit
fixture comparing two independent live/offline-style resolves. `GICP` remains
an explicit legacy PCL bridge, and unknown methods fail closed without NDT
fallback. No full real-bag R4 replay, external-DSO promotion, or README claim
is authorized by this slice.

### M4a deterministic measurement and receipt gate

The reusable
[`run_offline_determinism_check.sh`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scripts/run_offline_determinism_check.sh)
harness now measures every backend run with `/usr/bin/time -v`, derives RTF from
the bag's declared duration, and emits per-run metrics plus YAML, JSON, and
Markdown summaries. It requires byte-identical loop edges, optimized
trajectory, and canonical `registration_plugin_receipt.yaml` across all runs;
the receipt is fail-closed for the `backend_loop`/host NDT identity and typed
NDT configuration. A versioned `.complete` marker stores the execution
identity (runner/script/setup/parameters/complete bag tree/optional input
hashes, overrides, and thread-affecting environment). Resume rejects changed
inputs or partial artifacts, while a non-empty output run is protected from
accidental overwrite.

The machine receipt includes git revision/dirty state, a tracked-diff and
untracked-content worktree fingerprint, compiler/CPU/memory/OpenMP settings,
and hashes for resolved dynamic dependencies. Optional RTF, peak-RSS, and
wall-time-CV limits are report-only when omitted and fail closed when supplied;
`--require-ape` similarly makes the existing reference-TUM scorer mandatory.
The fixture suite covers success, byte/receipt mismatch, malformed thresholds,
required-APE failure, marker/input invalidation, and stale-output protection.
This is an **Implemented / experimental** measurement tool. The formal pinned
MID-360 result below applies the existing R4/README evidence policy: it proves
three-run artifact determinism and reports a strict performance-gate failure,
not a new accuracy or SOTA claim. No default switch or new README claim is
authorized by M4a. Local fixture/documentation checks on 2026-08-21 passed
(15 harness tests, 6 docs-entrypoint tests, strict MkDocs, and diff-check).

#### M4a pinned MID-360 three-run receipt (2026-08-21)

The formal receipt used the frozen backend-input bag
`/media/sasaki/aiueo1/benchmarks/mid360_public/backend_input_20260713/backend_input`
(`341.250776531 s`), the Jazzy install, and the checked-in
`lidarslam_mid360_rko_graph.yaml` (`ndt_num_threads=0`, `DIRECT7`; inherited
OpenMP environment empty/default). The complete local receipt is
`/tmp/lidarslam-m4-r4-mid360-final.jSP22j`. Its reproducibility identifiers
are recorded here so a future run can be rejected if its execution identity
changes:

```text
execution_identity_sha256=f7bf00d1d3cb4ed91eaf5522dc6bdb48e169061b119c94677b4de9c86ae7e0dc
runner_sha256=04382d6e77a7faf362e9a21ea26be559a00ee082c728b536a974e5c5fa469103
script_sha256=7afae5d4e73c29fbd8a567938a69b2079784019e6bf2524db5281df957d85c1b
```

All three runs completed with byte-identical loop-edge, optimized-trajectory,
and `registration_plugin_receipt.yaml` artifacts. The measured rows were
`wall_sec=343.16/339.08/343.61`,
`RTF=1.005594781/0.993638765/1.006913460`, and
`peak_rss_mib=548.10/548.26/547.95`; wall-time CV was `0.595904306%`.
With `max_rtf=1.0`, `max_peak_rss_mib=672`, and
`max_wall_cv_percent=5.0`, the receipt is **FAIL only because the maximum RTF
was 1.006913460**. RSS, CV, and determinism passed. This is a performance-gate
result, not a trajectory or registration-receipt compatibility regression.

The optional historical reference produced an APE-like proxy of
`1.1072674022 m` from 218 associated pairs (358 rejected). It is a
GLIM/reference trajectory proxy rather than ground truth, so it is not an
accuracy, SOTA, or competitor claim. README claims remain unchanged.

#### M4b bounded target-cache implementation slice (2026-08-21)

The first M4b code slice is implemented without changing the generic plugin
API or the frontend default. `BackendCore::TargetCloudCacheValue` now retains
only the representation required by the candidate variant: distance-only
candidates retain the final filtered target, ScanContext retains its filtered
target, and the 3D-BBS variant additionally retains its unfiltered BBS
aggregate. The variant is part of the exact key, so these representations
cannot cross-hit; bounded capacity, revision/pose invalidation, and the
existing 3D-BBS path remain fail-safe.

`NdtOmpRegistration` has an opt-in deterministic target-cell LRU. Capacity
zero is the default for frontend and external requests. The backend live and
offline canonical request sets the typed integer
`target_cell_cache_capacity=3`, which is emitted in the registration receipt.
Each entry retains the target shared pointer and a separately configured NDT
instance; a hit switches instances without copying `target_cells_`, while
prior, distance, exception, and reset guards remain per active instance.
Unit coverage includes representation variants, capacity/range, hit/miss,
deterministic eviction, reset, and backend receipt identity. This is an
implementation/compatibility result only; the MID-360 performance gate has
not been rerun and no speedup or RSS claim is authorized yet.

#### M4b performance-margin policy (fulfilled for pinned MID-360; holdouts pending)

The M4b acceptance policy targets **5% headroom** (`max RTF <= 0.95` across
three runs), while retaining the current hard gate of `max RTF <= 1.0`. Keep
the bag tree, params,
Jazzy/toolchain/dependency fingerprint, `ndt_num_threads=0`, OpenMP environment,
ROS domain isolation, and measurement script fixed. Profile first, change one
performance mechanism at a time, and record fresh runner/script/execution
identity hashes. Reject a candidate if required trajectory, loop-edge, or
registration-receipt hashes change, wall CV exceeds 5%, or peak RSS exceeds the
agreed non-regression budget. The formal pinned MID-360 receipt below meets
these performance/determinism gates. The same policy remains required for
HILTI exp04/exp07, GT holdout, and any broader production or accuracy claim;
no README/SOTA claim is implied by this profile-scoped result.

#### M4b stride-5 MID-360 development-profile candidate (2026-08-21)

Candidate 2 was evaluated with an explicit
`loop_search_query_stride:=5` override on the pinned MID-360 backend-input bag,
then frozen in `lidarslam/param/lidarslam_mid360_rko_graph.yaml` as a
MID-360 development-profile setting. The run kept `ndt_num_threads=0`, unset
the OpenMP environment, and used the Jazzy runner with ROS domain isolation.
The complete receipt is
`/tmp/lidarslam-m4b-mid360-stride5.JN9qW8`; its execution identity is
`54423801ae9bd9691f129f84f064dde6f5cb3e629356e334d0b280a3594839cf`.

The single run measured wall `85.020 s`, RTF `0.249142290`, peak RSS
`565.320 MiB`, and CPU `495%`. It accepted edge `6 -> 604` with fitness
`0.47313574856385182`. Loop edges, raw/optimized/refined trajectories, and
the refinement report were byte-identical to the stride-1 run; the
registration receipt was also identical. The historical GLIM/reference
trajectory proxy was unchanged at APE RMSE `1.1072674021694795 m` from 218
pairs. This is a strong compatibility/performance candidate, but it is one
run only: three-run wall-CV/determinism, HILTI exp04/exp07 regression, and
ground-truth holdout gates are still required. The proxy is not GT, an
absolute-accuracy result, or an SOTA/competitor claim, and no README claim or
global parameter default is authorized by this receipt.

#### M4b formal pinned MID-360 three-run gate (2026-08-21)

The frozen MID-360 development profile was then executed three times with
the same backend-input bag
`/media/sasaki/aiueo1/benchmarks/mid360_public/backend_input_20260713/backend_input`
(`341.250776531 s`), checked-in params, Jazzy runner, `ndt_num_threads=0`,
unset OpenMP environment, and ROS domains 212--214. The command supplied
`--require-ape`, `--max-rtf 0.95`, `--max-peak-rss-mib 672`, and
`--max-wall-cv-percent 5.0`. The complete receipt is
`/tmp/lidarslam-m4b-mid360-stride5-gate.VZX2kH` with execution identity
`5e06bf535bbf75e1de53c122655e55bcbd4d459d7723989b4472de2e0c220781`.
The runner, script, params, and bag-tree hashes were respectively
`321b231d212ef804169f0dfb2aa509067000a40e2393aa66a2c0603f823c5357`,
`a7fa30b9a53ee635654360bb08992970a0dd79a5400235739ff9299647d1afc0`,
`534db7c3d9d41baa9d3bc95f9d2b87277479f9cf7c84b80e2182cfab45958cb6`, and
`408e6988cc7b1bb960000c66e798520b85ae7c0ffe29445394964f1751920016`.

The machine receipt, YAML/JSON/Markdown summaries, and all three completion
markers report `PASS`. Wall time was `85.05/88.98/90.17 s` (mean
`88.066666667 s`, CV `2.484173052%`), RTF was
`0.249230202/0.260746659/0.264233831`, and peak RSS was
`564.718750/565.035156/565.222656 MiB`. The three-run artifact hashes were
identical:

```text
loop_edges_sha256=5e30036d9190933e9659805caac032cc675ba968700a096c788f12560026c6d
trajectory_raw_sha256=19e0b2c1c4cf3ec85446fd6354f0014d26a0af5cbe7e03966f9b2de993360378
trajectory_optimized_sha256=9b8fd9faff62e7f8a20073373c7d85f5153bee0d18128a4e5ed957df351574ac
trajectory_refined_sha256=2f80eabff43ed11eb1760f1ff7cc165aa5240e3c2bf00b787354be9a0d875042
refinement_report_sha256=a02355cc8b8e02a888ae348ae2ec9528c5ee391c8bab8a1d96b290e4a5add1d4
registration_receipt_sha256=7c4f90c758abcba03fd7bae1f709364e496d47a11a68ebc33ae0d1d325d8822f
```

The receipt remained `role=backend_loop`, host `lidarslam_builtin/NdtOmp`,
and `target_cell_cache_capacity=3` on every run. The accepted loop edge was
`6 -> 604` with fitness `0.47313574856385182`. Required APE completed with
the GLIM/reference trajectory proxy at `1.1072674021694795 m` (218 pairs,
358 rejected); it is not ground truth or an absolute-accuracy/SOTA result.
This closes the pinned MID-360 development-profile performance and
determinism gate only. HILTI exp04/exp07, GT holdout, Humble/toolchain matrix,
and any global default or README claim remain separate pending gates. The
earlier M4a strict-RTF **FAIL** receipt is intentionally retained as history.

#### M4c HILTI backend regression gates (2026-08-21)

M4c extends the same fixed-run measurement contract to two HILTI backend-input
receipts without changing the default parameter profile or the absolute map
quality profile. Both gates used Jazzy, the current installed runner, unset
OpenMP variables, three isolated ROS domains, `--require-ape`, and the fixed
RTF/RSS/CV limits (`0.95`, `672 MiB`, `5%`).

The exp04 receipt is
`/tmp/lidarslam-m4c-hilti-exp04-gate.pzufEB`, execution identity
`d991e3c17399cb6c7f974a79ac58321fbdfc179efc7c37d02d4b81ea2796765d`. All
three runs are byte-identical and pass the performance gate: wall
`2.71/2.90/2.73 s` (CV `3.066357758%`), maximum RTF `0.010347643`, and peak
RSS `272.167968750 MiB`. The optimized trajectory is the old artifact exactly
(SHA-256 `3e94a3482b67e56de515cb830fcd374dd0b6cab5ffb7c150e1ee414c5922b390`),
and the loop-edge SHA-256 is
`4c90fb5921414b59a43d90c7c5ff9bef82681c77254fa48f9ba683dff8f3768d`.
The old-map/current-map paired checker passes at the 2% budget. The
unchanged `indoor_construction` absolute profile still reports
`mme_valid_fraction=0.816687654` for the old map and `0.816816915` for the
current map, both below `0.90`; this is recorded as profile applicability,
not hidden by the paired pass.

The exp07 receipt is
`/tmp/lidarslam-m4c-hilti-exp07-gate.zCELMb`, execution identity
`bd3cd4496f5c4c6c28d274397a632c497aac19a6c6911dda0cc3ff7e165ea67e`. The
three-run machine YAML/JSON/Markdown summaries and completion markers all
report `PASS`. Wall time is `1.90/1.90/1.97 s` (CV `1.715683698%`), maximum
RTF `0.050166829`, and peak RSS `201.136718750 MiB`. The optimized trajectory
matches the old artifact exactly (SHA-256
`353226fb6b7b8ff430dca063224fe3b059a1d44eace7bb32204044964620162a`), with
loop edges SHA-256
`4c90fb5921414b59a43d90c7c5ff9bef82681c77254fa48f9ba683dff8f3768d`.
Historical interpolated APE is `0.6186851452574647 m` from 5/6 sparse GT
points, matching the stated old value; one sparse reference point is
rejected by the 2 s matching limit. Each run records the canonical
`backend_loop` host NDT receipt and `target_cell_cache_capacity=3`.

These are compatibility, determinism, and resource-regression gates for the
named HILTI inputs. They are not official dense-GT scores or SOTA/competitor
comparisons. M5 must add the dense GT protocol, holdout datasets, and any
toolchain matrix before a broader accuracy or production claim; README remains
unchanged.

### M2 startup preflight gate

The component-owned preflight is the smallest live integration slice. With the
selector disabled, the normal one-argument `ScanMatcherComponent` constructor
still builds the legacy same-translation-unit NDT path (and the existing direct
GICP/FAST/SMALL paths). With the selector enabled, the component builds the
shared typed request, resolves the explicit host or external class, validates
API major/minor, SPDX license, capabilities, typed configuration, method and
Small variant, and injects the session before creating publishers or sensor
subscriptions. Unknown, unavailable, mismatched, or invalid classes fail
closed; no fallback or runtime hot reload is allowed.

The parameters are read-only after construction:

| Parameter | Default | Gate |
| --- | --- | --- |
| `registration_plugin_enable` | `false` | Must be paired with a non-empty class ID. |
| `registration_plugin_class` | `""` | Explicit `lidarslam_builtin/<name>` host ID or external pluginlib ID. |
| `registration_plugin_allow_external` | `false` | Required for external pluginlib DSOs; it is an explicit risk acceptance, not a fallback switch. |

The offline runner uses the same component-owned resolution and reads the
resolved session/provenance for its receipt; it does not resolve or inject a
second session. The acceptance tests cover default behavior, host success and
provenance, external wiring with explicit risk acceptance, unknown/missing
selectors, method/variant mismatch, read-only mutation rejection, and session
lifetime. The external success test is wiring-only. The independent DSO replay
and ODR gate remains No-Go, so this slice does not change the default or permit
README superiority claims.

### M0 clean external consumer gate

The reproducible install-space check is
[`scripts/run_registration_plugin_consumer_check.sh`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scripts/run_registration_plugin_consumer_check.sh).
It starts from `/opt/ros/$ROS_DISTRO/setup.bash`, copies the public interface and
shell loader into an isolated underlay, builds the fake external package in a
separate overlay with `-DCMAKE_CXX_STANDARD=14`, and runs the underlay loader
test after sourcing only those temporary installs. It does not source the
repository's `install/setup.bash`; `--keep-work-dir` preserves logs and the
receipt for inspection. The C++14 claim applies to the public interface and
external fake consumer; the shell loader itself remains C++17 as documented in
the ADR.

The required receipt fields are:

```text
clean_external_consumer_proof=pass
ros_setup=/opt/ros/<distro>/setup.bash
base_packages=lidarslam_plugin_interfaces,lidarslam_registration_loader
external_package=lidarslam_fake_registration_plugins
external_cxx_standard=14
gtest_filter=RegistrationPluginLoader.DiscoversInstalledFakeExternalClasses:RegistrationPluginLoader.LoadsExternalPluginAndKeepsLoaderAlive
repository_install_sourced=false
```

Both matrix legs have passed with the two selected tests. Jazzy was run from
the local `/opt/ros/jazzy` installation. The Humble proof used the read-only
repository mount and `ros:humble-ros-core` image digest
`sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988`.
Its toolchain receipt was: GCC `11.4.0-1ubuntu1~22.04.3`, PCL
`1.12.1+dfsg-3build1`, Eigen `3.4.0-2ubuntu2`, and
`ros-humble-pluginlib 5.1.4-1jammy.20260717.234837` from `/opt/ros/humble`.
The C++14 claim remains limited to the installed public interface and external
fake consumer; the shell loader is C++17.

The Humble receipt fields were:

```text
clean_external_consumer_proof=pass
ros_setup=/opt/ros/humble/setup.bash
base_packages=lidarslam_plugin_interfaces,lidarslam_registration_loader
external_package=lidarslam_fake_registration_plugins
external_cxx_standard=14
gtest_filter=RegistrationPluginLoader.DiscoversInstalledFakeExternalClasses:RegistrationPluginLoader.LoadsExternalPluginAndKeepsLoaderAlive
repository_install_sourced=false
```

### M1 external author template gate

The first author-facing SDK slice is deliberately a small, buildable package,
not a second algorithm implementation. The template at
`examples/lidarslam_registration_plugin_template` demonstrates the complete
metadata, capability, typed-configuration, target, alignment, reset, and
pluginlib manifest contract. Its identity alignment returns a transformed
`aligned_source` consistent with the requested initial guess; it makes no
accuracy or SOTA claim. The negative fake package remains separate and is used
only by loader failure tests.

The reproducible proof is:

```bash
bash scripts/run_registration_plugin_template_check.sh --keep-work-dir
```

It starts from `/opt/ros/$ROS_DISTRO/setup.bash`, copies only
`lidarslam_plugin_interfaces` and `lidarslam_registration_loader` into a clean
underlay, builds the template in a separate overlay, and runs both the direct
C++14 contract fixture and the C++17 shell-loader fixture. The template test
checks configure acceptance/rejection, finite target/source validation,
capability mismatch, enabled/disabled initial-guess behavior, aligned-cloud
consistency, and reset to `kNotConfigured`. The loader test destroys the
loader before using the retained session, and checks exact XML/metadata class
identity, SPDX license acceptance, manifest/library provenance, configuration,
alignment, and reset.

The local Jazzy receipt is:

```text
m1_template_proof=pass
ros_setup=/opt/ros/jazzy/setup.bash
base_packages=lidarslam_plugin_interfaces,lidarslam_registration_loader
template_package=lidarslam_registration_plugin_template
template_class=lidarslam_registration_plugin_template/Identity
plugin_cxx_standard=14
loader_test_cxx_standard=17
metadata_class_id=lidarslam_registration_plugin_template/Identity
metadata_license=BSD-2-Clause
repository_install_sourced=false
```

The workflow runs this proof for both Humble and Jazzy after dependency
installation and before the normal workspace build. Both matrix legs have
also passed independently. The Humble run used a read-only repository mount
and the same `ros:humble-ros-core` image digest recorded by M0
(`sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988`).
The C++14 source claim applies to the public plugin and its direct consumer,
while the shell loader remains C++17.

## 1. Goal

Make `lidarslam_ros2` a platform where an external contributor can add a
registration method, place recognizer, loop verifier, optimizer, map refiner,
or exporter without editing the central ROS components.

Success means:

- a useful extension can live in a separate ROS package and repository;
- the default `RKO-LIO + graph_based_slam` workflow remains unchanged;
- online ROS components and deterministic offline runners use the same typed
  contract; explicit live startup preflight is implemented, while external
  live-plugin promotion remains pending its safety gates;
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

1. the live scan-registration default remains a string-driven legacy path inside
   `ScanMatcherComponent`; the typed runtime slot and host resolver now support
   explicit startup preflight and offline injection, while later call sites
   still require migration;
2. the live and offline loop-registration NDT paths now share a typed host
   session, canonical backend role/request, and receipt; the live and offline
   GICP compatibility paths retain explicit legacy construction until their
   separate resolver characterization;
3. four place-recognition implementations and their databases are compiled
   directly into `BackendCore`, whose public getters expose their concrete
   database types;
4. loop verification, pose-graph optimization, map refinement, and export are
   callable modules but not external extension boundaries;
5. the registration role now has an installed versioned source API,
   capability negotiation, plugin manifests, and shell contract tests; the
   equivalent kit does not yet exist for backend, recognition, optimization,
   refinement, or export roles;
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
