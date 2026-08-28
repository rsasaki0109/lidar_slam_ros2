# Extensible OSS architecture roadmap

## Future-only systems direction (post-SOTA evidence phase)

After the current evidence phase and its independent SOTA gates are complete,
future work is split into two tracks. Both tracks are exploratory and additive:
they are not implemented here, make no present benchmark or SOTA claim, do not
authorize replay/GT/scorer activity, and do not change the current SLAM default.

### Track A — lifelong and long-term SLAM

Track A studies a durable map lifecycle rather than a new default estimator. A
multi-session map would be a version DAG with session and submap provenance,
explicit branch/merge and rollback semantics, and change classification for
dynamic, seasonal, and structural changes. The lifecycle would also study
bounded storage and compaction, catastrophic-forgetting detection,
relocalization and recovery, fleet federation with conflict resolution, and
privacy/security controls for retained observations and shared map state.

The differentiator is deliberately an evidence contract around map history,
change decisions, recovery, and privacy—not a claim that the estimator is
better than other OSS. Each capability below is a future gate with a declared
dependency and time horizon; a missing dependency is a No-Go, never an
implicit pass.

| Track A capability | User value | Technical capability to prove | Public dataset or self-contained protocol | KPI and Go/No-Go | Dependencies | Timing |
| --- | --- | --- | --- | --- | --- | --- |
| Long-term change robustness | Keep localization useful across seasons, weather, construction, and dynamic-object turnover. | Classify dynamic/seasonal/structural change while preserving stable geometry and session provenance. | Repeated-route 24-hour/7-day simulation plus controlled weather, construction, and dynamic-object interventions. | False-change rate, localization availability, drift versus session age; **Go** only with predeclared bounds and crash-safe replay, otherwise **No-Go**. | Versioned observations, calibration identity, change-policy plugin, replay harness. | Mid-term |
| Multi-session merge and localization | Reuse prior maps and recover after disconnected or interrupted sessions. | Version-DAG branch/merge, provenance-aware relocalization, conflict detection, and deterministic rollback. | Public multi-session robotics sequences where available, plus a reproducible repeated-route protocol with injected branch conflicts. | Merge conflict loss, availability, relocalization time, drift, rollback determinism; any nondeterministic merge is **No-Go**. | Map schema/migrations, append-only event log, stable localization API. | Mid-term |
| Change detection and map aging | Prevent stale or transient observations from silently becoming permanent map state. | Age/retire evidence, distinguish transient from structural change, and expose reviewable map events. | Seasonal/revisit sequences and staged layout/roadway changes with held-out intervention schedules. | Structural-change precision/recall, false alarm rate, time-to-review, stale-map rate; missing provenance is **No-Go**. | Change classifier, policy plugin, session clock, review/export tooling. | Near-to-mid term |
| Bounded-memory map lifecycle | Keep long deployments operational under finite storage. | Crash-safe compaction, retention policies, submap eviction, and version-preserving recovery. | 7-day stress protocol with restart/crash injection and declared storage quota. | Storage growth bound, compaction loss rate, recovery time, byte-for-byte replay; quota breach or data loss is **No-Go**. | Versioned map store, transactional event log, filesystem durability contract. | Mid-term |
| Failure detection and rollback | Return to a known-good localization state after bad data, software, or storage events. | Detect divergence/catastrophic forgetting, select a known-good version, and restore host-owned state transactionally. | Fault-injection protocol covering process crash, corrupted event, rejected candidate, and sensor interruption. | Detection latency, recovery success/time, rollback determinism, zero partial publication; leaked state is **No-Go**. | Activation transaction, health/observability API, durable checkpoints. | Near-to-mid term |
| Fleet federation and privacy | Share useful map updates across robots without forcing raw-data centralization. | Conflict-aware cross-robot merge, bounded synchronization, access policy, provenance, and privacy-preserving retention. | Multi-robot intermittent-link protocol with divergent branches, delayed merges, and redacted/raw-data policy variants. | Cross-robot consistency, convergence time, bandwidth/storage bound, conflict loss, privacy-policy violations; violation is **No-Go**. | Federation transport, conflict resolver, key/identity policy, secure storage. | Long-term |

Track A is not exit-ready until a reproducible harness covers all of the
following, with crash-safe replay and an auditable lineage for every result:

| Exit metric | Required future evidence |
| --- | --- |
| Long-duration operation | At least 24-hour and 7-day simulation or replay runs, with restart/crash injection. |
| Repeated-route aging | The same routes revisited across months and multiple sessions, with immutable session identities. |
| Localization availability | Availability and recovery distributions, including relocalization after map branch/rollback and sensor interruption. |
| Drift versus session age | Drift curves indexed by session age and map-version depth, with confidence bounds. |
| False change rate | Dynamic/seasonal/structural classification error, including false structural-change alarms. |
| Rollback determinism | Replaying the same event prefix produces the same selected version, map bytes, and localization result. |
| Storage growth bound | Measured growth, compaction safety, and a bounded-retention guarantee under the declared workload. |
| Recovery time | Time to restore a usable map/localization state after process, host, or storage failure. |
| Cross-robot consistency | Version/conflict convergence and localization agreement across independently operating robots. |

### Future-only lifecycle invariants and candidate promotion targets

The Track A capabilities above are intentionally decomposed into invariants so
that a future implementation cannot pass by reporting only average accuracy.
The numerical values in this table are proposed planning targets, not current
benchmark thresholds or SOTA evidence. They must be ratified in a versioned
future profile before any acquisition, replay, GT, or scorer work; until then
every row is **NOT_READY**. A missing, non-reproducible, or non-finite metric is
an unconditional **No-Go**.

| Future invariant | Candidate KPI and minimum evidence | Go / No-Go | Explicit non-goal |
| --- | --- | --- | --- |
| Map-generation DAG and safe merge | 100% of sessions/submaps/events have immutable parent and source hashes; 100 deterministic replays produce identical lineage and map bytes; injected branch conflicts have 0 silently dropped updates. | **Go** only with append-only provenance, conflict receipts, and transactional merge/rollback; any orphan, silent overwrite, or nondeterminism is **No-Go**. | No automatic merging of unreviewed maps and no claim that a DAG alone improves accuracy. |
| Seasonal/time change detection | Report precision, recall, F1, false-structural-change rate, and time-to-review separately for dynamic, seasonal, and structural classes on a held-out intervention set; candidate target F1 >= 0.95 for structural changes and false-structural-change rate <= 0.02. | **Go** only when class labels, holdout identity, and review latency are independently auditable; missing class separation is **No-Go**. | No silent deletion or promotion of map state from an unverified classifier. |
| Aging, forgetting, and bounded storage | Under a declared retention policy, 7-day runs stay within the configured byte quota, have 0 orphaned chunks, preserve a recoverable lineage, and show byte-for-byte replay after compaction/restart. | **Go** only if aging/forgetting decisions are reversible or provenance-preserving; quota breach or unrecoverable forgetting is **No-Go**. | No claim of indefinite memory, lossless retention beyond the declared quota, or deletion without policy evidence. |
| Rollback and multi-session relocalization | 100% recovery in injected bad-update, process-crash, storage-fault, and sensor-gap cases; report p50/p95 relocalization and rollback time with 0 partial publications. | **Go** only with a known-good checkpoint and deterministic recovery receipt; one leaked bad state is **No-Go**. | No safety certification or guarantee of recovery from arbitrary physical damage. |
| Fleet federation, privacy, and safe merge | In intermittent-link multi-robot trials, 100% of accepted updates retain robot/session identity, 0 cross-branch ID collisions or policy violations occur, and repeated merges converge to identical bytes; report bandwidth and storage bounds. | **Go** only with explicit authorization, redaction, key/identity, and conflict receipts; any raw-data policy violation or lost conflict is **No-Go**. | No mandatory cloud, unrestricted raw-data upload, or identity/privacy guarantee without an independently reviewed threat model. |
| Long-duration operation | Complete one 24-hour and one 7-day run per declared hardware/profile, including restart and crash injection, with availability, drift-vs-age, storage growth, and recovery distributions. | **Go** only when every interval and failure is accounted for; timeout, missing intervals, or a substituted shorter run is **No-Go**. | No extrapolation from short runs to 24-hour/7-day reliability. |

These targets remain future-only even when an experimental implementation is
available. They are separate from the current competitive SOTA claim gate and
cannot satisfy, replace, or relax its accuracy, runtime, memory, map-quality,
fresh-holdout, or legal-provenance requirements.

### Track B — application differentiation

Track B explores opt-in application layers around stable SLAM contracts. Each
application must prove user value independently; a plugin seam or service
interface is not evidence that the underlying SLAM algorithm is superior.

| Application | User value | Differentiator / technical capability | Plugin seam | Validation dataset or scenario | KPI | Go/No-Go | Dependencies | Timing | Non-goals |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Warehouse/factory change intelligence | Detect layout, inventory, and safety-relevant changes between routine passes. | Separate operational change from seasonal motion and maintenance noise. | Versioned change classifier and map-event policy. | Repeated aisle/plant routes with staged pallet, rack, and access changes. | Change precision/recall, alert latency, false structural-change rate. | **Go** only on held-out interventions; otherwise **No-Go**. | Track A change aging; session provenance; review UI. | Near term | No autonomous workcell command, safety certification, or default-map mutation. |
| Mining and construction equipment | Keep localization and site maps useful while terrain, stockpiles, and equipment move. | Robust change classes and branch/rollback for large, partially observed sites. | Site adapter, submap policy, event exporter. | Multi-week quarry/site protocol with staged earthworks, occlusion, and GNSS degradation. | Availability, drift, change F1, recovery time, storage growth. | **No-Go** without degraded-sensor and restart evidence. | Long-term map lifecycle; calibration; rugged storage. | Mid term | No machine-control or progress-certification claim. |
| Agriculture and forestry | Support repeatable navigation as crops, foliage, and paths evolve. | Distinguish seasonal biological change from traversable structure. | Vegetation/change policy and domain dataset adapter. | Repeated row/forest routes across growth/weather phases with held-out paths. | Relocalization availability, drift by season, false structural-change rate. | **Go** only with seasonal holdout and privacy review. | Change classifier; multisession schema; sensor calibration. | Mid term | No agronomic diagnosis or yield claim. |
| Underground and GNSS-denied robotics | Recover and maintain maps where global positioning is absent or intermittent. | Local branch/recovery semantics and bounded offline event logs. | Offline recovery and map-store adapter. | Tunnel/mine protocol with blocked routes, sensor interruption, and restart injection. | Availability, time-to-relocalize, recovery success, bytes transferred. | Any unrecoverable branch or unbounded log is **No-Go**. | Crash-safe storage; relocalization API; fault injector. | Mid term | No mine-safety certification or guaranteed coverage. |
| Autoware localization/map maintenance | Keep road maps and localization assets synchronized with verified roadway changes. | Provenance-aware promotion and deterministic rollback. | Localization/map adapter, migration, approval-policy APIs. | Repeated urban routes with controlled lane, curb, and construction changes. | Availability, drift, rollback time, rejected-change rate. | **Go** only with approved-map audit trail; no production claim otherwise. | Stable versioned APIs; map migrations; Autoware adapter. | Mid term | No driving policy, planning, or production Autoware claim. |
| Construction progress and digital twin | Track geometry and site-state evolution for planners and owners. | Link spatial changes to time/session provenance and reviewable versions. | Event log and digital-twin schema adapter. | Multi-week staged site scans with occlusion and planned demolition/build phases. | Registration consistency, phase-change F1, review latency, storage growth. | **No-Go** without independent phase labels and crash-safe replay. | Map schema; export API; compaction. | Mid term | No contractual progress certification or BIM replacement. |
| Urban infrastructure inspection | Compare bridges, tunnels, utilities, or roads over repeated inspections. | Distinguish structural change from viewpoint, weather, and seasonal effects. | Inspection feature/change policy and evidence exporter. | Repeat inspections with controlled defects and weather/lighting variation. | Defect recall/false alarm rate, revisit alignment, recovery time. | **Go** only for evidence-assisted review; engineering sign-off remains out of scope. | Inspection dataset/protocol; provenance; reviewer tooling. | Mid term | No engineering sign-off or unverified defect claim. |
| Fleet/cloud-edge map federation | Share useful updates while robots remain productive offline. | Conflict-aware merge, bounded synchronization, and edge-first privacy. | Federation transport, resolver, and policy interfaces. | Several robots with intermittent links, divergent branches, and delayed merges. | Convergence time, cross-robot consistency, bandwidth/storage bound, conflict loss. | Any raw-data policy violation or lost conflict is **No-Go**. | Identity/key policy; transport; conflict resolver. | Long term | No mandatory cloud dependency or fleet-scale claim. |
| Disaster and low-connectivity response | Maintain localization/map continuity when infrastructure is damaged or disconnected. | Crash-safe local branches with explicit recovery and later reconciliation. | Offline event-log, recovery, and privacy/security plugins. | Intermittent-network, degraded-sensor, blocked-route, and restart scenarios. | Availability, time-to-relocalize, recovery success, reconnect bytes. | **Go** only as a non-safety-critical aid with recovery evidence. | Offline storage; relocalization; secure reconciliation. | Long term | No emergency-response certification or safety-critical authorization. |
| General autonomous-driving/robotics integrations | Give downstream systems a reviewable map lifecycle without forking core SLAM. | Stable adapters, policy plugins, observability, and migration contracts. | Versioned SDK and dataset adapters. | Public robotics/vehicle sequences plus a reproducible multi-session protocol. | API compatibility, replay determinism, availability, drift, storage bound. | **No-Go** until at least one concrete domain gate above passes. | Versioned APIs, schema migrations, benchmark kit. | Long term | No universal autonomy or SOTA claim. |

### Shared OSS differentiation and promotion stages

Future OSS differentiation should be expressed through stable versioned APIs,
map schemas and migrations, an append-only event log, policy plugins,
observability hooks, dataset adapters, and reproducible benchmark kits. The
promotion sequence is deliberately conservative:

1. research spike;
2. experimental plugins;
3. shadow mode;
4. opt-in production;
5. default candidate review.

Each promotion preserves the existing deterministic replay, accuracy, runtime,
memory, map-quality, and Autoware bundle gates. Until the Track A and Track B
evidence is independently accepted, all of this remains future-only planning:
no implementation, default behavior change, README/SOTA claim, or production
authorization follows from this document.

> Status: proposed, 2026-08-24. This roadmap changes architecture and contributor
> experience, not the default SLAM algorithm. Every migration step must preserve
> the existing deterministic replay, accuracy, runtime, memory, map-quality, and
> Autoware bundle gates.

### Additive competitive execution-selection handoff (2026-08-26)

The r3 execution-selection work is a future promotion handoff only. Its
candidate is `NOT_READY` and non-promoting while legal/source, dataset/fresh
holdout, machine, image, and toolchain closures remain external. It does not
alter the active profile, historical receipts, or any Track A/Track B claim.
Any future promotion requires an independently reviewed READY receipt and a
canonical profile reseal; unsigned handoff artifacts are review inputs only.

The additive r3 external-capture utility now provides a bounded observation
boundary for the three pinned image identities and an explicit opt-in,
network-none/read-only toolchain probe. Its immutable outputs remain
`NOT_REVIEWED_EXTERNAL` and non-promoting, including when all synthetic probes
pass; missing local images and probe failures remain `PENDING`. Custodian review
and independent execution receipts are still required before any promotion.

## 0. Current status matrix (2026-08-23)

### Current-source registration matrix audit (2026-08-24)

The earlier Humble/Jazzy external-consumer and author-template entries below
are historical claims, not current-source promotion.  The additive
`registration-plugin-matrix-current-2026-08` profile and
`scripts/audit_registration_plugin_matrix.py` now require a content-hashed
manifest covering the current transaction/provenance/ODR changes.  The local
result is Jazzy `PASS_HOST_TOOLCHAIN_ONLY`; the host has no `/opt/ros/humble`,
so Humble is `NO_GO` here.  Historical claims without immutable receipt
sidecars, and the missing `/tmp` ODR artifacts, are explicitly superseded.
No build, container, replay, GT, scorer, map, or SOTA claim follows from this
audit; a clean C++14 external build/load receipt remains a distro-specific
gate.  The new current-source release matrix is stricter and has four required
legs (Humble/Jazzy × optional-dependency absent/present).  Its present legs are
now `READY_PINNED_STATIC_REVIEW`: official immutable fast_gicp and small_gicp
archive, source-tree, and license pins are checked in, but no distro build/load
PASS is claimed.  Archive verification precedes extraction and post-fetch
network-capable commands are forbidden.  The release status remains pending
until all four immutable runtime receipts are `PASS`.

| Area | Status | Evidence and next gate |
| --- | --- | --- |
| C++14 `RegistrationPlugin` API, typed request/result, capabilities, and failure contract | **Implemented** | [`registration-plugin-api.md`](../architecture/registration-plugin-api.md); installed interface and contract tests pass. The public C++14 interface is consumed by the clean external fixture below. |
| Shell `pluginlib` loader and host/pluginlib hybrid resolver | **Implemented / experimental** | Offline and explicit live-startup discovery, provenance, API/capability/config validation, and failure tests pass. Actual DSO load/session, constructor rejection, activation rejection with lease preservation, post-commit rollback, scanmatcher resource rollback, and backend preflight rejection now pass targeted gates. Broader live-bag characterization remains pending. |
| External author SDK/template and contract guide | **Implemented / distro matrix passed** | [`registration-plugin-authoring.md`](../registration-plugin-authoring.md) and the C++14 template build as clean external consumers on Humble and Jazzy. The installed interface now exports its contract helper, removing the source-tree reach-back. CI adoption and additional third-party author feedback remain next. |
| Built-in NDT same-translation-unit adapter path | **Implemented** | The numerical/configuration default remains unchanged, but startup and processing now use the host-resident session and activation transaction even when the explicit plugin selector is disabled. The HILTI exp04 baseline/adapter frontend and map-artifact receipts are byte-identical (the existing indoor absolute profile still has its documented violation). |
| GICP and optional Small GICP/VGICP adapters | **Experimental / Humble+Jazzy build verified** | Real optional-dependency builds and all direct-fixture adapter tests pass on Humble and Jazzy. These are compatibility results, not absolute-accuracy claims; real-bag characterization remains pending. |
| FAST_GICP / FAST_VGICP | **Implemented conditionally / Humble+Jazzy build verified** | The optional `fast_gicp`-only DSO provides typed `FastGicp`/`FastVGicp` adapters, plugin XML, variant-specific host factories, canonical preflight mapping, and fail-closed absence behavior. Dependency-enabled compilation and direct-fixture equality pass on both supported distros; real-bag accuracy/runtime characterization remains pending. |
| Backend loop-registration/plugin seams | **Implemented / experimental** | Live and offline `graph_based_slam` NDT now resolve the same host-resident `lidarslam_builtin/NdtOmp` `backend_loop` request/session before observable processing; `BackendCore` consumes only the typed interface. R2, the path-independent R4 provenance fixture, the M4a receipt/parser fixture, and the pinned MID-360 three-run artifact comparison pass. That historical receipt **fails only the strict max-RTF gate** (`1.006913460 > 1.0`); the M4b bounded-cache implementation/tests and formal stride-5 MID-360 development-profile gate pass (`max RTF=0.264233831`, wall CV `2.484173052%`, peak RSS `565.222656250 MiB`). M4c HILTI exp04 and exp07 three-run backend regression gates also pass with old optimized artifacts exact; the paired exp04 map check passes at 2%, while the unchanged indoor absolute profile fails on both old/current reports. This closes cache/general regression for these receipts only; official dense-GT/SOTA comparison and broader promotion remain M5 pending. GICP stays an explicit legacy bridge. |
| Competitive SOTA evidence validator | **Implemented / fail-closed; identity frozen, evidence pending** | Additive schema-v2 mode in `evaluate_competitive_suite_gate.py` separates historical exp02/03/21 regression slots from the primary-fresh partition. Every system must provide every dataset in both partitions with exactly three run records; completion, RTF/RSS, map, and per-sequence regression checks cover both, while aggregate APE and hierarchical CI use fresh only. It requires profile-assigned fresh slots (selection/input/reference/calibration hashes), all rivals, pinned per-system provenance, a common scorer fingerprint, an equal canonical seven-field thread policy, and the remaining safety evidence. Exp14/16/18 are now `frozen_unopened` after a read-only deep verification of the managed root; the execution-identity preflight is `PASS` before first run, while benchmark evidence remains `INCOMPLETE` until all required 3-system x 3-slot x 3-run records exist. Exposed exp02/03/21 assets cannot be relabelled fresh. Synthetic boundary/negative tests pass and no README claim is authorized. |
| M6a GT-blind execution harness | **Implemented / campaign4 GT-blind complete; RTF gate not passed** | `scripts/run_competitive_gt_blind_benchmark.py` emits an explicit 27-attempt plan and supports read-only dry-run/preflight plus a separately gated execute mode. M6a2 is retained as an immutable infrastructure-failed parent (`a5abafde…f002`), not replaced; campaign3 remains an incomplete lineage record. M6a3 repairs the ours RKO runtime contract, attempt-scoped GLIM ROS state, and FAST loopback-only ROS1 startup. The rebuilt ours image is `sha256:18198c…288bd`; synthetic full-path smoke passes for all three wrappers. M6a7 fixes and audits the comparable aggregate process-tree RSS contract (`memory.max=max`, OOM delta zero). Campaign4 completed all 27 GT-blind attempts with valid integrity evidence, but the finite processing-RTF gate is not passed (`max RTF=1.173445611` from FAST-LIVO2, threshold `<=1.0`). No accuracy, map-quality, performance-superiority, SOTA claim, or M6b authorization follows. |
| M6a10 online-compute phase contract | **v1 immutable / fixed10-v2 FAIL_CLOSED / fixed10-v3/v4/v5 preflight FAIL_CLOSED / fixed10-v6 runner-not-started FAIL_CLOSED / fixed10-v7 identity FAIL_CLOSED / fixed10-v8 identity FAIL_CLOSED / fixed10-v9 quiescence FAIL_CLOSED / fixed10-v10 completed GT-blind functional / FAST v2c retry-v2 runner-contract FAIL_CLOSED / FAST v2c retry-v3 quiescence FAIL_CLOSED** | Fixed10-v2 retains exact consumer PASS evidence but is invalid overall: exit `125` for missing `input_end`, callback max `0.371609411 > 0.25` s, and RSS jitter `139.5325092% > 100%`; all hashes are pinned, retry `0`, GT/scorer false. Fixed10-v3, v4, and v5 each had one immutable read-only quiescence preflight failure and therefore never started a runner: v3 exceeded CPU/load with eight compilers, v4 exceeded CPU/load with seven compilers, and v5 recorded CPU busy `25.717884130982366% > 5%` with one compiler (load1/CPU `0.2575 <= 0.5`). Fixed10-v6 had a PASS quiescence receipt (CPU busy `3.0264817150063053%`, load1/CPU `0.03875`, no forbidden processes), but its preflight-to-run continuity window was lost before runner start; it is retained as immutable `runner_not_started_after_preflight` `FAIL_CLOSED` with retry `0`. Fixed10-v7 ran once and failed closed before quiescence because its launcher tree hash `bcbb4c86…f1b8ee6eb` did not match the preregistered profile hash; independent canonical recomputation matches the corrected 64-hex tree identity. Fixed10-v8 ran once and failed closed before quiescence because the adapter bound a malformed 69-character image digest while Docker inspection returned the full pinned 71-character image ID; marker/closure hashes are recorded in the profile, runner start was not attempted, and retry remains `0`. Fixed10-v9 passed its read-only identity receipt but its one owned quiescence attempt failed closed at load1/CPU `0.72375 > 0.5` (CPU busy `2.4439405391786346%`, no forbidden process); no runner started and retry remains `0`. Fixed10-v10 then ran exactly once from a single launcher process with successful quiescence continuity. It completed the GT-blind functional unpaced-ack contract with `230895` expected/received/processed messages, zero drops/overflow/failures, EOF and empty drain, callback maximum `0.109221778` s, and online compute RTF `0.126860008821509`; aggregate process-tree RSS was `868278272` bytes, cgroup total peak `2750537728` bytes, `memory.max=max`, OOM delta zero, and the trajectory hash was byte-equivalent for raw/corrected output. The closure receipt SHA is `e7224eb29dc547a5119cb7ecc1d51d009ced43cfca8e23c6c6d2fc1977e2e82b`; no map artifacts, GT content access, scorer, retry, accuracy claim, performance comparison, SOTA claim, or M6b authorization follows. FAST-LIVO2 v2c retry-v2 was then quiescent (busy `3.7616763443574857%`, load1/CPU `0.415`) but failed closed in the host runner before Docker because `safety.ground_truth_mount_exposed` was absent; closure receipt `c76ce0a5…ffdf0`, no bag replay/container, retry `0`. FAST-LIVO2 v2c retry-v3 then failed its single quiescence attempt before runner start (busy `98.275%`, load1/CPU `0.69375`, seven C++ compilers and one rustc); closure receipt `eed5aa50…bc399`, retry `0`. |
The fixed10-v2 image-bound launch overlay is separately content-addressed at
`d45545717f90f6877b5f281fc5623df04b824f7a236d2fe73b91c2dd3714371c` and is
installed/label-checked in image
`m6a10-v2a-fixed10-v2-lidarslam-ours:jazzy` (`sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69`).
The preregistration remains unexecuted; the fixed9/v1 failure lineage is
unchanged.

GLIM fixed10-v4 is a separate completed functional GT-blind closure, not a
rewrite of the ours fixed10-v10 or the GLIM fixed10-v3 failure lineage. Its
common phase finalizer receives the preregistered backlog bound `100000`; the
single run passed exact consumer counts, EOF/drain, callback bound, RSS/OOM,
and input-only mount checks. Closure receipt SHA is
`f33953c4126939dfce841b030cdb2e5560615d42963ae4aaaf4ab0aad7a7f6c1`.
Online RTF remains diagnostic and no accuracy, SOTA, or M6b claim is
authorized.

Fixed10-v10 is recorded as completed functional GT-blind validation under
`m6a10-v2a-ours-rko-unpaced-ack-fixed10-v10`. Its independent launcher is
`scripts/run_m6a10_fixed10_v10.py` (SHA-256
`f1d7844eaf8f2d5c22431fed5abcc84ae07f9f34cbd92de3b515c61d20ce78ed`) and its
test SHA is
`b89afc720fec4cc0af2d0736367407cbb6dc4b8789a26e3a0a3adf56530b2283`. The
launcher pins v9 source SHA
`879fa1c44aa093b158e405806276935bc41bf3bf4d9134a4534cd4922312254f`, the
complete image ID
`sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69`,
and canonical input tree
`0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb926`.
The read-only identity receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v10_identity_preflight/identity_receipt.json`
with SHA-256 `2211744e3691df3fc71f7b0a81248a2e75daa4c5b15a23c8a0f8acab4b360ab2`;
its read-only identity receipt was followed by exactly one launcher-owned
quiescence and Docker run. The execution root and closure receipt are
content-addressed in the profile; runner return code, consumer/phase status,
RSS/OOM, no-map, and GT-blind completion checks all passed. This remains
functional validation only, not a performance comparison or SOTA claim, and
M6b remains unauthorized.

### M6a10-v2b GLIM consumer hook (2026-08-23)

GLIM's benchmark-only consumer hook image build and installed-image identity
verification pass. One fixed10-v2 functional replay was executed once, but
the overall closure is `INVALID_SAFETY` and is not promoted.
Pinned core and ROS2 source revisions (`faa264a1…` and `4a9e7a4c…`) receive
content-addressed patches that record callback acceptance, exact NTU topic
counts (LiDAR 5793, IMU 225102, image 5792), EOF before `wait()`, and final
queue drain before save. Queue observability and high-water bounds are
required; unknown drops or overflow are invalid rather than assumed zero.
The runner binds the preregistered input/config tree hashes and image labels,
including `BUILD_WITH_CV_BRIDGE=ON`; its host wall RTF remains diagnostic.
The verified image is
`m6a10-v2b-20260823-glim-cpu-benchmark:competitive-v1` with ID
`sha256:010c0019a077116edf4d1e7462dfa28561c4fb17db3b5db52e3652c8a875eb41`.
The read-only build receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-glim-closure-20260823/build_receipt.json`
(SHA-256
`fe4d7e3b3b3fec28185f5faa016b19dfb30bd9a52928b099a68d6679e0a09df2`).
The system-level execution identity was independently preflighted in a
read-only, network-none container. The PASS receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-preflight-20260823-v2/execution_identity_preflight.json`
with SHA-256
`0ae210d6df457fb6bebf7c23ee33cb4e9299bcfe5f0ddde2ad009f79b1dd5179`.
It binds the current runner/wrapper and recipe hashes, image ID and OCI
labels, installed callback markers, and system-container toolchain probe;
no bag, GT, scorer, or replay was opened. The earlier generator receipt is
retained as superseded because it used a non-contract config tree hash; the
v2 receipt uses `relative_path_size_content_sha256_v1`.
The synchronized profile canonical SHA is
`800f07184b728623710375c778624a4f623bc706a1f8c05b19b544847d6e3830`; its
execution-selection file binding is
`9038c02be377a9ac9cc6fc15a34e9f23fc0181b57f31a5053d8f7c1a4aa00db2`.
The immutable closure receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-closure-20260823/closure_receipt.json`
(SHA-256
`72d8ef9cd2c8e26d4c2120463fdbff3057a75e3f81203863049c11028baa41d5`), with
output tree SHA-256
`f0db95c9c700b8c9650e9ce2751eb716c2cc271c664ee971f337ec6c01da2552`.
The wrapper/phase sub-result passed exact expected/received/processed
`236687`, zero drops/overflow/failures, EOF and empty drain, callback maximum
`0.0149106` seconds, process-tree RSS `1334648832` bytes,
`memory.max=max`, and OOM delta zero; online acknowledgement RTF
`0.29150756632664043` is diagnostic only. The first direct launch mounted the
release parent directory, making sibling `ntuviral_gt` reachable even though
no GT file was opened. Strict GT-unreachable closure is therefore false; a
future attempt must mount only the canonical input directory and use a fresh
root. Retry is zero. The atomic consumer evidence and EOF sidecar remain
immutable; no scoring, accuracy, performance comparison, SOTA claim, or M6b
authorization follows.

The fixed10-v3 successor narrowed the Docker input bind to the canonical ROS2
directory only (no release-parent or `ntuviral_gt` sibling mount). Its identity
preflight is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-preflight-20260823-v3/execution_identity_preflight.json`
(SHA-256
`9b8cc1d089a73113930557c6558a32a5ded3725d386b20a78c61fc14e740e190`). The
single replay retained exact consumer PASS evidence (`236687` messages,
zero drop/overflow/failure, EOF, final queue `0`, callback maximum
`0.025782` seconds), RSS/cgroup evidence, `memory.max=max`, and OOM delta zero.
It remains `INVALID_SAFETY`, however, because the generic phase finalizer used
its default backlog bound `0` instead of the preregistered `100000`, rejecting
the observed high-water `249`. The immutable v3 closure is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-closure-20260823-v3/closure_receipt.json`
(SHA-256
`492d39b1f7c4a1a2e86893dd979b0ed57b3f37267cb7f21849b990d95bcc32ff`). The
output tree is `3ab59eed2f20c3bf179bbfa05d0b5a4ffed723137ea81ebc2427619fd26306c5`.
No retry, GT/scorer access, performance comparison, or M6b authorization
follows. The synchronized profile canonical SHA is
`d83a3b96ea224f3f4673dd48a6252af488834dc272f735c5d5e147ce6a7a0ec4`; the
execution-selection file SHA is
`6ddcc24b5d221ef96ac746a7f0b4c6c3e261f5c002d115d51d19eb5fa8e072f4`.

FAST-LIVO2 fixed10-v4 is likewise retained as an immutable `FAIL_CLOSED`
diagnostic, not a performance result. Its host stop/force-kill sequence had
no OOM evidence, and first-ACK absence is not proven because v4 had no
progress checkpoint. The preregistered v5 observability slice adds atomic,
line-buffered feeder progress, stable container name/cidfile, and
non-destructive reconnectable Docker lifecycle snapshots. It preserves the
algorithm, input, mount graph, and fairness contract, requires a rebuilt image
label, and has not started quiescence or replay. No FAST performance or M6b
claim follows.

| Live-node plugin preflight | **Implemented / experimental** | Read-only `registration_plugin_enable`, `registration_plugin_class`, and `registration_plugin_allow_external` are validated before pub/sub creation. The default selector preserves built-in behavior through the same host-session processing boundary; runtime hot reload is rejected. External DSO promotion remains **No-Go** until independent ODR, lifecycle, rollback, and Humble/Jazzy replay gates pass. |
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
has a separate `fresh_holdout_slots` contract. M5b preregistered Exp14, Exp16,
and Exp18 in `configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml`;
M5d now records their deep-verified identities with status `frozen_unopened`.
The execution preflight is ready/PASS, but any real v2 evidence invocation
remains `INCOMPLETE` until every system/run repeats those identities. The
selection contains no performance data and does not authorize a README or
SOTA claim.

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
has no run records yet and cannot support a README or SOTA claim.

### M5b fresh-holdout selection and identity freeze (2026-08-21)

The read-only exposure audit selected three HILTI 2022 additional sequences
with official dense 6DoF IMU references and the same Phasma sensor/calibration
contract: Exp14 Basement 2, Exp16 Attic to Upper Gallery 2, and Exp18 Corridor
Lower Gallery 2. The machine-readable selection receipt is
`configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml`; its
profile hash is repeated in all three fresh slots. The receipt pins official
revision `e62017f907007fdc5ab8c721842e4ae7359d7f49`, bag sizes and LFS hashes,
GT file blob identities, calibration YAML blob identities, license, exposure
audit commands, and the blind policy. It intentionally leaves downloaded
input-manifest, GT-content, and calibration-archive SHA-256 fields were null
at preregistration time. The external managed root is now deep-verified
read-only: all three slots are `frozen_unopened`, with opaque GT hashes,
canonical calibration-tree hashes, canonical ROS2 tree/semantic hashes,
input-manifest hashes, manifest file hashes, preparation-receipt file hashes,
and plan identity recorded in the selection and profile. The frozen marker and
manifests remain bound to the committed preregistration SHA; the enriched
selection receipt records that SHA as an explicit lineage anchor, and the
verifier accepts only the current SHA or that declared anchor. No GT content,
trajectory metric, benchmark run, or performance result was opened or
recorded. The execution identity receipt is `ready` and its read-only preflight
is `PASS`; the schema-v2 evidence gate remains `INCOMPLETE` until all required
3-system x 3-slot x 3-run records exist. No README/SOTA claim is authorized.
Freshness remains bounded to the recorded repository/workspace/media audit and
is not proof of external historical-use absence.

After a separate review has produced all three `frozen_unopened` slots, the
independent read-only verifier is:

```bash
python3 scripts/verify_competitive_frozen_holdouts.py \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --output <out>/frozen_holdouts_deep_verification.json
```

It requires exactly `exp14`, `exp16`, and `exp18`; checks marker/selection/
plan binding, raw bag bytes/LFS SHA and calibration bytes/Git blobs, safe
canonical ROS 2 metadata/tree and seven-topic semantic identity,
input-manifest payload, and
preparation runtime/argv hashes. The preparation receipt's
`manifest_sha256` is checked by rebuilding its deterministic pre-finalization
manifest view and hashing the canonical JSON file bytes, including its
trailing newline (the payload hash is distinct). Raw bag content is checked by expected byte count and the
selection's LFS SHA-256; its preregistered Git-blob OID is the pointer
provenance, not a content-blob comparison. Ground truth is hashed as an opaque
stream only: no GT path, text, or metric is printed. Manifest and
preparation-receipt file SHAs are included in the machine-readable summary.
It is independent of the freezer; this M5d checkpoint reran it against the
managed root without parsing GT. Its output records both the current enriched
selection SHA and the accepted preregistration lineage anchor.

### M5b execution-identity freeze (2026-08-21)

The next pre-run artifact is
`configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml`.
It records the ours/GLIM/FAST-LIVO2 repository revisions and URLs, runner and
configuration hashes, container recipe/tag/digest state, toolchain source,
scorer fingerprint, machine-fingerprint source, exact `Release`, and the
seven-key thread policy (`cpu_affinity`, `max_threads`, `OMP`, OpenBLAS, MKL,
TBB, and accelerator policy). The receipt now records a verified clean ours
revision, fresh machine fingerprint, and CPUs 0--7 with all thread limits set
to 8. GLIM's pinned image is now verified at
`sha256:3702b73873395880e1dcdb91394232f1a9932194de22214e1aacb9626ced5846`
with toolchain fingerprint
`805429f650c6d927497e3c06169f9763c2cfd8e8101f02e6fcd565c0972924a2`; PCL is
explicitly `not_applicable` for its CPU path. The execution identity receipt
is now `ready` after the deep-verified fresh-input identity was recorded; this
is a pre-run readiness state, not benchmark evidence. Ours is recorded at image
`sha256:4426d334ee7014d6387694df8957285a56123c31576b17e30de655248dd91930`
with toolchain fingerprint
`6cdff5854c86cc820d6781700d7613b2e529e75d58c9fb0830fd2b0f792adca5`, and
FAST-LIVO2 is recorded ready at its pinned image/toolchain receipt. The
recorded policy must be enforced with
`taskset`/Docker `--cpuset-cpus` plus the explicit OMP/OpenBLAS/MKL/TBB
environment variables before execution. FAST's upstream visual configuration is
an observed external-container artifact at
`/opt/fast_livo_ws/src/FAST-LIVO2/config/HILTI22.yaml`, SHA-256
`efae9e702c71c770b19002b6e19d4e1b6f46c67df3727e984981d932258f0b4a`, bound to
the immutable image
`sha256:ddc75b574f8cca1e111332153e31a65c74ccdb11f8059da3797ab130814ce17e`.
The checker validates that container-path/digest binding without reading a host
path. GLIM's CPU track disables visual input while preserving the canonical
camera messages for the cross-system input contract.

`scripts/check_competitive_execution_selection.py` is a read-only,
fail-closed preflight. It verifies the profile's receipt path/SHA, file and
tree hashes, 40-hex revisions, `sha256:<64hex>` container digests, exact
`Release`, scorer/machine files, and complete equal thread policy. It emits
machine-readable JSON/YAML and reports missing values as `INCOMPLETE`; it does
not build, download, open GT, or run SLAM. The M5d identity checkpoint now
reports this preflight `PASS` after the deep-verified frozen fresh inputs were
recorded; the separate schema-v2 evidence gate remains `INCOMPLETE` until
benchmark run receipts exist.
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

### M5d pre-run identity checkpoint (2026-08-22)

The managed root
`/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821` was
deep-verified without parsing GT or running SLAM. The enriched selection
receipt is `frozen_unopened`; its current file SHA is
`2bfc541a8d6127599f7a36e66c08da44488a08a55a4d9c4709703223be8bdd2b`, and its
explicit lineage anchor is the committed preregistration SHA
`cfc106a0396cc032f297f72f10e549ebef55c39f492afe9c1d25ded0d4307dde`.
The verifier accepted only those two identities and passed all three Exp14,
Exp16, and Exp18 manifests, opaque GT hashes, calibration tree, canonical ROS2
tree/semantic report, input manifest, and preparation receipt identities.

The historical M5d profile canonical SHA was
`5a8b81b7483ce9921fdfb4393ed006390fc5f0b2553b5e29976de96725a4da39`; the
execution receipt raw file SHA at that checkpoint was
`ce6d03a273ec12d6191f9d802d1ee24892292d3333375fe10500c15a32a4abaf` and its
status was `ready`. `check_competitive_execution_selection.py` reported
`PASS`, including all three pinned systems, common machine/thread/scorer
identity, and exact `Release`. This is a pre-run readiness checkpoint only:
the schema-v2 evidence gate remains `INCOMPLETE` until every system supplies
three complete runs for every historical and fresh dataset. No GT metric,
performance result, README change, or SOTA claim is authorized.

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
GT was opened. The M5d receipt is now `ready` because the fresh identity was
deep-verified; the evidence gate is still `INCOMPLETE` until benchmark run
records are reviewed. A future run must still enforce the pinned system
containers, machine, Release, and seven-key thread policy.
Both commands accept explicit repeatable `--source SYSTEM=PATH` and
`--image SYSTEM=TAG` bindings; absent bindings produce an exact read-only
compiler/linker/ROS/PCL/Eigen/OpenMP probe manifest rather than guessed
readiness. The finite-state verifier compares measured revision, clean
provenance, image digest, toolchain fields/fingerprint, machine identity, and
canonical thread policy. Thus a complete ready/frozen synthetic contract can
be `PASS`, but no pending production receipt can be promoted implicitly.

### M6a0/M6a1 GT-blind execution harness (2026-08-22)

The first execution slice is intentionally a runner and safety checkpoint, not
a benchmark result. `scripts/run_competitive_gt_blind_benchmark.py` fixes the
27-attempt order as `ours -> glim_cpu -> fast_livo2`, then
`fresh_1(exp14) -> fresh_2(exp16) -> fresh_3(exp18)`, with repetitions 1--3
inside each slot. The default is read-only planning; `--preflight` additionally
inspects the receipt's immutable image digests and hashes the frozen raw and
canonical inputs, while `--execute` is a separate explicit action.

Each attempt is started in an unowned `.part` directory and atomically renamed
to its final directory for both success and failure. The manifest records the
schedule index, canonical profile/receipt/selection identities, raw and
canonical/semantic input hashes, config/revision/container/toolchain identity,
CPU/thread environment, exact command and mounts, host timing/RSS, artifact
hashes, and a GT-blind proof. GT paths are used only for metadata reachability
guards; their contents are never opened or passed to a container. No scorer,
APE, or map-quality command is part of this harness.

The managed input root is
`/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821` and the
intended results root is
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_20260822`.
The M6a0 plan has schedule SHA
`bb3484c32615a1c6af8b2f1f2e823bd7f06e8131d546a6f91a104707eb9766ce` and was
generated without launching a container. M6a1 rebuilt the ours image from the
fixed recipe at algorithm revision `866f733677e92ecb08d67126e463da99dd140d46`.
The image ID/digest is
`sha256:0680ae359deb2da45ff16ecf1c5d92c0510dc51d48bd06c0fcd93ce1d33ff3fb`;
its labels verify the `ndt_omp_ros2` gitlink, the RKO-LIO gitlink, and the
exact local RKO archive. The separate GT-blind harness/orchestrator revision
is `4701f0084d6b0fff475a62bec7eeb6d807561821`; it is not the algorithm
revision. The receipt and profile hashes were resynchronized without changing
the canonical profile identity. Read-only dry-run and preflight passed all
27 scheduled attempts, with no results-root marker created at that checkpoint.
Preflight hashes each frozen slot once, even though the schedule repeats it
across systems and repetitions.

### M6a2 GT-blind execution attempt (2026-08-22)

After the M6a1 checkpoint, the fixed schedule was executed exactly once in the
managed results root
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_20260822`.
The run used the committed `3bd10254972ebbcbdc3f904f13f9a79f60e37837`, the
M6a1 preflight identity, CPU set 0--7, and the quiescence snapshot
`ae07b16bb19227468fa5f61327a9a343a2f43c5f5ebe6ade17b712d5b134db54`.
All 27 immutable attempt directories were created and no `.part` directory
remained. The GT-blind completion manifest is
`a5abafdeb420619a1460737a3cf91862fa41ce7930c041324b1f38c00b16f002` and has
status `INCOMPLETE`: ours had 9 exit-1 startup failures because `rko_lio` was
not discoverable in the runtime, GLIM had 9 exit-250 failures while creating
its ROS log directory on the read-only container, and FAST-LIVO2 had 9 exit-20
failures because its ROS 1 master could not self-connect with the network
disabled. Every attempt records `ground_truth_content_opened: false` and
`scorer_invoked: false`; no trajectory or performance result is valid. These
runtime contracts must be repaired and independently smoke-tested before any
rerun. No GT, APE, map-quality, accuracy, or SOTA claim follows from M6a2.

### M6a3 GT-blind remediation and preflight closure (2026-08-22)

M6a2 remains immutable and is explicitly recorded as an infrastructure-failed
parent, not a replacement campaign. Its completion manifest SHA is
`a5abafdeb420619a1460737a3cf91862fa41ce7930c041324b1f38c00b16f002`; the
checkpoint lineage is commit `972300263476a113bd32e78b69babdbce5753895`.
The algorithm revision remains
`866f733677e92ecb08d67126e463da99dd140d46`.

M6a3 repairs and synthetic smoke evidence are:

- ours image `sha256:18198c17627459e96c574b1bf3093064c9c092f4fc2f89594b7e4b14705288bd`,
  with the pinned RKO package/archive/runtime layout verified;
- GLIM attempt-scoped ROS state and process-group clean shutdown;
- FAST-LIVO2 network-none, loopback-only ROS1 master/node/bag path;
- all three synthetic contract markers report `performance_run: false` and
  `gt_mounted: false`.

Normalized smoke evidence is stored outside the repository at
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a3_20260822/smoke/normalized`.
The tree hashes for ours/GLIM/FAST are
`12e56407c0750bf47ff3aae307c4a96e78e056f84121421de10ed6d9825216ec2`,
`2e74d9659c04007af20e4fb5999087f1a6cbaac4a4cf3df47f6eacc8562edbed`, and
`8293e1f1771f9b60a8d11a43998f538e6b965341bc3399b4aeda81c1a209ce17`.

The disclosed rerun uses
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign2_20260822`.
Its dry-run is 27/27 with plan SHA
`465e856846eb4bcf15e125285a914d2defda72413b5016e2965b884cda2f8bd8`; its
preflight is `preflight_ready`, 27/27, with plan SHA
`7aeac10cdeb1c8356e282b53a62f4f3a05621f37942d325d58302a8f6464cd43`.
The preflight records canonical profile SHA
`5a8b81b7483ce9921fdfb4393ed006390fc5f0b2553b5e29976de96725a4da39` and
checks all frozen input identities, image digests, CPU/thread policy,
GT-unreachable mounts, and scorer-free commands. At this M6a3 checkpoint no
`--execute` had been run; the later M6a4 partial record below is the complete
execution status. No GT content, scorer, accuracy, performance, map-quality, or
SOTA claim is authorized.

### M6a4 partial campaign closure (2026-08-22)

Campaign2 was executed once after the fixed preflight. Final attempts 001--018
are immutable and all passed their GT-blind completion checks (ours 001--009,
GLIM 010--018; exit 0, complete output, no GT/scorer access). FAST attempt 019
reached wrapper shutdown and emitted its top-level status artifacts, but driver
finalization failed on the unreadable root-owned mode-600
`attempt_019.part/ros_home/rospack_cache_15823137030970321179`. The `.part`
directory was not changed, renamed, or deleted; no final attempt 019 manifest
was produced and attempts 020--027 were not started.

The atomic partial manifest is external:
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign2_20260822/partial_campaign_manifest.json`,
SHA-256
`df91a0f0852911790f3fabb5b5638938b8e1222208943be01678a99fbd062978`.
Its root identity excludes the manifest itself and has projection SHA
`ceafd30fbfe4f0099ba32e4a79390f8dd5cc0e6ea428254c8d021c11231bd16e`.

This is an `INCOMPLETE` infrastructure record, not accuracy or performance
evidence. Host `/usr/bin/time -v` wrapped the Docker client, so its RSS values
are not valid container/cgroup peak-memory measurements. GT content and scorer
remain untouched. The immutable M6a2 parent SHA is
`a5abafdeb420619a1460737a3cf91862fa41ce7930c041324b1f38c00b16f002`; a
planned campaign3 must add resilient attempt finalization and cgroup RSS
accounting before any M6b evaluation or README/SOTA claim.

### M6a5 measurement contract and campaign3 preflight (2026-08-22)

The M6a5 remediation makes container cgroup-v2 `memory.peak` the only
comparable RSS field (`container_cgroup_peak_bytes`).  The host Docker-client
measurement (`docker_client_peak_rss_kb`) is diagnostic only.  `memory.max=max`
is a valid unlimited-limit observation; missing, malformed, unreadable, or
non-atomic memory evidence fails closed.  The helper, all three wrappers, and
the driver are bound in the execution receipt, together with the immutable
campaign1 completion and campaign2 partial-manifest lineage.  The synthetic
known-allocation evidence is
`e2df6b70dcec703406a406ed7a4c45aaac87b69987d87966f4d10bceee1c85bb` and is
not a performance result.

Campaign3 uses
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign3_20260822`.
Its external preflight evidence reports 27/27 readiness with no attempts,
GT content access, or scorer invocation.  The final dry-run plan SHA is
`bef6184b506b852288cf07b94772442cdc47c6c58e2cef45da5140249729d082`, the
preflight plan SHA is
`362ad49f85491bfd74ddda181ec0a334c972e4be2818374a05b0db3c5724087b`, and
the summary SHA is
`9ae7115d10dee09f5287317b5e9a3912580f7ae31cfd93e8686162149e108bf0`.
The receipt/profile identities at this historical M6a5 checkpoint were raw
receipt
`d89d30d9e516f7d7211536bd1ea4f837ae95c4f7aa4527af60a5f7699c3d677f` and
canonical profile
`5a8b81b7483ce9921fdfb4393ed006390fc5f0b2553b5e29976de96725a4da39`.
M6a7 resynchronizes the current values in the section below.
This is an execution-readiness checkpoint only; M6b scoring and any
accuracy, performance, map-quality, or README/SOTA conclusion remain
unauthorized.

### M6a6 campaign3 GT-blind closure (2026-08-22)

The fixed campaign3 schedule reached 27/27 immutable final attempts with no
remaining `.part` directory and no GT/scorer access.  Ours and GLIM are 9/9
complete.  FAST-LIVO2 is 6/9 complete; its `fresh_2` attempts 022--024 are
preserved exit-22 failures (`complete=false`).  Completion manifest SHA:
`31df60ff2775d2f2a699e7559c16cc5d732a91e175f12af1db06bc47e4b8cd5b`.
Integrity summary SHA:
`af27ae2c7d019db790cace3b38b250c2a5dcd3340e58fa5a91158293b2bf2024`.
The external closure manifest is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a6_20260822/closure/m6a6-campaign3-closure.json`
with SHA
`44d50669e33dbb105300b2b3f926c99f891341373e6a1dfa71ae9c85acf954b8`.

The campaign is `INCOMPLETE`, not a performance or accuracy result.  Its
`container_cgroup_peak_bytes` is a cgroup total footprint including page
cache, not process RSS; multiple attempts reached the 4-GiB memory cap and
the observed cold/warm spread prevents separating reclaim/cache effects.
The Docker-client RSS is diagnostic only.  `memory.events` was not captured
by the M6a5 schema, so pressure/reclaim behavior is explicitly unknown.  RTF,
RSS, accuracy, map-quality, and SOTA gates therefore remain invalid or
insufficient.  The disclosed lineage is campaign1 immutable failure,
campaign2 immutable partial, campaign3 incomplete, and campaign4 planned;
M6b remains blocked.

The reproducibility slice also owns the image recipes and their build entrypoint:
`docker/ours_competitive_benchmark.Dockerfile`,
`docker/glim_cpu_benchmark.Dockerfile`,
`docker/fast_livo2_benchmark.Dockerfile`, and
`scripts/build_competitive_benchmark_images.sh`. Each recipe pins an immutable
ROS base digest, source revisions, and the CPU-only thread environment. FAST
LIVO2 no longer relies on the historical undocumented
`fast-livo2-benchmark:noetic`/`hdl_localization_noetic:local` base; its pinned
full-length Sophus compatibility commit is recorded explicitly. GLIM's CPU
track explicitly marks unused PCL as
`not_applicable`; its container probe fingerprints that sentinel rather than
adding an unrelated dependency. The
ours recipe uses a Dockerfile-only context, clones the pinned public repository,
initializes only the build-required `Thirdparty/ndt_omp_ros2` gitlink, and
checks HEAD/clean/submodule status before building. The pinned
`Thirdparty/rko_lio` object is supplied as an exact local archive whose SHA is
checked against the `622b74778a41f753d47aa5918043755ebcbd4c75` gitlink; no
public-mirror substitution is used. The static contract test binds every
recipe and entrypoint SHA to this receipt. FAST-LIVO2's pinned
image/toolchain, GLIM's pinned image/toolchain, and the rebuilt ours image now
have observed ready receipts. This proves reconstruction metadata and
GT-blind preflight only: the M5d
receipt is ready for a first run, but no benchmark, accuracy, performance, or
SOTA claim follows from this slice.

### M6a7 process-tree RSS audit and campaign4 gate (2026-08-22)

The M6a7 final read-only audit is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/v3_final_audit_tool_final_20260822/v3_run_audit.json`
with SHA-256
`bd7f57cd2cb6fe8b93a9c28d7b193968d3e865180f2985fb66c44c736e7cd818`; its
normalized PASS receipt is the adjacent
`m6a7_v3_final_receipt.json` with SHA-256
`06985d4c4900dbe1aa27687c023c4b6c2ddd3c362ed6782c92a048f5e1eae62e`.
The corrected auditor handles leading whitespace in GNU `time` exit lines and
fail-closes missing, duplicate, or unexpected AB/BA directories.  It verified
20 pairs/40 runs, all complete, with GT/scorer/campaign4 access false.  Earlier
failed audit directories remain preserved as disclosed lineage.

Campaign4's only comparable memory metric is
`aggregate_process_tree_peak_rss_bytes`: the sum of per-process peak `VmRSS`
(shared pages may be recounted).  The sampler is fixed at 250 ms/nice 10;
cgroup v2 `memory.max=max` is required and every OOM/`oom_kill` delta must be
zero.  Cgroup total footprint and Docker-client RSS are diagnostics only.
The M6a7 measurement contract passed its fixed overhead gate (median absolute
1.8513%, bootstrap upper 4.1914%), signal/atomic-evidence checks, allocation vs
page-cache separation, and all three synthetic wrapper smokes.  These are
measurement and isolation receipts only; they are not accuracy, performance,
or SOTA evidence.

The profile/receipt use `canonical_profile_sha256_v1`: only the registered
execution-receipt file SHA is excluded from the profile projection.  The M6a7
audit/receipt/summary SHAs are explicit external bindings and do not depend on
mtime or a self-referential hash.  At that M6a7 checkpoint, campaign4 was a
fresh-root, GT-blind/scorer-free dry-run and preflight with zero attempts; no
real replay was authorized in that checkpoint.

At that pre-run checkpoint, the campaign4 fresh results root was
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign4_20260822`
and had zero attempts.  Its final deterministic read-only dry-run plan SHA is
`1ffa6836abc9aa94bb41e310de4ff2e50c9f336335b23779f0ccfa1236ea09f2` (a
repeat is byte-identical); the 27/27 preflight plan SHA is
`a8c953e59b6a7dd70891fcdf3c57c791f9735ea6173dda114cd365e188562e4e`.
Both bind selection `2bfc541a…dd2b`, algorithm revision
`866f733677e92ecb08d67126e463da99dd140d46`, profile canonical
`cbb09323…6cc25`, execution receipt `c58d1188…d14f7`, and the M6a7
process-RSS audit receipt `06985d4c…ae62e`; neither plan starts a container or
scorer.  The subsequent GT-blind execution is recorded below; the plans and
fixed execution identity remain unchanged.

### M6a8 campaign4 GT-blind completion (2026-08-22)

The fixed campaign4 root was executed only after the third quiescence window
passed (`30744959f67ef3c6b9f67aa8e8663ba1412c2fc65ae65ef7e4b949dc81b214ca`).
The pre-run machine snapshot is
`07f97ffc4241be205a0678fa58bd5defc66fa6bc818d83716d901e3177ec2341`.
Two earlier starts failed before any attempt because an already-created empty
root was rejected by the driver's no-overwrite contract; their immutable
evidence is `4bda0f3811e7bf3f9f4d9e38ea91b6f9c9be965e8272824c1862bb955fab9bd4`
and `26873c3a35b5c70894ebccc24dd694c05060abc450195d7930423c9f6f9119fd4`.
The verified empty root was then removed and the driver-owned mkdir recovery
was used; those starts created no containers and no attempts.

All 27 scheduled attempts (ours/GLIM/FAST-LIVO2 × exp14/16/18 × three
repetitions) exited zero and completed the GT-blind output contract. The
completion manifest is
`f63b14f52c0b22b957f897568aa38f5a2fce26fea47bd2ca5917f6fec43c1c74`, the
integrity manifest is
`f28ba05ea5c8fce0a6944ddcc7353d3dd5ea38dca927a7dcef7e4131983b6e5b`, and
the closure manifest is
`f2b1b6e943f1c30cd54636c9221bd1eb38d18192cb1cce1f583fd17bf8b1c59a`.
There are no `.part` files, Docker containers, or sampler residuals. Every
attempt has valid aggregate process-tree RSS evidence, `memory.max=max`, and
zero cgroup OOM/`oom_kill` deltas; Docker-client RSS and cgroup total memory
remain diagnostics, not the comparison metric.

The closure is deliberately
`VALID_COMPLETE_GT_BLIND_RTF_GATE_NOT_PASSED`: all finite processing RTFs were
recorded from bag metadata, but FAST-LIVO2 reached `1.173445611`, above the
registered `<=1.0` limit. No GT file was opened, no scorer was invoked, and
no accuracy, map-quality, aggregate APE, SOTA, or M6b claim is authorized.
The fixed profile canonical SHA remains
`cbb093233b2740e0624fbd348ac293a705fd69e7fa1825723a4e7e493736cc25` and the
execution-receipt SHA remains
`c58d11881f88dd7ea6fef05f1e91edf901ca5eaa23076ff673306580885d14f7`.

### M5c fresh-holdout acquisition checkpoint (2026-08-21)

The download implementation is `scripts/freeze_competitive_fresh_holdouts.py`.
It is intentionally a checkpoint workflow, not a benchmark runner. The exact
order is: independently review the selection receipt, run read-only `plan`,
run explicit `download` (or `download --resume` only for the same managed
identity), run `verify`, perform an independently reviewed ROS 2 conversion and
semantic-equivalence report, then run `finalize`. Only the last step can move a
slot to `frozen_unopened`; selection/execution receipts and the competitive
profile are not modified by the downloader and require a separate review.

The plan binds the official selection-receipt SHA, destination identity, and
the runtime SHA-256 of the producer script. That producer hash is copied into
the managed-root marker and every slot manifest, so a changed downloader or
selection contract cannot silently resume an old staging tree. Final slots are
fully re-verified and skipped; mixed final/staging, changed markers, stale
partials, path traversal, symlinks, size/hash mismatches, and calibration
tampering fail closed. Manifests and state transitions use canonical JSON and
atomic replacement.

Raw HILTI bags are streamed and checked against their declared size and LFS
SHA-256. Ground-truth files remain opaque: the tool records only bytes and
SHA-256, never parses, prints, or scores their content. Calibration uses both
storage paths and canonical logical paths, Git blob IDs, and a tree hash. The
destination is the explicitly supplied
`/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821` tree.
The external M5c acquisition/preparation was completed before the M5d review;
the independent deep verifier rechecked the final tree and all three slots are
now `frozen_unopened`. This records identity only: no GT content, trajectory
metric, benchmark run, or performance result was opened. The schema-v2
evidence gate remains `INCOMPLETE` until the required run receipts exist; no
README or SOTA claim is authorized.

The next conversion command is
`scripts/prepare_competitive_fresh_ros_inputs.py --all` (or
`--sequence exp14`). It requires exactly `rosbags==0.11.0`, rechecks only the
raw-bag bytes/SHA, and never opens the manifest GT path. The fixed
`rosbags-convert --src BAG --dst STAGING --dst-storage sqlite3 --compress none
--src-typestore ros1_noetic --dst-typestore copy` argv is recorded together
with Python/NumPy versions, converter/comparator hashes, canonical ROS 2 tree
hash, and the seven-topic semantic report hash. Conversion and comparison are
staged under `canonical_ros2.part`/`semantic_equivalence.json.part`, then
atomically published with `preparation_receipt.json`; `--resume` is accepted
only when that receipt, managed plan, raw identity, versions, commands, and
all output hashes still match. The final receipt is the commit marker; a
converter/comparator crash or either first publish rename can resume only when
each artifact has exactly one `.part`/final form; a staged receipt must have
full identity equality, while a converter/comparator partial without a
receipt must pass safe tree/report validation. The resulting canonical
tree/report are passed to the downloader's separate `finalize` step. The
recorded M5d receipt/profile update is a separate reviewed operation after
finalization; the freezer itself never edits those files.

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
maps the legacy method name to a host-resident class, resolves it through the
same typed request, and commits a session before creating publishers or sensor
subscriptions. This preserves the built-in numerical/configuration defaults
without leaving a raw-PCL processing fallback. With the selector enabled, the
component instead resolves the explicit host or external class and validates
API major/minor, SPDX license, capabilities, typed configuration, method, and
variant before the same transactional activation. Unknown, unavailable,
mismatched, or invalid classes fail closed; no fallback or runtime hot reload
is allowed.

The parameters are read-only after construction:

| Parameter | Default | Gate |
| --- | --- | --- |
| `registration_plugin_enable` | `false` | Must be paired with a non-empty class ID. |
| `registration_plugin_class` | `""` | Explicit `lidarslam_builtin/<name>` host ID or external pluginlib ID. |
| `registration_plugin_allow_external` | `false` | Required for external pluginlib DSOs; it is an explicit risk acceptance, not a fallback switch. |

The live frontend hot path calls only `RegistrationPluginSession::align()` and
`setInputTarget()` after activation. The backend live/offline shells share the
same session-owned boundary; a read-only source audit rejects direct raw-PCL or
raw-plugin processing calls in all three production shells. The 2026-08-28
Jazzy Release build and all three registration-focused scanmatcher test targets
pass, and the production-boundary source audit reports `PASS`. The
external success test remains wiring-only. The independent DSO replay, ODR,
optional-dependency, and Humble/Jazzy matrix gates remain No-Go, so this slice
does not permit README superiority claims.

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

The historical two-leg receipt claimed a pass with the two selected tests;
that evidence is superseded by the current-source four-leg release matrix and
must not be used for promotion. Jazzy was run from the local `/opt/ros/jazzy`
installation. The historical Humble proof used the read-only
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

The historical workflow ran this proof for both Humble and Jazzy after
dependency installation and before the normal workspace build. Those matrix
legs are superseded and do not satisfy the current release gate. The
historical Humble run used a read-only repository mount
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
