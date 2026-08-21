# Benchmarking And Release Gate

This page describes the recommended benchmark path and the release/readiness
gate used for the default permissive workflow.

## Recommended Benchmark

The standard benchmark path for this repository is:

```bash
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_rko_lio_graph_benchmark.sh
```

## FAST-LIVO2 head-to-head

Use the exact same bag, sensor messages, calibration, trajectory reference, and
evaluation alignment for both systems. Record each result in this compact JSON
shape (unknown metrics should be omitted, never estimated):

```json
{
  "system": "lidarslam_ros2",
  "dataset": "hilti2022_exp04",
  "trajectory": {"ape_rmse_m": 0.07146},
  "geometry": {
    "plane_thickness_mean_m": 0.0599,
    "planar_coverage": 0.5355
  },
  "colour": {
    "heldout_rgb_l2_median": 36.37,
    "heldout_rgb_inlier_20": 0.3536
  }
}
```

Add RPE and runtime keys only after measuring them. Create a second manifest
with `"system": "FAST-LIVO2"` and run:

```bash
python3 scripts/compare_fast_livo2.py \
  --ours output/head_to_head/lidarslam_ros2.json \
  --fast-livo2 output/head_to_head/fast_livo2.json \
  --out output/head_to_head/comparison.json
```

The command also writes `comparison.md`. It scores APE, RPE, real-time factor,
peak memory, plane thickness, planar coverage, held-out RGB error, and held-out
RGB inlier rate. A metric only counts when both systems provide it; values
within 1% are ties. This prevents an attractive map image or a single trajectory
number from being presented as an overall win.

## Competitive victory evidence (schema v2)

`scripts/run_cross_repo_slam_benchmark.py` and the legacy `--gate` mode of
`scripts/evaluate_competitive_suite_gate.py` remain report-only compatibility
paths. They do not authorize a system-level SOTA claim. The explicit v2 gate
is fail-closed and consumes one machine-readable evidence document:

```bash
python3 scripts/evaluate_competitive_suite_gate.py \
  --evidence <competitive_evidence_v2.yaml> \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --output <out>/competitive_evidence_v2.json \
  --yaml-output <out>/competitive_evidence_v2.yaml
```

The document must declare `schema_version: 2`,
`evidence_kind: competitive_slam_victory_evidence`, a profile-matched
`fresh_holdout_slots` selection receipt, and all three required systems
(`ours`, `glim`, and `fast_livo2`). Fresh slots are separate from the exposed
historical `holdout_slots` (`exp02`, `exp03`, and `exp21`); those historical
sequences can never be relabelled as fresh. Every profile fresh slot must be
`frozen_unopened`/`frozen` with a selection-receipt, input-manifest,
ground-truth, and calibration SHA-256, and the evidence must match all of
them. The profile records the reviewed, deep-verified Exp14, Exp16, and Exp18
identities in
`configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml` with
status `frozen_unopened`. The receipt fixes the official source revision, bag
sizes/SHA-256 values, opaque GT identities, calibration tree, canonical ROS2
and semantic/input-manifest identities, exposure audit, and blind-order
policy. The execution-identity receipt/preflight is ready/PASS before first
run; a real v2 evidence receipt remains `INCOMPLETE` until all required runs
exist, and no self-declared evidence can promote it.

The evidence contract has explicit `partitions.historical` (role
`regression`) and `partitions.fresh` (role `primary_fresh`) blocks. Every
system must provide exactly three complete runs for every dataset in both
partitions. Historical rows are not used for the aggregate victory APE/CI, but
they remain mandatory regression coverage.

Each system contains pinned provenance (`revision`, container digest, and
64-hex toolchain/config fingerprints) plus common scorer fingerprint,
input/reference/calibration, hardware, machine, thread-policy, and exact
`Release` identity. The thread policy must include `cpu_affinity`,
`max_threads`, `omp_num_threads`, `openblas_num_threads`, `mkl_num_threads`,
`tbb_num_threads`, and `accelerator_policy`; all systems must match the
canonical mapping hash. Each run repeats the dataset identity hashes, so a
global input hash cannot hide a cross-dataset mismatch. Historical identity
uses the profile's `input_manifest_sha256`, `ground_truth_sha256`, and
`calibration_archive_sha256`; fresh identity additionally includes the
selection receipt hash. Failed runs remain records and force `FAIL`; an
omitted run is `INCOMPLETE`. A complete run must include finite positive APE,
processing RTF at most 1.0, peak RSS, all map metrics, completion/exit status,
zero catastrophic failures/verified false loops, and trajectory/map SHA-256
artifacts.

The accuracy gate selects the lowest aggregate APE rival and requires ours to
improve by at least 10%. Its 95% superiority interval is a true two-stage
hierarchical bootstrap: fresh datasets are resampled as clusters with fixed
seed `20260821`, then ours and each rival's three runs inside every selected
dataset are independently resampled before their means are compared (10,000
draws). Runs are explicitly not treated as pseudo-independent datasets. All
rivals must have a positive lower CI bound. Each dataset also has a 2%
primary-APE regression limit against that sequence's best rival, and map
non-regression is checked separately for every dataset/rival pair before any
suite aggregation. The command writes matching JSON and YAML receipts with
`PASS`, `FAIL`, `INCOMPLETE`, or `INVALID`, including profile/evidence SHA-256
identities; missing real competitor evidence therefore cannot become a
victory by aggregation.

The scorer fingerprint is not a hand-entered label: the preflight checker
sorts scorer entries by name and hashes canonical JSON containing each entry's
name, repository-relative path, measured file SHA-256, and declared policy.
The receipt fingerprint must equal that recomputed digest. The checker also
requires every system's revision/container/toolchain status to be `ready`,
`frozen`, or (for source revisions only) `pinned`, rejects dirty worktrees,
and requires clean tracked-diff/untracked-content provenance before a run.
Pending status values remain `INCOMPLETE` even if a placeholder digest is
present.

Before any competitor run, freeze the execution identity with the separate
preflight receipt:

```bash
python3 scripts/check_competitive_execution_selection.py \
  --receipt configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --output <out>/competitive_execution_preflight.json \
  --yaml-output <out>/competitive_execution_preflight.yaml
```

The profile records the receipt path and full-file SHA-256. The checked-in
receipt is now `ready`: the ours clean revision, machine fingerprint,
eight-thread policy, all three pinned container/toolchain identities, and the
deep-verified fresh input identities are recorded. Missing values remain
`INCOMPLETE`; malformed or changed paths/digests are `INVALID`. This check is
read-only and performs no container build, dataset download, ground-truth
inspection, or benchmark run. The identity records exact `Release`,
revision/config/container/toolchain/scorer/machine/thread fields, plus the
modality/calibration policy: GLIM CPU is lidar+IMU, FAST-LIVO2 is
lidar+IMU+five-camera visual, and ours is the lidar+IMU track. These are
fairness constraints, not performance evidence. Before any run, enforce the
recorded policy with `taskset` and the matching Docker `--cpuset-cpus` setting,
and explicitly export `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`,
`MKL_NUM_THREADS`, and `TBB_NUM_THREADS` as recorded; this receipt update does
not change the benchmark runners.

The profile/receipt registration uses the non-cyclic
`canonical_profile_sha256_v1` contract. It parses the complete
`competitive_slam_profile` YAML mapping, removes only
`evidence_gate_v2.execution_selection_receipt_sha256`, then serializes the
mapping as UTF-8 JSON with sorted keys, compact separators, and
`ensure_ascii=true` before hashing with SHA-256. The receipt stores that value
as `common_identity.profile_sha256` plus its hash kind. The profile continues
to store the raw full-file SHA of the receipt, so YAML formatting changes are
visible there without creating a mutual-hash cycle. Any other profile field
mutation changes the canonical profile hash; missing, wrong-kind, or mismatched
values remain fail-closed (`INCOMPLETE` for unresolved pending data and
`INVALID` for malformed/tampered data).

To refresh the pending identity without touching the reviewed receipt, create
an observation artifact and then finalize it against the same receipt:

```bash
python3 scripts/capture_competitive_execution_identity.py capture \
  --receipt configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --output <out>/execution_identity_capture.json \
  --yaml-output <out>/execution_identity_capture.yaml
python3 scripts/capture_competitive_execution_identity.py finalize \
  --receipt configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --capture <out>/execution_identity_capture.yaml \
  --output <out>/execution_identity_finalize.json \
  --yaml-output <out>/execution_identity_finalize.yaml
```

Both commands are read-only with respect to the receipt. Capture records the
current worktree provenance, machine fingerprint, OpenMP-related environment,
and locally available Docker image IDs. For an explicitly bound local image,
capture also runs bounded `--pull=never --network none --read-only` probes for
compiler/linker/ROS/PCL/Eigen/OpenMP and binds the result to the inspected
image digest; a source checkout binding supplies Git provenance only. It does
not pull/build images or open fresh bags/GT. Finalize refuses a capture
from another receipt and cannot promote `pending` to `ready`/`frozen`. The
current worktree therefore produces `INCOMPLETE`, as required; only a later
reviewed clean revision with system-container toolchain identities and a
complete equal thread policy can be explicitly frozen.

For a measured local checkout or image, bindings are explicit and repeatable;
they never clone, build, or download anything:

```bash
python3 scripts/capture_competitive_execution_identity.py capture \
  --source ours=/path/to/ours \
  --source glim=/path/to/glim \
  --source fast_livo2=/path/to/FAST-LIVO2 \
  --image glim=glim-cpu-benchmark:competitive-v1 \
  --image fast_livo2=fast-livo2-benchmark:ros1-pinned \
  --receipt <receipt.yaml> --profile <profile.yaml> \
  --output <out>/capture.json
```

When a rival checkout or local image is not bound, the observation contains a
machine-readable probe manifest with the exact compiler/linker/ROS/PCL/Eigen/
OpenMP commands still required; it does not infer readiness. A complete
synthetic or clean ready/frozen contract can return `PASS`; any receipt left
pending remains `INCOMPLETE` until an operator explicitly reviews and updates
it. The current M5d execution receipt is ready, while the evidence gate still
awaits benchmark run records.

### M5c fresh-holdout download checkpoint (2026-08-21)

Fresh input acquisition is a separate, opaque-hash-only checkpoint. The
selection receipt is reviewed independently first; this tool never edits the
selection receipt or competitive profile. Use an explicit destination on the
benchmark storage volume. The read-only `plan` action is the first step and
does not access the network or dataset contents:

```bash
python3 scripts/freeze_competitive_fresh_holdouts.py plan \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --output /tmp/fresh-holdout-plan.json
```

After a separate review of that plan and selection receipt, run the actions in
this order:

```bash
python3 scripts/freeze_competitive_fresh_holdouts.py download \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821
# If a transfer is interrupted, use this instead of repeating download:
python3 scripts/freeze_competitive_fresh_holdouts.py download --resume \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821
python3 scripts/freeze_competitive_fresh_holdouts.py verify \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821
```

Before `finalize`, prepare the canonical ROS 2 tree and semantic report from
the verified raw bags. The preparation command is sequence-scoped or can cover
all managed manifests; it requires `rosbags==0.11.0` and uses the fixed
`rosbags-convert` command recorded in its preparation receipt:

```bash
python3 scripts/prepare_competitive_fresh_ros_inputs.py \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --all
# Or, for one slot (and --resume only after an interrupted preparation):
python3 scripts/prepare_competitive_fresh_ros_inputs.py \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --sequence exp14 --resume
```

It rechecks only the raw-bag byte count/SHA from each
`downloaded_hashed` manifest and never opens the manifest's GT path. Each
conversion is written to `slots/<seq>/canonical_ros2.part`, checked for the
seven-topic contract, compared against the raw ROS 1 bag with
`compare_rosbag_semantic_inputs.py`, then atomically published as
`canonical_ros2/`, `semantic_equivalence.json`, and
`preparation_receipt.json`. The receipt binds the plan SHA, manifest/raw
identity, Python/NumPy/rosbags versions, converter/comparator script hashes,
exact argv, ROS 2 tree hash, and semantic report hash. Existing output is
accepted only when that receipt and all hashes still match; the final receipt
is the commit marker. A crash after conversion, comparison, or either of the
first two atomic renames is resumable only when each artifact has exactly one
of its `.part`/final forms; a staged receipt must validate its full identity,
while a converter/comparator partial without a receipt is only accepted after
its safe tree/report validation. Mixed or symlinked output fails closed. After
this step, pass
`slots/<seq>/canonical_ros2` and `slots/<seq>/semantic_equivalence.json` to
the downloader's `finalize` command. The external managed root used for this
checkpoint has been converted and deep-verified; its fresh slots are now
`frozen_unopened`. Receipt/profile updates are a separate reviewed operation.

Only after preparation and its separate review, publish the downloader's
final state:

```bash
python3 scripts/freeze_competitive_fresh_holdouts.py finalize \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --ros2-root exp14=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp14/canonical_ros2 \
  --ros2-root exp16=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp16/canonical_ros2 \
  --ros2-root exp18=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp18/canonical_ros2 \
  --semantic-report exp14=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp14/semantic_equivalence.json \
  --semantic-report exp16=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp16/semantic_equivalence.json \
  --semantic-report exp18=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp18/semantic_equivalence.json
```

`--resume` is only for a managed, identity-matching staging directory; a
complete final slot is re-verified and skipped, while a stale or mixed final/
staging tree fails closed. The plan and managed-root marker bind the selection
receipt SHA and the runtime SHA-256 of this producer script, so changing the
producer or selection contract cannot reuse an old download. Raw bags are
checked by expected byte count and official LFS SHA-256. Ground truth is never
parsed or printed: only its opaque byte count/SHA-256 is recorded. Calibration
files are checked by bytes, SHA-256, Git blob identity, and a canonical logical
tree hash; storage paths are kept separate from logical paths. `finalize`
verifies every manifest before calculating the canonical ROS 2 input identity,
and publishes each state atomically. The output root is
`/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821` for the
current preregistration. The M5d review deep-verified this managed root and
recorded selection/profile/execution identities. Ground truth remains opaque
and no benchmark/metric was run; no README/SOTA claim follows from acquisition
metadata.

After a reviewed `frozen_unopened` tree exists, run the independent deep
verifier before using it in a benchmark:

```bash
python3 scripts/verify_competitive_frozen_holdouts.py \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --output <out>/frozen_holdouts_deep_verification.json
```

The verifier requires exactly Exp14/16/18 and rechecks the managed marker,
selection/plan identities, official bag byte/LFS-SHA identity, calibration
byte/Git-blob identity, and every final manifest. The bag's preregistered
Git-blob OID is the immutable Git LFS pointer provenance; it is format-checked
and reported, never compared with the downloaded bag's content blob. Ground
truth remains an opaque stream: no GT path, text, trajectory, or score is printed.
The verifier recomputes the safe
canonical ROS 2 metadata/tree hash, seven-topic semantic report hash,
input-manifest payload hash, and the preparation receipt's deterministic
pre-finalization manifest **file** hash (canonical compact JSON plus its
trailing newline; distinct from the newline-free payload hash). Its JSON
summary includes each manifest and
preparation-receipt file SHA. Missing slots, symlink/path traversal, stale
receipt/runtime/argv identity, or artifact tampering are hard failures. The
M5d invocation passed against the managed root and records the enriched
selection SHA plus its explicit committed preregistration anchor; no GT
content or metric was parsed. The selection/profile/execution identity update
was reviewed separately from the freezer.

### Pinned benchmark image recipes

The checked-in execution receipt now names a repo-owned build recipe and build
entrypoint for each system. Run the entrypoint only after the source revision
and execution identity have been reviewed:

```bash
bash scripts/build_competitive_benchmark_images.sh --system all
```

The recipes use immutable `sha256` base-image references, pin the ours/GLIM/
FAST-LIVO2 source revisions, and set the recorded CPU-only thread environment.
The ours recipe receives a Docker context containing only its Dockerfile. It
clones the public `lidar_slam_ros2` repository at `OURS_REVISION`, verifies the
`ndt_omp_ros2` gitlink and initializes only that build-required submodule, then
checks the detached HEAD, clean status, and initialized submodule status before
rosdep or compilation. The pinned `rko_lio` gitlink is intentionally left
uninitialized: its object is unavailable from the public mirror and it is not
needed by this `BUILD_TESTING=OFF --packages-up-to lidarslam graph_based_slam`
image target. The recipe fails closed if either gitlink changes or `rko_lio`
appears in colcon discovery. This prevents a dirty host checkout or a source
archive with missing submodule contents from entering the image.
GLIM's CPU path does not consume PCL; its receipt explicitly records `pcl` as
`not_applicable`, and the container probe fingerprints that sentinel rather
than installing an unused package or falling back to the host. The capture
tool only permits this exception for GLIM/PCL; compiler, linker, ROS, Eigen,
OpenMP, and all ours/FAST fields remain mandatory and fail closed.
The FAST-LIVO2 recipe builds its ROS 1 workspace under `/opt/fast_livo_ws`;
its pinned image and system-container toolchain probe are now observed ready;
it does not depend on the historical undocumented
`fast-livo2-benchmark:noetic` or `hdl_localization_noetic:local` images. Its
legacy Sophus compatibility commit is an explicit full-length build-time pin.
FAST's upstream HILTI22 configuration is now recorded as an external-container
artifact at `/opt/fast_livo_ws/src/FAST-LIVO2/config/HILTI22.yaml`, with SHA-256
`efae9e702c71c770b19002b6e19d4e1b6f46c67df3727e984981d932258f0b4a`. The entry is
`observed` and is bound to the immutable FAST image
`sha256:ddc75b574f8cca1e111332153e31a65c74ccdb11f8059da3797ab130814ce17e`;
the checker never treats that container path as a host file. Fresh execution
inputs remain pending.
`--pull=false` is
intentional: a missing base image or source ref must fail rather than silently
changing the identity.

This recipe wiring is provenance infrastructure, not benchmark evidence. The
three pinned image/toolchain observations and the fresh-input identity are now
marked ready after clean builds, read-only probes, and deep verification. The
checked-in receipt is ready for a first run; this does not constitute benchmark
evidence or an accuracy/SOTA claim.

The checked-in synthetic tests cover the exact 10% boundary, missing and
failed runs, old schema, false freshness, pending slots, dataset hash
mismatches, invalid fingerprints, identity/RTF failures, within-run CI
variance, sequence collapse, per-dataset map regression, and all-rival
bootstrap superiority. No real fresh-slot competitor receipt currently
satisfies this contract; existing exp02/exp21 assets with failures or RTF
above one remain negative evidence. README superiority claims are unchanged.

## SLAM candidate regression

Run plane-revisit OFF/ON with the same backend input and reference:

```bash
bash scripts/run_plane_revisit_candidate_benchmark.sh \
  --dataset mid360_public --bag <backend-input-bag> \
  --reference-tum <reference.tum> --fixed-loop-edges <verified.csv> \
  --output-dir /media/<ssd>/benchmarks/phase7/mid360
```

Repeat with `hilti_exp04` and `rtkslam_construction_seq2`. Construction Seq2 is
the required second positive sequence; its total-station checkpoints contain
positions but no surveyed orientations, so rotational RPE is forbidden.
`--dry-run` prints the pipeline without starting ROS. For submap-rate backend
poses, supply the matching frontend trajectory with `--dense-raw-tum`.

To create a deterministic backend input from the original sensor bag:

```bash
ROS_DOMAIN_ID=210 ROS_LOCALHOST_ONLY=1 \
bash scripts/record_backend_input.sh --output-dir <work>/backend_input -- \
  bash scripts/run_rko_lio_graph_benchmark.sh \
    --bag <construction_seq2> --lidar-topic /livox/points \
    --imu-topic /livox/imu --skip-reference-gen \
    --reference-tum <construction_seq2_gt.tum> \
    --reference-meta <construction_seq2_reference.json> \
    --rko-param configs/mid360_robot/rko_lio_mid360_low_voxel_no_deskew.yaml \
    --lidarslam-param lidarslam/param/lidarslam.yaml \
    --output-dir <work>/source_run --offline-timeout-secs 5400
```

The recorder refuses overwrite, flushes MCAP on exit, and requires non-empty
`/rko_lio/odometry` and `/rko_lio/frame` topics. When `--dense-raw-tum` is
supplied to the candidate runner, its timestamp span is used as the runtime
denominator; backend bags use processing time, not original sensor time.
Compare the three dataset pairs:

```bash
python3 scripts/evaluate_slam_candidate_regression.py \
  --baseline <mid360-off>/cross_repo_benchmark.json \
  --baseline <hilti-off>/cross_repo_benchmark.json \
  --baseline <construction-seq2-off>/cross_repo_benchmark.json \
  --candidate <mid360-on>/cross_repo_benchmark.json \
  --candidate <hilti-on>/cross_repo_benchmark.json \
  --candidate <construction-seq2-on>/cross_repo_benchmark.json \
  --output output/phase7/candidate_regression.json --require-pass
```

Promotion requires complete reports, matching inputs, bounded runtime and map
quality, and independently improved MID-360 and Construction Seq2 trajectories.

When aggregate ATE hides which surveyed positions changed, generate a
checkpoint-level JSON and Markdown report:

```bash
python3 scripts/analyze_sparse_checkpoint_errors.py \
  --reference-tum <surveyed-positions.tum> \
  --reference-csv <checkpoint-labels.csv> \
  --estimate raw=<raw.tum> --estimate baseline=<off-dense.tum> \
  --estimate candidate=<on-dense.tum> --baseline-label baseline \
  --output <suite>/checkpoint_errors.json
```

Each trajectory is independently SE(3)-aligned without scale before per-point
errors are compared, matching the position-only public-suite ATE semantics.

That wrapper:

- uses the bundled NTU VIRAL `rosbag2`
- runs `RKO-LIO + graph_based_slam`
- saves raw and corrected trajectories
- computes APE against the Leica prism reference
- verifies the Autoware map bundle when present
- writes `metrics.json` for the reporting pipeline

## KITTI / LiDAR-Only Evaluation

The public default benchmark remains `RKO-LIO + graph_based_slam`. For KITTI
Odometry, use the separate LiDAR-only path because the Velodyne dataset does
not provide IMU messages.

```bash
bash scripts/download_kitti_odometry.sh --velodyne
export KITTI_ODOMETRY_ROOT="$PWD/datasets/KITTI_odometry"
bash scripts/run_kitti_odometry_benchmark.sh --sequence 00 --small-gicp --force-prepare
```

For frontend tuning, run the sweep wrapper:

```bash
bash scripts/sweep_kitti_small_gicp.sh \
  --dataset "$KITTI_ODOMETRY_ROOT" \
  --sequences "00 05 07"
```

The LO and `small_gicp` wrappers generate a rosbag2 QoS override so PointCloud2
playback uses `best_effort`, matching the frontend sensor-data subscriptions.

## Optional 3D-BBS Verification

`graph_based_slam` can build MIT-licensed 3D-BBS support from
`Thirdparty/3d_bbs`. This is an optional verifier for Scan Context loop
candidates, not part of the default public benchmark path.

Build behavior:

- enabled at build time when `GRAPH_BASED_SLAM_ENABLE_3D_BBS=ON` and the vendor
  headers are present
- disabled at runtime unless `use_3d_bbs_for_scan_context: true` is set
- force-disabled with
  `colcon build --symlink-install --cmake-args -DGRAPH_BASED_SLAM_ENABLE_3D_BBS=OFF`

MID-360 wrapper example (research track, `report_only_until: v0.4` in
`scripts/release_profiles.yaml`):

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh \
  --use-3d-bbs-for-scan-context true
```

Typical outputs are written under:

- `output/bench_rko_lio_ntu_viral_<name>/traj_raw_prism.tum`
- `output/bench_rko_lio_ntu_viral_<name>/traj_corrected_prism.tum`
- `output/bench_rko_lio_ntu_viral_<name>/ape_raw_vs_gt.txt`
- `output/bench_rko_lio_ntu_viral_<name>/ape_corrected_vs_gt.txt`
- `output/bench_rko_lio_ntu_viral_<name>/metrics.json`

## Loop Cloud-Overlap Gate

After registration, the backend can require a fraction of aligned source
points to have target-cloud support. `loop_min_overlap_ratio: 0.0` keeps the
gate disabled for backward compatibility; `loop_overlap_max_distance_m`
defines the nearest-neighbor support radius. The cheap fitness and correction
gates run first, so rejected registrations do not pay the KD-tree cost.

Construction Seq2 validated the following dataset-specific candidate:

```yaml
loop_min_overlap_ratio: 0.76
loop_overlap_max_distance_m: 0.5
```

The ratio rejected the harmful `57 -> 123` revisit and its adjacent
substitutes while retaining five beneficial loop edges. Do not promote this
threshold to a general default until it passes the other release datasets.

For a cross-sensor candidate, the source-overlap threshold can be explicitly
relaxed only when registration applies a large translation correction:

```yaml
loop_min_overlap_ratio: 0.76
loop_min_overlap_ratio_large_correction: 0.70
loop_overlap_large_correction_translation_m: 1.0
loop_overlap_max_distance_m: 0.5
```

The effective threshold is 0.76 below 1.0 m correction and 0.70 at or above
it. Leaving either large-correction parameter at zero disables the override.
The candidate preserved the established MID-360 loop, rejected the HILTI
exp04 false loop, retained Construction Seq2's five verified edges, and
preserved KITTI 00's `28 -> 176` loop (source overlap 0.864002) with
byte-identical edge and trajectory artifacts. The generic YAML default remains
disabled while broader release validation is pending. Reverse and harmonic
overlap are emitted in debug logs for diagnosis, but are not acceptance gates
because target aggregation extent biases them.

Accepted candidates and debug attempts also report `support_rmse_m` and
`support_p90_m`. These are nearest-neighbour distances for source points that
fall within `loop_overlap_max_distance_m`; they reuse the overlap KD-tree and
do not launch another search. Treat them as diagnostics, not gates: repeated
geometry can produce low support residuals at the wrong longitudinal offset.
The p90 calculation uses linear-time selection rather than sorting all
supported points.

HILTI exp01/exp07 can be frozen and compared end to end with one command. Raw
bags and generated backend MCAPs stay on the external SSD by default:

```bash
bash scripts/run_hilti_overlap_crossval.sh --sequence all --runs 2
```

Use `--dry-run` to inspect every command, `--record-only` to stop after input
capture, or `--offline-only --resume` to reuse an existing capture. Each
sequence writes `comparison.json` and `comparison.md` beside `gate_off/` and
`gate_adaptive/`. The capture stage generates a parameter snapshot that keeps
submap publication active but disables expensive live loop registration; both
offline variants then consume the exact same odometry/cloud pairs.

## Summaries And HTML Report

To summarize all collected runs:

```bash
python3 scripts/benchmark_summary.py \
  --root output \
  --write-md output/benchmark_summary.md \
  --write-csv output/benchmark_summary.csv
```

To generate the static HTML report:

```bash
python3 scripts/generate_html_report.py \
  --root output \
  --out output/latest_report.html
```

To generate a short public-beta readiness report from the current local
artifacts:

```bash
python3 scripts/generate_v2_beta_readiness_report.py
```

By default this writes:

- `output/v2_beta_readiness_<YYYYMMDD>.md`

To generate a short public-facing map-authoring positioning report from the
tracked benchmark, GNSS, dynamic-filter, and classic-path artifacts:

```bash
python3 scripts/generate_map_authoring_report.py \
  --out output/map_authoring_report_$(date +%Y%m%d).md \
  --write-json output/map_authoring_report_$(date +%Y%m%d).json
```

To stage a reusable submission-style bundle from an existing run directory:

```bash
bash scripts/create_map_authoring_submission_bundle.sh \
  output/bench_rko_lio_ntu_viral_fresh_20260324 \
  output/submission_bundle_ntu_viral_fresh \
  --report output/map_authoring_report_$(date +%Y%m%d).md \
  --verify-map
```

That bundle standardizes:

- `pointcloud_map/`
- `map_projector_info.yaml`
- `metrics.json` when present
- trajectories and key logs when present
- focused reports under `reports/`, with sibling `json/svg` copied automatically when present
- `map_qa_summary.md`
- `manifest.json`

To generate a separate stress-validation report that distinguishes the current
default path from older long-loop and hard-dataset evidence:

```bash
python3 scripts/generate_stress_validation_report.py
```

By default this writes:

- `output/stress_validation_report_<YYYYMMDD>.md`

To summarize dynamic-object-filter behavior across the tracked Leo Drive
save-time benchmarks:

```bash
python3 scripts/generate_dynamic_object_filter_validation_report.py \
  --out output/dynamic_object_filter_validation_report_$(date +%Y%m%d).md \
  --write-json output/dynamic_object_filter_validation_report_$(date +%Y%m%d).json \
  --write-svg output/dynamic_object_filter_validation_report_$(date +%Y%m%d).svg
```

The default report compares the tracked `bag1` and `bag6` dynamic-filter
benchmarks, so point reduction and voxel-removal behavior can be discussed as
cross-dataset evidence rather than a single-case anecdote. It also reports
coarse tile-footprint preservation via shared metadata tiles, tile jaccard,
and filtered-tile overlap ratio.

To promote an already-recorded aligned cross-validation run such as the MID360
long-loop check into `metrics.json` so it appears in `benchmark_summary.md` and
`latest_report.html`:

```bash
python3 scripts/write_aligned_trajectory_metrics.py \
  --out-dir output/bench_rko_lio_mid360_v3 \
  --bag demo_data/glim_mid360/rosbag2_2024_04_16-14_17_01 \
  --reference-tum output/glim_mid360_reference.tum \
  --corrected-tum output/bench_rko_lio_mid360_v3/traj_corrected.tum \
  --raw-tum output/bench_rko_lio_mid360_v3/traj_raw.tum \
  --graph-log output/bench_rko_lio_mid360_v3/graph_slam.log \
  --reference-source glim_mid360_reference \
  --reference-kind cross_validation \
  --reference-label GLIM \
  --points-topic /livox/lidar \
  --points-frame livox_frame \
  --robot-frame livox_frame
```

The summary/report pipeline now exposes the reference kind, so `ground_truth`
and `cross_validation` runs do not appear as if they were the same type of APE.

For a public-facing snapshot built on top of these artifacts, see
`docs/comparison.md` and `docs/releases/v0.2.2.md`.

To rerun the current MID360 cross-validation benchmark end-to-end:

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh
```

This MID360 wrapper defaults to a tuned `RKO-LIO + graph_based_slam` profile
with `voxel_size=0.5`, `max_range=80.0`, `search_submap_num=5`,
`loop_edge_dedup_index_window=20`, and `loop_edge_info_weight=200`.

To benchmark the real open-data Leo Drive `driving_30_kmh` bag with mixed
RTK/non-RTK GNSS quality:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

That wrapper writes a local `Applanix_GSOF49` reference trajectory,
`traj_raw.tum`, `traj_corrected.tum`, and `metrics.json` so the run appears in
`benchmark_summary.md` and `latest_report.html`.

When the main bag already contains native `sensor_msgs/msg/NavSatFix` or
`sensor_msgs/msg/Imu`, the same wrapper now prefers those real topics before it
falls back to Applanix sidecar generation.

Current Leo Drive packet-path evidence is:

- `driving_30_kmh`, GNSS-only classic path: `APE RMSE 195.285 m`
- `bag1_front`, `no_imu`: `APE RMSE 0.248 m`
- `bag1_front`, native `/sensing/imu/imu_data`: `APE RMSE 0.251 m`
- `bag6_front`, `no_imu`: `APE RMSE 0.422 m`
- `bag6_front`, native `/sensing/imu/imu_data`: `APE RMSE 0.365 m`

The important result is that packet IMU deskew is usable on the native
`all-sensors` bags, but only when the benchmark is replayed conservatively.
The wrapper now auto-selects `rate=1.0` whenever `--use-imu=true` and `--rate`
is omitted. The earlier `20m+` regressions were runtime-sensitivity artifacts,
not a proof that the deskew math itself was fundamentally broken. To reproduce
the current experimental IMU result on the driving bag:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --use-imu true \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --robot-frame-id base_link \
  --imu-frame-id base_link \
  --verify-map
```

To compare the same packet path on `all-sensors-bag6` while isolating IMU
deskew from GNSS:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/front/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --use-gnss false \
  --verify-map

bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/front/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --use-gnss false \
  --use-imu true \
  --verify-map

bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/left/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --use-gnss false \
  --use-imu true \
  --imu-rotation-use-orientation false \
  --verify-map
```

To summarize the current cross-dataset odom-prior validation evidence after the
classic-path runs have been recorded:

```bash
python3 scripts/generate_odom_prior_validation_report.py \
  --out output/odom_prior_validation_report_$(date +%Y%m%d).md \
  --write-json output/odom_prior_validation_report_$(date +%Y%m%d).json \
  --write-svg output/odom_prior_validation_report_$(date +%Y%m%d).svg
```

This report intentionally compares `driving_30_kmh` and `bag6_front` side by
side, because the current velocity-based prior helps the fallback classic path
on one dataset and hurts or helps differently on another.

To validate packet IMU deskew as a repeatable matrix on real open data, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_packet_imu_deskew_validation_matrix.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg
```

That matrix compares `no_imu` and native-IMU runs for the default `bag1_front`
and `bag6_front` cases at `rate=1.0` and emits:

- `packet_imu_deskew_validation.md`
- `packet_imu_deskew_validation.json`

The report is generated by `generate_packet_imu_deskew_validation_report.py`
and fails if any case violates the configured path-coverage, RMSE-regression,
or matched-pose thresholds.

The same bag also exposes native `/gnss/fix`. The backend now falls back to
receive time when the NavSatFix header stamp is far from ROS time
(`gnss_header_stamp_max_skew_sec`, default `30 s`), which lets the graph attach
GNSS edges on `all-sensors-bag6`. In practice that native `/gnss/fix` still
disagrees with the `GSOF49` reference enough to degrade the cross-validation
APE, so `all-sensors-bag6` is useful for georeferenced smoke tests but not a
clean GNSS benchmark source.

To compare place-recognition behavior on MID360, rerun the same benchmark with
and without an optional descriptor family and then render the short report:

```bash
bash scripts/run_place_recognition_benchmark.sh
```

To compare the current experimental BEV-assisted distance rerank instead:

```bash
bash scripts/run_place_recognition_benchmark.sh --candidate-mode bev_rerank
```

The report shows:

- runtime `use_scan_context`
- accepted/attempted loop counts
- accepted loop source counts
- observed `ScanContext loop candidate` count
- observed `BEV rerank hint` count
- observed `SOLiD rerank candidate` count
- `APE RMSE` delta between the two runs
- optional JSON summary via `--write-json`
- optional SVG summary via `--write-svg`

The report is generated by `generate_place_recognition_report.py`.

Current checked-in evidence is:

- fair current-code baseline rerun:
  `output/bench_rko_lio_mid360_current_default_rerun_20260326/metrics.json`
  (`APE RMSE 4.096 m`)
- current best checked-in Scan Context candidate with DB/index fix,
  aggregated descriptor/registration cloud, and `scan_context_threshold=0.55`:
  `output/bench_rko_lio_mid360_sc055_yawguess_scagg_screg_20260326/metrics.json`
  (`APE RMSE 3.568 m`)
- current experimental BEV-assisted distance rerank:
  `output/bench_rko_lio_mid360_20260326_202840/metrics.json`
  (`APE RMSE 3.607 m`)
- best observed BEV-assisted distance rerank:
  `output/bench_rko_lio_mid360_20260326_202119/metrics.json`
  (`APE RMSE 3.533 m`)
- short comparison report:
  `output/place_recognition_report_20260326.md`

That candidate currently beats both the fair rerun baseline and the published
`3.641 m` default artifact, but the accepted loop still comes from the
distance-based path. Treat `use_scan_context=true` as an opt-in tuning path
rather than the repository default.

The BEV path is now more useful as a sensor-agnostic distance-candidate rerank
than as a standalone loop source. It has shown better-than-baseline runs, but
its rerun variance is still too large for a default-on setting.

To summarize the current stop/go decisions for place recognition and the
classic fallback path in one short report:

```bash
python3 scripts/generate_exploration_closeout_report.py \
  --out output/exploration_closeout_report_$(date +%Y%m%d).md \
  --write-json output/exploration_closeout_report_$(date +%Y%m%d).json
```

A local snapshot can be written to:

- `output/exploration_closeout_report_20260327.md`

That report fixes the current repository position in one place:

- public default place recognition remains the distance-based path
- `Scan Context` stays opt-in
- `BEV-assisted rerank` stays experimental
- `SOLiD` stays experimental/off by default
- the classic path remains a fallback workflow rather than the main public path

## Dynamic Object Filter Benchmark

The dynamic-object filter is save-time only. It does not change live odometry
or loop closure, so the right comparison is the saved map output with the same
bag and the same backend settings.

Run the paired comparison on the open-data bag6 smoke path:

```bash
bash scripts/run_dynamic_object_filter_benchmark.sh
```

That wrapper:

- runs `run_open_data_gnss_smoke.sh` twice on the same bag
- saves `no_filter/` and `dynamic_filter/` outputs under one root
- renders `dynamic_object_filter_report.md`,
  `dynamic_object_filter_report.json`, and `dynamic_object_filter_report.svg`

The report is generated by `generate_dynamic_object_filter_report.py` and
tracks:

- Autoware map verify result for both runs
- projector type
- saved grid cell count
- metadata tile count
- total saved point count
- filter candidate/kept/removed voxel counts
- saved-point reduction ratio

The current checked-in evidence is:

- baseline smoke:
  `output/open_data_gnss_smoke_bag6_autodetect_throttled_20260325`
- filtered smoke:
  `output/open_data_gnss_smoke_bag6_dynamic_filter_20260326`
- benchmark report bundle:
  `output/dynamic_object_filter_benchmark_bag6_20260326`

In that checked run, the saved map went from `138732` to `87861` points while
keeping `verify_autoware_map.py` at `PASS`.

## Leo Drive Classic Path Benchmark

To compare the current classic `scanmatcher + graph_based_slam` path on the
mixed-quality Leo Drive `driving_30_kmh` open-data bag, run:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_classic_path_benchmark_suite.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

This wrapper emits:

- `classic_path_report.md`
- `classic_path_report.json`
- `classic_path_report.svg`

The report is generated by `generate_classic_path_report.py`.

The checked-in snapshot is:

- `output/classic_path_report_20260327.md`

Current evidence is:

- `no GNSS`: `APE RMSE 313.695 m`
- `GNSS only`: `APE RMSE 195.285 m`
- `GNSS + velocity-based planar odom prior`: `APE RMSE 175.732 m`
- `GNSS + IMU`: `APE RMSE 271.144 m`

So the classic path still needs work, but the direction is clearer now:
backend GNSS helps substantially, and a velocity-based planar odom prior helps
further on `driving_30_kmh`, while the current packet IMU path is still not a
default recommendation.

## Release/Readiness Gate

To run the local readiness gate in one command:

```bash
bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10
```

That wrapper can run:

- default build and package tests
- benchmark summary generation
- HTML report generation
- optional public MID-360 segment-reset completion gate
- standalone public MID-360 continuous kidnap-relocalization gate
- optional Autoware dogfood

With `--ape-threshold`, the gate is hard:

- it exits non-zero if any selected run is missing APE
- it exits non-zero if any selected run exceeds the threshold
- by default `run_release_readiness_checks.sh` applies that hard gate only to
  `ground_truth` runs; `cross_validation` runs stay visible in reports without
  blocking release

For the public MID-360 segment-reset completion evidence, add:

```bash
bash scripts/run_release_readiness_checks.sh \
  --skip-default-ci \
  --skip-benchmark-summary \
  --public-mid360-completion
```

That hook runs `scripts/run_mid360_robot_public_completion_gate.py` as a hard
gate and writes its JSON/Markdown under the release-readiness output directory.

### Paired map-quality non-regression

When a candidate map has a like-for-like baseline report, the release wrapper
can add a fail-closed paired check without changing the existing absolute
profile gate:

```bash
bash scripts/run_release_readiness_checks.sh \
  --skip-default-ci \
  --skip-benchmark-summary \
  --map-quality-pcd /path/to/candidate/map_refined.pcd@configs/map_quality_profiles/indoor_construction.yaml \
  --map-quality-baseline-report /path/to/baseline/map_quality_report.yaml \
  --map-quality-max-regression-percent 2.0
```

`run_map_quality_check.sh` evaluates the candidate's run-1 report against the
named baseline with `scripts/check_map_quality_regression.py`. The five paired
metrics are plane thickness mean/p95 (lower is better), planar coverage and
mean-map-entropy valid fraction (higher is better), and entropy value (higher,
or less negative, is worse). Reports must have finite values, meaningful
planes, and identical extraction settings; a missing field, zero baseline
denominator, or mismatch is invalid and fails closed. The paired budget never
relaxes `indoor_construction.yaml` or any other absolute profile. The command
writes `paired_regression_verdict.yaml` and `.json` beside the map-quality
summary, plus human-readable rows in `paired_regression_verdict.txt`.

The HILTI exp04 current-vs-old map reports used in the M4c diagnostic pass this
2% paired check. Both reports independently violate the indoor profile's
`mme_valid_fraction_min` threshold, so that absolute-profile result remains a
separate applicability issue rather than being hidden by the paired pass.

M4c also closed the fixed backend regression receipts for two HILTI inputs:

- exp04: `/tmp/lidarslam-m4c-hilti-exp04-gate.pzufEB`, three-run artifact
  identity and old optimized trajectory exact, wall `2.71/2.90/2.73 s`,
  maximum RTF `0.010347643`, peak RSS `272.167968750 MiB`, and wall CV
  `3.066357758%`.
- exp07: `/tmp/lidarslam-m4c-hilti-exp07-gate.zCELMb`, three-run artifact
  identity and old optimized trajectory exact, historical interpolated APE
  `0.6186851452574647 m` from 5/6 sparse GT points, wall `1.90/1.90/1.97 s`,
  maximum RTF `0.050166829`, peak RSS `201.136718750 MiB`, and wall CV
  `1.715683698%`.

Both receipts pass the fixed RTF/RSS/CV gates and record the canonical host NDT
`backend_loop` receipt with `target_cell_cache_capacity=3`. These are named
input compatibility/resource gates, not official dense-GT or SOTA comparisons;
the M5 fresh-holdout and competitor protocol remains pending. The old/current
indoor absolute-profile violation is reported separately and is not relaxed.

For the continuous RKO-LIO kidnap-relocalization evidence, run:

```bash
python3 scripts/run_mid360_robot_public_continuous_relocalization_gate.py
```

That gate checks the merged public `outdoor_kidnap_a+b` run for full-duration
RKO output, at least one global relocalization event, loop-alignment PASS,
public loop endpoint closure at the GT start/end stamps, Autoware map verify
PASS, offline completion, and tracked kidnap recovery config matching the run
config. The endpoint closure check prevents a local revisit from being counted
as continuous kidnap relocalization.

## CI Coverage

CI exercises the reporting path in two ways:

- a passing synthetic benchmark fixture must generate summary and HTML report
- a failing synthetic benchmark fixture must trip the threshold gate with
  exit code `2`

The fixture generator is:

```bash
python3 scripts/generate_sample_benchmark_metrics.py \
  --root /tmp/ci_fixture \
  --profile passing
```

Use `--profile failing` to create a negative-path fixture.

## Recommended Artifacts To Publish

If you want benchmark results to be easy to consume, publish:

- `metrics.json`
- `benchmark_summary.md`
- `benchmark_summary.csv`
- `latest_report.html`
- the exact param file used for the run
- `docs/comparison.md` when publishing the current positioning of the repo
- `docs/releases/v0.2.2.md` when publishing the current public beta scope
- `v2_beta_readiness_<YYYYMMDD>.md` when preparing a public beta snapshot
- `stress_validation_report_<YYYYMMDD>.md` when discussing long-loop or
  aggressive-motion evidence

## Related Commands

- Autoware quickstart: `docs/autoware-quickstart.md`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- public comparison page: `docs/comparison.md`
- end-to-end dogfood: `bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20`
