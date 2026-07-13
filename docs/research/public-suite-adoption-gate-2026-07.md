# Public-suite integration and adoption gate (2026-07-13)

## Decision

The current defaults remain unchanged. The shared evaluation loop is complete
for one HILTI geometry/trajectory capture, one Localization Zoo KITTI odometry
capture, and two independent AIST real-RGB captures, but no candidate improves
two distinct datasets. The aggregate verdict is therefore `DO_NOT_ADOPT`; this
is the intended conservative outcome, not a failed benchmark.

## Mechanized contract

`run_cross_repo_slam_benchmark.py` now supports both trajectory datasets and
report-only sensor-quality datasets. A profile that requires `trajectory` runs
Localization Zoo on raw and graph-corrected TUM files. A profile without that
requirement records geometry, held-out alignment, colour, and resource metrics
without inventing a trajectory-accuracy claim.

Each manifest records both repository revisions and hashes all inputs. The AIST
runner additionally hashes the complete rosbag directory, official camera-LiDAR
extrinsic, frontend trajectory, and coloured PLY. Directory hashes include
relative filenames, so file replacement, removal, addition, or content drift is
detectable.

`evaluate_public_suite_gate.py` groups repeated captures by dataset before
counting improvements. It requires:

- complete manifests from at least two distinct datasets;
- the profile's minimum number of independently improved datasets;
- no primary-metric regression beyond the frozen ceiling;
- finite realtime factor and peak RSS with successful process exit;
- all recorded source and derived artifact hashes to remain valid.

## Frozen evidence

| dataset | captures | evidence | primary result |
|---|---:|---|---|
| HILTI 2022 exp04 | 1 | trajectory, geometry, runtime/memory | graph ATE tied (0.07156 m) |
| KITTI Odometry 00 | 1 | Localization Zoo LO, graph, runtime/memory | graph RPE tied (1.01864%) |
| AIST Ouster RGB | 2 | geometry, held-out RGB, colour, runtime/memory | chromatic fraction 59.52%, 64.14% |

The generated suite report records:

- unique complete datasets: 3;
- improved datasets: 0 (required: 2);
- worst primary regression: effectively 0.0% (`8.72e-14%` floating-point
  residue);
- completeness, multi-dataset, regression, runtime/memory, and raw-integrity
  gates: pass;
- minimum-improved-datasets gate: fail;
- verdict: `DO_NOT_ADOPT`.

External artifact:
`/media/sasaki/aiueo/benchmarks/public_suite_20260713/baseline_adoption_gate.json`.
Its SHA-256 after the three-dataset audit is
`2ce9836a8e2b435126352ccfb138dfe4672378d04c34f57f90a889271b9ee234`.

The KITTI row is the current Localization Zoo default TrICP-LO profile
(automatic overlap 0.8, no GT seed), not the earlier overlap-0.9 research
capture. It processes all 4,541 scans at 1.018635% 100 m translational RPE.
The fixed graph backend consumes all 4,541 odometry/cloud pairs, creates 356
submaps, accepts zero loops, and reproduces the same trajectory byte-for-byte
twice. Isolated frontend plus one backend run take 1,509.10 s wall time
(3.2069x the official 470.5816 s sequence span) with 168.08 MiB pipeline peak
RSS. The manifest hashes the raw PCD tree, fixed backend bag, trajectory,
runtime logs, and graph edge set.

## Strict strided Scan Context candidate

The later opt-in KITTI candidate (`scan_context_threshold=0.55`, query stride
4, Scan Context NDT fitness threshold 0.2) improves KITTI 00 translational RPE
from 1.018635% to 1.016778% and first-aligned ATE from 17.645 m to 16.012 m.
Two graph runs produce one byte-identical `28 -> 176` loop and trajectory.
HILTI exp04 remains byte-identical to its no-loop baseline and AIST is
unchanged, so this is only one improved dataset.

The candidate suite gate reports three complete datasets, one improved
dataset, zero primary regression, and passes completeness, multi-dataset,
runtime/memory, raw-integrity, and maximum-regression checks. It fails only
the required two-improved-datasets check and therefore remains
`DO_NOT_ADOPT`. The report is
`/media/sasaki/aiueo/benchmarks/public_suite_20260713/sc055_stride4_gate0p2_adoption_gate.json`
with SHA-256
`7a10b32925e5623da8a001f62a2e5d18771bd9f80372d5afbcd08bb9ec751802`.

## Reproduction

Generate one `cross_repo_benchmark.json` per dataset using the commands in the
main README and the AIST RGB benchmark note. Then aggregate them:

```bash
python3 scripts/evaluate_public_suite_gate.py \
  --manifest /path/to/hilti/cross_repo_benchmark.json \
  --manifest /path/to/kitti/cross_repo_benchmark.json \
  --manifest /path/to/aist/cross_repo_benchmark.json \
  --out /path/to/public_suite/adoption_gate.json
```

An experimental default may be promoted only when this output is `ADOPT` and
the relevant implementation tests and deterministic artifacts also pass.
