# Public-suite integration and adoption gate (2026-07-13)

## Decision

The current defaults remain unchanged. The shared evaluation loop is complete
for one HILTI geometry/trajectory capture and two independent AIST real-RGB
captures, but no candidate improves two distinct datasets. The aggregate verdict
is therefore `DO_NOT_ADOPT`; this is the intended conservative outcome, not a
failed benchmark.

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
| AIST Ouster RGB | 2 | geometry, held-out RGB, colour, runtime/memory | chromatic fraction 59.52%, 64.14% |

The generated suite report records:

- unique complete datasets: 2;
- improved datasets: 0 (required: 2);
- worst primary regression: 0.0%;
- completeness, multi-dataset, regression, runtime/memory, and raw-integrity
  gates: pass;
- minimum-improved-datasets gate: fail;
- verdict: `DO_NOT_ADOPT`.

External artifact:
`/media/sasaki/aiueo/benchmarks/public_suite_20260713/baseline_adoption_gate.json`.

## Reproduction

Generate one `cross_repo_benchmark.json` per dataset using the commands in the
main README and the AIST RGB benchmark note. Then aggregate them:

```bash
python3 scripts/evaluate_public_suite_gate.py \
  --manifest /path/to/hilti/cross_repo_benchmark.json \
  --manifest /path/to/aist/cross_repo_benchmark.json \
  --out /path/to/public_suite/adoption_gate.json
```

An experimental default may be promoted only when this output is `ADOPT` and
the relevant implementation tests and deterministic artifacts also pass.
