# Contributing

Thanks for contributing to `lidarslam_ros2`.

This repository is trying to be a practical default for ROS 2 LiDAR SLAM and
Autoware pointcloud-map generation. Contributions are most useful when they are
easy to reproduce, easy to compare, and clearly scoped.

Participation is governed by the [Code of Conduct](CODE_OF_CONDUCT.md).
The [Governance](GOVERNANCE.md), [Support](SUPPORT.md), and
[Security](SECURITY.md) policies explain decisions, help routes, and private
vulnerability reporting.

## What To Send

Useful contributions include:

- bug fixes in the default permissive workflow
- Autoware pointcloud-map integration fixes
- benchmark results with reproducible commands and artifacts
- parameter improvements backed by logs and trajectory metrics
- documentation improvements that reduce setup time

## Default Workflow Policy

The default and recommended workflow in this repository is permissive-license
only. Contributions that change the default path should preserve that property.

Current default path:

- `RKO-LIO + graph_based_slam`
- local Scan Context implementation
- Autoware-compatible pointcloud map output

Research frontends with other licenses can still be discussed, but they should
not silently become the default path.

## Autoware Naming And Trademark Guidance

Keep Autoware references descriptive.

Preferred wording:

- `Autoware-compatible pointcloud map`
- `pointcloud-map workflow for Autoware`
- `works with Autoware`
- `built on Autoware`

Avoid branding this repository or a derived product as if it were an official
Autoware product or a Foundation-approved distribution.

Avoid wording such as:

- `Autoware-ready` as a product tag line
- `official Autoware`
- `Autoware <product-name>`
- `certified by Autoware`
- `endorsed by the Autoware Foundation`

If in doubt, prefer compatibility language over product-name language.

## Before Opening An Issue

Please collect the smallest reproducible case you can.
Use the structured bug, feature, sensor-support, benchmark, or Autoware issue
form that best matches the request. Usage questions must include the diagnostic
bundle listed in [SUPPORT.md](SUPPORT.md).

For Autoware-related reports, include:

- exact map bundle path or a minimal reproduction bundle
- `map_projector_info.yaml`
- result of `python3 scripts/verify_autoware_map.py <pointcloud_map_dir>`
- exact command used to stage or view the map
- whether GNSS was enabled

For benchmark-related reports, include:

- bag or dataset name
- exact command line
- param file path
- `metrics.json`
- `ape_corrected_vs_gt.txt` and `ape_raw_vs_gt.txt` when available
- logs needed to understand failures or regressions

For a docs-only onboarding trial, use the
[Independent First-Map Validation](https://github.com/rsasaki0109/lidar_slam_ros2/issues/new?template=first-map-validation.yml)
form. Record the first attempt before private maintainer guidance, including
failures, exact commands, tested revision or image digest, elapsed time and
non-sensitive verifier/diagnosis evidence. The
[public validation ledger](docs/evidence/independent-first-map-validations.md)
defines which reports count toward the v1.0 external-adoption gate.

## Recommended Local Checks

For code changes that touch the default workflow:

```bash
bash scripts/run_default_ci_checks.sh
```

For benchmark/reporting changes:

```bash
bash scripts/run_release_readiness_checks.sh --skip-default-ci --ape-threshold 0.10
```

For Autoware pointcloud-map changes:

```bash
bash scripts/run_autoware_quickstart.sh
```

## Benchmark Result Submissions

If you want to contribute benchmark results, prefer opening the benchmark report
issue template and include:

- ROS 2 distro and Ubuntu version
- sensor topics and frames
- bag duration
- parameter file
- command used to run the benchmark
- output directory
- key metrics from `metrics.json`
- whether the generated map passed Autoware verification

If possible, attach or link:

- `metrics.json`
- `benchmark_summary.md`
- `latest_report.html`
- the exact param YAML used for the run

## Pull Requests

Please keep PRs narrow and explicit.

- explain the operator-visible change first
- include exact commands used for verification
- mention whether the change affects the default workflow
- call out license implications if any dependency choice changes
- link related benchmark or Autoware issues when relevant

## Entry Points

The three official beginner-facing product entrypoints are:

- fixed Docker MID-360 demo (the README Docker command)
- own-bag wrapper: `scripts/run_autoware_map_beginner.sh`
- fixed NTU VIRAL source quickstart:
  `scripts/download_ntu_viral_tnp01.sh` then
  `scripts/run_autoware_quickstart.sh`

All other scripts and launch files are advanced, benchmark, migration, or
research interfaces. See the [Product Contract](docs/product-contract.md) for
the supported boundary.

Useful references:

- Autoware quickstart: [docs/autoware-quickstart.md](docs/autoware-quickstart.md)
- comparison page: [docs/comparison.md](docs/comparison.md)
- benchmarking and release gate: [docs/benchmarking.md](docs/benchmarking.md)
- current release notes: [docs/releases/v0.6.0.md](docs/releases/v0.6.0.md)
- product roadmap: [docs/roadmap/v0.9.md](docs/roadmap/v0.9.md)
- benchmark fixture generator: `scripts/generate_sample_benchmark_metrics.py`
