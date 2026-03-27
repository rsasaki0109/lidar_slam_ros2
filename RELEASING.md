# Releasing

This repository is currently prepared for a public beta release at `0.2.2`.

## Release Scope

The intended public release scope is:

- default permissive-license workflow
- `RKO-LIO + graph_based_slam`
- Autoware-compatible pointcloud map generation
- benchmark summary / HTML report / release-readiness gate

## Pre-Release Checklist

1. Clean the worktree so generated outputs do not leak into the release commit.
2. Run local checks:

```bash
bash scripts/run_default_ci_checks.sh
bash scripts/run_release_readiness_checks.sh --skip-default-ci --ape-threshold 0.10
bash scripts/run_autoware_quickstart.sh
```

3. Push the branch and verify GitHub Actions are green.
4. Confirm `VERSION`, `CHANGELOG.md`, `docs/comparison.md`,
   `docs/releases/v0.2.2.md`, and the core package versions match.
5. Review README, `docs/autoware-quickstart.md`, `docs/benchmarking.md`,
   `docs/comparison.md`, and `CONTRIBUTING.md` for operator-facing accuracy.

## Automated Publication

Two GitHub Actions workflows now matter for release:

- `.github/workflows/main.yml` runs the continuing CI matrix, release-readiness
  fixture checks, and weekly scheduled validation.
- `.github/workflows/release.yml` publishes a prerelease when `v*` tags are
  pushed and uses `docs/releases/v<version>.md` as the release body.

## Suggested Tagging

Package versions are currently `0.2.2`.

Suggested Git tag:

```bash
git tag v0.2.2
git push <remote> v0.2.2
```

If you want to market this as the `v2` public beta, keep the GitHub release
title explicit, for example:

- `lidarslam_ros2 v2 beta (package version 0.2.2)`

## Suggested Release Notes

Include at least:

- the default supported workflow
- the Autoware pointcloud-map scope
- the current comparison page and benchmark snapshot
- the benchmark / release-readiness artifacts
- current limitations, especially around lanelets and full production support
