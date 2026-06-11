# Releasing

This repository is currently prepared for a public release at `0.5.0`.

## Release Scope

The intended public release scope is:

- default permissive-license workflow
- `RKO-LIO + graph_based_slam`
- Autoware-compatible map bundle generation (pointcloud_map / projector info /
  lanelet2)
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
4. Confirm `VERSION`, `CHANGELOG.md`, the per-package `CHANGELOG.rst` files,
   `docs/comparison.md`, `docs/releases/v0.5.0.md`, and the core package
   versions match (`test_release_metadata_and_core_package_versions_match`
   enforces most of this).
5. Review README, `docs/autoware-quickstart.md`, `docs/benchmarking.md`,
   `docs/comparison.md`, and `CONTRIBUTING.md` for operator-facing accuracy.

## Automated Publication

Two GitHub Actions workflows matter for release:

- `.github/workflows/main.yml` runs the continuing CI matrix, release-readiness
  fixture checks, and weekly scheduled validation.
- `.github/workflows/release.yml` publishes a prerelease when `v*` tags are
  pushed and uses `docs/releases/v<version>.md` as the release body.

## Tagging

Package versions are currently `0.5.0`.

```bash
git tag v0.5.0
git push <remote> v0.5.0
```

## Binary release to rosdistro (bloom)

The apt/buildfarm release procedure — dependency analysis, the
`ndt_omp_ros2`-first ordering, and the exact `bloom-release` answers — lives
in [`docs/rosdistro-release.md`](docs/rosdistro-release.md). Repository-side
prep (aligned versions, SPDX license tags, per-package `CHANGELOG.rst`) is
part of the normal pre-release checklist above; keep the `CHANGELOG.rst`
files updated with each version bump.

## Suggested Release Notes

Include at least:

- the default supported workflow
- the Autoware map-bundle scope
- the current comparison page and benchmark snapshot
- the benchmark / release-readiness artifacts
- current limitations, especially around lanelets and full production support
