# Releasing

The repository root `VERSION` file is the release version source of truth.
Tagged `0.x` releases remain prereleases until the v0.9 roadmap's stable
promotion gate is completed.

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
4. Run `python3 scripts/check_version_alignment.py`. `VERSION` is the only
   version source of truth; the checker requires `CHANGELOG.md`, the
   per-package `CHANGELOG.rst` files and `package.xml` versions,
   `docs/comparison.md`, current release links,
   `docs/releases/v${VERSION}.md`, and `CITATION.cff` to match it.
5. Review README, `docs/autoware-quickstart.md`, `docs/benchmarking.md`,
   `docs/comparison.md`, and `CONTRIBUTING.md` for operator-facing accuracy.
6. Confirm the tagged checkout can build both `ROS_DISTRO=humble` and
   `ROS_DISTRO=jazzy` Docker targets. The release workflow will refuse to
   create the GitHub Release unless both published digests pass the installed
   `lidarslam-map --version` smoke test.

## Automated Publication

Two GitHub Actions workflows matter for release:

- `.github/workflows/main.yml` runs the continuing CI matrix, release-readiness
  fixture checks, and weekly scheduled validation.
- `.github/workflows/release.yml` validates the tag, publishes and attests
  `v<version>-humble` and `v<version>-jazzy` GHCR images, smoke-tests both by
  registry digest, then publishes the prerelease using
  `docs/releases/v<version>.md` as the release body. The release assets include
  the source bundle plus one `release-image-<distro>.json` installation
  evidence file per image.

The image build emits an OCI SBOM and maximum-mode BuildKit provenance.
GitHub artifact attestations cover each image digest and the source release
bundle. Verify an image after publication:

```bash
gh attestation verify \
  "oci://ghcr.io/rsasaki0109/lidar_slam_ros2:v${VERSION}-jazzy" \
  -R rsasaki0109/lidar_slam_ros2
```

## Tagging

```bash
VERSION="$(tr -d '\n' < VERSION)"
test -f "docs/releases/v${VERSION}.md"
git tag "v${VERSION}"
python3 scripts/check_version_alignment.py --tag "v${VERSION}"
git push <remote> "v${VERSION}"
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
