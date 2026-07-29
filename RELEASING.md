# Releasing

The repository root `VERSION` file is the release version source of truth.
Versions below `0.9.0` publish as GitHub prereleases. v0.9.x is the stable
release-candidate line and publishes as a normal GitHub Release; it is not a
claim that the separate v1.0 readiness gate is complete.

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
   Also inspect the cross-phase product audit; release candidates may remain
   `NOT_READY`, but an invalid contract must stop the release:

```bash
python3 scripts/check_v1_readiness.py --json
```

If the distribution gate is still open on `ndt_omp_ros2`, inspect it without
mutating GitHub or rosdistro:

```bash
python3 scripts/check_ndt_omp_release_readiness.py
```

Only `READY_TO_TAG` authorizes proceeding to the separately documented
maintainer commands; `LOCAL_READY` is the offline CI result, not remote
publication proof.

After Bloom packages enter ROS testing, run
`.github/workflows/package-manager-install-upgrade.yml` for both Humble and
Jazzy. Capture clean-install evidence and, when an older main version exists,
main-to-testing upgrade evidence before the sync. After sync, repeat
clean-install against `main`; see `docs/rosdistro-release.md` for exact
dispatch inputs.

4. Set `VERSION="$(tr -d '\n' < VERSION)"` and confirm `CHANGELOG.md`, the
   per-package `CHANGELOG.rst` files, `docs/comparison.md`,
   `docs/releases/v${VERSION}.md`, `CITATION.cff`, and the core package versions
   match (`test_release_metadata_and_core_package_versions_match` enforces most
   of this).
5. Review README, `docs/autoware-quickstart.md`, `docs/benchmarking.md`,
   `docs/comparison.md`, and `CONTRIBUTING.md` for operator-facing accuracy.
6. Confirm the tagged checkout can build both `ROS_DISTRO=humble` and
   `ROS_DISTRO=jazzy` Docker targets. The release workflow will refuse to
   create the GitHub Release unless both published digests pass the installed
   `lidarslam-map --version` smoke test.

For a v1.0 release, require the complete product gate. This includes the
tracked independent-user ledger rather than checking it as an isolated proxy:

```bash
python3 scripts/check_v1_readiness.py --require-complete
```

Prerelease candidates validate the ledger without pretending that 0/3 is
complete:

```bash
python3 scripts/check_external_first_map_readiness.py --json
```

Build the exact curated bundle before creating a new tag:

```bash
VERSION="$(tr -d '\n' < VERSION)"
python3 scripts/build_release_bundle.py \
  --tag "v${VERSION}" \
  --candidate \
  --output "/tmp/lidarslam_ros2_v${VERSION}_release_bundle.tar.gz"
```

The command requires a clean worktree, refuses an existing tag that names a
different commit, refuses to overwrite its output, and embeds a
schema-validated SHA-256 inventory. Repeating it from the same commit produces
the same bundle bytes.

## Automated Publication

Two GitHub Actions workflows matter for release:

- `.github/workflows/main.yml` runs the continuing CI matrix, release-readiness
  fixture checks, and weekly scheduled validation.
- `.github/workflows/release.yml` validates the tag, publishes and attests
  untagged Humble and Jazzy candidate digests, smoke-tests both by registry
  digest, verifies their source revision, SBOM, BuildKit provenance, and
  GitHub attestation, then promotes the pair to
  `v<version>-humble` and `v<version>-jazzy`. Promotion preflights both
  digests before creating either tag. A matching existing tag is reused;
  a different digest fails closed and is never overwritten. The workflow then
  publishes the GitHub Release using
  `docs/releases/v<version>.md` as the release body. The release assets include
  the source bundle plus one `release-image-<distro>.json` installation
  evidence file and one digest-pinned `rollback-plan-<distro>.json` per image,
  plus `release-promotion.json`.
  Retain the prior release's JSON assets as the last-known-good recovery
  record; do not move a tag to perform rollback.

After creating the GitHub Release, the workflow runs an independent,
read-only publication audit:

```bash
python3 scripts/check_published_release.py --require-published
```

It requires a non-draft, non-prerelease v0.9 release, resolves the tag commit,
downloads exactly the six release assets, validates every JSON schema and
cross-file identity, and verifies the embedded bundle manifest against every
archived file size and SHA-256. HTTP/API failures are `BLOCKED`; only explicit
tag/release 404 responses mean `NOT_PUBLISHED`. Retain the uploaded
`published-release-audit.json` Actions artifact with the release evidence.

The curated bundle contains `release-bundle-manifest-v1.json`, including the
exact tag commit and every bundled file hash. The image build emits an OCI
SBOM and maximum-mode BuildKit provenance. GitHub artifact attestations cover
each image digest and the source release bundle. Verify an image after
publication:

```bash
lidarslam-map rollback-plan release-image-jazzy.json
```

## Tagging

```bash
VERSION="$(tr -d '\n' < VERSION)"
test -f "docs/releases/v${VERSION}.md"
git tag "v${VERSION}"
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
