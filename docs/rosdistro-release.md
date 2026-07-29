# rosdistro Binary Release (bloom) — Runbook

Goal: `sudo apt install ros-humble-lidarslam` (and the Jazzy equivalent)
installs the four core packages from the ROS buildfarm.

This page records the dependency analysis and the exact release procedure.
The repository-side prep (versions, SPDX license tags, per-package
`CHANGELOG.rst`) landed with v0.5.0 and is maintained through v0.7.0; what remains is the bloom/rosdistro
procedure itself, which requires the maintainer's GitHub account.

## Released package set

| Package | Version | Notes |
|---|---|---|
| `lidarslam_msgs` | 0.7.0 | messages only |
| `scanmatcher` | 0.7.0 | NDT frontend (FastGICP / SmallGICP optional, off on the farm) |
| `graph_based_slam` | 0.7.0 | backend + `/map_save` Autoware bundle |
| `lidarslam` | 0.7.0 | launch + param presets |

These are the only `package.xml` files in the repository outside
`Thirdparty/`, so bloom's package discovery picks up exactly this set.
`Thirdparty/` consists of git submodules, which `git archive` (and therefore
bloom's upstream import) excludes — intended.

## Dependency readiness

| Dependency | rosdep key state | Action |
|---|---|---|
| rclcpp, rclcpp_components, tf2\*, \*_msgs, pcl_conversions, std_srvs | released ROS packages | none |
| `libg2o` | released (`ros-<distro>-libg2o`; verified 2026-06-11: `rosdep resolve libg2o` → `ros-humble-libg2o` on jammy, `ros-jazzy-libg2o` on noble) | none |
| `libpcl-all-dev` | standard rosdep key (system PCL) | none |
| **`ndt_omp_ros2`** | **not in rosdistro**; upstream fork `0.1.0` metadata and Humble/Jazzy Bloom/deb gates are ready at `8b77fa5` | tag `0.1.0`, bloom-release, and submit it first (see below) |
| `rko_lio` | PRBonn `0.3.2-1` is registered and built in testing for Humble/Jazzy; main currently has Humble `0.3.0` and Jazzy `0.2.0`; declared as `rko_lio >= 0.3.2` after the official-binary gate passed | wait for `0.3.2` to sync to main before the normal apt path |

This table was rechecked directly against the
[Humble distribution](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml)
and
[Jazzy distribution](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml)
in `ros/rosdistro` on 2026-07-29. Neither distribution contains
`ndt_omp_ros2`; both register `rko_lio` `0.3.2-1` from
[`PRBonn/rko_lio`](https://github.com/PRBonn/rko_lio) and
`ros2-gbp/rko_lio-release`.

The amd64 apt indexes were also checked on 2026-07-29. The ROS testing
repository contains `0.3.2-1` builds for both distributions. The main
repository, which normal users install from, still contains Humble `0.3.0`
and Jazzy `0.2.0`; do not describe `0.3.2` as synced to main until those
indexes change.

### ndt_omp_ros2 must be released first

`scanmatcher` and `graph_based_slam` declare `<depend>ndt_omp_ros2</depend>`.
The dependency is consumed as the submodule
`https://github.com/rsasaki0109/ndt_omp_ros2` (branch `humble`) — a fork
maintained by the same owner, BSD licensed, with a unique name in rosdistro.
Before the first lidarslam release, use the following maintainer sequence.
The source repository currently has no `0.1.0` tag and
`rsasaki0109/ndt_omp_ros2-release` does not exist (checked 2026-07-29), so
all first-release steps below are required.

Run the read-only preflight immediately before doing any publication work:

```bash
python3 scripts/check_ndt_omp_release_readiness.py
python3 scripts/check_ndt_omp_release_readiness.py \
  --require-ready-to-tag \
  --output-json /tmp/ndt-omp-release-preflight.json
```

The first command describes the current state; the strict command exits 1
unless the exact reviewed candidate is `READY_TO_TAG`. It validates the
parent gitlink, submodule HEAD and cleanliness, package metadata, changelog,
CMake install/export contract, and Bloom CI assets. Its remote inspection
then verifies `origin/humble`, source tag, release-repository existence, and
both rosdistro keys. A GitHub 404 means an initial artifact is absent; any
other HTTP or network error is `BLOCKED`, never mistaken for absence.

CI runs `--offline`, whose successful state is only `LOCAL_READY`. After
publication, use `--require-released`; it passes only when the tag, release
repository, and Humble and Jazzy rosdistro entries all exist. `IN_PROGRESS`
means publication is partial and the report lists the missing next steps.
The JSON contract is
[`ndt-omp-release-readiness-v1.schema.json`](schemas/ndt-omp-release-readiness-v1.schema.json).
The checker is read-only; it never creates a tag, repository, or PR.

1. Confirm fork commit `8b77fa5` is green. Its package metadata is `0.1.0`
   with `BSD-2-Clause`, a reachable fork maintainer, `CHANGELOG.rst`, exported
   `ndt_omp` CMake target, and installed-consumer tests. Public
   [CI run 30369808717](https://github.com/rsasaki0109/ndt_omp_ros2/actions/runs/30369808717)
   passed the Humble and Jazzy build/test plus Bloom-generated Debian package
   gate.
2. Create and push source tag `0.1.0`, then create the separate
   `ndt_omp_ros2-release` repository.
3. Run one new Bloom track for each ROS distribution. Since the repository is
   not yet in either distribution file, pass the release repository URL
   explicitly.
4. Wait for the rosdistro PR to merge; the lidarslam release can be submitted
   as soon as the key exists in the distribution file (it does not need to be
   built yet).

#### NDT 0.1.0 exact commands

These commands deliberately fail closed if the remote branch, package version,
or tag state differs from the reviewed release candidate:

```bash
git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
cd ndt_omp_ros2
git fetch --prune origin

RELEASE_COMMIT=8b77fa5a6cdcad45bf35918361c892b6d94a287e
test "$(git rev-parse origin/humble)" = "$RELEASE_COMMIT"
test -z "$(git status --porcelain)"
test "$(python3 -c \
  'import xml.etree.ElementTree as E; print(E.parse("package.xml").findtext("version"))')" \
  = "0.1.0"
test -z "$(git ls-remote --tags origin refs/tags/0.1.0)"

git tag -a 0.1.0 "$RELEASE_COMMIT" -m "ndt_omp_ros2 0.1.0"
git push origin refs/tags/0.1.0
```

The tag is intentionally **not** `v0.1.0`: the Bloom track below uses the
default `:{version}` release-tag template. Do not tag the parent
`lidarslam_ros2` repository.

Create the public release repository once, without generated files, then give
Bloom the `master` branch it expects:

```bash
gh auth status
gh repo create rsasaki0109/ndt_omp_ros2-release \
  --public \
  --description "Bloom release repository for ndt_omp_ros2"
gh auth setup-git

RELEASE_REPO_DIR="$(mktemp -d)/ndt_omp_ros2-release"
git clone https://github.com/rsasaki0109/ndt_omp_ros2-release.git \
  "$RELEASE_REPO_DIR"
git -C "$RELEASE_REPO_DIR" commit --allow-empty -m "Initialize release repository"
git -C "$RELEASE_REPO_DIR" branch -M master
git -C "$RELEASE_REPO_DIR" push -u origin master
```

Install Bloom from the target ROS/Ubuntu package repository, then release
Humble first and Jazzy second:

```bash
sudo apt update
sudo apt install python3-bloom python3-catkin-pkg python3-rosdep

RELEASE_REPO=https://github.com/rsasaki0109/ndt_omp_ros2-release.git
bloom-release ndt_omp_ros2 \
  --rosdistro humble \
  --track humble \
  --new-track \
  --override-release-repository-url "$RELEASE_REPO"

bloom-release ndt_omp_ros2 \
  --rosdistro jazzy \
  --track jazzy \
  --new-track \
  --override-release-repository-url "$RELEASE_REPO"
```

The command options have distinct jobs:

| Option | Why it is present |
|---|---|
| `ndt_omp_ros2` | rosdistro **repository** key; it also matches the single package name here |
| `--rosdistro <distro>` | selects the Humble or Jazzy distribution file and Debian target |
| `--track <distro>` | selects the independently maintained Bloom track |
| `--new-track` | creates that track on its first release; omit it on later releases |
| `--override-release-repository-url` | supplies the release repository before a rosdistro entry exists |

Use these first-run track answers; press Enter for the values shown as
defaults:

| Prompt | Answer |
|---|---|
| repository name | `ndt_omp_ros2` |
| upstream repository | `https://github.com/rsasaki0109/ndt_omp_ros2.git` |
| upstream type | `git` |
| version | `:{auto}` |
| release tag | `:{version}` |
| upstream devel branch | `humble` |
| ROS distro | `humble` or `jazzy`, matching the command |
| patches directory | empty / `None` |
| release repository push URL | empty; the HTTPS origin is already authenticated by `gh auth setup-git` |

Bloom pushes generated branches and tags to the release repository and then
offers the corresponding `ros/rosdistro` PR. Verify that each generated PR
only adds `ndt_omp_ros2`, uses the release URL above, and reports version
`0.1.0-1`. Do not use `--non-interactive` for this first release. If either
command fails, fix the cause and rerun it; do not accept Bloom's offered
force-push without first inspecting the release repository.

### rko_lio binary compatibility is proven for the golden path

The RKO-LIO flagship launch (`rko_lio_slam.launch.py`) uses the `rko_lio`
package at runtime. `lidarslam/package.xml` now declares
`<exec_depend version_gte="0.3.2">rko_lio</exec_depend>`, so an installed
flagship path cannot silently resolve to the older main-repository builds.

An official `rko_lio` release line and binary packages are now present for
both supported distributions, with the versions differing between testing
and main as recorded above. The product currently pins the
`rsasaki0109/rko_lio` fork at package version `0.2.0`; that fork contains
substantial offline-completion, recovery, diagnostics, and opt-in research
changes after its common upstream base. The successful gate proves the
official binary for the maintained MID-360 golden path; it does not claim
equivalence for those fork-only research features, and the fork must not be
bloomed under the already-owned `rko_lio` package name.

The resolution gate completed in
[public run 30412938777](https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/30412938777):

1. Run `.github/workflows/official-rko-binary-compatibility.yml`. It builds
   the release-shaped source tree without the RKO-LIO submodule, installs the
   official `ros2-testing-apt-source` configuration and the exact `0.3.2-1`
   testing candidate, builds production packages with source-only fork
   research tests disabled, proves `offline_node` resolves from
   `/opt/ros/<distro>`, and runs the pinned MID-360 E2E independently on
   Humble and Jazzy. The workflow records the full Debian build version and
   executable SHA-256 in its non-geometry evidence artifact.
2. Both passed all 18 E2E checks. The permanent non-geometry record is
   [`official-rko-binary-compatibility-2026-07-29.md`](evidence/official-rko-binary-compatibility-2026-07-29.md).
   `lidarslam` therefore requires `rko_lio >= 0.3.2`; fork-only features
   remain evaluation scope.
3. If a scheduled rerun regresses, upstream the minimum product fixes to
   `PRBonn/rko_lio` or give the maintained fork a non-conflicting package
   identity before releasing it. Do not silently substitute untested source
   code.

Do not advertise `sudo apt install ros-<distro>-lidarslam` as the golden path
until `ndt_omp_ros2` is released, `rko_lio 0.3.2` is synced to main, and the
package-manager installation E2E gate passes.

The compatibility workflow runs when its product boundary changes on
`develop`, is scheduled weekly to detect repository drift, and can be
dispatched manually. It intentionally uses the maintained MID-360 preset:
`lidarslam/param/rko_lio_mid360.yaml` only uses parameters implemented by the
official `0.3.2` line. Research presets containing fork-only recovery,
diagnostic, radar, visual, or gravity parameters are outside this gate. The
NTU-VIRAL preset is also outside the first adoption gate because it currently
contains fork-only timestamp-offset and ICP keypoint controls.

## Distro targets

Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04). Both are already exercised by
the CI matrix on every push, including the `ndt_omp_ros2` submodule build, so
no new build risk is expected on the farm. All ament tests run on synthetic
fixtures (no datasets, no network, no GPU), which matches buildfarm
constraints.

## Release procedure (maintainer)

One-time setup:

```bash
pip3 install -U bloom catkin_pkg
# a GitHub token with repo scope, configured for bloom:
#   https://bloom.readthedocs.io/ -> "Automated PR opening"
```

Per release:

```bash
# 0. ensure develop is green and this repo's release prep is merged
# 1. tag the release commit (release.yml publishes the GitHub release)
VERSION="$(tr -d '\n' < VERSION)"
git tag "v${VERSION}" && git push origin "v${VERSION}"

# 2. create an empty release repo once (first release only):
#    https://github.com/rsasaki0109/lidar_slam_ros2-release

# 3. release into each distro (first run is interactive; answers below)
bloom-release --rosdistro humble lidarslam_ros2
bloom-release --rosdistro jazzy lidarslam_ros2
```

First-run interactive answers:

| Prompt | Answer |
|---|---|
| repository name | `lidarslam_ros2` |
| upstream repository | `https://github.com/rsasaki0109/lidar_slam_ros2.git` |
| upstream type | `git` |
| upstream branch | `develop` |
| version | `:{auto}` (reads package.xml) |
| release tag | `v:{version}` (upstream tags are v-prefixed; matches the `v*` trigger in `release.yml`) |
| release repository | `https://github.com/rsasaki0109/lidar_slam_ros2-release.git` |

bloom ends by offering to open the `ros/rosdistro` PR with the configured
GitHub token — submit it, answer review comments (license string, maintainer
email, description quality are the usual ones), and wait for the buildfarm.
Binaries appear in the ROS testing repo first, then sync to main with the
next distro sync (typically 2–6 weeks).

## After the first sync

- README: add the `sudo apt install ros-humble-lidarslam` install path next
  to the source build.
- Subsequent releases: update each package's `CHANGELOG.rst`, bump all four
  `package.xml` versions together with `VERSION` (see `RELEASING.md`), tag,
  and re-run `bloom-release` (non-interactive after the first time).

## Known caveats

- `lidarslam/images/` demo media (~25 MB of GIF/mp4/png) ships inside the
  source package because the README references those paths. Acceptable for
  now; moving demo media out of the package directory is a possible later
  cleanup to slim the source deb.
- The repository-level `CHANGELOG.md` is the human-facing project changelog;
  the per-package `CHANGELOG.rst` files are what bloom turns into deb
  changelogs (REP-132). Both need updating per release.
- `git archive` drops submodules, so the upstream tarball bloom imports
  contains no `Thirdparty/` sources. Do not move release-relevant code into
  `Thirdparty/`.
