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
| **`ndt_omp_ros2`** | **not in rosdistro** | **bloom-release it first** (see below) |
| `rko_lio` | PRBonn `0.3.2-1` is registered and built in testing for Humble/Jazzy; main currently has Humble `0.3.0` and Jazzy `0.2.0`; **not yet** a core `package.xml` dependency | run version-pinned official-binary compatibility E2E before declaring it as the flagship runtime dependency |

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
Before the first lidarslam release, in the fork repo:

1. Bump `package.xml` `<version>` from `0.0.0` to `0.1.0`, set the SPDX
   license string (`BSD-2-Clause`), and add/confirm a reachable
   `<maintainer>` (rosdistro review requires one).
2. Tag and bloom-release it into `humble` and `jazzy` (same procedure as
   below, separate release repo `ndt_omp_ros2-release`).
3. Wait for the rosdistro PR to merge; the lidarslam release can be submitted
   as soon as the key exists in the distribution file (it does not need to be
   built yet).

### rko_lio exists, but binary compatibility is not yet proven

The RKO-LIO flagship launch (`rko_lio_slam.launch.py`) uses the `rko_lio`
package at runtime, but no core package declares it in `package.xml`, so the
four core packages can be built and released without it. That is only a
buildfarm fact: the resulting apt installation does not install RKO-LIO.

An official `rko_lio` release line and binary packages are now present for
both supported distributions, with the versions differing between testing
and main as recorded above. The product currently pins the
`rsasaki0109/rko_lio` fork at package version `0.2.0`; that fork contains
substantial offline-completion, recovery, diagnostics, and opt-in research
changes after its common upstream base. It would be incorrect either to claim
an official binary is equivalent without testing that exact version or to
bloom the fork under the already-owned `rko_lio` package name.

The resolution gate is:

1. Run `.github/workflows/official-rko-binary-compatibility.yml`. It builds
   the release-shaped source tree without the RKO-LIO submodule, installs the
   official `ros2-testing-apt-source` configuration and the exact `0.3.2-1`
   testing candidate, builds production packages with source-only fork
   research tests disabled, proves `offline_node` resolves from
   `/opt/ros/<distro>`, and runs the pinned MID-360 E2E independently on
   Humble and Jazzy. The workflow records the full Debian build version and
   executable SHA-256 in its non-geometry evidence artifact.
2. If both pass, add `<exec_depend>rko_lio</exec_depend>` to `lidarslam`,
   retain fork-only features as evaluation scope, and add package-manager
   install/upgrade evidence.
3. If either fails, upstream the minimum product fixes to `PRBonn/rko_lio`
   or give the maintained fork a non-conflicting package identity before
   releasing it. Do not silently substitute untested source code.

Do not advertise `sudo apt install ros-<distro>-lidarslam` as the golden path
until `ndt_omp_ros2` is released, this compatibility decision is complete,
and the installation E2E gate passes.

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
