# rosdistro Binary Release (bloom) — Runbook

Goal: `sudo apt install ros-humble-lidarslam` (and the Jazzy equivalent)
installs the four core packages from the ROS buildfarm.

This page records the dependency analysis and the exact release procedure.
The repository-side prep (versions, SPDX license tags, per-package
`CHANGELOG.rst`) landed with v0.5.0; what remains is the bloom/rosdistro
procedure itself, which requires the maintainer's GitHub account.

## Released package set

| Package | Version | Notes |
|---|---|---|
| `lidarslam_msgs` | 0.5.0 | messages only |
| `scanmatcher` | 0.5.0 | NDT frontend (FastGICP / SmallGICP optional, off on the farm) |
| `graph_based_slam` | 0.5.0 | backend + `/map_save` Autoware bundle |
| `lidarslam` | 0.5.0 | launch + param presets |

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
| `rko_lio` | not in rosdistro, and **not** a `package.xml` dependency | not a blocker; see below |

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

### rko_lio is not a blocker

The RKO-LIO flagship launch (`rko_lio_slam.launch.py`) uses the `rko_lio`
package at runtime, but no core package declares it in `package.xml`, so the
binary release neither pulls it in nor needs it to build. Consequences for
apt users, to keep documented honestly:

- `lidarslam.launch.py` (scanmatcher frontend) works out of the box.
- The RKO-LIO frontend needs a source workspace (or the ghcr Docker image)
  until `rko_lio` itself is released by its upstream — that decision is not
  ours to make.

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
git tag v0.5.0 && git push origin v0.5.0

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
| release tag | `v:{version}` (upstream tags are v-prefixed: `v0.5.0`; matches the `v*` trigger in `release.yml`) |
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
