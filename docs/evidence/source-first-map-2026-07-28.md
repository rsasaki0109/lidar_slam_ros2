# Source-workspace first-map trial — 2026-07-28

## Result

A maintainer-operated clean Jazzy trial cloned the repository recursively,
installed dependencies, built all six product packages, ran the installed
product CLI against the tracked MID-360 bag and produced a verified Autoware
map bundle.

The first attempt also found a source-onboarding defect: a fresh ROS base image
has no apt package index, so the documented direct call to `rosdep install`
reported every apt dependency as unavailable. Running `apt-get update` before
`rosdep install` fixed the failure. The canonical source-build instructions now
include that step.

This is direct evidence for the source-workspace path. It is not one of the
three independent-user reports required before v1.0.

## Trial environment

| Item | Recorded value |
| --- | --- |
| Host | `sasaki-pc`, x86_64 |
| Clean base | `ros:jazzy-ros-base` |
| Base digest | `sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f` |
| Ubuntu / ROS | 24.04 / Jazzy |
| Source revision | `ed9dc989cd5c7a438af7ba0a0c3d23473354f02d` |
| Clone mode | HTTPS, `--recursive`, single branch |
| Input | Koide, *Driving SLAM Test with Livox MID360*, read-only extracted rosbag2 |
| Input MD5 identity | `0836c50859bb1af591966b69da166186` |

The source workspace and all installed dependencies were created inside a
disposable container. The input bag was mounted read-only; this trial did not
repeat the already-recorded public download test.

## Failure and correction sequence

### Attempt 1 — fresh apt index blocked rosdep

Following the documented sequence from a fresh base reached:

```text
E: Unable to locate package ros-jazzy-sophus
E: Unable to locate package ros-jazzy-pcl-conversions
E: Unable to locate package ros-jazzy-libg2o
```

The same failure affected Ubuntu packages such as `libpcl-dev`,
`nlohmann-json3-dev` and `python3-scipy`. The rosdep keys were correct; the
image's apt lists were empty. `apt-get update` made all required packages
resolvable.

### Attempt 2 — clean build passed

After updating the apt index, rosdep reported:

```text
#All required rosdeps installed successfully
```

The Release build completed all six packages in 7 minutes 21 seconds. The
trial harness then stopped while sourcing `install/setup.bash` because the
harness itself had enabled Bash `set -u`; the documented user shell does not
enable that mode.

### Attempt 3 — installed CLI and first map passed

The corrected harness repeated the clean install and built all six packages in
7 minutes 25 seconds. The installed CLI reported `lidarslam_ros2 0.6.0`.
`lidarslam-map doctor` inspected the first PointCloud2 record, found the
per-point `timestamp` field and recommended the maintained PointCloud2 + Imu
path. The source-built CLI then ran:

```bash
lidarslam-map run /input/mid360 \
  --profile rko_lio_graph_mid360_preset \
  --output-dir /evidence/mid360_source_map
```

## Acceptance results

| Check | Result |
| --- | --- |
| Recursive clone | PASS — both tracked submodules checked out |
| rosdep | PASS — all required dependencies installed |
| Release build | PASS — 6 packages, 7 min 25 s |
| Installed CLI version | `lidarslam_ros2 0.6.0` |
| Doctor / PointCloud2 inspection | PASS — `timestamp` accepted |
| Map process exit | `0` |
| Manifest | schema `2`, status `succeeded`, profile `rko_lio_graph_mid360_preset` |
| Lifecycle | `complete`, runner exit `0`, verification enabled |
| Diagnosis | `success` |
| Autoware verification | PASS — 8 pass, 0 warn, 0 fail |
| Corrected/raw poses | 577 / 2,430 |
| Point-cloud tiles | 366 |
| Lanelets | 42 relations |
| Output size | 127 MB |
| `.partial` sibling | absent |

Selected final artifact identities:

| Artifact | SHA-256 |
| --- | --- |
| `run_manifest.json` | `63f77b2d5b9c5cc752b6d63213adcfb0e9b7090be5f3c77c12b699d0e083ed59` |
| `autoware_map_diagnosis.json` | `457576f7be8fc9974f8413a4d1532f3cd5d308f48a111b632789b94fbe11590b` |
| `verify_autoware_map.log` | `9a5499493e057ec2c379982d585b1c20bab216ac082c6304ff9aa68a4343a54c` |
| `pointcloud_map/pointcloud_map_metadata.yaml` | `c2a4d1fcc21728909b7c7e7f2cbb8d7a7c561c15584863c8fabc11227ad45305` |
| `traj_corrected.tum` | `08e3e4537ed74f1ff86dfe51152bef8ba9ae2598eedcba3231473dfe7d70c39f` |

## Remaining findings

- A clean source install pulls a large PCL/VTK development dependency set.
  Disk and time expectations should be made more explicit for new users.
- The schema-v2 manifest records product and ROS versions, but
  `software.git_commit` was `null` in the source-built installed CLI run.
- This trial covers one amd64 Jazzy environment and a headless known bag. It
  does not replace Humble source validation, viewer validation, arm64
  evaluation or independent-user first-map reports.
