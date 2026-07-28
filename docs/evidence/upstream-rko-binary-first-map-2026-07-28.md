# Upstream RKO-LIO binary first-map trial — 2026-07-28

## Result

The Humble product golden path completed from a clean ROS container while
using the public `ros-humble-rko-lio` binary instead of the repository's
maintained research fork. The trial built the five required source packages,
processed the full public MID-360 bag, finalized the product manifest and
produced an Autoware-compatible map bundle.

This resolves the RKO-LIO side of the ROS buildfarm product boundary for the
standard MID-360 profile. It does not prove that the lidarslam packages are
available from apt: `ndt_omp_ros2` must still be released first, and Jazzy
must pass the equivalent installed-dependency gate.

## Trial boundary

| Item | Recorded value |
| --- | --- |
| ROS distribution | Humble |
| Base image | `docker.io/library/ros@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16` |
| RKO-LIO source | Ubuntu/ROS apt repository |
| RKO-LIO package | `ros-humble-rko-lio` |
| RKO-LIO version | `0.3.0-1jammy.20260617.082534` |
| RKO-LIO prefix | `/opt/ros/humble` |
| Source RKO-LIO fork | Excluded from the trial workspace |
| Product source revision | `589b0a303d7bc45f914e8da06ae40ba733e409b2` plus the dependency-boundary patch under test |
| Profile | `rko_lio_graph_mid360_preset` |
| Input duration/messages | 277.167 seconds / 58,217 |
| Input metadata SHA-256 | `65d66875f49248e38ff14d80e6e749fb50606f6f80bd4be337160e3752691e9a` |
| Input database SHA-256 | `3bbd390a97e57af47ad6699baa36eb4c5f39f61b35275505ecaf221c126354f5` |

The source workspace contained `lidarslam_msgs`, `scanmatcher`,
`graph_based_slam`, `lidarslam`, and the unreleased `ndt_omp_ros2`
dependency. It deliberately did not contain `Thirdparty/rko_lio`.
`rosdep` and apt therefore supplied the public RKO-LIO runtime.

## Build result

The clean workspace built all five packages in 422.57 seconds. The packaging
boundary keeps the backend's own degeneracy tests enabled and skips only
integration tests that directly include headers unique to the maintained
research fork. This prevents research-only headers from becoming an
undeclared build dependency of the binary product.

## First-map result

| Check | Result |
| --- | --- |
| Process exit code | `0` |
| Manifest schema/status | schema `2`, status `succeeded` |
| Lifecycle | stage `complete`, runner exit code `0`, verification enabled |
| Diagnosis | `success`; RKO started and graph backend initialized |
| Autoware verification | PASS — 8 pass, 0 warn, 0 fail |
| Point-cloud tiles | 360 |
| Corrected/raw poses | 576 / 2,430 |
| Output size | 127 MiB |
| Atomic output | Final directory committed; no `.partial` run directory |

Selected artifact identities:

| Artifact | SHA-256 |
| --- | --- |
| `run_manifest.json` | `23f2890d42e257794952f4eefa47c39cc2bf7bc76264c61391430adffcb2a438` |
| `autoware_map_diagnosis.json` | `818df628446a5b2199d767a82d43861257d3041ad2544440e404536523241c6d` |
| `verify_autoware_map.log` | `f25d5abb99f60686122f08bfd327e3fbe95e34e7412abae785c040b0d40ac0b5` |
| `pointcloud_map/pointcloud_map_metadata.yaml` | `e7267257b247026d2a5b247144394e6d8746214aa9b4b285d5a6d961797fe37a` |
| `traj_corrected.tum` | `43cebdbc3e60a559f9f462165281e7fb086394d1222c81583657767f56726ed9` |

## Product decision

The public RKO-LIO package is the runtime dependency of the binary
golden-path profile. The recursive repository checkout remains supported and
supplies a maintained fork with experimental degeneracy, radar, intensity and
visual-fusion work. Those fork-only profiles are source/research capabilities;
they are not part of the apt product claim.

The remaining binary-release blocker is `ndt_omp_ros2`, which is still absent
from Humble and Jazzy rosdistro. The apt install path remains unsupported
until that package and the four core packages are released and an installed
Humble/Jazzy acceptance gate passes.
