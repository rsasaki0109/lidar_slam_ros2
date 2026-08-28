# Registration adapter HILTI exp04 gate receipt

Date: 2026-08-20
Status: frontend precision/performance and deterministic map-artifact
non-regression gates passed; the absolute indoor profile retains one baseline
violation

## Scope

This receipt evaluates the Phase 1 built-in NDT adapter after static production
integration into `scanmatcher`. It compares the adapter with the pre-migration
frontend at the same source revision. The comparison is deliberately the
offline `scan_matcher_offline_runner` path: it exercises the production
`ScanMatcherComponent` and, only when `--save-map` is requested, exports the
world-frame merge of the received `MapArray` submaps. It does not run
`graph_based_slam`, loop closure, colored map authoring, or backend map
refinement.

The input is the locally held HILTI 2022 `exp04` sequence:

| item | value |
| --- | --- |
| rosbag2 | `/media/sasaki/aiueo1/datasets/hilti2022/exp04_ros2` |
| LiDAR topic | `/hesai/pandar` |
| IMU topic | `/alphasense/imu` |
| bag duration | 125.814128037 s |
| LiDAR / IMU messages | 1,258 / 50,198 |
| metadata SHA-256 | `f256bd10ec4a65fec68ab91455108ba73ac3791043f81e05846be93922d21100` |
| control-point GT | `/media/sasaki/aiueo1/datasets/hilti2022/exp04_construction_upper_level_gt.txt` |
| GT SHA-256 | `38cf516e51113254e4ae0207c790f740b19dee08665063e0d8df7bd277040c20` |
| parameters | `configs/hilti2022/lidarslam_competitive_v2.yaml` |
| parameter SHA-256 | `53312d748bc6f6ba8f12fab2a11490c5dc2bcbb5b722f318b48500237aac3e17` |

The baseline is `/tmp/lidarslam-baseline-Vn3TiJ` at commit
`0c08b58f8524ea8ee5288982ca4a1b86450161b2`, with the adapter changes absent.
The current run uses the working tree at the same commit with the built-in
adapter implementation included in the `scanmatcher_component.cpp` translation
unit. Both trees received the same runner-only `--save-map` export patch; the
baseline patch does not contain any registration implementation. The baseline
and current parameter files have the same SHA-256.

## Reproduction

Build the two isolated workspaces with the same Jazzy environment, then run
three sequential runs for each workspace:

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/run_frontend_determinism_check.sh \
  --bag /media/sasaki/aiueo1/datasets/hilti2022/exp04_ros2 \
  --cloud-topic /hesai/pandar --imu-topic /alphasense/imu \
  --params configs/hilti2022/lidarslam_competitive_v2.yaml \
  --runs 3 --ros-domain-base 210 \
  --save-map \
  --output-dir /tmp/hilti-exp04-current-3x \
  --reference-tum \
    /media/sasaki/aiueo1/datasets/hilti2022/exp04_construction_upper_level_gt.txt

bash /tmp/lidarslam-baseline-Vn3TiJ/scripts/run_frontend_determinism_check.sh \
  --bag /media/sasaki/aiueo1/datasets/hilti2022/exp04_ros2 \
  --cloud-topic /hesai/pandar --imu-topic /alphasense/imu \
  --params /tmp/lidarslam-baseline-Vn3TiJ/configs/hilti2022/lidarslam_competitive_v2.yaml \
  --runs 3 --ros-domain-base 220 \
  --save-map \
  --output-dir /tmp/hilti-exp04-baseline-3x \
  --reference-tum \
    /media/sasaki/aiueo1/datasets/hilti2022/exp04_construction_upper_level_gt.txt
```

The APE gate uses the historical dense-trajectory convention, not the
runner's default nearest-neighbour report:

```bash
python3 scripts/ape_from_tum.py --interpolate --max-time-diff 3.0 \
  --ref /media/sasaki/aiueo1/datasets/hilti2022/exp04_construction_upper_level_gt.txt \
  --est <run>/trajectory_frontend.tum --out <run>/ape_historical_interpolate.txt
```

The timed receipt is in `/tmp/hilti-exp04-time.Vqxhvd`. Each run was executed
sequentially with `/usr/bin/time -v` and a private ROS domain. No output from
the data directory was copied into the repository. Those resource numbers are
the existing frontend runtime gate without `--save-map`; map export is an
evidence-only artifact mode and is not used to claim a runtime budget.

## Results

All six runs produced 1,258 poses and 10 frontend submaps. Every baseline run
and every adapter run was byte-identical:

| artifact | baseline runs 1–3 | adapter runs 1–3 |
| --- | --- | --- |
| `trajectory_frontend.tum` MD5 | `59f87bc57455e69b23ce05c07e47b3b2` | `59f87bc57455e69b23ce05c07e47b3b2` |
| `submaps_frontend.csv` MD5 | `16743fe8d14fa35a623e921d9da37930` | `16743fe8d14fa35a623e921d9da37930` |
| `map.pcd` MD5 | `16454b536af73b5486af2460c0920507` | `16454b536af73b5486af2460c0920507` |

The compressed binary `map.pcd` is 1,921,184 bytes and has SHA-256
`bbf742a1d223c0d21b9a85be705282bde71e73cae0e941f20a85049b472550fd` on both
sides. The export applies each submap pose to its local cloud in message order,
matching `ScanMatcherComponent::publishMap()`; it does not re-run registration
or use adapter-specific state.

Using `--interpolate --max-time-diff 3.0`, all six runs scored identically:

| metric | baseline | adapter |
| --- | ---: | ---: |
| paired GT points | 7 / 7 | 7 / 7 |
| APE translation RMSE (SE(3) Umeyama) | 10.439764002361335 m | 10.439764002361335 m |
| APE max time gap | 0.10001611709594727 s | 0.10001611709594727 s |

The per-run `/usr/bin/time -v` measurements were:

| run | baseline wall [s] | adapter wall [s] | baseline peak RSS [KiB] | adapter peak RSS [KiB] |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 48.44 | 49.95 | 112,016 | 112,508 |
| 2 | 48.95 | 48.49 | 111,772 | 111,888 |
| 3 | 49.94 | 48.29 | 112,400 | 112,088 |
| median / max | 48.95 | 48.49 | 112,400 | 112,508 |

The adapter wall median is -0.94% versus baseline; peak RSS is +0.10%.
The corresponding median processing RTF (`wall / bag_duration`) is 0.3891
versus 0.3854 (-0.94%). Both are within the Phase 1 +5% wall/RTF and +10%
RSS budgets. This gate therefore shows no HILTI exp04 frontend precision or
resource regression, but it does not establish an absolute accuracy claim
against the separate README RKO-LIO/GLIM comparison.

## Map-quality gate status

The geometric evaluator was run with the frozen common command
`scripts/run_map_quality_check.sh --setup install/setup.bash --downsample 0.1`
and no threshold profile on `run1/map.pcd` from each side. All six reports were
byte-identical within
each side and across baseline/current: report MD5
`4825eec68fb4442de70cc39d42b0b53a`. The shared report is:

| metric | baseline | adapter |
| --- | ---: | ---: |
| input / evaluated points | 129,726 / 110,283 | 129,726 / 110,283 |
| mean map entropy [nats] | -1.858672907 | -1.858672907 |
| MME valid fraction | 0.788852316 | 0.788852316 |
| plane patches | 1,882 | 1,882 |
| thickness RMS mean [m] | 0.044694491 | 0.044694491 |
| thickness RMS p95 [m] | 0.105756330 | 0.105756330 |
| planar coverage | 0.369340696 | 0.369340696 |
| occupied root voxels | 12,579 | 12,579 |

This closes the frontend map-artifact determinism and adapter non-regression
gate: no metric or map bytes changed. As a separate absolute reference,
`configs/map_quality_profiles/indoor_construction.yaml` was run on both
artifacts. It gives the same single violation on both sides,
`mme_valid_fraction=0.788852316 < 0.90`; its other four thresholds pass. This
is a pre-existing profile failure, not an adapter regression, so this receipt
does not claim an absolute indoor release-profile pass.

The colored profile
`configs/colored_map_quality_profiles/hilti_exp04_report_only.yaml` was not
passed to the geometric evaluator: it contains RGB/image alignment and APE
fields and is not a valid `run_map_quality_check.sh` profile. No colored-map
quality claim is made because this frontend runner emits no images, colors,
or backend `pointcloud_map` bundle. The map-quality evidence here is the real
world-frame PCD plus the existing deterministic geometric evaluator, not a
trajectory/submap proxy.

The raw command outputs remain in `/tmp/hilti-exp04-map-*` and
`/tmp/hilti-exp04-quality-*`; they are intentionally not copied into the
repository.

## Verification

The current tree was built with
`colcon build --packages-select scanmatcher graph_based_slam --symlink-install
--cmake-args -DCMAKE_BUILD_TYPE=Release`. The scanmatcher CTest suite passed
10/10, including the two new `test_offline_map_export` cases. The changed C++
files passed `ament_copyright` and `ament_cpplint`. Final documentation checks
are `python3 -m mkdocs build --strict` (exit 0),
`python3 -m pytest graph_based_slam/test/test_docs_entrypoints.py -q` (6
passed), and `git diff --check` (pass).
