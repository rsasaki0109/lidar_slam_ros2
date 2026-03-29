# Social Copy: v0.2.2

Suggested attachment:

- `lidarslam/images/social_autoware_map_authoring.png`
- `lidarslam/images/social_autoware_map_authoring_demo.mp4`

## Japanese

### Short

`lidarslam_ros2 v0.2.2` を公開しています。  
ROS 2 の non-GPL public path で、Autoware-compatible な
`pointcloud_map/` と `map_projector_info.yaml` を作れる構成です。

- `RKO-LIO + graph_based_slam`
- `NTU VIRAL` current default `APE RMSE 0.952 m`
- `MID360` current default `APE RMSE 3.641 m`
- Leo Drive `bag6` の save-time dynamic filter で saved points を約 `50%` 削減

Quickstart:

```bash
bash scripts/run_autoware_quickstart.sh
```

Release:

- <https://github.com/rsasaki0109/lidarslam_ros2/releases/tag/v0.2.2>

### Medium

`lidarslam_ros2 v0.2.2` を公開しています。  
この repo は汎用 SLAM の最小構成を目指すより、
Autoware-compatible な `pointcloud_map/` を作る workflow を整えることに寄せています。

- non-GPL default path
- `RKO-LIO + graph_based_slam`
- `pointcloud_map/` と `map_projector_info.yaml`
- GNSS georeference
- save-time dynamic-object cleanup
- benchmark / report / release-readiness artifacts

Quickstart:

```bash
bash scripts/run_autoware_quickstart.sh
```

Docs:

- <https://github.com/rsasaki0109/lidarslam_ros2/blob/develop/docs/autoware-map-authoring.md>

## English

### Short

`lidarslam_ros2 v0.2.2` is out.  
The public path is a non-GPL ROS 2 stack for Autoware-compatible
`pointcloud_map/` generation.

- `RKO-LIO + graph_based_slam`
- `NTU VIRAL` current default `APE RMSE 0.952 m`
- `MID360` current default `APE RMSE 3.641 m`
- Leo Drive `bag6` save-time dynamic filtering cuts saved points by about `50%`

Quickstart:

```bash
bash scripts/run_autoware_quickstart.sh
```

Release:

- <https://github.com/rsasaki0109/lidarslam_ros2/releases/tag/v0.2.2>

## Alt Text

Promotional card for `lidarslam_ros2 v0.2.2` highlighting a non-GPL ROS 2 map
authoring workflow, current benchmark numbers on NTU VIRAL and MID360, GNSS map
metadata support, and save-time dynamic-object cleanup.
