# AWSIM × lidarslam 自動運転チュートリアル

AWSIM シミュレータ上で lidarslam を使って 3D マップを作成し、Autoware で自動運転を実現するまでの手順。

## 概要

```
AWSIM (西新宿シミュレーション)
  → rosbag2 記録 (LiDAR + IMU)
    → lidarslam (SLAM → pointcloud_map.pcd)
      → simple_lanelet2_generator (軌跡 → lanelet2_map.osm)
        → Autoware (自作マップで自動運転)
```

## 前提条件

| 項目 | 要件 |
|------|------|
| OS | Ubuntu 22.04 / 24.04 |
| GPU | NVIDIA RTX 2080 以上 |
| VRAM | 16GB 推奨 |
| RAM | 64GB 推奨 |
| Docker | NVIDIA Container Toolkit 付き |
| ROS 2 | Jazzy (ホスト) / Humble (Docker内) |

## セットアップ

### 1. Autoware Docker イメージ

```bash
docker pull ghcr.io/autowarefoundation/autoware:universe-cuda
```

### 2. Autoware ML アーティファクト

```bash
bash scripts/download_autoware_artifacts.sh
```

### 3. AWSIM ダウンロード

```bash
mkdir -p /path/to/awsim
cd /path/to/awsim
wget -O awsim_labs_v1.6.1.zip \
  "https://github.com/autowarefoundation/AWSIM-Labs/releases/download/v1.6.1/awsim_labs_v1.6.1.zip"
unzip awsim_labs_v1.6.1.zip
chmod +x awsim_labs_v1.6.1/awsim_labs.x86_64
```

付属スクリプトのデフォルトは `/workspace/ai_coding_ws/awsim` です。別の場所に展開した場合は環境変数で指定します:

```bash
export AWSIM_DIR=/path/to/awsim
export AWSIM_MAP_PATH=/path/to/sample_map/nishishinjuku_autoware_map
export AUTOWARE_IMAGE=ghcr.io/autowarefoundation/autoware:universe-cuda
```

### 4. CycloneDDS 設定

```bash
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks><WhcHigh>500kB</WhcHigh></Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
EOF
```

DDS カーネルパラメータ（要 sudo）:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo ip link set lo multicast on
```

### 5. lidarslam ビルド

```bash
cd /path/to/ros2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Step 1: AWSIM から rosbag2 を記録

### AWSIM を起動

AWSIM はホストの ROS 2 Jazzy と衝突するため、Docker 内で起動する必要があります。

```bash
xhost +local:docker

docker run -d --name awsim_demo \
  --net=host --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="${DISPLAY}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/cyclonedds.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${HOME}/cyclonedds.xml:/cyclonedds.xml:ro" \
  -v "/path/to/awsim/awsim_labs_v1.6.1:/awsim:rw" \
  ghcr.io/autowarefoundation/autoware:universe-cuda \
  bash -c "export LD_LIBRARY_PATH=/awsim/awsim_labs_Data/Plugins:\${LD_LIBRARY_PATH} && \
    /awsim/awsim_labs.x86_64 -force-vulkan --config /awsim/awsim_labs_Data/StreamingAssets/config.json"
```

### Autoware を起動（データ記録用）

```bash
docker run -d --name autoware_recorder \
  --net=host --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY="${DISPLAY}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=/cyclonedds.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${HOME}/cyclonedds.xml:/cyclonedds.xml:ro" \
  -v "/path/to/sample_map:/autoware_map:ro" \
  -v "${HOME}/autoware_data:/root/autoware_data:rw" \
  ghcr.io/autowarefoundation/autoware:universe-cuda \
  bash -c "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && \
    ros2 launch autoware_launch e2e_simulator.launch.xml \
      vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit \
      map_path:=/autoware_map"
```

### rosbag2 を記録

```bash
docker exec -d autoware_recorder bash -c "source /opt/ros/humble/setup.bash && \
  ros2 bag record \
    /sensing/lidar/top/pointcloud_raw_ex \
    /sensing/imu/tamagawa/imu_raw \
    /sensing/gnss/pose_with_covariance \
    /clock /tf /tf_static \
    -o /tmp/awsim_bag --max-bag-duration 120"
```

120秒後に停止してコンテナからコピー:

```bash
docker exec autoware_recorder bash -c "pkill -f 'ros2 bag record'"
docker cp autoware_recorder:/tmp/awsim_bag ./awsim_rosbag2/
```

## Step 2: lidarslam でマップ生成

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lidarslam rko_lio_slam.launch.py \
  lidar_topic:=/sensing/lidar/top/pointcloud_raw_ex \
  imu_topic:=/sensing/imu/tamagawa/imu_raw \
  bag_path:=./awsim_rosbag2/awsim_bag \
  use_sim_time:=true \
  deskew:=false
```

> **注意**: AWSIM の LiDAR にはポイントタイムスタンプがないため `deskew:=false` が必須。

SLAM 完了後、マップを保存:

```bash
ros2 service call /map_save std_srvs/srv/Empty
```

出力:
- `pointcloud_map/` — グリッド分割 PCD ファイル
- `pose_graph.g2o` — ポーズグラフ（軌跡情報を含む）

## Step 3: PCD を AWSIM 座標系に変換

SLAM の出力はローカル座標系なので、AWSIM の map 座標系（MGRS 54SUE）に変換する必要があります。

ego の開始位置と yaw（`config.json` から取得）を使って回転+平行移動を適用します。

詳細は `scripts/simple_lanelet2_generator.py` および本リポジトリの `output/awsim_shinjuku_slam/` を参照。

## Step 4: 軌跡から lanelet2 を生成

```bash
python3 scripts/simple_lanelet2_generator.py \
  --input output/awsim_shinjuku_slam/traj_map.tum \
  --output output/awsim_shinjuku_slam/autoware_map/lanelet2_map.osm \
  --lane-width 7.0 \
  --origin-lat 35.686 --origin-lon 139.689 \
  --resolution 2.0
```

### Autoware 互換のポイント

- **lanelet を複数に分割**: Autoware のルートプランナーは lanelet 間の接続グラフを使用。~50m 間隔で分割し、境界ノードを共有。
- **projector_type**: `local`（lat=local_y, lon=local_x として直接マッピング）
- **車両方向**: ゴールの orientation をレーン方向に合わせないと `Goal's footprint exceeds lane` エラー。

## Step 5: Autoware で自動運転

### マップディレクトリ構成

```
autoware_map/
├── pointcloud_map.pcd      # マージ済み PCD (binary format)
├── lanelet2_map.osm         # 生成した lanelet2
└── map_projector_info.yaml  # projector_type: local
```

### NDT 閾値の調整

SLAM 品質によっては NDT スコアが閾値を下回る場合があります:

```yaml
# ndt_scan_matcher.param.yaml
score_estimation:
  converged_param_nearest_voxel_transformation_likelihood: 1.5  # default: 2.3
```

`scripts/run_awsim_selfmade_map_demo.sh` は `NDT_OVERRIDE` を Autoware コンテナへ mount します。デフォルト以外の場所に置く場合は指定してください:

```bash
export NDT_OVERRIDE=/path/to/ndt_scan_matcher.param.yaml
```

### ワンコマンドデモ

```bash
export AWSIM_DIR=/path/to/awsim
export MY_MAP=/path/to/autoware_map
export NDT_OVERRIDE=/path/to/ndt_scan_matcher.param.yaml
bash scripts/run_awsim_selfmade_map_demo.sh        # デモ実行
bash scripts/run_awsim_selfmade_map_demo.sh true    # 動画付き
```

## トラブルシューティング

| 症状 | 原因 | 対策 |
|------|------|------|
| AWSIM segfault | ホスト Jazzy と衝突 | Docker 内で起動 |
| `Point cloud needs timestamps` | AWSIM LiDAR にタイムスタンプなし | `deskew:=false` |
| `Data corruption` in PCD | binary_compressed の不正マージ | LZF 解凍して binary で再書き込み |
| NDT `Score is below threshold` | マップ品質/解像度 | 閾値を 1.5 に下げる |
| `vector map is not ready` | MGRS 投影でロード失敗 | `projector_type: local` を使用 |
| `Goal's footprint exceeds lane` | 車両方向がレーンと不一致 | ゴールの orientation をレーン方向に合わせる |
| `Failed to plan route` | lanelet 未接続 or 1つだけ | 複数 lanelet に分割し境界ノード共有 |

## 関連スクリプト

| スクリプト | 用途 |
|-----------|------|
| `scripts/simple_lanelet2_generator.py` | TUM 軌跡 → lanelet2 OSM |
| `scripts/run_awsim_autoware_demo.sh` | サンプルマップ AWSIM デモ |
| `scripts/run_awsim_selfmade_map_demo.sh` | 自作マップ自動運転デモ |
| `scripts/download_autoware_artifacts.sh` | Autoware ML モデルダウンロード |
| `scripts/test_awsim_setup.sh` | セットアップ確認 |
