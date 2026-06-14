# 3DGS closed-loop sensor-sim — Phase 2 ノード骨格 (2026-06-15)

3DGS sim2real トラックの Phase 2。LiDAR-primed 3DGS シーンを **closed-loop に置き**、
実画像で学習した知覚スタック（例: Autoware perception）を合成ではなく photoreal な
render に対して走らせるための ROS 2 ノード骨格。Phase 0
（`docs/research/3dgs-sim2real-gap-phase0.md`）で走行スケールが ±1 m 横ずれに
耐えると分かったのを受けて着手。

## 成果物

- `tools/gaussian_splatting/gaussian_renderer.py` — `GaussianRenderer`:
  gaussians を GPU に**常駐**させ `render(viewmat, K, w, h) -> uint8 RGB`。
  pure な pose 数学 `pose_to_viewmat` / `transform_from_pos_quat` 同梱（CPU テスト）。
- `tools/gaussian_splatting/sensor_sim_node.py` — rclpy ノード。ego pose
  (`Odometry` or `PoseStamped`) を購読 → render → `sensor_msgs/Image`(rgb8) +
  `CameraInfo` を配信。`extrinsic`(base←camera_optical) / `align`
  (model_world←pose_world) / `scale` をパラメータ化。
- `tools/gaussian_splatting/pose_player.py` — localiser のスタンドイン。
  `transforms.json` のカメラ姿勢(=inv viewmat)を `Odometry` で配信し、模型自身の
  frame でループを閉じる。後で Autoware `kinematic_state` に差し替える部品。
- runner `scripts/run_sensor_sim_node.sh`、CPU テスト
  `graph_based_slam/test/test_gaussian_splatting_renderer.py`（12 ケース、ament_flake8
  clean。pose_player↔node の往復が元 viewmat にバイト整合することも検証）。

## 検証

- **リアルタイム性（本命の成立条件）**: 4070 Ti SUPER / gsplat 1.5.3、
  毎フレーム GPU 再アップロードする `render_frames` 経由でも
  isuzu(496k gaussians, 720幅) ~112 FPS、koide(660k, 2448幅フル) 16.8 FPS /
  1224幅 31.5 FPS。実用解像度で 30+ FPS → **レンダはボトルネックでない**。
  常駐レンダラはアップロードを 1 回に減らすのでさらに余裕。
- **正しさ**: `GaussianRenderer.render` の出力は `render_frames` と
  バイト一致（`np.array_equal` True）。
- **ROS end-to-end**: ノードを別プロセス起動し `/ego_odom` に `Odometry` を
  publish → `/sensor_sim/image_raw`(512×612, rgb8, frame=camera_optical) と
  `/sensor_sim/camera_info`(fx 363.2) を受信できることを確認。
- **closed-loop デモ**: `pose_player`(koide 軌跡 6Hz) → `sensor_sim_node`
  (808×676) → image を ROS 経由でキャプチャ → `output/sim2real_gap/
  closed_loop_koide.mp4`(80 frames)。photoreal な軌跡追従レンダがパイプラインを
  通って出ることを確認。`pose_topic` を Autoware の localization に差し替えれば
  そのまま実走 closed-loop になる。

## フレーム規約（重要）

- ego pose = `world<-base_link`。`extrinsic` は `base_link<-camera_optical`
  で、OpenCV optical frame（+x 右, +y 下, +z 前 = 学習 transforms と同じ）を指すこと。
- localiser の world frame が 3DGS モデルの frame と違う場合は `align`
  (`model_world<-pose_world`) を設定。一致するなら identity。
- camera-in-model = `align @ T_world_base @ T_base_cam`、viewmat はその逆行列。

## 次アクション

- AWSIM×Autoware パイプライン（`docs/awsim-autonomous-driving-tutorial.md`）の
  localization pose を `pose_topic` に繋ぎ、実シーンの 3DGS（要 align 推定）で
  perception を回す統合。
- 動的アクタ（歩行者/車）は vanilla 3DGS では出ないので Phase 3 で compositing。
- render 結果の exposure/tone を実カメラに寄せる後処理（sim2real gap の更なる縮小）。

## Addendum (2026-06-15): co-registered Autoware マップ束

クリーン construction 3DGS と同一 SLAM frame の Autoware マップ束を生成
（`output/rtkslam_autoware_map/`、ライブ Autoware 接続の前提セット）:

- `pointcloud_map.pcd` — NDT 用点群（lidar_init = LiDAR 蓄積雲、400k 点、
  open3d で PLY→PCD）。
- `lanelet2_map.osm` — `simple_lanelet2_generator.py` で SLAM 軌跡から生成
  （28 lanelets、隣接で境界ノード共有 = Autoware ルーティング要件）。
- `map_projector_info.yaml` — `projector_type: local`（geo 投影なし、SLAM frame）。

3 成果物 + 3DGS が全て RKO-LIO SLAM frame を共有 → **align = identity** で
`sensor_sim_node` に繋がる。座標確認: cloud bbox x[-77,25] y[-12,78] と
trajectory bbox x[-71,0.4] y[-42,70] は同一メトリック frame で重畳。
注意: PCD は 3DGS と同じ 480-545s 窓のみ、lanelet2 は全 738s 軌跡（窓内は整合）。

**残り（ライブ Autoware-in-the-loop）**: Autoware (Humble, CUDA docker) を上記
マップで起動 → bag の `/livox/points` で NDT localization → `/localization/
kinematic_state` を `sensor_sim_node` の `pose_topic` に接続。ただし
construction_seq1 は屋内マシンホールの徒歩シーンで、自動運転/ルーティングとしては
人工的。意味のある走行 closed-loop には屋外路上シーン（isuzu クラス、±1m 耐性）が適切。
