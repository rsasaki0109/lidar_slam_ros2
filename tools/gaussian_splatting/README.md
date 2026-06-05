# tools/gaussian_splatting — LiDAR-primed 3DGS map deliverable (opt-in)

SLAM の出力（最適化軌跡 + pointcloud_map）から **3D Gaussian Splatting** の
photorealistic map / novel-view 成果物を後処理で再構成するための opt-in ツール群。

設計の全体像・スコープ・ライセンス判断・PoC 計画は
[`docs/research/3dgs-postprocess-map-design.md`](../../docs/research/3dgs-postprocess-map-design.md)
を参照。

## 立ち位置（重要）

- これは **後処理ツール**であって SLAM 本体ではない。RKO-LIO / graph_based_slam
  は触らない。
- 3DGS は pointcloud_map を **置き換えない**。Autoware の localization は従来どおり
  PCD/NDT。3DGS は人間向け検査 / digital-twin / NVS の追加成果物。
- **opt-in**。`colcon` パッケージではない。CUDA を C++ ビルド/標準 CI に持ち込まない。

## ライセンス方針

- 本家 INRIA `gaussian-splatting` は **non-commercial** ライセンスのため**不採用**。
- rasterizer / 学習コアは **gsplat (Apache-2.0)** を前提とする。
- 本リポジトリの BSD-2/MIT 商用フリー方針を維持する。詳細は設計 doc §2。

## 構成

| ファイル | 役割 | 依存 | テスト |
|---|---|---|---|
| `posed_images.py` | GPU/ROS 非依存コア。TUM 軌跡パース、SLERP ポーズ補間、外部標定合成、Nerfstudio `transforms.json` 出力。 | numpy のみ | `graph_based_slam/test/test_gaussian_splatting_posed_images.py`（ament pytest 登録済み） |
| _(予定)_ `extract_posed_images.py` | rosbag2 から画像 + `camera_info` を取り出し、`posed_images` で各画像の `world<-camera` を解決して `transforms.json` + 画像を書き出す CLI。 | rosbag2_py, cv_bridge | bag fixture |
| _(予定)_ `train_gsplat.py` | `transforms.json` + LiDAR 点群初期化で gsplat 学習 → `.ply` + viewer。 | torch, gsplat (CUDA) | GPU 環境のみ |

現状は **コア (`posed_images.py`) のみ実装済み**。これは GPU/ROS 不要で、既存の
ament pytest harness（`run_default_ci_checks.sh`）でそのまま検証される。

## 座標系の約束

- SLAM/TUM ポーズは ROS 右手系の `world <- body`。
- ROS camera optical frame は x-right, y-down, z-forward。
- Nerfstudio/OpenGL カメラは x-right, y-up, z-back。
- `transforms.json` の `transform_matrix` は OpenGL 規約の camera-to-world。
  `posed_images.ROS_OPTICAL_TO_OPENGL = diag(1,-1,-1,1)` を右から掛けて変換する。

## first-light（M1）の想定データ

`demo_data/koide_lidar_camera_calib`（ローカル）が最有力。
`/image` + `/camera_info` + `/livox/points` + `/livox/imu` が同期収録されており、
新規データ取得なしで PoC できる。詳細・他データセットは設計 doc §3 / §6。
