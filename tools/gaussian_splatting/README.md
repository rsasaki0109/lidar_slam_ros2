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
| `posed_images.py` | GPU/ROS 非依存コア。TUM 軌跡パース、SLERP ポーズ補間、外部標定合成、Nerfstudio `transforms.json` 出力。 | numpy のみ | `test_gaussian_splatting_posed_images.py`（19）|
| `extract_posed_images.py` | rosbag2 から画像 + `camera_info` を取り出し、各画像の `world<-camera` を解決して `transforms.json` + 画像を書き出す CLI。`sensor_msgs/Image` は **numpy で生復号**（cv_bridge 非依存）、`rosbag2_py` は遅延 import。 | rosbag2_py（実行時のみ）| `test_gaussian_splatting_extract.py`（17、ポーズ/外部標定/復号ロジックは ROS 非依存）|
| `pointcloud_io.py` | 最小 PLY 入出力（xyz[+rgb]）＋ voxel 間引き＋ **画像投影による点群着色**（`colorize_by_projection`）。 | numpy のみ | `test_gaussian_splatting_pointcloud.py`（12）|
| `build_lidar_init.py` | bag のスキャンを SLAM 軌跡で world 系に蓄積 → **LiDAR-primed init 点群** PLY。FILE-compressed(zstd) bag 対応。`--color-transforms` で transforms.json の posed 画像を投影して着色（品質中立だが検査用に有用）。 | rosbag2_py（実行時のみ）| 同上（`transform_points` 等の純粋部）|
| `train_gsplat.py` | `transforms.json` + 画像で gsplat 学習 → INRIA 標準 `.ply` 出力。OpenGL c2w を OpenCV w2c に変換。`--init-ply` で **LiDAR-primed init**（位置＋色 seed）、`--densify` で gsplat `DefaultStrategy` の adaptive density control、`--ssim-lambda`（既定 0.2）で INRIA 標準 **L1+D-SSIM 損失**、`--knn-scale-init` で点群の局所密度から per-Gaussian スケール seed、`--sh-degree D` で **視点依存カラー（SH 次数 D、INRIA 標準 f_dc+f_rest 出力）**、`--antialiased` で gsplat の antialiased rasterize mode、`--mcmc`（+`--mcmc-cap`）で MCMCStrategy（LiDAR-primed init では DefaultStrategy 優位＝既定）、`--optimize-extrinsic` で共有 6-DoF extrinsic の photometric 自己校正。学習終了時に全ビューの PSNR/SSIM を出力。 | torch, gsplat (CUDA) | `test_gaussian_splatting_train.py`（20、純粋部）|
| `selftest_gpu.py` | opt-in GPU セルフテスト。合成シーンを描画→`transforms.json`→学習→`.ply` の全鎖を検証。 | torch, gsplat (CUDA) | 手動実行（CI 非対象）|

GPU 不要の純粋部は ament pytest harness（`run_default_ci_checks.sh`）で **計 42 ケース**
検証される。CUDA を要する学習部はテストを skip せず CI 面から分離（opt-in）。

## 使い方

```bash
# 1) bag から posed 画像 + transforms.json を抽出（ROS 環境）
#    --time-offset auto: カメラと LiDAR がセンサ内蔵クロックの別基準でも
#    bag 受信時刻から skew を相殺（Livox+cam bag で頻出）
python3 tools/gaussian_splatting/extract_posed_images.py \
  --bag demo_data/koide_lidar_camera_calib/livox/rosbag2_2023_03_09-13_42_46 \
  --traj output/<run>/traj_corrected.tum \
  --camera-topic /image --camera-info-topic /camera_info \
  --extrinsic configs/gaussian_splatting/<lidar_camera_extrinsic>.yaml \
  --time-offset auto --clock-reference-topic /livox/points \
  --out output/<run>/gsplat

# 2a) LiDAR-primed init 点群を構築（COLMAP 不要の幾何事前）
python3 tools/gaussian_splatting/build_lidar_init.py \
  --bag <bag> --traj output/<run>/traj_corrected.tum \
  --points-topic /livox/points --voxel 0.05 \
  --out output/<run>/gsplat/lidar_init.ply

# 2b) gsplat 学習 → .ply（GPU）。--init-ply で LiDAR-primed init、
#     --densify で adaptive density control（鮮鋭化）
python3 tools/gaussian_splatting/train_gsplat.py \
  --transforms output/<run>/gsplat/transforms.json \
  --init-ply output/<run>/gsplat/lidar_init.ply \
  --densify --out output/<run>/gsplat/point_cloud.ply --iters 5000

# GPU 動作確認（合成データ、bag 不要）
python3 tools/gaussian_splatting/selftest_gpu.py --out /tmp/gsplat_selftest

# 実データ first light をワンコマンド再現（SLAM→extract→train）
bash scripts/run_koide_3dgs_firstlight.sh
```

実データ first light の結果・品質要因・次レバーは
[`docs/research/3dgs-koide-first-light.md`](../../docs/research/3dgs-koide-first-light.md)。

## 動作確認済み環境

`selftest_gpu.py` は **NVIDIA RTX 4070 Ti SUPER (16GB) / CUDA 12.0 / torch 2.10 /
gsplat 1.5.3** で PASS（合成 12 視点、photometric MSE 0.298 → 0.009、`.ply` 出力）。
gsplat はネイティブ install 済みのため Docker は必須ではない（再現性のため別途
Dockerfile を将来追加予定）。

実データ koide first light では **random → LiDAR-primed → densify で PSNR
15 → 20.5 → 24.8dB**。詳細: [`docs/research/3dgs-koide-first-light.md`](../../docs/research/3dgs-koide-first-light.md)。

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
