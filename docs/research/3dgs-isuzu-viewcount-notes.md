# 3DGS view-count lever on Autoware Leo Drive isuzu (2026-06-07)

koide first light（`docs/research/3dgs-koide-first-light.md`）の実用上限 ~25dB の主因が
「視点数 30 / 単一短セグメント」と推定されたため、**視点数を増やす**レバーを
Autoware Leo Drive isuzu（`demo_data/autoware_leo_drive_isuzu/all-sensors-bag1_compressed`、
カラー3カメラ・地上車両・36s・97m 走行）で検証した記録。
**正直な結論: 視点を 21 倍（30→640）にしても koide を超えられず、むしろ悪化した。
視点数よりキャプチャの一貫性・シャープネスが支配的。**

## やったこと

- **圧縮 bag 対応**: isuzu は `compression_mode: FILE`（zstd）。`extract_posed_images.py` /
  `build_lidar_init.py` を metadata 検出で `SequentialCompressionReader` に切替（plain
  reader は `.db3.zstd` を sqlite として開こうとして失敗する）。`ros2 bag play` は透過
  処理するので SLAM フロントエンドは無改修。
- **SLAM**: `/sensing/lidar/concatenated/pointcloud`（base_link, 365 scans, time field
  なし → deskew off）に `lidarslam_kitti_velodyne.yaml` で frontend のみ。286 poses /
  97.3m の直線軌跡。NDT fitness gate の POSE_REJECT が多発（param 不一致）だが軌跡自体は
  幾何的に妥当。
- **extrinsic**: `/tf_static` から `base_link → sensor_kit_base_link →
  camera_top/camera_link → camera_top/camera_optical_link` を合成して
  `configs/gaussian_splatting/isuzu_camera_top_extrinsic.yaml`。手動校正不要。
- **抽出**: camera_0(=camera_top) 640 views、`--undistort`（plumb_bob k1=-0.28 を pinhole 化）。
- LiDAR-primed init(400k 点) + SSIM densify で学習。

## 結果

| 構成 | views | PSNR | SSIM |
|------|-------|------|------|
| koide（校正級・近接密・低速）| 30 | **24.3 dB** | 0.81 |
| isuzu 全区間 | 640 | 14.5 dB | 0.61 |
| isuzu 前半区間（短く=ドリフト小）| 150 | 16.9 dB | 0.69 |
| isuzu 前半 + extrinsic 自己校正 | 150 | 17.6 dB | 0.68 |

比較画像 `assets/3dgs_isuzu_viewcount.png`（左 GT / 右 render）。

## 診断（なぜ視点を増やしても悪化したか）

1. **GT 画像自体がモーションブラー** — camera_top は横向き、車両は ~2.7 m/s で走行。
   横向き高速スイープのため各フレームが流れており、3DGS の教師信号が既にボケている。
2. **視点重複が小さい** — 横向き高速移動で隣接フレームの視野が大きく入れ替わり、
   「多視点で同一構造を観測」になりにくい（見かけの視点数ほど制約が増えない）。
3. **軌跡ドリフト** — frontend-only（graph backend OFF）で 97m を積分。全区間 14.5dB →
   前半 150 views 16.9dB と短くすると改善＝距離方向の不整合が効いている。
4. **シーンが屋外・植生・広域** — 樹木/芝の高周波・確率的テクスチャは gaussian で
   表現しにくく、近接密の人工物（koide のパネル・建物）より本質的に不利。
5. **extrinsic 単独は主因でない** — 自己校正は +0.7dB のみ（回復 tau は大きいのに利得が
   小さい＝photometric 信号が不整合で校正が収束しきらない）。

## 学び（position の補強）

- **3DGS 品質は「視点数」より「キャプチャ特性」が支配的**: シャープ（ブラー無し）・
  視点重複大・近接密・人工構造・pose 一貫（loop-closed / 校正 extrinsic / HW クロック同期）。
  koide はこれらを満たす校正級キャプチャ、isuzu は走行中の横向きカメラで満たさない。
- 視点数レバーを活かすには、単にフレームを増やすのではなく **pose 一貫性を保ったまま**
  増やす必要がある（graph backend ON、`direct_visual_lidar_calibration` 級 extrinsic、
  rolling-shutter/motion-blur の小さい静止寄りキャプチャ）。
- → 当面の 3DGS 成果物は **koide 型（近接・密・カラー・低速・校正級）** を対象にする
  という position を、別データセットでも再確認した（[[3dgs-ntu-viral-notes]] と同じ方向）。

## 残した汎用機能

- `extract_posed_images.py` / `build_lidar_init.py`: FILE-compressed(zstd) bag 対応
  （Autoware 系 bag で再利用可能）。
- `configs/gaussian_splatting/isuzu_camera_top_extrinsic.yaml`: TF 由来 extrinsic の例。
