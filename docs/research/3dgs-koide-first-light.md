# 3DGS koide first light (2026-06-06)

`docs/research/3dgs-postprocess-map-design.md` の M1 first light を、ローカルの
`demo_data/koide_lidar_camera_calib`（Livox + 単眼カメラ同期 bag）で実走した記録。
**実 bag → SLAM → posed 画像 → 3DGS `.ply` の全鎖が通った**ことの確認。

再現: `bash scripts/run_koide_3dgs_firstlight.sh`

## パイプライン

```
demo_data/koide_lidar_camera_calib/livox/rosbag2_2023_03_09-13_42_46  (15.6s)
  │
  ├─[1] lidarslam frontend (mid360 noimu)  → traj_map_livox_frame.tum  (30 poses)
  ├─[2] extract_posed_images.py            → 30 posed images + transforms.json
  │        --time-offset auto (camera→/livox/points クロック整合: 21.88s)
  │        --extrinsic 近似 frame-convention (並進ゼロ)
  └─[3] train_gsplat.py (gsplat, 60k init, 1500 iter) → point_cloud.ply (4.1MB)
```

## 結果（3段階の品質向上）

| 指標 | random init | + LiDAR-primed | + **densification** |
|---|---|---|---|
| 学習 photometric MSE | 0.035 | 0.0082 | **0.0024** |
| レンダ PSNR（view 0/15/29） | 14.6 / 15.1 / 14.7 | 20.7 / 20.1 / 20.8 | **25.4 / 22.9 / 26.1 dB** |
| gaussians | 60k | 200k | 220k（適応増加）|
| iter | 1500 | 1500 | 3000 |

共通: 入力 30 views（2448×2048, fl_x≈1453）／ RTX 4070 Ti SUPER ／ gsplat 1.5.3。

![上から random / LiDAR-primed / densify、各 左=GT・右=render (view 15)](assets/3dgs_koide_progression.png)

- **LiDAR-primed init**（設計の核）で +5.5dB。`build_lidar_init.py` が bag のスキャンを
  SLAM 軌跡で world 系に蓄積（151 scans → voxel 0.05 → 200k 点）し、
  `train_gsplat.py --init-ply` が Gaussian 位置（＋色）を seed。COLMAP SfM 不要で
  メートル単位の幾何事前が入る。
- **densification**（`--densify`, gsplat `DefaultStrategy`）でさらに +4.3dB。
  adaptive density control で Gaussian を増減し細部を鮮鋭化。**最終 PSNR ~24.8dB**、
  render は太陽光パネル列・建物エッジ・橙色フレームまで GT にほぼ一致。
  ※短い run では opacity reset が逆効果なので `reset_every` を無効化している。

`random → LiDAR-primed → densify` で **PSNR 15 → 20.5 → 24.8dB**、設計の方向性が
実データで裏付けられた。

## さらなる改善レバー（効果順）

1. **カメラ外部標定が近似** — `configs/gaussian_splatting/koide_lidar_camera_extrinsic_approx.yaml`
   は livox→camera の**フレーム規約のみ**（並進ゼロ・回転のみ）。koide は本来
   `direct_visual_lidar_calibration` 用データなので、その calib 結果を入れれば
   multi-view 整合がさらに上がる（残ブラーの主因と推定）。
2. **frontend-only odometry のドリフト** — graph backend OFF。15s なので軽微だが
   backend ON or ループ補正で pose 品質向上。
3. **LiDAR init の色付け** — 現状は位置のみ seed（色は学習）。画像投影で色を付ければ
   さらに収束が速い。
4. iteration/視点が少ない（3000 iter, 30 views）。長 run + SSIM/LPIPS 損失で更に向上。

## わかったこと（実運用の知見）

- **LiDAR とカメラがセンサ内蔵クロックの別基準**だった（header stamp が ~21.9s ずれ）。
  bag 受信時刻で skew を相殺する `--time-offset auto` を実装して解決。同種の Livox+cam
  bag で再利用可能。
- `cv_bridge` は当環境の numpy 2.4.4 で ImportError。`sensor_msgs/Image` を numpy 生
  復号（`decode_image`）して回避済み。

## 次アクション

- (A) `train_gsplat` に **LiDAR-primed init**（pointcloud_map / scan 点群からの Gaussian
  初期化）と densification を追加 → 品質の本丸。
- (B) koide の **正規 extrinsic**（direct_visual_lidar_calibration）を入れて再評価。
- (C) NTU VIRAL（ステレオ + GT）で pose 品質と 3DGS 品質の相関を見る（設計 doc M2）。
