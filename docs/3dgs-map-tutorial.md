# 3DGS Photoreal Map (post-process) — operator tutorial

LiDAR-SLAM の出力（最適化軌跡 + LiDAR 点群）と同期カメラ画像から、**3D Gaussian
Splatting (3DGS) の photorealistic map / novel-view 成果物**を後処理で作る手順。
研究ノートに散らばった知見を、再現可能な 1 本のガイドにまとめたもの。

## これは何で、何でないか

- **後処理の成果物**。SLAM 本体（RKO-LIO / graph_based_slam）は触らない。
- **localization マップではない**。Autoware の自己位置推定は従来どおり PCD/NDT。
  3DGS は人間向け検査 / digital-twin / NVS 用の追加成果物。
- **opt-in**。`colcon` パッケージではなく `tools/gaussian_splatting/` の Python 群。
  CUDA を C++ ビルド/標準 CI には持ち込まない。
- ライセンス: rasterizer/学習コアは **gsplat (Apache-2.0)**。INRIA 本家
  `gaussian-splatting` は non-commercial のため不採用。

## 前提

- CUDA GPU + `torch` + `gsplat`（動作確認: RTX 4070 Ti SUPER / CUDA 12 /
  torch 2.10 / gsplat 1.5.3、ネイティブ install）。
- **同期した LiDAR + カラーカメラの bag**。カメラ無し（MID-360 実機等）は対象外。
- 良いシーン特性が品質を支配する（後述）: 近接・密・カラー・低ブラー・視点重複大。

## いちばん簡単な再現（koide ワンコマンド）

ローカルの `demo_data/koide_lidar_camera_calib`（Livox + 単眼カメラ同期）で、
SLAM → posed 画像抽出 → LiDAR-primed init → gsplat 学習までを一括実行する。
既定はベスト構成（SH 次数 1 / DefaultStrategy / iter 9000）。

```bash
bash scripts/run_koide_3dgs_firstlight.sh
# 出力: output/koide_3dgs_firstlight/gsplat/point_cloud.ply  (~25dB)
```

主要な環境変数で構成を変えられる: `ITERS`（既定 9000、15000 で +0.3dB）、
`SH_DEGREE`（既定 1）、`DENSIFY`、`LIDAR_PRIMED`、`NUM_INIT`。

## 手動パイプライン（任意の bag 向け）

`<bag>` をカメラ + LiDAR の同期 bag、`<run>` を作業ディレクトリとする。

### 1. SLAM 軌跡を作る

LiDAR で SLAM を回し、TUM 軌跡（`world <- body`）を得る。Livox 系は例:

```bash
bash scripts/compare_with_glim.sh --bag <bag> --skip-glim \
  --points-topic /livox/points --imu-topic /livox/imu --no-imu \
  --no-graph-based-slam --param lidarslam/param/lidarslam_mid360_noimu.yaml \
  --robot-frame-id livox_frame --base-frame livox_frame --lidar-frame livox_frame \
  --out-dir <run>
```

### 2. posed 画像 + transforms.json を抽出

各画像の `world <- camera` を解決して Nerfstudio `transforms.json` と画像を書き出す。

```bash
python3 tools/gaussian_splatting/extract_posed_images.py \
  --bag <bag> --traj <run>/lidarslam/traj_map_*.tum \
  --camera-topic /image --camera-info-topic /camera_info \
  --extrinsic configs/gaussian_splatting/<lidar_camera_extrinsic>.yaml \
  --time-offset auto --clock-reference-topic /livox/points \
  --out <run>/gsplat
```

- `--time-offset auto`: カメラと LiDAR がセンサ内蔵クロックの別基準でも、bag 受信
  時刻から skew を相殺（Livox+cam bag で頻出。koide は ~21.9s ずれていた）。
- `--extrinsic`: camera optical ← body の外部標定 YAML（`matrix` または
  `translation`+`rotation_xyzw`）。`/tf_static` から合成した例が
  `configs/gaussian_splatting/` にある。
- camera_info が bag に無いときは `--intrinsics-yaml`、歪みがあれば `--undistort`、
  大型 bag は `--start-time/--end-time` で窓切り出し。
- FILE-compressed (zstd) の bag（Autoware 系）も自動検出して読める。

### 3. LiDAR-primed init 点群（設計の核）

bag のスキャンを SLAM 軌跡で world 系に蓄積した点群を Gaussian の初期位置に使う
（COLMAP SfM 不要でメートル単位の幾何事前が入る）。

```bash
python3 tools/gaussian_splatting/build_lidar_init.py \
  --bag <bag> --traj <run>/lidarslam/traj_map_*.tum \
  --points-topic /livox/points --voxel 0.05 --max-points 200000 \
  --out <run>/gsplat/lidar_init.ply
```

### 4. gsplat 学習 → .ply

```bash
python3 tools/gaussian_splatting/train_gsplat.py \
  --transforms <run>/gsplat/transforms.json \
  --init-ply <run>/gsplat/lidar_init.ply \
  --densify --sh-degree 1 --ssim-lambda 0.2 --iters 9000 \
  --out <run>/gsplat/point_cloud.ply
# 末尾に全ビューの PSNR/SSIM を表示。出力は INRIA 標準 .ply。
```

## 推奨構成と、効いた/効かなかったレバー

koide（近接密カラー、30 視点）で 8 つの品質レバーを検証した結論。
**ベスト = LiDAR-primed init + DefaultStrategy + SSIM 損失 + SH 次数 1 + iter 9000〜15000
= PSNR 25.2〜25.5dB**。

| レバー | 効果 | 既定 |
|--------|------|------|
| LiDAR-primed init (`--init-ply`) | +5.5dB（幾何事前） | ON |
| densification (`--densify`, DefaultStrategy) | +4.3dB | ON |
| 学習 iter 3000→9000→15000 | **+1.1 / +0.3dB（最大）** | 9000 |
| SH 次数 1 (`--sh-degree 1`) | +0.27dB（視点依存色） | ON |
| L1+D-SSIM 損失 (`--ssim-lambda 0.2`) | 知覚 +0.03（PSNR 中立） | ON |
| k-NN scale init (`--knn-scale-init`) | 中立 | OFF |
| init 着色 (`--color-transforms`) | 中立（検査用には有用） | OFF |
| extrinsic 自己校正 (`--optimize-extrinsic`) | 中立（要・不良 extrinsic 用） | OFF |
| antialiased (`--antialiased`) | **-0.6dB（逆効果）** | OFF |
| MCMCStrategy (`--mcmc`) | **-1.3〜-1.5dB（LiDAR-primed と不適合）** | OFF |

詳細な ablation:
[`research/3dgs-koide-first-light.md`](research/3dgs-koide-first-light.md) /
[`research/3dgs-sh-degree-notes.md`](research/3dgs-sh-degree-notes.md) /
[`research/3dgs-mcmc-notes.md`](research/3dgs-mcmc-notes.md)。

**学んだ要点**: 当初 ~24dB を「データ上限」と見ていたが、実は 3000 iter が
under-training だった。十分回せば 25.5dB に届く。MCMC / antialiased が負なのは、
**LiDAR-primed の強い幾何事前**があるため（clone/split が再配置より有利、opacity 補正が
裏目）。

## データ適性（重要）

3DGS の品質は**キャプチャ特性が支配的**。視点数を増やしても素性が悪いと改善しない。

- **向く**: koide 型 = 近接・密・カラー・低ブラー・低速・視点重複大・pose 一貫（校正級）。
- **向かない（検証済みの負例）**:
  - Autoware Leo Drive isuzu（横向きカメラ走行、640 視点）: モーションブラー + 視点
    重複小 + frontend-only ドリフトで ~14dB。視点 21 倍でも koide に届かない
    （[`research/3dgs-isuzu-viewcount-notes.md`](research/3dgs-isuzu-viewcount-notes.md)）。
  - NTU VIRAL（mono・広域・疎）: ~10dB
    （[`research/3dgs-ntu-viral-notes.md`](research/3dgs-ntu-viral-notes.md)）。
  - グレースケール / camera_info・extrinsic 欠落のデータ（newer_college 等）はカラー
    3DGS の前提を満たさない。

## 出力 .ply の閲覧

`point_cloud.ply` は INRIA 標準レイアウト（`f_dc` + SH の `f_rest`）。
[SuperSplat](https://playcanvas.com/supersplat/editor) などの web ビューアにドラッグ
&ドロップで開ける。

## トラブルシュート

- 全フレーム drop → カメラと LiDAR のクロック別基準。`--time-offset auto`
  + `--clock-reference-topic` を使う。
- pose lookup 失敗 → `--robot-frame-id/--base-frame/--lidar-frame` を点群の
  `header.frame_id` に揃える。
- レンダが霧状 → extrinsic / pose の不整合かモーションブラー。データ適性を疑う。
- `.db3.zstd` を sqlite として開けない → 圧縮 bag は自動検出済み（古い版なら更新）。

## 関連

- 設計・スコープ・ライセンス:
  [`research/3dgs-postprocess-map-design.md`](research/3dgs-postprocess-map-design.md)
- ツール詳細: `tools/gaussian_splatting/README.md`
- 再現スクリプト: `scripts/run_koide_3dgs_firstlight.sh`
