# LiDAR 深度教師あり 3DGS — 「綺麗だが幾何が空洞」を実面に整列させる (2026-06-13)

これまでの 3DGS トレーナは **photometric（見た目）損失のみ**で Gaussian を最適化して
いた。LiDAR は init（`--init-ply`、LiDAR-primed）にしか使っておらず、学習中は幾何を
拘束していない。本ノートは **LiDAR の深度を学習中の教師信号**として効かせる
`--lidar-depth-lambda` を `train_gsplat.py` に追加し、koide で検証した記録。

**核心の発見**: photometric-only の 3DGS は**見た目は良い（PSNR 23dB）が、新規視点の
幾何が約 10 倍ズレている**。LiDAR 深度教師ありはこれを実面に整列させ、新規視点の深度
誤差を **2〜4 桁**改善する。これは本リポジトリの「メトリックマップ成果物」という位置
づけ ([3dgs-postprocess-map-design.md](3dgs-postprocess-map-design.md)) に直結する。

## 動機 — photometric-only の幾何は信用できない

3DGS は半透明 Gaussian をアルファ合成して画像を作る。**訓練視点の見た目さえ合えば
良い**ので、Gaussian を「真の面より手前」に大量に置いて（floater）合成結果だけ正解に
する解に容易に落ちる。訓練視点では破綻しないが、**新規視点ではその floater 配置が幾何
的に意味をなさない**。マップとして使うなら致命的。

koide の保留視点（held-out）で実測した深度分布（同一視点、3000 iter）:

| モデル | レンダ深度 中央値 | (5,50,95) pct | GT(LiDAR) (5,50,95) pct |
|--------|------------------|----------------|--------------------------|
| **base（photometric only）** | **3.3 m** | (1.9, 3.3, 6.1) | (8.7, 26.5, 52.2) |
| λ=0.002 | 26.2 m | (8.6, 26.2, 51.3) | (8.7, 26.5, 52.2) |
| λ=0.5 | 26.4 m | (8.1, 26.4, 52.2) | (8.7, 26.5, 52.2) |

真の面が **26m** にあるのに、photometric-only は Gaussian を **3m** に置いている。
見た目（PSNR 23dB）には現れないが、**深度は約 10 倍の嘘**。深度教師ありは λ=0.002 でも
レンダ深度を GT にほぼ一致させる。

## 実装

- `--lidar-depth-lambda L`（opt-in、既定 0=従来どおり）。`--init-ply`（LiDAR 雲）必須
  — 無いと `ValueError`。
- **疎 GT 深度マップ**: `pointcloud_io.project_depth_maps` が world 点群を各視点へ投影し、
  **画素ごとに最近傍 z（z-buffer）**を取って疎な GT 深度を作る（手前の面が背後の点を
  自然に遮蔽）。純 numpy・CI テスト付き。
- **損失**: gsplat の `render_mode='RGB+ED'`（expected depth）でレンダ深度を得て、GT が
  ある疎画素で **メトリック L1**（`L * |depth_render - depth_lidar|`）を photometric 損失
  に加算。深度はカメラ系 z（メートル）なので両者の単位は一致。
- 学習終了時に保留視点ではなく**訓練視点**の深度 median abs error を表示
  （`depth_mae`）。
- band-0 / SH / densify / MCMC いずれの経路にも差し込めるよう `render_view` に
  `with_depth` を追加。深度なし時は完全後方互換。

## 結果（koide firstlight, 24 train / 6 held-out, SH deg1, 3000 iter）

| λ | held-out PSNR | held-out 深度 MAE |
|---|---------------|--------------------|
| 0（base） | **23.5 dB** | **~21 m** |
| 0.002 | 22.5 dB | **0.38 m** |
| 0.005 | 21.9 dB | 0.15 m |
| 0.01 | 21.3 dB | 0.17 m |
| 0.02 | 20.6 dB | 0.077 m |
| 0.1 | 19.2 dB | 0.013 m |
| 0.5 | 18.2 dB | 0.003 m |

- **単調**: λ↑ で幾何 MAE は桁単位で改善、PSNR は漸減。**幾何 ↔ 見た目の明快なノブ**。
- **knee = λ≈0.002**: 深度 MAE を **21m → 0.38m（98% 減）** に落とすのに PSNR は
  **−1dB だけ**。1dB の見た目を払って「幾何的に意味のないモデル」を「メトリックに忠実な
  マップ」に変える、良いトレード。
- 「完全に無償」の幾何は無い（最小 λ でも ~1dB は払う）。これは深度教師ありが
  densification と競合し、3000 iter では見た目の収束を一部犠牲にするため。

## 2 シーン目クロス検証 — RTK-SLAM 近接 construction (2026-06-13)

退化/近接シーンで一般化を確認。RTK-SLAM construction の歩行窓
(`transforms_walk.json`, 600x440, 74 train / 19 held-out, init=`lidar_init_minr.ply`):

| λ | held-out PSNR | held-out 深度 MAE |
|---|---------------|--------------------|
| 0（base） | 23.83 dB | **3.46 m** |
| 0.002 | 23.63 dB | **0.35 m** |
| 0.02 | 23.42 dB | 0.20 m |

- 同じく **幾何を 10 倍改善**（3.46m → 0.35m）。レバーはシーンを跨いで一般化する。
- **PSNR 代償が桁違いに小さい**: λ=0.002 で **−0.2dB**（koide の −1dB に対し）。近接・
  高重複だと baseline 幾何が元々マシ（3.46m vs koide 21m）で、深度を実面に寄せても見た目
  をほとんど壊さない。
- **実用的結論**: PSNR 代償はシーンの**深度スケール・視点疎度に比例**する。本パイプラインが
  実際に狙う**近接・高重複マッピングでは深度教師ありはほぼ無償の「幾何保険」**になる。

（注: hold-every-N は近接内挿評価なので、軌跡から外れる外挿（崩壊半径 0.4m,
[3dgs-trajectory-flythrough-notes.md](3dgs-trajectory-flythrough-notes.md)）は突いていない。
外挿下での幾何保持は次段の検証課題。）

## 位置づけと既定

- **既定 OFF（opt-in）**。他の品質レバー同様、PSNR を払うので default では入れない。
- **幾何忠実度が要るとき**（マップ計測用途、点群との整合、下流の localization 検討）に
  `--lidar-depth-lambda 0.002` 程度を推奨。
- これは「綺麗な render」ではなく「**幾何的に正しい 3DGS マップ**」を作るレバー。SLAM が
  メトリックな点群を持つ本パイプラインだからこそ COLMAP 無しで深度教師信号を供給できる
  （[LiDAR-primed の核](3dgs-postprocess-map-design.md) の自然な延長）。

## 再現

```bash
# 幾何忠実な 3DGS マップ（LiDAR 深度教師あり）
python3 tools/gaussian_splatting/train_gsplat.py \
  --transforms output/koide_3dgs_firstlight/gsplat/transforms.json \
  --init-ply  output/koide_3dgs_firstlight/gsplat/lidar_init.ply \
  --out model_depthsup.ply --sh-degree 1 --iters 9000 \
  --lidar-depth-lambda 0.002
```

## 残課題 / 次レバー

- 退化シーン（HILTI exp07 廊下、RTK-SLAM 歩行窓）での効果測定。floater が支配的なほど
  深度教師ありの相対利得は大きいはず。
- depth loss の warmup / densify 後半のみ適用で PSNR 代償を圧縮できるか。
- scale-invariant / 相対深度損失で大深度シーンの重み付けを改善。
- 深度教師あり前提なら **法線/平面正則化**（2DGS 系）も自然な次段。
