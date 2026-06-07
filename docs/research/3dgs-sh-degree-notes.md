# 3DGS spherical-harmonics (view-dependent colour) on koide (2026-06-07)

これまでの 3DGS トレーナは **SH band-0（フラット色、`f_dc` のみ）** で、視点依存の
見え（鏡面・反射）を表現できなかった。INRIA 標準の **SH 次数 >0** を `train_gsplat.py`
に追加し、koide で検証した記録。**SH degree 1 で初めて PSNR が正方向に動いた**
（他の品質レバー＝extrinsic 自己校正・視点数・init 着色はいずれも PSNR 中立/悪化だった）。

## 実装

- `--sh-degree D`（opt-in、既定は従来の band-0）。色 param を **`sh0`(N,3) DC + `shN`
  (N,(D+1)^2-1,3) 高次** に分割し、INRIA 流に DC は `lr*3`、高次は `lr*3/20` の低 LR で
  最適化。`gsplat.rasterization(..., sh_degree=D)` がカメラ中心から視点依存色を評価する。
- 出力は INRIA 標準 `.ply`（`f_dc_0..2` + `f_rest_0..{3*(K-1)-1}` channel-major）。
  SuperSplat 等でそのまま開ける（degree 3 で f_rest 45 個を確認）。
- band-0 既定パスは完全に従来どおり（後方互換）。

## 結果（koide, LiDAR-primed init + SSIM densify, 3000 iter, 学習ビュー評価）

| 色モデル | PSNR | SSIM |
|----------|------|------|
| band-0（従来） | 23.79 dB | 0.8412 |
| **SH degree 1** | **24.06 dB** | **0.8424** |
| SH degree 3 | 23.93 dB | 0.8412 |

- **SH degree 1 が +0.27dB / +0.0012 SSIM** と小幅だが明確な改善。太陽光パネル等の
  弱い視点依存成分を 1 次 SH が拾う。
- **SH degree 3 は +0.14dB と頭打ち**。45 個の高次係数は **30 視点・3000 iter では
  情報不足で overfit/ノイズ**になりやすい。これは視点数の知見（`3dgs-isuzu-viewcount-notes.md`）
  と整合: 高次 SH は多数の一貫した視点があって初めて効く。

## antialiased rasterization と学習 iter の追検証（2026-06-07）

`--antialiased`（gsplat `rasterize_mode='antialiased'`、opacity 補正付き screen-space
filter）と **学習 iter 数**を同時に振った。

| 構成 | iter | PSNR | SSIM |
|------|------|------|------|
| SH deg1 classic | 3000 | 24.06 dB | 0.8424 |
| SH deg1 **antialiased** | 3000 | 23.44 dB | 0.8283 |
| SH deg1 antialiased | 9000 | 24.60 dB | 0.8489 |
| **SH deg1 classic** | **9000** | **25.18 dB** | **0.8569** |
| SH deg1 classic | 15000 | **25.47 dB** | **0.8617** |

- **antialiased は koide で一貫して -0.6dB と逆効果**（3000: 24.06→23.44、9000:
  25.18→24.60）。短 run + densify 設定では opacity 補正が裏目に出る。フラグは他シーン用に
  残すが既定 OFF。
- **学習 iter を 3000→9000 にすると +1.12dB（24.06→25.18dB）と最大の利得**。15000 で
  25.47dB（+0.29）と逓減。収束カーブは **3k 24.06 → 9k 25.18 → 15k 25.47**、9k が
  compute/品質の knee。
- **これまで ~24dB を「data-bound の上限」と見ていたが、一部は単に under-training だった**
  ── 3000 iter は未収束で、十分回せば 25.5dB 帯に届く（per-view では既に 25.4dB を観測して
  いたのとも整合）。15k ベスト構成のレンダは GT とほぼ一致（パネル列・建物・橙フレームまで鮮鋭）:

![左 GT / 右 render (SH deg1, 15k, 25.47dB), view 0/15/29](assets/3dgs_koide_best_sh1_15k.png)

## 推奨（更新）

- koide 型（少視点・近接密）では **`--sh-degree 1` + classic + iter は長め（>=9000）** が
  実用上のベスト（25.2dB）。`--antialiased` は koide では使わない。
- 多視点・pose 一貫データが用意できれば degree 3 が活きる余地（未検証）。
- **重要な更新**: 「~24dB は data-bound」という以前の結論は **一部 under-training の誤読**
  だった。iter を伸ばせば 25dB 超まで動く（ただし 3倍の compute で +1dB の逓減）。残る
  上限突破レバーは依然 capture/pose 一貫性だが、まず十分な iter を回すことが先決。
