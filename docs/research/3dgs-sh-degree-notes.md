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

## 推奨

- koide 型（少視点・近接密）では **`--sh-degree 1`** が無難な品質上乗せ。
- 多視点・pose 一貫データが用意できれば degree 3 が活きる余地（未検証）。
- いずれにせよ PSNR ~24dB 帯は崩れず、上限は依然 capture/pose 一貫性が支配的という
  position は不変。SH は「効く向きの正のレバー」だが小幅。
