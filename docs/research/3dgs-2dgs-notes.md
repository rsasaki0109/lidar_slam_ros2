# 2DGS (surfel) + 平面正則化 — LiDAR-primed では負の結果 (2026-06-13)

[深度教師あり](3dgs-depth-supervision-notes.md)で「位置（深度）」は実面に固定できたが、
**外挿下の RGB floater は残る**（深度は正しいが視点依存の見えが崩れる）。これを surfel
primitive と平面正則化で抑えられないか検証した記録。**結論: LiDAR-primed パイプラインでは
2DGS は 3DGS+深度教師ありに勝てず、productionize しない**（[MCMC](3dgs-mcmc-notes.md) と
同じ理由）。

## 試したこと

gsplat 1.5.3 はネイティブ 2DGS (`rasterization_2dgs`) を持ち、**既存と同じ
(means, quats, scales, opacities, colors) コンテナ**をそのまま使える（DefaultStrategy は
`key_for_gradient='gradient_2dgs'` で対応）。これに 2DGS 標準の 2 正則化を加えた:

- **normal 一貫性**: レンダ法線 `render_normals` を深度勾配由来の `surf_normals` に揃える
  `(1 - <render_normals, surf_normals>)`。
- **depth distortion**: `render_distort`（レイ方向の重み分散）を最小化＝floater を潰す狙い。

RTK-SLAM walk (74 train / 19 held-out) で、[外挿 dolly 深度プローブ](3dgs-depth-supervision-notes.md)
（テスト視点を横へ 0〜0.8m ドリーし render 深度 vs LiDAR 投影深度の MAE）で比較。

## 結果（外挿 dolly 深度 MAE, cm）

| 構成 | 0.0m | 0.2m | 0.4m | 0.8m |
|------|------|------|------|------|
| 3DGS base（photometric） | 349 | 330 | 290 | 184 |
| **3DGS + 深度教師 λ0.02** | **20** | **26** | **30** | **34** |
| 2DGS + normal + dist | 223 | 117 | 124 | 262 |
| 2DGS + normal + dist + depth | 41 | 38 | 50 | 320 |
| 2DGS pure + depth | 35 | 44 | 208 | 44 |

- **全 2DGS 変種が 3DGS+深度教師ありに負ける**。しかも offset により 200〜320cm へ
  **発散**（surfel レンダは外挿下で不安定）。
- 2DGS+正則化（深度なし）は base よりマシだが深度教師ありに遠く及ばない（局所 surfel
  一貫性は**絶対的な実面アンカーにならない**）。
- RGB を外挿レンダしても floater/霧は 3DGS と同程度かむしろ悪化（grazing 角の surfel が
  画面空間で霧になる）。

## 洞察

**LiDAR のメトリック深度事前が、2DGS の自己教師な幾何正則化を包含する**。2DGS の
normal/distortion 損失は「画像だけから面の向きと厚みを一から推定する」ためのもので、
**ground-truth の LiDAR 深度が既にある本パイプラインでは冗長**。むしろ surfel primitive は
外挿下で 3D Gaussian より不安定だった。これは [MCMC が負だった理由](3dgs-mcmc-notes.md)
（LiDAR-primed の強い幾何事前があると、自前で再配置/再発見する手法より素直な手法が優位）
と一貫する。

**外挿下の RGB floater は依然未解決**だが、その本丸は surfel 化ではなく**視点依存の見え**
（色・grazing opacity・モーションブラー）のモデル化。photoreal novel-view はキャプチャ品質
律速（[trajectory-flythrough](3dgs-trajectory-flythrough-notes.md)）という結論は変わらない。

## 注意（フェアネス）

normal 損失の warmup スケジュールや重み・iter 数は網羅的にチューニングしていない（2DGS 論文は
normal 損失を後半から効かせる）。ただし**判断基準は「深度教師ありに勝てるか」**で、素直な
3DGS+深度教師ありが既に幾何を解いている以上、2DGS は明確な利得なく大きなコード経路と外挿
不安定性を持ち込むだけ、と判断した。spike コードは `output/twodgs_spike/spike.py`(gitignore)。
