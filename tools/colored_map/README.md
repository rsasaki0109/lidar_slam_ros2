# tools/colored_map — カメラ着色点群マップとエクスポート

カメラ画像を LiDAR 点群へ投影して実色の SLAM マップを作り、GIS / mesh /
LAS / CAD-BIM へ書き出すためのツール群。**3D Gaussian Splatting には依存しない**
(古典的な投影 + z バッファ遮蔽 + medoid 色決定。レンダも `--device cpu` の
numpy ラスタライザで CUDA / torch 不要)。歴史的経緯で
`tools/gaussian_splatting/` に同居していたが 2026-07 に分離した。旧パスは
互換 shim で import / 実行とも動き続ける。

## 主なエントリポイント

| ツール | 役割 |
|---|---|
| `colored_map_pipeline.py` | bag + TUM 軌跡 → posed images → 着色マップ → 品質ゲートまでの一括実行 |
| `extract_posed_images.py` | bag から姿勢付きカメラ画像 (`transforms.json`) を抽出 |
| `build_lidar_init.py` | スキャン蓄積 + 着色 (`--color-robust --color-image-margin --color-vignette-gain-limit --color-min-samples`) |
| `render_map_flythrough.py` | 着色マップの三人称フライスルー動画 (`--color-mode rgb --device cpu --soft-edge-px`) |
| `colorize_from_bag.py` | SLAM なし・静的 extrinsic での単発着色 (マルチカメラ融合対応) |
| `map_export.py` / `las_export.py` / `mesh_export.py` | GIS delimited text / LAS 1.2 / 色付き mesh 出力 |
| `bim_export.py` / `bim_pipeline.py` | 平面抽出 → IfcSlab/IfcWall/IfcDoor/IfcWindow/IfcSpace (IFC4) |

品質評価は `scripts/evaluate_heldout_point_colors.py --image-margin`(忠実度)、
`scripts/evaluate_colored_map_appearance.py`(彩度保持・胡椒ノイズ・coverage)、
`scripts/check_colored_map_quality.py`(ゲート統合)の3点セットで行う。
RTK-SLAM construction_seq1 の較正済み report-only 閾値は
`configs/colored_map_quality_profiles/rtkslam_seq1_report_only.yaml` にある。
品質ゲートのレポート引数はプロファイルが参照する領域だけ指定すればよい。
`appearance_planar_roughness_*` 閾値を持つプロファイルでは平面限定roughnessも
パイプラインが自動計算する。
realtime nodeの出力確認には`scripts/evaluate_realtime_colored_map.py`を使い、
confirmed coverageとchromaをJSON保存できる。
CPU rendererの`--soft-edge-px 1`は不透明surfaceを変えず黒い隙間だけをfadeで
埋める。mesh exportは`--thin-voxel`でmulti-million-point入力を事前に間引ける。
RTK-SLAMで検証済みのビネット補正は `--color-image-margin 120
--color-vignette-gain-limit 2.5`。補正はgain limitが1のとき無効で、従来出力を
維持する。

## 引き継ぎ文書

- [`COLORING_HANDOFF.md`](COLORING_HANDOFF.md) — 着色品質枝 (2026-07-18)
- [`BIM_HANDOFF.md`](BIM_HANDOFF.md) — Scan-to-BIM 枝 (2026-07-11)

チュートリアル: [`docs/3dgs-map-tutorial.md`](../../docs/3dgs-map-tutorial.md)
(フライスルー生成)、[`docs/workflows.md`](../../docs/workflows.md)。
