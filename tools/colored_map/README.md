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
| `build_lidar_init.py` | スキャン蓄積 + robust着色（overlap RGB balance / view confidence対応） |
| `recolor_pointcloud.py` | 既存PLYのXYZを保持してcamera画像から再着色し、coverage JSONを出力 |
| `render_map_flythrough.py` | 着色マップ動画（cinematic path / surface splat / 描画指標対応） |
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
K3構成ではさらに `--color-overlap-balance --color-view-confidence
--color-normal-voxel 0.12 --color-view-score-power 1` を使う。前者は同じ3D点を
見る画像間のRGB差から露出・white balanceを安定化し、後者はsurface normalの
入射角と投影解像度で観測を順位付けする。いずれもdefault-off。

README動画の再現設定は `render_map_flythrough.py --device cpu
--soft-edge-px 1 --surface-splat --surface-aspect-limit 2.5
--surface-normal-voxel 0.12 --camera-preset cinematic --render-voxel 0.03
--render-workers 4 --metrics-out metrics.json`。legacy camera、円形splat、直列描画は
既定値のままで互換性を維持する。

## 引き継ぎ文書

- [`COLORING_HANDOFF.md`](COLORING_HANDOFF.md) — 着色品質枝 (2026-07-18)
- [`BIM_HANDOFF.md`](BIM_HANDOFF.md) — Scan-to-BIM 枝 (2026-07-11)

チュートリアル: [`docs/3dgs-map-tutorial.md`](../../docs/3dgs-map-tutorial.md)
(フライスルー生成)、[`docs/workflows.md`](../../docs/workflows.md)。
