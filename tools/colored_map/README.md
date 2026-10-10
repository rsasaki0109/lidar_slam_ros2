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
| `attach_dynamic_image_masks.py` | 外部の動的物体PNG maskを検証し、hash/coverage付きmanifestへ接続 |
| `build_lidar_init.py` | スキャン蓄積 + robust着色（overlap RGB balance / view confidence対応） |
| `refine_camera_poses.py` | 画像特徴点とLiDAR地図で view ごとにカメラ姿勢を補正（held-out ペアのエピポーラ誤差が改善したときだけ採用） |
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

`evaluate_heldout_point_colors.py` の露出正規化は、着色に使う学習画像の
輝度中央値を基準にする。レポートの `exposure_reference` は
`training_views`（正規化無効時は `null`）。全画像の中央値を使っていた旧版の
スコアとは直接比較せず、同じ版・分割・設定で再評価する。
パイプラインの held-out 評価は、地図生成コマンドの実効着色設定を
`--fusion-options` で受け渡し、同じ着色処理を学習画像だけで実行する。
レポートに設定と学習・評価画像のインデックスを保存する。単独実行でこの指定を
省略した場合は従来の既定設定による再着色を維持する。評価画像の露出補正と
評価用 `--image-margin` は別設定で、パイプラインは露出設定を着色側と揃える。
以前のパイプライン評価値とは条件が変わるため、候補と基準をともに再評価する。
`--use-pointcloud-colors` は着色に使った画像集合や未着色点を検証しないため、
その出力だけで held-out 精度を主張しない。

パイプラインは各段階の成功した実行条件を
`pipeline_stage_commands.json` に保存し、同じ条件の出力を再利用する。
着色設定を変更すると地図と評価を、カメラ時刻・外部較正・画像設定を変更すると
画像とその後続を再生成する。着色だけの変更では画像や較正を再計算しない。
中断・失敗した段階と未実行の後続は再開時に作り直す。`--dry-run` は履歴を変更しない。
旧出力など実行履歴がない場合は、一度元のbagから再生成する必要がある。
`--force-images` / `--force-map` / `--force-quality` は従来どおり明示的な再生成に使える。
判定にはコマンドと作業ディレクトリ、および対応する入力の更新時刻を使う。
ソフトウェアの更新や更新時刻を保持した入力の置換は検出しないため、その場合は
該当段階を `--force-*` で再生成する。

`refine_camera_poses.py` は着色前に view ごとのカメラ姿勢を補正する。
近い view 同士（既定 1・3 フレーム差）で ORB 特徴点を対応付け、片方の view の
深度画像で LiDAR 面に持ち上げた点に、もう片方の view を PnP で合わせる。
これを全 view で数回繰り返す。2 フレーム差のペアは当てはめに使わず、
そのエピポーラ（Sampson）誤差が `--min-improvement` 以上下がったときだけ
補正後の姿勢を書き出す（下がらなければ元の姿勢をそのまま書く）。色は使わないので、
着色後の held-out 色誤差は独立した確認になる。近い view 同士の食い違い
（色ぼけの原因）は数回で消えるが、全 view に共通する誤差（一定の外部較正誤差）は
ほとんど観測できない。RTK-SLAM construction_seq1（260 view）での結果は
下記「カメラ姿勢の補正」を参照。

```bash
python3 tools/colored_map/refine_camera_poses.py \
  --transforms posed/transforms.json --pointcloud geometry.ply \
  --out posed/transforms_refined.json --workers 4
python3 tools/colored_map/recolor_pointcloud.py \
  --input geometry.ply --transforms posed/transforms_refined.json --out colored.ply
```

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

品質を優先する場合は、既存の実行コマンドに `--color-max-samples 32` を追加して
比較できる（既定値12）。固定geometry・姿勢・学習/評価画像分割で、この値だけを
変更した結果は次の通り。数値はheld-out画像とのRGB L2誤差の平均で、小さい方がよい。

| データ | 12観測 | 32観測 | 改善率 |
|---|---:|---:|---:|
| AIST 162554 | 37.442 | 35.807 | 4.37% |
| AIST 162651 | 31.327 | 29.978 | 4.31% |
| RTK construction_seq1 K4 | 61.850 | 51.329 | 17.01% |

各走行の着色被覆と評価対象点は同一。ただし視点別中央値はAISTで各8視点、
RTKで12視点が悪化した。反射面や誤投影の解決、測色的な真値への精度向上を
証明するものではない。手元のGo2/JEPLO屋内5本は画像topicがなく、着色は未評価。

RTK-SLAMの配布bagでは、カメラとIMUの約−20.6msの差は補正済みである
（[公式の時刻説明](https://rtk-slam-dataset.github.io/#download)）。
保存較正の `timeshift_cam_imu` を画像headerへ再度加えない。
配布bagの補正済み時計と同じ時計の軌跡を使う場合は、既存の
`--time-offset 0 --time-offset-adjustment 0` で追加補正を明示的に無効にできる。
独自に変換したbagや別時計の軌跡では、その生成履歴を先に確認する。
この指定はscan内の点時刻によるdeskewや外部較正を代替しない。

RTK約491万点の単回比較では、12→32観測で処理時間274→426秒、ピークRSS
2.97→4.24 GiBとなった（同じ旧medoid実装）。現在のmedoid実装は全観測間の
距離配列をソートと累積和に置き換え、32観測で色・未着色mask・全品質評価値を
完全一致させたままRSSを4.01 GiBに削減した。時間は430秒で、全処理の高速化は
確認していない。さらに深度・品質の観測順位配列を共有し、同じRTK32設定の
新しい比較でRSSを4.01→3.42 GiB（14.6%減）に削減した。点群色・未着色mask・
全52評価視点の品質値は完全一致し、AIST 2本×12/32観測でも出力が完全一致した。
この比較の時間は276→392秒だが、変更後の実行中に別の計算処理を観測しており、
速度への影響は未確定。比較は単回で、実行順・cache・外部負荷の影響を含む。
観測保持用メモリは残るため、余裕がない環境では既定値12を使う。

検証記録（2026-09-25、ローカルデータルート
`/media/sasaki/aiueo2/jeplo_data/experiments/colorization_accuracy/`）:
`aist_all_training_samples_r1/`、`rtk_samples_paired_r1/comparison.json`、
`medoid_memory_validation/`、`rtk_medoid_memory_r1/comparison.json`、
`sample_rank_memory/{aist_completed,rtk_comparison,host_load_observation}.json`。
入力・実効設定・ソースhashと、disjoint画像分割を各実験に保存している。

地図geometry自体の動的障害物は、任意依存の
[`dynamic-object-removal`](https://github.com/rsasaki0109/dynamic-3d-object-removal)
0.5以降を導入し、`--dynamic-map-cleaner fusion`で除去できる。各LiDAR scanを
trajectoryでworld座標へ変換した点と同じ時刻のsensor originをcleanerへ渡すため、
単純な完成地図の点数削減ではない。大規模bagでは
`--dynamic-map-cleaner-evidence-stride N`で判定用scanを間引ける。除去点数、比率、
使用scan数、実装versionは`dynamic_map_cleaning.json`へ保存する。この機能も
default-offで、静的構造の保持と既存quality profileをpaired評価してから有効化する。

```bash
pip install 'dynamic-object-removal>=0.5'
python3 tools/colored_map/colored_map_pipeline.py BAG TRAJECTORY OUT \
  --dynamic-map-cleaner fusion --dynamic-map-cleaner-workers 4 \
  --dynamic-map-cleaner-evidence-stride 5
```

物体境界の色滲みを抑えるgeometry-aware fusionもdefault-offで利用できる。
`--color-geometry-aware`は1 pixel z-buffer近傍で、手前silhouetteの隣に投影された
背景点と、深度不連続の両側をRGB候補から除外する。外部segmentationのPNGを
`--dynamic-mask-dir`で接続し`--color-dynamic-exclusion`を指定すると動的領域も
除外する。`--refine-spatiotemporal-calibration`と
`--color-calibration-sigma-multiplier`を組み合わせると、較正の7DoF不確実性と
camera速度をpixel半径へ伝播し、各guardを観測ごとに拡張する。棄却数は
`fusion_diagnostics`としてmap/recolor reportへ残る。

```bash
python3 tools/colored_map/colored_map_pipeline.py BAG TRAJECTORY OUT \
  --extrinsic BODY_CAMERA.json --refine-spatiotemporal-calibration \
  --color-geometry-aware \
  --dynamic-mask-dir dynamic_masks --color-dynamic-exclusion \
  --color-dynamic-mask-margin-px 2 \
  --color-calibration-sigma-multiplier 1.0
```

maskは各posed imageと同じstemのPNGで、非zero pixelを除外領域とする。
補間着色では色を混ぜる画素の範囲も除外判定する。edge-awareも保守的に同じ範囲を確認し、
除外画素に接する観測を使わないため、境界で着色される点が減る場合がある。動的除外を
有効にする場合は全frameのmaskが必須。露出・画像間色合わせ・周辺減光の補正量も
除外領域を使わず推定する。全域除外などで支持がない画像は補正の根拠にしない。詳しい設計と安全条件は
[`colored-map-geometry-aware-fusion-2026-07.md`](../../docs/research/colored-map-geometry-aware-fusion-2026-07.md)
を参照。
silhouette/depth-edge marginはConstruction Seq1の全量候補が既存planar quality
gateを通らなかったため既定0。dataset固有のpaired A/Bと既存profileを通すまで
明示的に有効化しないこと。

edge-aware samplingは4 cornerの巨大な一時stackを作らず、同じcornerからRGBの
min/maxをin-place更新する。旧式との完全一致testに加え、Construction Seq1の
paired screenでPLY SHA-256とreportが一致し、wall timeを25.0%短縮した。詳細は
[`colored-map-fusion-performance-2026-07.md`](../../docs/research/colored-map-fusion-performance-2026-07.md)。

README動画の再現設定は `render_map_flythrough.py --device cpu
--soft-edge-px 1 --surface-splat --surface-aspect-limit 2.5
--surface-normal-voxel 0.12 --camera-preset cinematic --render-voxel 0.03
--render-workers 4 --metrics-out metrics.json`。legacy camera、円形splat、直列描画は
既定値のままで互換性を維持する。

## 引き継ぎ文書

- [`COLORING_HANDOFF.md`](COLORING_HANDOFF.md) — 着色品質枝 (2026-07-18)
- [`BIM_HANDOFF.md`](BIM_HANDOFF.md) — Scan-to-BIM 枝 (2026-07-11)
- [`colored-map-release-readiness-2026-07.md`](../../docs/research/colored-map-release-readiness-2026-07.md) — 公開判定と制約

チュートリアル: [`docs/3dgs-map-tutorial.md`](../../docs/3dgs-map-tutorial.md)
(フライスルー生成)、[`docs/workflows.md`](../../docs/workflows.md)。

### Camera models and posed-image export

The shared transforms loader uses one intrinsic matrix and image size for all
frames. Per-frame `fl_x`, `fl_y`, `cx`, `cy`, `w`, and `h` may repeat the root
values, but differing overrides are rejected. Split inputs by camera calibration
instead of silently coloring or rendering them with the root camera.

Use `extract_posed_images.py --undistort` for the internal pinhole training,
rendering and recoloring tools. The integrated pipeline does this by default.
Its shared transforms loader rejects nonzero distortion and non-pinhole camera
models, including zero-coefficient fisheye. Clearing JSON coefficients without
rectifying the images does not produce a valid dataset.

Raw export preserves ordinary OpenCV coefficients, or writes fisheye images as
`OPENCV_FISHEYE` with `k1` through `k4`, for compatible external consumers
([Nerfstudio model mapping](https://github.com/nerfstudio-project/nerfstudio/blob/main/nerfstudio/cameras/cameras.py)).
Nonzero rational denominator or extended coefficients cannot be represented by
this exporter: use `--undistort`; raw export fails before writing images rather
than dropping coefficients. This does not add raw-image support to the internal
pinhole tools. For direct bag coloring without resampling the image,
`colorize_from_bag.py --no-undistort` projects using the CameraInfo lens model.

## カメラ姿勢の補正（RTK-SLAM construction_seq1）

K4 採用構成の地図形状（4.9M 点）と posed images（260 view）を固定し、
`refine_camera_poses.py --workers 4`（既定設定、14 分、最大 RSS 1.5 GB）で補正した。

| 指標 | 記録姿勢 | 補正後 |
|---|---:|---:|
| held-out ペアのエピポーラ誤差（中央値 / p90、800x600 px、143 ペア） | 3.82 / 8.58 | 0.69 / 2.46 |
| held-out 色誤差 RGB L2（中央値 / p90） | 40.02 / 151.80 | 37.67 / 150.45 |
| RGB L2 ≤ 20 の割合 | 0.290 | 0.314 |
| 評価 view ごとの中央値 | — | 41/52 view で改善（最大悪化 +2.9） |

補正量は回転 中央値 0.40°（p90 1.56°）、カメラ中心 中央値 3.6 cm（p90 18 cm）。
色誤差は 5 枚に 1 枚（index mod 5 = 1）を評価に残し、残りの 208 枚で K4 の
着色設定のまま着色して、評価画像に投影して測った（両条件とも同じ分割・設定・
形状）。評価画像の姿勢も補正するが、補正には特徴点の対応と LiDAR 形状だけを使い、
地図の色は使わない。画像だけから推定した F 行列の誤差は同じ対応で約 0.34 px なので、
補正後もまだ差が残る。

## README の着色フライスルー（RTK-SLAM Stadtgarten 2）

`lidarslam/images/map_flythrough_stadtgarten.{webp,mp4,gif}` は stadtgarten_seq2 の
20–180 s（公園の約 110 m）から次の手順で作った。`refine_camera_poses.py` の held-out
エピポーラ誤差は 4.02 → 0.67 px（640 view、447 ペア）。

```bash
# 1) 軌跡: construction_seq1 と同じ rko_params（deskew:false、initialization_phase:false）。
#    機材が約 20° 前傾しているため地図全体が傾くが、19 測量点への SE(3) ATE は 1.67 m
#    （initialization_phase:true は 3.23 m）。描画はカメラ軌跡に沿うので傾きは問題にならない。
ros2 run rko_lio offline_node --ros-args --params-file rko_params.ros.yaml \
  -p bag_path:=<stadtgarten_seq2> -p imu_topic:=/livox/imu -p lidar_topic:=/livox/points \
  -p base_frame:=base_link -p dump_results:=true -p results_dir:=<out> -p run_name:=sg2_rko
# 2) posed images
python3 tools/colored_map/extract_posed_images.py --bag <bag> --traj <tum> \
  --camera-topic /camera/image_raw/compressed \
  --intrinsics-yaml configs/gaussian_splatting/rtk_slam_cam0_intrinsics.yaml \
  --extrinsic configs/gaussian_splatting/rtk_slam_cam0_extrinsic.yaml \
  --undistort --time-offset 0 --start-time 20 --end-time 180 --stride 5 \
  --max-extrapolation 0.2 --out <out>/posed
# 3) 形状: 屋外の疎な遠方地面を残すため --min-neighbors 2。一緒に歩く人物の軌跡は
#    動的除去で消す（pip install 'dynamic-object-removal>=0.5'）。
python3 tools/colored_map/build_lidar_init.py --bag <bag> --traj <tum> \
  --points-topic /livox/points --start-time 20 --end-time 180 --voxel 0.015 \
  --min-range 1.5 --max-range 60 --max-points 12000000 --min-neighbors 2 \
  --sparse-voxel 0.1 --dynamic-map-cleaner fusion --out geometry.ply
# 4) 姿勢補正と着色（K4 の着色設定 + 空の色の混入除去）
python3 tools/colored_map/refine_camera_poses.py --transforms <out>/posed/transforms.json \
  --pointcloud geometry.ply --out <out>/posed/transforms_refined.json --workers 4
python3 tools/colored_map/recolor_pointcloud.py --input geometry.ply \
  --transforms <out>/posed/transforms_refined.json --out colored.ply \
  --exposure-scale-limit 1.5 --max-samples 12 --min-samples 3 --image-margin 120 \
  --vignette-gain-limit 2.5 --overlap-balance --view-confidence --normal-voxel 0.12 \
  --sky-rejection
# 5) 描画（CPU、surface splat）と README アセット
python3 tools/colored_map/render_map_flythrough.py --pointcloud colored.ply \
  --transforms <out>/posed/transforms_refined.json --color-mode rgb --frames 240 \
  --fps 30 --point-size 0.03 --scale 0.375 --device cpu --camera-preset cinematic \
  --surface-splat --loop-fade 12 --label "Camera-coloured LiDAR map (RTK-SLAM Stadtgarten 2)" \
  --mp4 master.mp4
ffmpeg -i master.mp4 -vf "fps=15,scale=600:-2:flags=lanczos" -loop 0 \
  -c:v libwebp -quality 78 map_flythrough_stadtgarten.webp   # + crf26 mp4 / palette gif
```

既知の残差: 開始直後に立ち止まっていた人物は動的除去で消えず、点として残る
（`--sky-rejection` で白さは減った）。どの view でも空を背にしか写らない枝は白いまま。
カメラは前向き 1 台なので、経路から外れた範囲は色が付かない。

## 空の色の混入除去（`--sky-rejection`、既定オフ）

屋外では細い枝や樹冠の縁が、姿勢のわずかなずれで枝の間の空に投影され、白や青で
塗られる。`--sky-rejection`（`build_lidar_init.py` と `colored_map_pipeline.py` では
`--color-sky-rejection`）は、視線がカメラの水平線より上を向き（既定 2°、
`--sky-min-elevation-deg`）、画素が「明るく彩度が低い」か「青い」サンプルを空らしい
サンプルとし、点にそれ以外のサンプルがあればそれだけで色を決める。空らしいサンプル
しかない点（白い外壁など）はそのまま。上方向は全 view のカメラ上向きの平均で、地図が
傾いていても使える。どの点に色が付くか、`--min-samples` が数えるサンプル数は変えない。

Stadtgarten 2（上の手順 4 の設定に追加、同じ形状 8,264,141 点）:

| 指標 | なし | `--sky-rejection` |
|---|---:|---:|
| coverage | 0.9417 | 0.9417 |
| chroma_retention | 0.993 | 1.024 |
| roughness median / p90 | 4.60 / 13.35 | 4.56 / 13.10 |
| planar roughness p90 | 12.92 | 12.45 |
| held-out 色誤差、正解画素が空でない点（84 %）median / p90 | 29.4 / 93.0 | 28.3 / 88.0 |
| 同 RGB L2 ≤ 20 の割合 | 0.367 | 0.379 |
| held-out 色誤差、正解画素が水平線より上で空らしい点（16 %）median | 49.6 | 92.3 |

held-out は偶数 view で着色し、奇数 view の 5 枚に 1 枚（64 枚、`--image-margin 140`）で
測った。正解画素が空の点で誤差が増えるのは、ずれた投影先の空を正解とするためで、
枝を空の色で塗った方が一致する。この指標は空の混入を検出できないので、改善は
同一視点グリッドの目視（樹冠と生垣の白斑が減る）と、空でない画素の誤差で確かめた。
