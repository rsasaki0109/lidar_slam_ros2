# 点群着色 品質改善 引き継ぎ (2026-07-18)

BIM 枝の `BIM_HANDOFF.md` と同じ趣旨の引き継ぎ文書。README の
camera-coloured flythrough の色が濁っていた問題を起点に、着色パイプライン・
リアルタイムノード・評価系を一気に改善した一連の作業(lidar_slam_ros2
PR #363/#364/#365/#366、全て develop に merge 済み)の現状・API・ハマり所・
再現手順・次の候補をまとめる。

## 1. 何が問題で、何を入れたか

README の RTK-SLAM flythrough (`lidarslam/images/map_flythrough_rtkslam.webp`)
の着色が「白カブリ + 色滲み + 胡椒ノイズ」だった。原因は3つ:

1. **アセットが着色改善前のコードで生成されていた** — 2026-07-12 20:33 生成の
   直後に edge-aware 補間 (#348) / 観測色 medoid (#347) / 露出ゲインクランプ
   (#349) が merge され、README には未反映のままだった。
2. **レンズビネット** — RTK-SLAM cam0 (1600x1200) は四隅が完全に黒く落ちる。
   縁の暗い/黒いピクセルがそのまま点に投影されていた。
3. **Livox `offset_time` 未対応** — `build_lidar_init.py` のデスキュー対象
   フィールド名に無く、mid360 bag は黙って剛体スキャン扱い
   (歩行 ~1 m/s で最大 ~10 cm のスキャン内スミア)。

入れたもの(全て default-off / default 値で既存挙動バイト同一):

| 機能 | 場所 | PR |
|---|---|---|
| `image_margin`(色サンプルのビネット帯除外) | `pointcloud_io.colorize_by_projection_robust` + `--color-image-margin` | #363 |
| `min_samples`(低観測数の色を unseen に降格) | `build_lidar_init._colorize` + `--color-min-samples` | #363 |
| `offset_time` (整数 ns) デスキュー | `build_lidar_init._read_pointcloud_xyz_time` | #363 |
| CPU レンダラ(CUDA/torch 不要) | `render_path.render_frames_cpu` + `--device cpu` | #363 |
| 上記2オプションの realtime 移植 | `scanmatcher/include/scanmatcher/point_colorizer.hpp`(`insideImageMargin` / `PointColor::confirmed`)+ node param `image_margin` / `min_color_samples` | #364 |
| 見た目指標(appearance) | `scripts/evaluate_colored_map_appearance.py` | #365 |
| held-out のビネット除外 truth | `evaluate_heldout_point_colors.py --image-margin` | #365 |
| appearance のゲート組込み | `check_colored_map_quality.py` `appearance_*` 閾値 + pipeline `appearance` ステージ + exp04 プロファイル | #366 |

## 2. 設計上の要点(触る前に知るべき不変条件)

- **margin の意味論**: z バッファ(遮蔽判定)は**フルフレームのまま**。
  margin は「色サンプルの採否」だけを絞る。縁でしか見えない点は
  他ビューの中央から色を得る。offline (`colorize_by_projection_robust`) と
  realtime (`insideImageMargin`) で同一の意味論に揃えてある — 崩さないこと。
- **min_samples は「降格」**: 色を書き換えるのではなく default_rgb (128,128,128)
  に戻して unseen 扱いにする。`render_map_flythrough --color-mode rgb` は
  unseen を drop する、という既存の流れに乗せている。
- **決定論**: これらのオプションは default(0/1)で従来とバイト同一。
  リポジトリの流儀として default-off を守る。
- **この ICP/描画系に GPU は無い**(マシン sasaki-pc は Iris Xe、torch 無し)。
  `render_frames(device='cpu')` が `render_frames_cpu` にディスパッチする。
  仕組み: 量子化深度と点 ID を int64 にパック → `np.minimum.at` 一発で
  ピクセル毎の可視性と勝者 ID を同時解決 → 2x supersample を box 縮小。
  ~0.7 s/frame @600x450, 4.8M 点。gsplat の soft splat と違い不透明ディスク。

## 3. 評価プロトコル(これで測ること)

**3点セット**で評価する。単独指標は騙される:

1. **忠実度**: `evaluate_heldout_point_colors.py --use-pointcloud-colors
   --image-margin <px>`。**非マスク版はビネット画素を正解として数えるため
   margin 付き着色を誤って劣位判定する**(RTK 実測: 非マスク 40.8 vs 42.7 で
   margin なし優位 → マスク版 43.5 vs 40.1 で反転、inlier_20 0.257→0.296)。
2. **見た目**: `evaluate_colored_map_appearance.py` —
   `chroma_retention`(マップ彩度÷画像彩度; washout で <1、モノクロ画像は null)、
   `roughness`(voxel 内色標準偏差 median/p90; 組織化テクスチャは低く
   胡椒ノイズは高い)、`coverage`(点を捨てるチート防止)。
   exp04 で視覚順位を再現済み: 旧着色→再着色→+margin で
   rough_p90 18.5→17.6→12.3、coverage 0.768→0.999。
3. **目視**: `render_map_flythrough.py --test-grid 4 --device cpu` の
   同一視点グリッド比較。

ゲート: `check_colored_map_quality.py --appearance-report ...`。
`configs/colored_map_quality_profiles/hilti_exp04_report_only.yaml` に
report-only の較正済み閾値あり(coverage≥0.95 / rough_med≤2.5 / rough_p90≤15。
旧着色は3つとも violation、現行+margin は余裕 PASS)。

## 4. 採用構成(README アセットの正)

RTK-SLAM construction_seq1、窓 480–545 s(60 m 歩行ループ)、RKO-LIO 軌跡:

```bash
# 1) 軌跡 (CS2 実績の rko_params.ros.yaml を流用、deskew:false / dual_downsample)
timeout -s INT 1800 ros2 run rko_lio offline_node --ros-args \
  --params-file rko_params.ros.yaml -p bag_path:=<construction_seq1> \
  -p imu_topic:=/livox/imu -p lidar_topic:=/livox/points -p base_frame:=base_link \
  -p dump_results:=true -p results_dir:=<out> -p run_name:=seq1_rko
# 2) posed images (time-offset 0 で良い; Kalibr -20.6ms は有意差なしを実測済み)
python3 tools/colored_map/extract_posed_images.py \
  --bag <bag> --traj <tum> --camera-topic /camera/image_raw/compressed \
  --intrinsics-yaml configs/gaussian_splatting/rtk_slam_cam0_intrinsics.yaml \
  --extrinsic configs/gaussian_splatting/rtk_slam_cam0_extrinsic.yaml \
  --undistort --time-offset 0 --start-time 480 --end-time 545 --stride 5 \
  --max-extrapolation 0.2 --out <out>/posed
# 3) 着色点群 (採用パラメータ)
python3 tools/colored_map/build_lidar_init.py \
  --bag <bag> --traj <tum> --points-topic /livox/points \
  --start-time 480 --end-time 545 --voxel 0.015 --min-range 1.5 --max-range 60 \
  --max-points 5000000 --min-neighbors 8 --sparse-voxel 0.1 \
  --color-transforms <out>/posed/transforms.json --color-robust \
  --color-image-margin 140 --color-min-samples 3 --out colored.ply
# 4) レンダ + README アセット
python3 tools/colored_map/render_map_flythrough.py \
  --pointcloud colored.ply --transforms <out>/posed/transforms.json \
  --color-mode rgb --frames 240 --fps 30 --point-size 0.02 --scale 0.375 \
  --loop-fade 12 --device cpu --mp4 master.mp4
ffmpeg -i master.mp4 -vf "fps=15,scale=600:-2:flags=lanczos" -loop 0 \
  -c:v libwebp -quality 78 map_flythrough_rtkslam.webp   # + crf26 mp4 / palette gif
```

## 5. データ・成果物の所在

- **RTK-SLAM bag**: `/media/sasaki/aiueo/datasets/rtk_slam/ros2/construction_seq1`
  (13,180,936,192 bytes)。**注意**: 一度 8.0/13.2 GB で切断破損していた
  ("database disk image is malformed")。`wget -c` (`scripts/download_rtk_slam_dataset.py`
  の URL) で再開修復できる。**使う前にサイズを SEQUENCES の期待値と照合**。
- **今回の RTK 成果物一式**:
  `/media/sasaki/aiueo/benchmarks/rtkslam_seq1_colored_map_20260718/`
  (RKO-LIO TUM 軌跡 `results/seq1_rko_0/`、posed_t0(260 views)、
  colored_{A,B,D}.ply(A=margin なし/B=margin140/D=採用構成)、
  heldout/app/align の各 json、flythrough_master.mp4、新旧比較 PNG)。
  D が README アセットの元。
- **exp04 検証一式**:
  `/media/sasaki/aiueo/datasets/public_validation/hilti_exp04_colored_map_recolor_20260718/`
  (README.md に A/B 表、旧着色との比較 ply、heldout/app json)。
- README アセット: `lidarslam/images/map_flythrough_rtkslam.{webp,mp4,gif}`。
- 経緯の一次資料: `docs/research/3dgs-trajectory-flythrough-notes.md` 追補3、
  `docs/3dgs-map-tutorial.md` の成果物例(再現コマンド)。

## 6. ハマり所(実際に踏んだもの)

- **非マスク held-out を信じない**(§3)。ビネット除外の効果測定には
  `--image-margin` を正解側にも付けるか appearance 指標を使う。
- **colcon の同名パッケージ衝突**: ws に `lidar_slam_ros2` と
  `lidar_slam_ros2_candidate2` が並存し scanmatcher が重複。
  ビルドは **ws ルートから** `source install/setup.bash` 後に
  `colcon build --base-paths lidar_slam_ros2_candidate2 --packages-select scanmatcher`。
  ws の `build/scanmatcher` キャッシュが旧 repo 由来だと
  "does not match the source ... used to generate cache" で落ちる → 消して再生成。
- **offline_node は bag 終端でハング**(仕様)→ `timeout -s INT`。
- **`point_colorizer.hpp` は純ヘッダ** — g++ 単体
  (`g++ -std=c++17 -I scanmatcher/include -I /usr/include/eigen3 ... -lgtest`)
  で colcon なしに即テストできる。
- **posed images のファイル名は `00000.png` 形式**(`frame_XXXXX.png` ではない)。
- **appearance の chroma_retention はモノクロ画像で null** — プロファイルに
  閾値を書くなら色付きリグのみ。
- **exp04 の posed images はグレースケール**(alphasense)。色の議論には
  RTK-SLAM か AIST bag (`/media/sasaki/aiueo/benchmarks/aist_rgb_map/`) を使う。

## 7. 次の候補(未着手、優先度順の私見)

1. **RTK-SLAM 用 colored_map_quality プロファイル**: 色付きリグなので
   `appearance_chroma_retention_min`(~0.9)を初めて有効化できる。
   `benchmarks/rtkslam_seq1_colored_map_20260718/` の json がそのまま較正データ。
2. **ビネットの推定補正**: margin は「捨てる」対策(着色数 -8%)。多数ビューの
   輝度中央値 vs 画像半径から radial gain を推定して「直す」方式なら
   coverage を犠牲にせず縁まで使える。`_sample_pixels` の手前に挟むのが自然。
3. **roughness の平面限定版**: 現行は全 voxel。`project_planar_voxels` で
   平面 voxel に限定すればテクスチャ境界の偽陽性が減り、閾値をさらに絞れる。
4. **CPU レンダラの soft 化**: 不透明ディスクゆえ未マップ領域の黒い隙間が
   gsplat よりも目立つ。supersample 3 化 or 半径依存の soft edge。
5. **リアルタイムノードの実 bag 検証**: #364 は unit test + build まで。
   all-sensors-bag1(`/home/sasaki/autoware_data/all-sensors-bag1`、
   lucid camera_0=camera_top/camera_optical_link、QoS は SensorDataQoS 対応済み)
   で live A/B するとよい。
6. **エクスポート枝への波及**: mesh/LAS/GIS(`mesh_export.py` 等)は改善前の
   色で検証されたまま。D 構成の ply で再検証すると数値が上がるはず。

## 8. 関連 PR / 経緯リンク

- #345/#346–#349: 旧アセット生成と直後の colorizer 改善(すれ違いの原因)
- #363: 着色品質改善 + README アセット再生成(比較画像は PR 参照)
- #364: realtime 移植 + exp04 検証
- #365: appearance 指標 + masked held-out
- #366: appearance のゲート組込み(本文書を含む)

## 9. 追記 (2026-07-18): tools/colored_map/ へ分離

着色は gsplat と無関係、という指摘を受けて本文書を含む着色・エクスポート系
14 モジュールを `tools/gaussian_splatting/` から `tools/colored_map/` へ移動した。
旧パスには **sys.modules リダイレクト式 shim** があり、旧来の
`sys.path.insert(gs_dir)` + `import pointcloud_io` も
`python3 tools/gaussian_splatting/build_lidar_init.py` も動き続ける
(shim は spec 名を先に `sys.modules` へ登録すること — dataclass が
`sys.modules[cls.__module__]` を引くため、忘れると collection で落ちる)。
3DGS 側に残る `train_gsplat` / `render_path` / `render_slam_3dgs_sidebyside`
への依存は、移動側 3 ファイル (build_lidar_init / render_map_flythrough /
colorize_planar_references) の先頭 bootstrap が解決する。
tests / scripts の旧パス参照は shim で無改修動作 — 新規コードは
`tools/colored_map/` を直接参照すること。
