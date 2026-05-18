# lidarslam-ros2 総合計画書

## 1. プロジェクト概要

### ゴール
MIT/BSD ライセンスで、Autoware ユーザーが使える高品質な LiDAR SLAM マッピングツール。
- GPL 汚染なし（商用利用可能）
- Autoware の `pointcloud_map_loader` 互換の PCD マップ出力
- GNSS 連携による地理座標系マッピング
- RKO-LIO フロントエンド + graph_based_slam ループクロージャーバックエンド

### 現在の状態
PR #2 (Ready for Review): https://github.com/rsasaki0109/lidarslam_ws/pull/2

---

## 1.1 追加トラック（2026-04）：KITTI Odometry で勝つための LO / small_gicp チューニング

### 目的
- **KITTI Odometry (Velodyne only, IMU なし)** で動く **LiDAR Odometry (LO) モード**を用意し、フロントエンドのパラメータをスイープして APE を詰める。
- 既存の既定パイプライン（RKO-LIO + graph_based_slam）を壊さずに、**追加の評価軸**として LO を整備する。

### 追加パイプライン（現状）
- **LO baseline**: `scanmatcher_node`（IMU 無効） + `graph_based_slam`  
  - launch: `lidarslam/launch/lo_slam.launch.py`
  - params: `lidarslam/param/lidarslam_lo.yaml`
  - bench: `scripts/run_lo_graph_benchmark.sh`
- **small_gicp LO**: `small_gicp_odom_node` + `graph_based_slam`  
  - launch: `lidarslam/launch/small_gicp_lo_slam.launch.py`
  - params: `lidarslam/param/small_gicp_kitti_velodyne.yaml`
  - bench: `scripts/run_small_gicp_graph_benchmark.sh`
  - sweep: `scripts/sweep_kitti_small_gicp.sh`

### ここまでの実行で見えた「ハマりどころ」（重要）
このセッションで、スクリプトは起動できたが **TUM が空（traj_raw.tum / traj_corrected.tum が 0 bytes）**になりやすい状況が確認できた。
主因は概ね次の 2 つ。

#### (A) rosbag2 play と購読 QoS の不一致（BestEffort vs Reliable）
- `small_gicp_odom_node` の購読は `rclcpp::SensorDataQoS()`（一般に **BestEffort**）。
- `ros2 bag play` の publisher 側が **Reliable** になり、接続が成立せず **コールバックが一度も来ない**ことがある。
- 目視の兆候:
  - `ros2 bag info` では `/kitti/velodyne/points` にメッセージが存在する
  - しかし `odom_to_tum` は “Subscribed ...” のログだけで、ファイルが増えない
  - `small_gicp_odom_node` 側ログは起動・終了しか残らない

**対策（実装済み）**
- `ros2 bag play --qos-profile-overrides-path <yaml>` を使い、`INPUT_CLOUD` の QoS を **best_effort + volatile** に固定する。
- `scripts/run_small_gicp_graph_benchmark.sh` / `scripts/run_lo_graph_benchmark.sh` が各出力ディレクトリに `rosbag2_play_qos.yaml` を生成して渡す。
- launch と logger の起動・初期化を待ってから bag 再生を始める。これで冒頭フレーム取り逃がしによる空 TUM を避ける。

#### (B) “graph の corrected が取れない” ＝ 失敗ではなく「短すぎる」場合がある
- 2 フレーム程度の短い bag だと `graph_based_slam` が `/modified_path` を出す前に終わることがある。
- その場合でも **フロントエンド odom（/small_gicp/odom）** が記録できていれば評価は可能。

**対策（実装済みの方針）**
- `traj_corrected.tum` が無い場合に `traj_raw.tum` をコピーして “corrected” として後段評価を続行する（スモーク用のフォールバック）。
- 本番スイープでは “corrected を必須” に戻すか、別指標として raw/corrected を分けて扱う。

#### (C) launch のパラメータ上書き順の罠
- `small_gicp_lo_slam.launch.py` の `parameters=[dict(overrides), param_file]` の順だと、**param_file が最後に勝って override が効かない**。
- override を効かせるには **param_file → overrides の順**にする必要がある。

**対策（実装済み）**
- `small_gicp_lo_slam.launch.py` の parameters の順序を調整し、CLI override が最優先で勝つようにした。

### スイープのやり方（引き継ぎ用・再現コマンド）
#### 最小（まず 00 だけ）
```bash
# KITTI のルートを指定（odometry dataset 構造）
bash scripts/sweep_kitti_small_gicp.sh --dataset /path/to/KITTI_odometry --sequences "00"
```

#### 複数（00/05/07）
```bash
bash scripts/sweep_kitti_small_gicp.sh --dataset /path/to/KITTI_odometry --sequences "00 05 07"
```

### 次にやること（TODO）
- `scripts/sweep_kitti_small_gicp.sh` の `CONFIGS` を広げる:
  - `ds`（downsampling_resolution）
  - `voxel`（IncrementalVoxelMap の voxel）
  - `corr`（max_correspondence_distance）
  - `range`（min/max）
  - `use_gicp`（ICP vs GICP）※GICP は共分散計算が重いので最後に
- ユーザー導線:
  - 公開推奨は引き続き **RKO-LIO + graph_based_slam**。
  - KITTI / Velodyne-only は評価・チューニング用の導線として README / docs から `download_kitti_odometry.sh` → `run_kitti_odometry_benchmark.sh --small-gicp` → `sweep_kitti_small_gicp.sh` に誘導する。
  - `datasets/`, `map.pcd`, `map_projector_info.yaml`, `pointcloud_map/` はローカルデータ/生成物として Git 管理外に置く。
- 成功条件:
  - `traj_raw.tum` が non-empty
  - `ape_raw_vs_gt.txt` が生成される（少なくとも raw 側）
  - スイープ後に `benchmark_summary.py` の md/csv が出る

---

## 1.2 追加トラック（2026-05）：STD/BTC 風 Triangle Descriptor 自前実装

### 目的
- **MID-360 のような narrow-FOV / 短距離 LiDAR** で Scan Context が縮退するケースに備え、edge-length 不変な三角形ハッシュ系の place recognition を **BSD-2 自前実装** で導入する。
- STD (Stable Triangle Descriptor) / BTC (Binary Triangle Code) の原理だけ拾い、GPLv2 / ライセンス不明な公式実装には触れずに書く（[[project_place_recognition_license]] 参照）。
- 既存の BEV / Scan Context / SOLiD と並列に「もう 1 段重ねる」選択肢として置く。default workflow には影響を与えない opt-in 機能として落とす。

### 投入した 13 PR（develop マージ済 / 2026-05-18 時点）

| PR | コミット | 内容 |
|----|---------|------|
| #135 | `f4b54ba` | BEV keypoint 抽出 / 三角形列挙 / Umeyama SVD 3点 SE(3)（gtest 15） |
| #136 | `9b9e19d` | quantizeEdges hash / TriangleDatabase / accumulateVotes / RANSAC findLoopCandidate（gtest 15） |
| #137 | `146002f` | `use_triangle_descriptor` + 関連 14 ROS パラメータ / `searchLoop` 配線 |
| #138 | `013ed10` | YAML preset（generic 360° + MID-360 短距離向け） |
| #139 | `4e775aa` | `generate_place_recognition_report.py` に triangle 統計を追加 |
| #140 | `ccd8b03` | 任意の BEV mutual-visibility cross-verify ゲート |
| #141 | `9abeef8` | 1 コマンド ablation runner `scripts/run_triangle_ablation.sh` |
| #142 | `571783e` | runner のログ拾いバグ修正 + report ヘッダ汎用化 |
| #143 | `e89f322` | triangle SE(3) を NDT 初期値として配線 + デフォルト閾値引き上げ |
| #145 | `b6b95de` | v4 ablation knobs を default 化: grid_cells 60→100, salience 0.3→0.8, max_keypoints 80→40, edge_bin 1.0→0.5, max_triangles 5000→3000, votes 10→6, inliers 5→4 |
| #146 | `cdaa22f` | `min_inlier_ratio` / `max_pairs` ROS パラメータ追加: 絶対 inlier 数だけでなく相対密度フロアでもゲートできる |
| #147 | `73ecf19` | 4-point consensus gate: 3-point RANSAC で選ばれた SE(3) に対し triangle 頂点以外の query keypoint を写して db 側 keypoint と一致するかをチェック。TriangleDatabase に submap-level keypoint 全保持を追加 |

### Pipeline 全体図
1. **keypoint 抽出**: BEV 投影 max-height local-maximum をキーポイント候補（`extractKeypointsBEV`）。
2. **三角形列挙**: 全 3-tuple から edge length が `[min_edge_m, max_edge_m]` に収まるものを抽出、edge sort 後に descriptor 化（`buildTriangles`）。
3. **ハッシュ**: 3 edge を `edge_bin_m` で量子化、`packHash` で uint64 化（`quantizeEdges` → `packHash`）。
4. **DB**: submap_id ごとに `(hash → DatabaseEntry[])` を保持。Entry は 3 頂点座標も含む（後段の幾何検証用）。
5. **投票**: query submap の三角形を hash ルックアップ、submap_id ごとに票を加算（`accumulateVotes`）。
6. **RANSAC 検証**: 最高得票の submap に対し、マッチした三角形ペアそれぞれから Umeyama で SE(3) を出し、他のペアと consensus を取る（`findLoopCandidate`）。
7. **NDT 初期値**: triangle 由来の SE(3) を NDT/GICP の initial guess として使う（PR #143）。
8. **オプション**: BEV mutual-visibility distance と AND ゲート（PR #140）。

### NTU VIRAL tnp_01 ablation（5 ラウンド）

`scripts/run_triangle_ablation.sh` で同一バッグを `use_triangle_descriptor` のみ切替えて 2 回回し、`generate_place_recognition_report.py --candidate-kind triangle_descriptor` で diff。

| ラウンド | 設定 | baseline APE | candidate APE | Triangle 候補 emit | Triangle 採用 | distance loop (cand/base) | 解釈 |
|---------|------|-------------|---------------|-------------------|--------------|--------------------------|------|
| v1 | min_inliers=3, min_votes=6, NDT初期値=pose+yaw | 1.440 m | 1.602 m | 4 | 0 | 21 / 26 | distance loop が dedup で押し出されて悪化 |
| v2 | min_inliers=5, min_votes=10, NDT初期値=triangle SE(3) | 1.509 m | 1.418 m | 0 | 0 | — | 全部閾値で弾かれ、変化なし |
| v3 | min_inliers=3, min_votes=6, NDT初期値=triangle SE(3) | 1.444 m | 1.497 m | 10 | 0 | — | SE(3) 初期値投入しても NDT 通らず |
| v4 | **keypoint tightening** (grid_cells 100, salience 0.8, max_kp 40, edge_bin 0.5) | 1.271 m | 1.312 m | 4 | **1** ✨ | 11 / 17 | 初の triangle 採用 (id=32 ↔ 95, 補正 0.49 m/1.06°)。ただし distance loop は依然減 |
| v5 | v4 + `min_inlier_ratio=0.15` + `max_pairs=24` + `min_4th_point_agreements=3` | 1.500 m | 1.478 m | **2** (半減) | 1 | **19** / 17 | 偽陽性 emit を半減しつつ legit 採用は維持、distance loop の押し出しも解消 (cand > baseline) |

**run-to-run variance** は ~0.1 m あり、APE 単独で確信は持てない。ただし **v5 で「triangle on にしても distance loop が減らず、triangle 由来 loop が 1 件追加で APE は variance 内で baseline 以下」** という形が初めて再現。これは「triangle stack が非破壊的に追加情報を提供する」状態に到達したことを意味する。

### 決定的な観察 (v1〜v3 時点)

`Triangle loop candidate:` ログを candidate run から並べると、**同じ submap_id=5 ペアで連続呼び出しの yaw 推定値が 51° → 100° → 123° → 130° → 145° と乱高下**していた。同一サブマップ間の真の相対 yaw は本来一つに収束すべきところ、毎回違う SE(3) が出ている。

つまり:
- Hash bucket（edge length match）の vote 自体は強い（id=5 が 200〜2300 票）
- 3 点対応付け段階で **keypoint extraction のノイズ**が支配的になり、RANSAC が偶然合意する 3〜4 inliers でランダムな SE(3) を出力している
- そのランダム SE(3) を NDT 初期値に入れても NDT が収束しない

### v4 / v5 で解消された段

v4 の keypoint tightening と v5 の inlier_ratio + 4-point gate を経て、根本原因の段別評価は次のように更新:

| 段 | v1〜v3 評価 | v5 評価 |
|----|------------|---------|
| Hash bucket matching (edge length) | ✅ OK | ✅ OK |
| 3 点対応付け（誰がどの頂点か） | ❌ keypoint ノイズで破綻 | ⚠️ tightening で id=5 false-bucket は解消、ただし依然 noisy |
| SE(3) 復元（Umeyama） | ⚠️ ノイズ入力でノイズ出力 | ⚠️ 改善はしたが完全ではない |
| 3-point inlier consensus | （単独で機能不全） | ✅ `min_inlier_ratio` で相対密度ゲートできる |
| 4-point consensus | （未実装） | ✅ triangle 頂点外の keypoint で SE(3) を独立検証 |
| NDT 検証 | ⚠️ noisy 初期値で発散 | ✅ tightening + 4-point gate を通った SE(3) は NDT 収束する (v4/v5 で 1 件 / 2 件 emit 中 1 件 accept) |

### 学び（v1〜v5 通算）

- **Hash voting と RANSAC SE(3) recovery は別レベルの難しさ**を持つ。「票が集まる」と「同じ submap を見ている」とは言えるが、「ある三角形 A が三角形 B にどう対応するか」までは情報が足りない。
- **デフォルト閾値を厳しくしただけでは救えない**。v2 のように閾値を上げると 1 つも emit しなくなり、検証データが取れなくなる。
- **SE(3) を NDT 初期値に入れただけでは救えない**。v3 のように noisy SE(3) を渡しても NDT 収束半径外。
- **keypoint 抽出の質が支配的だった**。v4 で salience filter + edge_bin 縮小だけで「同じ submap に全部寄る」failure mode が解消、初の採用が出た。
- **3-point RANSAC は最小自由度ゆえ偶然合意しやすい**。v5 の 4-point consensus + inlier_ratio で偽陽性 emit を半減できることを実証した。
- **run-to-run variance ~0.1 m を見落とすと改善誤判定する**。1 回比較で「APE 改善した」と判定するのは早計。

### 3-Dataset 検証結果と Default 判断 (2026-05-18 更新)

| Dataset | LiDAR | 環境 | Votes/submap | Inliers | Triangle 採用 |
|---------|-------|------|--------------|---------|--------------|
| NTU VIRAL tnp_01 | Ouster OS1 (360° wide vertical) | outdoor open | 200-2800 | 3-5 | 1 (v5, variance 内 APE 改善) |
| MID-360 glim | Livox MID-360 (narrow FOV) | outdoor mixed | 97-1037 | 1-2 | 0 (2 周試行, ともに 0 emit) |
| Newer College math_hard | Ouster OS0-32 (360° narrow vertical) | indoor | 100-200 | 1-2 | 0 (1 周試行, 76 votes 全件 reject) |

**結論:** Triangle descriptor (BEV max-height keypoint + 3-point RANSAC) は **spinning 360° + wide vertical FOV + outdoor** に限定して機能する。narrow FOV (MID-360) または indoor 構造化シーン (Newer College math_hard) では keypoint repeatability が破綻し、Hash votes は集まるものの 3-point RANSAC inliers が 1-2 で頭打ちになる。

**Default 設計判断:**
- `use_triangle_descriptor: false` を **全 preset で維持** — NTU 単独の 1 採用は variance 範囲内、複数 dataset で価値示せず default on の根拠なし
- `min_4th_point_agreements: 0` を **維持** — 4-point gate は NTU 以外では発火前に inlier gate で死ぬため検証不能、デフォルトを上げる根拠なし
- これらは「opt-in 研究機能」として develop に landing 済。production default に上げるには別の keypoint 抽出方式が必要

### 残った次の打ち手（優先度順）

1. **MID-360 / indoor 向け keypoint 抽出の再設計**
   - 現実装の BEV max-height では narrow FOV / indoor の repeatability 不足が 3 dataset で確定
   - 候補: corner/edge keypoint (FPFH 風), density 変化点 (PointNet++ 系), planar/non-planar 分類
   - もしくは triangle stack を「OS1/OS0 wide vertical FOV 専用」として明示し、MID-360 / indoor 用には Scan Context / SOLiD など別系統に委ねる
2. **NTU v5 reproducibility — 2-3 周回して variance 内に APE 改善が安定するか確認**
   - 現状 1 採用は 1 回観測 (v5)。v5 を 3 周回して採用ペアが (a) 毎回出るか (b) 毎回同じ submap_id か検証
   - 安定すれば NTU プリセット限定で `use_triangle_descriptor: true` も検討余地あり
3. **Leo Drive driving bag への展開** (要 PointCloud2 化と reference 整備)
4. **コンポーネント単体テスト** (searchLoop に triangle path をモック注入する gtest)

### ステータスと運用方針

- triangle descriptor stack は「**実装完了・1 データセットで PoC 効果・2 データセットで非効果**」段階。v0.4 リリースでは引き続き default off (`use_triangle_descriptor: false`) で opt-in 機能として提供。
- v0.4 release notes には「STD/BTC 風 place recognition の opt-in 実装あり、NTU VIRAL tnp_01 で 1 採用ループ確認 (variance 範囲)、MID-360 / Newer College では発火せず — wide-FOV spinning 360° outdoor 限定の研究機能」と書く。
- 4-point gate と `use_triangle_descriptor` の default 判断は 3-dataset 検証によりクローズ済。次は keypoint 抽出の根本見直しか、対応 LiDAR/環境を明示しつつ機能領域を狭く維持。

詳細メモは [[project_triangle_descriptor_stack]] に保存済。

---

## 2. ベンチマーク結果

### 2.1 Newer College math-hard (320m, Ouster OS0-128, IMU あり)

#### LIO + ループクロージャー

| 順位 | 手法 | RMSE (m) | ライセンス | 備考 |
|------|------|----------|-----------|------|
| 1 | DLIO | 0.070 | MIT | 最良精度だが DDS 問題で他ノードと共存不可 |
| 2 | **RKO-LIO + loop closure** | **0.078** | MIT | graph_based_slam, info=1000, Scan Context |
| 3 | RKO-LIO raw | 0.082 | MIT | ループ補正なし |

#### LO (LiDAR-Only)

| 順位 | 手法 | RMSE (m) | ライセンス | 備考 |
|------|------|----------|-----------|------|
| 1 | GenZ-ICP (tuned) | 0.112 | MIT | planarity=0.5, deskew=true, 再現性にバラつき |
| 2 | KISS-ICP | 0.440 | MIT | 安定、リファレンス |
| 3 | lidarslam NDT baseline | 24.286 | BSD | 元の baseline |

### 2.2 NTU-VIRAL tnp_01 (580s, Ouster OS1-16, VN-100 IMU)

| 手法 | RMSE (m) | ループ | 備考 |
|------|----------|--------|------|
| RKO-LIO raw | 1.246 | - | |
| **RKO-LIO + loop closure** | **0.869** | 1回 | 30% 改善 |
| RKO-LIO + loop closure (14回) | **1.314** | 14回 | 検証実行 |

### 2.3 MID-360 (277s, Livox MID-360, 内蔵 IMU, vs GLIM 参照)

| 手法 | RMSE vs GLIM (m) | ループ | 備考 |
|------|-------------------|--------|------|
| RKO-LIO raw | 10.3 | - | |
| RKO-LIO + loop closure (best) | **4.00** | 1回 | info=100, threshold=15.0 |

**MID-360 の限界**: 非360 FOV のため Scan Context 無効、中間ドリフトの補正にループが不足。

---

## 3. 実装済み機能

### 3.1 graph_based_slam 改善

| 機能 | 状態 | 説明 |
|------|------|------|
| Odometry 直接入力モード | ✅ | `use_odom_input` で RKO-LIO/DLIO の Odometry を直接受信 |
| Cloud-driven サブマップ生成 | ✅ | Odom + Cloud の同期サブマップ作成 |
| GPL フリー Scan Context | ✅ | IROS 2018 論文からフルスクラッチ実装 |
| BSD-2 Triangle Descriptor stack | ✅ (opt-in) | STD/BTC 風 keypoint+hash+RANSAC+SE(3) initial guess を自前実装。default off。詳細は §1.2 |
| BEV mutual-visibility cross-verify | ✅ (opt-in) | triangle 候補を BEV mutual visibility distance で AND ゲート |
| Robust kernel 切替 | ✅ | Huber / DCS / Cauchy をパラメータで切替（`loop_edge_robust_kernel_type`） |
| PCD ディスクキャッシュ | ✅ | OOM 対策、サブマップを逐次 PCD 保存 |
| 情報行列バグ修正 | ✅ | ループエッジを固定重み、オドメトリエッジに `adjacent_edge_info_weight` |
| 隣接エッジ情報重み auto-scale (Level 1) | ✅ (opt-in) | NIS median トラッキングで `adjacent_edge_info_weight` を EMA 自動調整 |
| IMU 回転制約 | ✅ | ジャイロ積分でロール・ピッチ制約 |
| GNSS 位置制約 | ✅ | NavSatFix → ENU 変換 → ユナリエッジ (未テスト) |
| Autoware グリッド PCD 出力 | ✅ | `pointcloud_map_metadata.yaml` + 分割 PCD (検証済み) |
| `map_projector_info.yaml` | ✅ | GNSS 原点の地理座標出力 (未テスト) |

### 3.2 scanmatcher 改善

| 機能 | 状態 | 説明 |
|------|------|------|
| 非単調タイムスタンプスキップ | ✅ | ROS2 bag 再生時の時刻逆転対応 |
| VoxelHashMap | ✅ | KISS-ICP 着想のボクセルマップ |
| 適応閾値 | ✅ | EMA ベースの correspondence distance 自動調整 |
| FAST_GICP / SMALL_GICP | ✅ | オプショナル依存 (`#ifdef` ガード) |
| cloud_queue_depth | ✅ | キュー深度パラメータ化 |

### 3.3 インフラ

| 機能 | 状態 | 説明 |
|------|------|------|
| `rko_lio_slam.launch.py` | ✅ | RKO-LIO + graph_based_slam 統合ランチファイル |
| `verify_autoware_map.py` | ✅ | Autoware 互換性検証スクリプト |
| `odom_to_tum.py` / `path_to_tum.py` | ✅ | 軌跡ロギングツール |
| CI ローカルビルド | ✅ | 全パッケージビルド + テスト 25/25 パス |
| README | ✅ | ベンチマーク結果、Autoware 使い方、パラメータ一覧 |

---

## 4. 各手法の深掘り分析

### KISS-ICP — なぜ LO 系で安定か

- **VoxelHashMap**: tsl::robin_map で O(1) ルックアップ、sub-voxel 距離チェック
- **27近傍探索**: 3x3x3 ボクセルキューブ、KDTree 不要
- **Robust kernel**: `w = σ² / (σ² + r²)` で外れ値自動排除
- **Adaptive threshold**: motion model error RMS で `τ = 3σ`
- **Constant velocity prediction**: 収束が速い
- **処理速度**: 20-30 fps、共分散計算なし

### GenZ-ICP — チューニング結果

- **最良設定**: `voxel_size=0.5, planarity=0.5, deskew=true`
- **結果**: RMSE 0.112m (KISS-ICP の 0.440m を大幅に上回る)
- **問題**: 再現性にバラつき (0.112〜0.146m)、rate や DDS 状態に依存
- **voxel_size=0.4 以下は劣化**、0.6 以上は発散

### DLIO vs RKO-LIO

| 要素 | DLIO (0.070m) | RKO-LIO (0.082m) |
|------|---------------|-------------------|
| IMU 統合 | Jerk ベース 3次連続モデル | 定加速度 + カルマンフィルタ |
| デスキュー | 各点ごとの SE(3) 補間 | フレーム境界間の補間 |
| マッチング | NanoGICP (共分散あり) | カスタム point-to-plane ICP |
| マップ | キーフレーム + 凸/凹ハル | Bonxai 疎ボクセルグリッド |
| **問題** | **DDS メッセージ遅延で他ノードと共存不可** | **安定、offline_node で統合成功** |

---

## 5. ライセンス調査

### 使える (MIT/BSD + ROS2 対応)

| 手法 | 分類 | ライセンス |
|------|------|-----------|
| KISS-ICP | LO | MIT |
| GenZ-ICP | LO | MIT |
| small_gicp | 登録ライブラリ | MIT |
| DLIO | LIO | MIT |
| RKO-LIO | LIO | MIT |

### ライセンス NG

| 手法 | ライセンス |
|------|-----------|
| FAST-LIO2 / Faster-LIO | GPLv2 |
| LIO-SAM | BSD だが GTSAM Jazzy 互換問題 |
| LiLi-OM / MULLS / MOLA | GPL |

---

## 6. Autoware 対応状況

### 検証済み ✅

| 項目 | 状態 | 詳細 |
|------|------|------|
| グリッド分割 PCD | ✅ PASS | 20x20m セル、binary_compressed |
| `pointcloud_map_metadata.yaml` | ✅ PASS | `filename.pcd: [int, int]` 形式、Autoware の yaml-cpp パーサー互換 |
| PCD ヘッダー | ✅ PASS | v0.7, FIELDS x y z intensity, float32 |
| orphan ファイル防止 | ✅ PASS | 出力前にディレクトリクリーンアップ |
| `map` フレーム座標系 | ✅ | REP-105 準拠 |

### 未検証 ⚠️

| 項目 | 状態 | 理由 |
|------|------|------|
| GNSS ポーズグラフ制約 | ⚠️ | 手元に有効な GNSS 付きデータセットがない |
| `map_projector_info.yaml` | ⚠️ | GNSS 未動作のため出力されず |
| Autoware 実環境読み込み | ⚠️ | Autoware 未インストール |

### Autoware ユーザーへのバリュー

1. **MIT ライセンスの SLAM** — LIO-SAM (GPL) の代替として商用利用可能
2. **ループクロージャー付き高品質マップ** — ドリフト補正済み PCD
3. **`pointcloud_map_loader` 直接互換** — 変換ツール不要
4. **GNSS 連携** (実装済み、テスト待ち) — 地理座標系マッピング

---

## 7. 既知の問題と制限

### 7.1 DDS メッセージ遅延
- **影響**: DLIO が他ノードと共存できない、online_node でスキャンドロップ
- **原因**: 大きな PointCloud2 メッセージ (6MB+) の DDS 転送遅延
- **回避策**: offline_node (RKO-LIO) でバッグを内部読み込み
- **根本解決**: FastDDS のシェアードメモリ設定、またはゼロコピー転送

### 7.2 MID-360 (固体 LiDAR) の限界
- 非 360 FOV のため Scan Context が無効
- 中間ドリフトの補正にループクロージャーが不足
- RMSE 4.0m (vs GLIM) が現状の限界
- BSD-2 自前実装の STD/BTC 風 triangle descriptor を 2026-05 に投入。NTU VIRAL ablation v4 で初の triangle 採用 (id=32↔95, 補正 0.49m/1.06°)、v5 で 4-point gate + inlier_ratio による偽陽性 emit 半減 (4→2) と distance loop 押し出し解消を確認。詳細は §1.2。default off の opt-in 機能として develop に landing 済。次の段は MID-360 demo bag 整備 → 同じ ablation を MID-360 でも回すこと。

### 7.3 GenZ-ICP の再現性
- DDS のメッセージ配送タイミングに結果が依存
- 同一設定で 0.112m〜26m の幅がある
- offline 実行モードが必要

### 7.4 small_gicp オドメトリの処理速度
- IncrementalVoxelMap の NN 探索が律速
- 共分散計算のオーバーヘッドでスキャンドロップ多発
- ICPFactor への切替で改善可能だが未実装

---

## 8. 今後のアクション候補

### 優先度: 高

| # | タスク | 理由 |
|---|--------|------|
| 1 | **GNSS 付きデータセットで GNSS 制約テスト** | Autoware の地理座標系マッピング機能が未検証 |
| 2 | **Autoware 実環境での読み込みテスト** | `pointcloud_map_loader` でのランタイム互換性確認 |
| 3 | **develop ブランチへのマージ** | PR #2 のコードレビュー対応 |

### 優先度: 中

| # | タスク | 理由 |
|---|--------|------|
| 4 | ~~Triangle keypoint 抽出質改善~~ | ✅ PR #145 v4 default に landing 済、初の採用ループ確認 |
| 4b | ~~4 点以上 consensus への拡張~~ | ✅ PR #147 で実装、v5 で偽陽性半減確認 |
| 4c | MID-360 demo bag の整備 | reference 軌跡 + 短距離ループありの bag が無いと triangle ablation を MID-360 で回せない |
| 4d | 別データセットで triangle stack 再現性検証 | NTU 単独では PoC 段階。Newer College / Leo Drive / MID-360 demo で同じ ablation を回したい |
| 5 | Robust kernel 導入 | 誤ループ検出への頑健性（既に DCS/Cauchy/Huber 切替は実装済） |
| 6 | キーフレーム選択ロジック | フロントエンドの品質指標に基づくサブマップ生成 |
| 7 | マルチセッションマッピング | 複数回走行データの統合 |

### 優先度: 低

| # | タスク | 理由 |
|---|--------|------|
| 8 | GTSAM 移行 | Jazzy での boost→std 互換問題の解決待ち |
| 9 | DLIO 統合 | DDS 問題の根本解決が先 |
| 10 | small_gicp オドメトリ高速化 | KISS-ICP / RKO-LIO が十分高精度 |

---

## 9. 技術的知見

### ループクロージャーのパラメータチューニング

| パラメータ | Newer College 推奨 | MID-360 推奨 | NTU-VIRAL 推奨 |
|-----------|-------------------|-------------|---------------|
| adjacent_edge_info_weight | 1000.0 | 100.0 | 1000.0 |
| threshold_loop_closure_score | 3.0 | 15.0 | 3.0 |
| distance_loop_closure | 100.0 | 100.0 | 100.0 |
| use_scan_context | true | false (非360 FOV) | true |
| scan_context_threshold | 0.3 | - | 0.3 |

**知見**: `adjacent_edge_info_weight` はデータセットの LIO 精度に依存。高精度 LIO (RKO-LIO on Newer College) では 1000 でオドメトリ重視、低精度時 (MID-360) では 100 でループ重視。

### Autoware マップフォーマット

```yaml
# pointcloud_map_metadata.yaml (Autoware 互換)
x_resolution: 20.0
y_resolution: 20.0
-80_-40.pcd: [-80, -40]    # 座標は整数必須 (yaml-cpp as<int>)
-60_-60.pcd: [-60, -60]

# map_projector_info.yaml (GNSS 原点)
projector_type: local
vertical_datum: WGS84
map_origin:
  latitude: 35.6812362
  longitude: 139.7671248
  altitude: 40.0
```

### 重要ファイル

| ファイル | 説明 |
|---------|------|
| `graph_based_slam/src/graph_based_slam_component.cpp` | バックエンド本体 |
| `graph_based_slam/include/graph_based_slam/scan_context.hpp` | GPL フリー Scan Context |
| `graph_based_slam/include/graph_based_slam/triangle_descriptor.hpp` | BSD-2 三角形 descriptor primitives（§1.2） |
| `graph_based_slam/include/graph_based_slam/triangle_descriptor_database.hpp` | hash DB + RANSAC findLoopCandidate（§1.2） |
| `graph_based_slam/include/graph_based_slam/bev_mutual_visibility.hpp` | FOV-aware BEV mutual visibility（triangle cross-verify でも利用） |
| `graph_based_slam/include/graph_based_slam/loop_edge_robustifier.hpp` | Huber / DCS / Cauchy 切替ヘルパ |
| `graph_based_slam/include/graph_based_slam/adjacent_edge_auto_scale.hpp` | NIS median ベースの adjacent edge info weight auto-scale |
| `scanmatcher/src/scanmatcher_component.cpp` | フロントエンド本体 |
| `scanmatcher/include/scanmatcher/voxel_hash_map.hpp` | VoxelHashMap |
| `lidarslam/launch/rko_lio_slam.launch.py` | RKO-LIO 統合ランチ |
| `scripts/verify_autoware_map.py` | Autoware 互換性検証 |
| `scripts/odom_to_tum.py` | 軌跡ロギング |
| `scripts/run_triangle_ablation.sh` | triangle on/off ablation を 1 コマンドで（§1.2） |
| `scripts/generate_place_recognition_report.py` | scan_context / BEV / SOLiD / triangle の loop 採用統計を md/JSON/SVG 化 |
