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

> 🧭 **2026-05-24/25 research arc closeout**: PR #183-#189 の variance / RANSAC-cost / max_pairs sweep の operator 向け narrative は [`docs/research/triangle-stack-2026-05-summary.md`](../docs/research/triangle-stack-2026-05-summary.md) を参照。本セクションは時系列の詳細記録 (実装 PR + 各 ablation 結果) を保持。

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

| Dataset | LiDAR | 環境 | Votes/submap | BEV inliers | edge_3d inliers | Triangle emit / accept |
|---------|-------|------|--------------|-------------|-----------------|------------------------|
| NTU VIRAL tnp_01 | Ouster OS1 (360° wide vertical) | outdoor open | 200-2800 | 3-5 | (未測定) | BEV: 1 採用 (v5, variance 内 APE 改善) |
| MID-360 glim | Livox MID-360 (narrow FOV) | outdoor mixed | 97-1037 | 1-2 (0 emit) | 1-4 (max=4 が稀) | edge_3d (tuned min=3): 1 emit / 0 accept (NDT が yaw 178° 弾く) |
| Newer College math_hard | Ouster OS0-32 (360° narrow vertical) | indoor | 100-200 | 1-2 (0 emit) | 1-3 (inliers=3 が 6 件) | edge_3d (tuned min=3): 2 emit / 0 accept (NDT が yaw 131°/158° 弾く) |

**結論 (BEV のみ):** BEV max-height keypoint + 3-point RANSAC は **spinning 360° + wide vertical FOV + outdoor** に限定して機能する。narrow FOV / indoor では keypoint repeatability が破綻し、Hash votes は集まるものの inliers が 1-2 で頭打ち。

**結論 (edge_3d 追加後):** PCA edgeness ベースの `edge_3d` keypoint mode (PR #153) を追加し、3-dataset 横断で **inliers 分布を 1 ステップ上にシフト** することを確認 (BEV: 1-2 → edge_3d: 1-3 / 1-4)。これで tuning (min_inliers=3 / min_votes=6) を組み合わせると narrow-FOV / indoor でも **初の emit に到達**。NDT validation gate がすべての yaw 反転偽陽性を弾いて採用 0 を維持しつつ、Newer College では candidate APE 0.087 < baseline 0.107 (-0.020m、variance 内) と初めて改善方向のサインが出た。採用 1 件まで持っていくには 4-point gate / inlier_ratio gate / edge_3d voxel チューニングのいずれかが必要。

**Default 設計判断:**
- `use_triangle_descriptor: false` を **全 preset で維持** — NTU 単独の 1 採用は variance 範囲内、複数 dataset で価値示せず default on の根拠なし
- `min_4th_point_agreements: 0` を **維持** — 4-point gate は NTU 以外では発火前に inlier gate で死ぬため検証不能、デフォルトを上げる根拠なし
- これらは「opt-in 研究機能」として develop に landing 済。production default に上げるには別の keypoint 抽出方式が必要

### 残った次の打ち手（優先度順）

1. **edge_3d keypoint extractor を 2026-05-19 に投入 (PR #153)** — PCA edgeness ベース、view-direction agnostic、KeypointMode enum で dispatch、MID-360 yaml は default を edge_3d に切り替え。3-dataset 横断検証で **inliers 分布を 1 ステップ上にシフト** することを確認。
   - **MID-360 ablation**: BEV 時代は inliers 100% が 1-2 で完全 0 emit だったが、edge_3d では稀に inliers 3, 4 が出現し、tuning (min_inliers 5→3, min_votes 8→6) で **初の emit 1 件** (id=22, inliers=4, yaw 178°) を確認。NDT が yaw 反転の偽陽性を弾いて採用 0 件、APE は variance 内 (-0.56m)。
   - **Newer College math_hard ablation**: BEV では 76 votes 全件 reject (inliers 1-2)、edge_3d で 1-3 まで上昇 (inliers=3 が 6 件)、tuned min=3 で **emit 2 件** (id=26 yaw 131°、id=0 yaw 158°)、NDT が両方弾いて採用 0 件、APE 0.087 vs baseline 0.107 で **初めて candidate < baseline** (variance 内、+方向)。
   - **edge_3d tighter + 4-point gate チューニング (同日)**: `edge_voxel_size=0.2, edge_neighbor_radius=0.6, edge_min_edgeness=0.6, min_4th_point_agreements=2` で再走。Newer College では **初の inliers=4 + legit yaw 13° emit** に到達。MID-360 では同条件で **emit 0 (regression)** — tighter voxel が narrow FOV outdoor では逆効果と判明。→ environment 別チューニングが必要 (Newer College: tighter / MID-360: default)。これを codify した graphbasedslam_indoor.yaml preset を PR #157 で投入。
   - **3-run variance test (2026-05-19, PR #158)**: PR #156 の APE -0.039m が再現するか 3 周検証。**結果: mean Δ +0.0038 m / std Δ 0.0216 m で variance >> mean Δ、APE 改善は variance 内のフロックと判明**。ただし emit reach は明確に進歩 (run1 で **inliers=5 yaw=3.6°** 出現 = 初の inliers=5、0° 付近の legit yaw 候補が増加)。NDT validation が依然 brake で採用 0 維持。
   - **Umeyama N-point SE(3) refinement 検証 (2026-05-19)**: `estimateRigidFromCorrespondences` を追加し winning T を全 inlier 三角形の 3*N 点対応で再推定する opt-in flag `refine_se3_with_all_inliers` を実装。合成データ gtest で 30 点 noisy correspondence の精度向上を確認。Newer College n=1 では emit 数が 3→1 に減少 (refined T が 4-point gate を超える形に動いた)、APE 0.07→0.12 で variance 内ながら方向違い。→ 仮説 (refinement で NDT correction <15m に収まる) は narrow-FOV/indoor で **検証されず**。inlier correspondence 自体が wrong-but-agreeing なケースが多く、refinement で wrong T を smooth しても本質的な improvement にならない。code は opt-in landing、indoor preset でも default false 維持。
   - **次の打ち手 (採用 1 件を狙う)**: (a) Newer College の loop_max_translation_delta 緩和の妥当性検討、(b) keypoint correspondence の質を上げる別 hashing/descriptor (4-point STD/BTC-style key)、(c) min_inlier_ratio gate と組み合わせた precision floor の効果測定
2. **NTU v5 reproducibility — 2-3 周回して variance 内に APE 改善が安定するか確認** (BEV のまま、edge_3d は narrow-FOV 向けなので NTU は別議題)
   - 現状 1 採用は 1 回観測 (v5)。v5 を 3 周回して採用ペアが (a) 毎回出るか (b) 毎回同じ submap_id か検証
   - 安定すれば NTU プリセット限定で `use_triangle_descriptor: true` も検討余地あり
4. **Leo Drive driving bag への展開** (要 PointCloud2 化と reference 整備)
5. **コンポーネント単体テスト** (searchLoop に triangle path をモック注入する gtest)

### ステータスと運用方針

- triangle descriptor stack は「**実装完了・NTU で 1 PoC 採用 (BEV)・MID-360 と Newer College で edge_3d により emit が 0→1-5 に進歩 (3-run variance、Newer で inliers=5 初到達)、APE 改善は variance 範囲内 (3-run 平均で mild regression)、採用は NDT 安全網で 0 維持**」段階。v0.4 リリースでは引き続き default off (`use_triangle_descriptor: false`) で opt-in 機能として提供。
- v0.4 release notes には「STD/BTC 風 place recognition の opt-in 実装あり、NTU VIRAL tnp_01 で 1 採用ループ確認 (BEV、variance 範囲)、MID-360 / Newer College は edge_3d keypoint mode で対応 — narrow-FOV / indoor では emit に至るも geometric consensus がまだ脆く、production gate は通っていない研究機能」と書く。
- 4-point gate と `use_triangle_descriptor` の default 判断は 3-dataset 検証によりクローズ済。edge_3d 追加で keypoint 抽出の根本見直しは部分的に進捗。次は edge_3d パラメータチューニングと Newer College 検証。

詳細メモは [[project_triangle_descriptor_stack]] に保存済。

### Triangle descriptor 後続 PR (#159-#162, 2026-05-21)

§1.2 までの 3-dataset 検証の延長で、**「wrong-but-agreeing 3-point RANSAC」のフロアを下げる**目的で 4 つの後続 PR を develop に投入：

| PR | コミット | 内容 |
|----|---------|------|
| #159 | `79a6b5f` | **Umeyama N-point SE(3) refinement** — winning RANSAC T を全 inlier 三角形の 3×N 点対応で再推定する opt-in flag `refine_se3_with_all_inliers`。3-point SVD だと 1σ ノイズが translation にそのまま乗り NDT correction が 50-70m もズレるという #158 の知見への対策 |
| #160 | `e905673` | **descriptor-source-only `loop_max_delta` override** — DISTANCE candidate は strict cap を維持しつつ、TRIANGLE / SCAN_CONTEXT / BEV / SOLID には大きめの NDT correction を許可するパスを追加。Newer College で legit yaw 3.6° emit が NDT fitness 11.9 < 15 を通っても world-frame correction が generic cap に弾かれていた問題への対策。default `-1.0` (disabled) で既存挙動は bit-for-bit 同一 |
| #161 | `5864503` | **STD/BTC-style quad-hash extension** — 三角形 hash bucket key に 4 次元目 (centroid → 最近傍 non-vertex keypoint の量子化距離、回転不変な local context) を opt-in で追加。corridor / parking row / parallel column geometry に対する false positive 削減狙い。`triangle_descriptor_quad_feature_bin_m` ROS param、default `0.0` (disabled) で legacy 3-edge path は bit-for-bit 同一 |
| #162 | `5e52f3b` | **inlier_ratio + eval_n surfacing** — `LoopCandidate::eval_n` と `LoopCandidate::inlier_ratio` を populate し、emit log + debug reject log に `inliers=N eval_n=M inlier_ratio=R` を出す。`min_inlier_ratio` ゲートは PR #146 で実装済だが eval_n が unobservable で blind tuning だったため、precision-floor recipe (`min_inliers` AND `min_inlier_ratio`) を tuning 可能にする (no behavior change) |

**位置付け**: いずれも default off (behavioral no-op) で、§1.2 の運用方針 (default `use_triangle_descriptor: false`、研究 opt-in) を変えずに「採用 1 件のフロアを下げる knob を 3 種類用意した」ということ。次は dataset を増やして 4-point quad-hash + N-point refinement + precision floor の組み合わせで何件 emit / accept できるか測定する段階。

### Triangle v5 reproducibility 3-run variance (2026-05-24)

§1.2 の「NTU v5 で 1 採用」「APE Δ -0.022 m 改善」は単発 (N=1) 観測だったので、現 develop HEAD (post PR #159-#162, 全 default off) で **同一 yaml・同一 bag を 3 回回して variance を測定**：

| run | baseline APE [m] | candidate APE [m] | Δ [m] | Triangle observed | Triangle accept | distance loops base/cand |
|-----|-----------------|-------------------|-------|-------------------|-----------------|--------------------------|
| 1   | 1.470           | 1.385             | -0.085 | 0                | 0               | 14 / 11                  |
| 2   | 1.436           | 1.338             | -0.098 | 0                | 0               | 11 / 11                  |
| 3   | 1.344           | 1.469             | +0.125 | 0                | 0               | 14 / 11                  |
| 平均 | 1.417 ± 0.064  | 1.397 ± 0.066    | **-0.019 ± 0.125** | 0/3            | 0/3             | 13 / 11                  |

**結論**:
1. **APE Δ -0.019 ± 0.125 m は variance 内** — v5 設定が NTU で APE を改善するという主張は N=1 ノイズだった。2026-05-18 の「Δ -0.022 m 改善」は今回の variance σ=0.125 m に十分含まれる
2. **Triangle observed=0/3 runs** — 2026-05-18 単発で「emit=2/accept=1」だった結果は **再現せず**
3. **コード regression ではない**：
   - 同一 submap id=16 の query を比較すると **votes 369→392 (UP), inliers 4→3 (DOWN)** — bucket 構造は変化なし (votes 増えてる)、verification が stochastic にずれた
   - 05-24 の 3 runs 間でも id=16 の inliers は {2, 3, 3} とばらつく → RANSAC + map_array timing の **run-to-run variance** で同じバイナリでも non-deterministic
   - PR #159-#162 の C++ 変更は全 default off, 該当 code path は #158 と bit-for-bit 同一 (diff 検証済)
4. **per-query inlier distribution**: 05-18 n=36 で P(≥4)=5.6% (1+1 tail), 05-24 n=66 で P(≥4)=0% — base rate ~5% per query × 20-30 queries/run なら 0/run は普通に起きる

**運用への含意**:
- v0.4 release notes から「NTU で 1 採用ループ」「APE 改善」の文言を削除すべき (再現性なし)
- 今後の ablation は **必ず ≥3 runs** で variance 込みで判断、単発で APE Δ を主張しない
- triangle on NTU は base rate が低すぎて NTU を primary 評価軸にしない方が良い — indoor / MID-360 (edge_3d 系) の方が emit reach が高い
- 検出率を底上げする方向: (a) `min_inliers: 4 → 3` + ratio gate で precision floor 維持, (b) `max_pairs: 24 → 48` で sample budget 増, (c) RANSAC 自体を deterministic 化 (seed 固定 + 並列度抑制)

詳細: [`output/triangle_ablation_ntu_v5_3run_20260524_083127/SUMMARY.md`](../output/triangle_ablation_ntu_v5_3run_20260524_083127/SUMMARY.md) (root cause analysis 含む)

### MID-360 3-run variance + 2026-05-19 emit reproducibility (2026-05-24)

NTU v5 で確立した「単発結果は信用しない」を 2nd dataset (MID-360) で検証。2026-05-19 単発「emit 1 件 (id=22 inliers=4 yaw=178°)」が再現するか、default config と tuned config (= 2026-05-19 設定) で各 3 runs:

| config | base APE [m] | cand APE [m] | Δ APE [m] | obs/3 | accept/3 |
|--------|-------------|--------------|-----------|-------|----------|
| default (`min_inliers=5, min_votes=8`) | 3.800 ± 0.270 | 4.498 ± 0.962 | +0.699 ± 1.231 | 0/3 | 0/3 |
| **tuned** (`min_inliers=3, min_votes=6`) | 3.793 ± 0.349 | 4.876 ± 0.456 | **+1.083 ± 0.128** | **2/3** | 0/3 |

**結論**:
1. **2026-05-19 「emit 1 件」は部分的再現** — tuned 2/3 runs で obs=1、specific submap_id/yaw は run ごとに stochastic。「emit ~1/run」は real、「id=22 yaw=178° specific」は noise
2. **Tuned config は robust APE regression** — Δ +1.083m, std 0.128m, **|Δ|/σ=8.5** (variance を超えた有意な悪化)。default config では variance 内 (+0.699 ± 1.231m) だが run2 outlier (+2.1m) が dominate
3. **新 meta-finding**: triangle pipeline 有効化だけで APE が +1m 悪化 (accept=0 にも関わらず)。仮説: triangle 計算 CPU cost → ROS executor scheduling shift → distance loop verification の message timing 変化。**wall-clock timing 依存が APE level で visible**
4. **NDT precision floor は機能している** — 全 emit が yaw 178° corridor flip → NDT fitness reject。MID-360 narrow-FOV では triangle 3-point RANSAC が本質的に corridor 偽陽性 prone、NDT が safety net

**運用判断**:
- MID-360 yaml default は `min_inliers=5` 維持 (`min_inliers=3` への tuning は dangerous)
- `use_triangle_descriptor: false` 維持
- v0.4 release notes: MID-360 で triangle を promote しない (default-off は正解)

**次の deep dive 候補**: Triangle pipeline を no-op compute load (votes 計算するが RANSAC スキップ) に減らして APE regression が消えるか → "executor scheduling cost" vs "RANSAC compute cost" を切り分け

詳細: [`output/triangle_ablation_mid360_3run_tuned_20260524_093504/SUMMARY.md`](../output/triangle_ablation_mid360_3run_tuned_20260524_093504/SUMMARY.md)

### MID-360 RANSAC cost isolation (2026-05-24, follow-up to MID-360 3-run)

PR #184 で発見した「triangle pipeline 有効化だけで +1m drift」の根本原因を切り分け。diagnostic flag `triangle_descriptor_skip_ransac` (default false) を追加し、`accumulateVotes` (hash lookup) は実行するが `findLoopCandidate` (RANSAC) を skip するモードを実装。同じ MID-360 tuned config で 3-run:

| condition | mean Δ APE [m] | std [m] | \|Δ\|/σ |
|-----------|----------------|---------|---------|
| RANSAC ON (PR #184 tuned)    | **+1.083** | **0.128** | **8.5** (有意) |
| RANSAC OFF (skip_ransac=true) | +0.604 | 1.258 | 0.48 (variance 内) |

**結論**: **RANSAC compute cost が systematic +1m drift の dominant source**。
- RANSAC OFF にすると |Δ|/σ が 8.5 → 0.48 に落ち、baseline と区別不能 (variance 支配的)
- mean Δ も 1.083 → 0.604 m に半減
- 残った +0.6m は run-to-run noise (std 1.258m に拡大)
- accumulateVotes (hash lookup, O(N)) は変動内、findLoopCandidate (RANSAC, O(N²) over max_pairs=32) が問題

**改善候補** (PR #186 で実装・確認済):
- ✅ MID-360 yaml で `triangle_descriptor_max_pairs: 32 → 16` 縮小 (RANSAC O(N²) を 1/4)
- RANSAC を `std::async` で非同期化 (searchLoop hot path から外す) — 未実装
- MID-360 で "vote-only mode" — RANSAC skip して candidate submap_id を NDT に渡す
- `triangle_descriptor_skip_ransac` flag は diagnostic として default false で残す

**運用判断は変わらず**:
- MID-360 default は `use_triangle_descriptor: false` 維持
- 改善 PR を出すなら max_pairs 縮小 + 別 dataset での再検証セット

詳細: [`output/triangle_ablation_mid360_skipransac_20260524_101218/SUMMARY.md`](../output/triangle_ablation_mid360_skipransac_20260524_101218/SUMMARY.md)

### max_pairs=16 fix 検証 (2026-05-24)

PR #185 で発見した「RANSAC compute cost が +1m drift の dominant source」を受けて、最も actionable な改善 (`max_pairs: 32 → 16`) を実装・3-run で検証。同じ MID-360 tuned config:

| condition | mean Δ APE [m] | std [m] | \|Δ\|/σ | emit/3 |
|-----------|----------------|---------|---------|--------|
| `max_pairs=32` (PR #184) | **+1.083** | 0.128 | **8.5** (有意悪化) | 2 |
| RANSAC OFF (PR #185) | +0.604 | 1.258 | 0.48 (variance 内) | 0 |
| **`max_pairs=16`** | **-0.292** | 0.607 | **0.48** (variance 内、僅か良化方向) | 0 |

**結果**: `max_pairs=16` で mean Δ APE が +1.083 → -0.292 m に swing (1.4m 改善方向)、|Δ|/σ が 8.5 → 0.48 に落ち **systematic regression が消えて variance 内に収まる**。

**Production trade-off**:
- emit reach は 2 → 0 に減るが、MID-360 narrow-FOV では NDT が triangle emit を 100% reject していたので **production accept rate は元々 0 で変化なし**
- `use_triangle_descriptor: false` (default) のユーザーには無影響
- opt-in したユーザーは APE regression に当たらなくなる = strict 改善

**Why max_pairs=16 が直接効くか**:
- findLoopCandidate は N² の `transformAgrees` (32² = 1024 vs 16² = 256, 4x 削減)
- tuned config では vote threshold を超える tick が頻発 → 毎回 1024 比較が wall-clock dominate
- 256 比較なら ROS executor scheduling が perturb されず、distance loop verification timing 維持 → APE 安定

**Generalization to other datasets (確認済)**:
- ✅ `graphbasedslam_indoor.yaml` (Newer College math_hard) は max_pairs=64 でも 2026-05-19 3-run で **Δ APE +0.004 ± 0.022 m (variance 内)** = drift 観測されない (variance_summary.json)。post-#186 でも Newer 経路に C++ 変更なし → 結果有効。**同 fix 不要**
- ✅ generic `graphbasedslam.yaml` (NTU 系 outdoor 360°) も PR #183 で variance 内 = **変更不要**
- ✅ **NTU skip_ransac 3-run (2026-05-24)** で **RANSAC compute が NTU APE に影響しないことを直接実験で裏付け**: RANSAC ON Δ -0.019 ± 0.125m vs RANSAC OFF Δ -0.013 ± 0.047m (両方 variance 内、mean 同等)。PR #187 の「MID-360 specific」結論を independent experiment で確証 (skip_ransac で std が 3x tighter = wall-clock variability 減少示唆)

**Generalization finding**: `+1m APE drift` は **MID-360 narrow-FOV 特有**で、indoor (Newer College) / outdoor 360° (NTU) には generalize しない。仮説:
- APE スケールが違う: MID-360 base ~3-5m vs Newer ~0.1m vs NTU ~1.4m
- submap topology: MID-360 narrow-FOV は keypoint repeatability が低く triangle vote threshold を頻繁に clear → RANSAC 呼び出し頻度が高い → wall-clock cost が distance loop verification timing を perturb
- 他データセットでは triangle pipeline の発火頻度が低く、cost が dominate しない

**運用判断**: `max_pairs` 縮小 fix は MID-360 yaml に限定 (PR #186 で完了)。他の preset は触らない。

詳細: [`output/triangle_ablation_mid360_maxpairs16_20260524_175503/SUMMARY.md`](../output/triangle_ablation_mid360_maxpairs16_20260524_175503/SUMMARY.md) + [`output/triangle_ablation_newer_college_edge3d_3runs/variance_summary.json`](../output/triangle_ablation_newer_college_edge3d_3runs/variance_summary.json)

### max_pairs sweep 完成: `=8` で regression 再発 (2026-05-24)

PR #186 (`max_pairs=16`) の後、「もっと下げれば更に良いのでは」を検証するため `max_pairs=8` で 3-run。仮説 (RANSAC compute cost) が monotonic なら更に改善するはず。

| max_pairs | mean Δ APE [m] | std [m] | \|Δ\|/σ | cand mean APE [m] | classification |
|-----------|----------------|---------|---------|---------------------|----------------|
| 32 (PR #184)     | **+1.083** | 0.128 | **8.5** | 4.876 | systematic regression |
| **16 (PR #186)** | **-0.292** | 0.607 | **0.48** | **3.812** | **within variance ✓ (sweet spot)** |
| 8 (this)         | **+0.768** | 0.167 | **4.6** | 4.644 | systematic regression 再発 |

(baseline 9-run aggregate: mean 3.92 ± 0.40 m)

**U字パターン**: max_pairs=16 が真の sweet spot。両側 (32 / 8) が baseline noise を超えて drift。candidate mean APE が baseline noise (3.92±0.40) に唯一収まるのは `=16` のみ。

**意味**:
- 「RANSAC compute 軽くすれば軽くするほど良い」という単純な monotonic 仮説は **棄却**
- PR #186 の `max_pairs=16` は単なる最適化ではなく **empirical sweet spot** — 16 ±N どちらに動かしても悪化
- max_pairs=8 で regression 再発する root cause は未解明 (wall-clock floor / RANSAC 内部の早期リターン経路 / accumulateVotes downstream の thread contention など)

**actionable conclusion**: PR #186 のままで OK、追加 fix 不要。`max_pairs=16` が production 最適値と empirical 確証。

詳細: [`output/triangle_ablation_mid360_maxpairs8_20260524_213619/SUMMARY.md`](../output/triangle_ablation_mid360_maxpairs8_20260524_213619/SUMMARY.md)

---

## 1.3 追加トラック（2026-05）：Dogfood wrapper measurement plumbing (PR #166)

### 目的
- `scripts/run_rko_lio_graph_autoware_dogfood.sh` は「ロボットが撮った bag を SLAM → corrected trajectory → Autoware map verify まで 1 コマンドで通す」操作員向け wrapper。これを **実環境の bag (frame name が launch default と違う / 長尺で /map_save 後も graph_based_slam が submap を処理し続ける) で安定動作させる**ために、計測系の plumbing を拡充した。

### 投入した PR
- **PR #165 (`129eb58`)** — `path_to_tum.py` / `odom_to_tum.py` の custom signal handler を削除。`rclpy.spin()` 中は Python signal handler が走らず、`kill -INT` で hang していた。rclpy の default SIGINT handler に委譲 + `KeyboardInterrupt` / `ExternalShutdownException` を catch して finally で `rclpy.try_shutdown()`。dogfood pipeline で観測された「`path_to_tum.py` subscriber が `Map outputs saved` 後も 40 分以上生き残る」問題を解決。
- **PR #166 (`5929728`)** — dogfood wrapper measurement plumbing 本体：
  - **frame override flags**: `--base-frame`, `--lidar-frame`, `--imu-frame` を追加。robot の frame name が launch default と異なるケース対応。
  - **quiescence-based offline completion**: `--offline-quiet-log-secs` を追加。RKO-LIO offline node の stdout に N 秒間ログが出なければ完了と判定。
  - **graph-drain wait**: `--graph-drain-secs` で、`/map_save` 前に graph_based_slam が buffered submap を消費し終えるまで待つ。長尺 bag で submap 残り処理中に `/map_save` が走って map が incomplete になる問題への対策。
  - **/modified_path → traj_corrected.tum 取込み**: `--capture-corrected-path`, `--corrected-path-topic` を追加し、ループクロージャ補正後の trajectory を録る。
  - **APE vs reference**: `--reference-tum FILE` で reference TUM 軌跡を渡せば evo APE を計算して `traj_corrected_ape.txt` を吐く。
  - **path_to_tum subprocess reap**: cleanup() で structured kill (SIGINT 先行 → 10s 後 SIGKILL guard)。

### Why
- dogfood wrapper が **production candidate session (PR #176)** の上流レイヤとして使われるため、frame 不一致 / offline 完了判定 / graph-drain / corrected path capture の 4 系列をすべて wrapper の責務に閉じ込めた。これにより `run_mid360_robot_production_candidate_session.sh --run` が wrapper を呼ぶときに、bag → corrected trajectory → APE まで一気通貫で取れる。
- PR #166 の measurement plumbing がなければ MID-360 chain (PR #168-#177) は意味のある "production-readiness" を gate できない。

---

## 1.4 追加トラック（2026-05）：README 操作員向け書き直し (PR #163/#164/#167)

### 目的
- README が status / scope の jargon-dense な metadata block で始まり、showcase 画像が line 97 まで埋まっていた。新規訪問者が「このリポジトリで何ができるか」を 1 スクロールで掴めない状態。
- 3 段階のリライトを経て **plain technical README @ ~110 行 / 5 分 quickstart / docs/ への deeper link table** に着地した。

### 投入した PR
- **PR #163 (`2926e92`)** — initial rewrite: badges + hero showcase 画像 + **'5 Minutes' quickstart** (clone → build → quickstart → `map_verify: PASS` 確認) + themed grouping (🗺️ Mapping, 🚗 Autonomous Driving, 🔁 Loop Closure, 📊 Benchmarks, 🧰 Operator Tooling) + docs category table。
- **PR #164 (`a5498bd`)** — tone-down: emoji / horizontal rules / "5 Minutes" framing / hero block を削除し plain technical README に。157 行（220 行 cap 内）。 `test_docs_entrypoints.py` の全 assertion を維持。
- **PR #167 (`dfeb2ab`)** — final simplify: 必須トピック table / dynamic-object-filter figure / 4 つの benchmark CLI 例を `docs/workflows.md` + `docs/benchmarking.md` に逃がす。README order を Install → Quickstart → "Use your own bag" → Features に。docs link を **Getting started / Pipelines / Benchmarking / Project** で grouping。最終的に ~110 行。

### 結果
- README は 1 スクロールで主要情報が読める長さに到達。
- `test_docs_entrypoints.py` で記載項目が継続的に gate されており、deeper detail が docs/ に逃げても link 健全性は保たれる。
- Phase 2 chain (#171-#177) を land する直前に README が片付いていたので、操作員向け entrypoint table (`scripts/run_mid360_robot_production_candidate_session.sh`, `scripts/import_mid360_robot_production_candidate_bundle.py`) を後で追加するときに整理しやすい状態を確保した。

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
| 3a | ~~MID-360 robot toolkit chain (操作員 pipeline)~~ | ✅ PR #168-#177 で 10 PR inside-out で landing 完了 (§10) |
| 3b | **実機 Jetson + MID-360 robot での dogfood 実走** | chain (§10) を組んだものの実機 bag での E2E 検証はまだ。dogfood-vs-bench の cloud distribution 不一致も併せて調査 |
| 3c | Jetson host readiness preflight PR | §10.5 残課題の自然な次。`check_jetson_mid360_host_readiness.py` + `jetson_mid360_host_tools.py` を 1 PR で land |

### 優先度: 中

| # | タスク | 理由 |
|---|--------|------|
| 4 | ~~Triangle keypoint 抽出質改善~~ | ✅ PR #145 v4 default に landing 済、初の採用ループ確認 |
| 4b | ~~4 点以上 consensus への拡張~~ | ✅ PR #147 で実装、v5 で偽陽性半減確認 |
| 4c | MID-360 demo bag の整備 | reference 軌跡 + 短距離ループありの bag が無いと triangle ablation を MID-360 で回せない |
| 4d | 別データセットで triangle stack 再現性検証 | NTU 単独では PoC 段階。Newer College / Leo Drive / MID-360 demo で同じ ablation を回したい |
| 4e | 4-point quad-hash (#161) + N-point refinement (#159) + precision floor (#162) 組合せ ablation | §1.2 の延長線。3 つの knob を組み合わせた最適 emit/accept 比率を測る |
| 4f | preflight 系 (`preflight_mid360_robot_bag.py`, `validate_mid360_robot_profile.py`, `rewrite_mid360_robot_bag_stamps.py`) を land | §10.5 残課題。Jetson host readiness の次の順序 |
| 4g | public_dataset 系 (~15 scripts: download / segments / loop_candidates / dataset_report) を land | §10.5 残課題。public bag の準備を独立 PR で済ませる |
| 4h | 3DGS visual QA/export track を設計・PoC | loop alignment / map split を operator が確認しやすい 3D artifact にする。core SLAM gate ではなく dashboard/bundle の optional artifact として扱う |
| 5 | Robust kernel 導入 | 誤ループ検出への頑健性（既に DCS/Cauchy/Huber 切替は実装済） |
| 6 | キーフレーム選択ロジック | フロントエンドの品質指標に基づくサブマップ生成 |
| 7 | マルチセッションマッピング | 複数回走行データの統合 |

### 優先度: 低

| # | タスク | 理由 |
|---|--------|------|
| 8 | GTSAM 移行 | Jazzy での boost→std 互換問題の解決待ち |
| 9 | DLIO 統合 | DDS 問題の根本解決が先 |
| 10 | small_gicp オドメトリ高速化 | KISS-ICP / RKO-LIO が十分高精度 |
| 11 | docs/jetson-mid360-robot-{runbook,scope,static-tf-worksheet}.md を mkdocs に組込み | §10.5 残課題。実機セットアップ手順をまとめる時に必要だが、§10.4 の codebase 側 land が先 |

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

---

## 10. MID-360 robot toolkit chain — 操作員向け session pipeline (2026-05)

### 10.1 何を作ったか

Jetson + Livox MID-360 を載せた robot で、現場の操作員が

  1. **bag を撮る → 録音を check → SLAM map を作る → public RKO ベースラインで設定の妥当性を gate → production-readiness を判定 → operator-facing dashboard + 配布可能 bundle を出す**

までを 1 コマンド (`run_mid360_robot_production_candidate_session.py --run …`) で通せる operator pipeline を、**10 PR (#168〜#177)** に inside-out で分割して develop に入れた。

ここでの「inside-out」は、**依存ツリーの leaf (foundation) → root (orchestrator) の順に narrow PR を積む**戦略のこと。9 PR 全部が `mid360_robot_tools` (PR #168) という self-contained foundation の上に乗っていて、各 PR は前段の PR でランドしたモジュールだけに依存する。

### 10.2 全 PR table

| Phase | PR | scripts/test 新規 | 役割 |
|-------|----|------|------|
| 1 | #168 | `mid360_robot_tools.py` (1423 行) + test | Foundation: RobotProfile / preflight builder / map-run planner / run-manifest writer / payload_to_json |
| 1 | #169 | `mid360_robot_loop_alignment_analyzer.py` (760 行) + `analyze_*.py` + test | PCD 由来の loop closure 候補に largest_component_ratio + local_cloud_checks |
| 1 | #170 | `mid360_robot_dashboard.py` (903 行) + `mid360_robot_production_candidate_bundle.py` (330 行) + CLI x2 + test x2 | Operator-facing HTML dashboard + tar.gz bundle (loop_alignment artifact が dashboard + bundle に統合され E2E 動作確認済) |
| 2 | #171 | `mid360_robot_record_tools.py` (220 行) + test | RobotProfile から `ros2 bag record` コマンドを構築 + 再現可能な manifest writer (json + md + profile snapshot) |
| 2 | #172 | `mid360_robot_production_readiness.py` (407 行) + `check_*.py` + test | 操作員向け production-readiness gate (bag_path / duration / topic rate / map verify / public RKO adoption gate を集約 PASS/FAIL + next_actions) |
| 2 | #173 | `mid360_robot_public_rko_quality_report.py` (838 行) + `mid360_robot_rko_config_adoption.py` (293 行) + CLI x2 + test x2 | sweep manifest → quality score + gate を計算、tracked config が gate-passing case と一致するか確認 |
| 2 | #174 | `mid360_robot_public_rko_sweep.py` (965 行) + `run_*.py` + test + configs/mid360_robot/rko_lio_mid360_*.yaml x2 | public MID-360 bag に対する RKO-LIO parameter sweep (per-case yaml override + timeout 付き subprocess 駆動 + runtime signature 解析) |
| 2 | #175 | `mid360_robot_public_rko_adoption_gate.py` (310 行) + `run_*.py` + test | sweep → quality → adoption を 1 entry に orchestrate (run / plan / from-existing mode) |
| 2 | #176 | `mid360_robot_production_candidate_session.py` (741 行) + `mid360_robot_production_candidate_bundle_import.py` (401 行) + CLI x3 (py + shell) + test x2 | **chain closing piece**: session orchestrator (recording → readiness → mapping → public gate → production gate → dashboard) + bundle import (tar.gz extract + verify + 再 gate) |
| 2 | #177 | `mid360_robot_recording_check_tools.py` (393 行) + `check_mid360_robot_recording.{sh,py}` + `record_mid360_robot_bag.sh` + `plan_mid360_robot_record.py` + `configs/mid360_robot/livox_mid360_default.yaml` + test | 録音後 check cascade (bag が record plan / robot profile に合っているか、topic 周波数 / frame id を確認)。PR #176 で skipif した record_only test がこれで自動 enable |

**Phase 1 (#168〜#170)**: 前セッションで先に landed (dashboard + bundle の loop_alignment 統合まで)。  
**Phase 2 (#171〜#177)**: 本セッションで連続 land。**7 PR / ~7400 行 / 全 5/5 CI green**。

### 10.3 依存ツリー

```
mid360_robot_tools (foundation, PR #168)
 ├── mid360_robot_record_tools (PR #171)
 │    └── mid360_robot_recording_check_tools (PR #177)
 ├── mid360_robot_loop_alignment_analyzer (PR #169)
 ├── mid360_robot_dashboard (PR #170)
 ├── mid360_robot_production_candidate_bundle (PR #170)
 ├── mid360_robot_production_readiness (PR #172)
 ├── mid360_robot_rko_config_adoption (PR #173)
 ├── mid360_robot_public_rko_quality_report (PR #173)
 ├── mid360_robot_public_rko_sweep (PR #174)
 │    └── mid360_robot_public_rko_adoption_gate (PR #175) — also depends on #173
 │         └── mid360_robot_production_candidate_session (PR #176) — depends on #170/171/172/175
 │              └── mid360_robot_production_candidate_bundle_import (PR #176)
 └── plan_mid360_robot_record (PR #177, uses #171)
```

各 PR は前段の PR の output (script + module API) だけに依存。CMakeLists.txt への `ament_add_pytest_test` の追加が PR 間で衝突 → develop merge ごとに rebase (10+ 回) して force-push-with-lease で揃えた。

### 10.4 ament lint の罠 (memory にも記録)

7 PR の連続 land 中に **3 種類の CI fail パターン**を踏んだ：

1. **`ament_copyright`**: `# Copyright 2026 Sasaki / # Software License Agreement (BSD 2-Clause Simplified License)` の 4 行 header だけだと `license=<unknown>` で fail。 `Redistribution and use ...` で始まる **BSD-2-Clause license body 全文 (約 23 行)** を入れる必要あり。テンプレは `test_aligned_trajectory_metrics.py` 先頭 29 行。
2. **`ament_flake8 I101` (Jazzy のみ)**: `from module import (A, B, C)` の中身は **case-insensitive アルファベット順**。`render_rko_quality_markdown` は `RKO_QUALITY_HTML` より前 (`r_e` < `r_k`)。lowercase が大文字より先に来る。
3. **`ament_flake8 I100`**: `from local_module import ...` を `import yaml` (third-party) より後に置くと "should be before 'import yaml'" で fail。**対策は `importlib.import_module('mid360_robot_*')` で lazy load する pattern**。test_mid360_robot_tools.py が既に使っている既存パターンを踏襲。

加えて **untracked dep**:

- `configs/mid360_robot/*.yaml` を PR に同梱しないと CI で `FileNotFoundError`。
- `scripts/run_*.sh` shell wrapper も untracked だと `subprocess.CalledProcessError: returncode 127`。

合計で **3 回の force-push rework + 1 個のテストを skipif で deferred** したが、最終的に 7 PR とも 5/5 green で landed。

### 10.5 残課題 (untracked)

- `check_jetson_mid360_host_readiness.py` + `jetson_mid360_host_tools.py` (Jetson 上の CPU / disk / cuda preflight) — 次の natural な PR
- `preflight_mid360_robot_bag.py`, `validate_mid360_robot_profile.py`, `rewrite_mid360_robot_bag_stamps.py` などの preflight 系
- `public_dataset` / `public_loop` / `sample_session` / `field_session` 系 (~15 scripts)
- 3DGS visual QA/export 系 (pointcloud_map + trajectory + loop candidates を operator が確認できる splat/HTML artifact にする)
- 関連 docs: `docs/jetson-mid360-robot-runbook.md`, `docs/jetson-mid360-robot-scope.md`, `docs/jetson-mid360-static-tf-worksheet.md`
- 実機 Jetson + MID-360 robot での dogfood 実走 (cloud distribution の dogfood-vs-bench discrepancy 調査込み)

### 10.6 重要ファイル (chain)

| ファイル | 説明 |
|---------|------|
| `scripts/mid360_robot_tools.py` | Foundation: RobotProfile / preflight / planner / payload_to_json |
| `scripts/mid360_robot_dashboard.py` | Operator HTML dashboard (loop_alignment 統合) |
| `scripts/mid360_robot_production_candidate_bundle.py` | tar.gz bundle 出力 |
| `scripts/mid360_robot_production_candidate_bundle_import.py` | tar.gz 受信 + verify + 再 gate |
| `scripts/mid360_robot_production_candidate_session.py` | Session orchestrator (chain closing piece) |
| `scripts/mid360_robot_production_readiness.py` | Production-readiness gate |
| `scripts/mid360_robot_recording_check_tools.py` | 録音後 check (bag ↔ record_plan ↔ profile) |
| `scripts/mid360_robot_public_rko_sweep.py` | public bag に対する RKO-LIO parameter sweep |
| `scripts/mid360_robot_public_rko_quality_report.py` | sweep manifest から quality gate report |
| `scripts/mid360_robot_public_rko_adoption_gate.py` | sweep → quality → adoption orchestrator |
| `scripts/mid360_robot_rko_config_adoption.py` | tracked RKO config と sweep best case の照合 |
| `scripts/mid360_robot_loop_alignment_analyzer.py` | PCD 由来 loop closure 候補の largest_component / local cloud check |
| `scripts/mid360_robot_public_segment_map_cloud_alignment.py` | reset済み start/end segment map をICPで剛体アラインし、loop drift をCloudAnalyzer gate化 |
| `scripts/mid360_robot_record_tools.py` | `ros2 bag record` コマンド + 再現可能 manifest |
| `scripts/run_mid360_robot_production_candidate_session.sh` | 操作員向け 1-コマンド エントリ |
| `scripts/import_mid360_robot_production_candidate_bundle.py` | 別マシンで bundle を受け取って recheck |
| `configs/mid360_robot/livox_mid360_default.yaml` | デフォルト robot profile (frames + expected topics) |
| `configs/mid360_robot/rko_lio_mid360_*.yaml` | sweep の base config (deskew off / low_voxel) |

### 10.7 3DGS visual QA/export candidate

3DGS (3D Gaussian Splatting) は入れる価値がある。ただし **SLAM の数値 gate
や production readiness の必須条件にはしない**。まずは operator / reviewer が
map の loop misalignment、split cloud、trajectory revisit を確認しやすくする
optional visual QA artifact として扱う。

#### 最終目標

**MID-360 で作った地図を、ブラウザで一発で見られる 3D map preview にする。**

成果物としては、RKO-LIO / graph_based_slam の `pointcloud_map/` から
`mid360_robot_3d_map_preview.html` を生成し、ブラウザで開くだけで map cloud、
trajectory、loop candidate marker を確認できる状態を目指す。これは 3DGS の
production training pipeline ではなく、3DGS 風の splat/point preview から始める。

#### 使いどころ

| Use case | 3DGS の役割 | core gate への扱い |
|---|---|---|
| loop alignment review | loop candidate 周辺を滑らかな splat scene として見せ、trajectory の往路/復路を重ねる | optional。PASS/FAIL は `mid360_robot_loop_alignment_analyzer.py` が持つ |
| map split diagnosis | connected components が分かれた場所を色分けして reviewer が見る | optional evidence |
| production candidate dashboard | `mid360_robot_session_dashboard.html` から 3D artifact へリンク | dashboard enhancement |
| bundle review | Jetson から持ち帰った bundle に軽量 3D preview を同梱 | bundle optional artifact |
| public demo | Autoware map verify PASS の map を人間に説明しやすくする | release/supporting material |

#### 重要な境界

- 現在の public MID-360 bags はカメラ画像を前提にしていない。したがって最初の
  3DGS は photorealistic radiance field ではなく、**LiDAR pointcloud 由来の
  geometry splat preview** として始める。
- synchronized camera images がある robot では、後で RGB 付き 3DGS training に
  拡張できる。しかし、現トラックでは camera calibration / image ingestion は
  production requirement に入れない。
- 外部 3DGS 実装を vendor しない。license / CUDA / PyTorch version / build time の
  リスクが大きいので、repo が最初に持つべき責務は **export manifest + lightweight
  viewer artifact + reproducible command**。
- 3DGS が綺麗でも、map verify / loop analyzer / production readiness が FAIL なら
  production candidate は FAIL のまま。3DGS は「説明」と「検査補助」であって、
  correctness proof ではない。

#### PoC design

最小 PoC は trainer ではなく exporter から始める。

| Phase | Artifact | 内容 | Test |
|---|---|---|---|
| A: splat export | `mid360_robot_3d_map_preview.json`, `mid360_robot_3d_map_preview.ply` | `pointcloud_map/` の PCD tiles を sample し、position / color を持つ preview PLY に変換。色はまず height-based | fixture PCD から deterministic PLY を生成 |
| B: loop overlay | `mid360_robot_3d_map_preview_overlay.json` | TUM trajectory、loop candidates、local cloud connected components を viewer overlay として出力 | fixture trajectory で candidate indices が JSON に残る |
| C: dashboard link | dashboard HTML | session dashboard から 3D preview artifact にリンク。bundle export/import でも optional artifact として保持 | dashboard test + bundle optional artifact test |
| D: viewer | `mid360_robot_3d_map_preview.html` | browser で map cloud + trajectory + loop marker を開ける軽量 viewer。重い dependency は optional | docs smoke + file existence |
| E: RGB 3DGS training | optional external command manifest | camera topics / calibration / images がある robot だけで trainer を呼ぶ。現段階では design only | no default CI |

#### Proposed module split

| Module | Responsibility |
|---|---|
| `scripts/mid360_robot_3d_map_preview.py` | PCD / trajectory / loop analyzer report を読み、HTML + PLY + overlay JSON を生成 |
| `scripts/export_mid360_robot_3d_map_preview.py` | CLI wrapper。`run_dir`, `--loop-alignment`, `--output-dir`, `--max-points` |
| `graph_based_slam/test/test_mid360_robot_3d_map_preview.py` | fixture binary/binary_compressed PCD と TUM から exporter を検証 |
| dashboard integration | `mid360_robot_dashboard.py` に optional 3DGS section |
| bundle integration | `mid360_robot_production_candidate_bundle.py` に optional include |

#### First implementation order

1. `mid360_robot_loop_alignment_analyzer.py` の PCD reader を再利用して PCD tiles を読む。
2. `max_points` と deterministic stride sampling で lightweight PLY を出す。
3. trajectory と loop candidates を overlay JSON に出す。
4. dashboard/bundle には artifact link だけ追加する。
5. public loop bag (`outdoor_kidnap_a + outdoor_kidnap_b`) の map run ができたら、loop
   analyzer report と 3DGS preview を並べて reviewer が確認する。

この順なら、3DGS を入れても SLAM core / Autoware map verify / production readiness
を汚さない。PoC が有用なら viewer と RGB training に進む。

#### Current public loop status (2026-05-25)

- `outdoor_kidnap_a + outdoor_kidnap_b` は raw sqlite merge 済み:
  `datasets/mid360_public_loops/outdoor_kidnap_raw/rosbag2`
  - `ros2 bag info`: 554.562s / 118,843 messages / 4.2GiB
  - topics: `/livox/points` PointCloud2 4,017, `/livox/imu` 110,612,
    `/livox/lidar` CustomMsg 4,214
  - split gap: 1.493804475s
- 実RKO-LIO投入:
  `output/mid360_public/outdoor_kidnap_ab_rko_tolerant`
  - `/map_save` 成功、Autoware map verify PASS
  - 3D map preview/dashboard 生成済み
  - ただし RKO trajectory は 203 poses / 28.4s / 43.1m で止まり、
    loop analyzer は FAIL (`nearest_revisit=22.120m`, loop candidates 0)
  - log: `Number of correspondences are 0` が 2,693 回、
    keypoint/drop 系 error が 3,814 回。`Received LiDAR scan ... delta` は 0
- bag側の実データ検証:
  - scan 203 から keypoint不足 zone が始まり、後段には再び有効scanがある。
    これは public `outdoor_kidnap` の kidnap/disconnected segment 性質で、
    旧continuous RKO-LIOでは post-kidnap を再捕捉できていなかった。
  - `scripts/analyze_mid360_robot_public_loop_cloud.py` は PASS。
    GT loop candidate 0 の実PointCloud2 overlap は
    median NN 0.250m / p90 0.548m / coverage within 1m 0.963。
    つまり public data の loop 自体は本物。
- continuous RKO-LIO kidnap relocalization:
  - RKO core に kidnap recovery path を追加。通常ICP失敗時に、pruneしない
    relocalization map へ coarse yaw search + ICP で再捕捉する。
    keypoint不足scanは時刻を進めてdropし、relocalization失敗時のみlocal resetへ
    fallbackする。
  - tracked config:
    `configs/mid360_robot/rko_lio_mid360_kidnap_tolerant.yaml` は
    `enable_kidnap_relocalization: true`,
    `reset_on_registration_failure: true`,
    `max_scan_delta_sec: 10000.0`。
  - 旧continuous gateは generic loop candidate だけを見ていたため浅かった。
    旧runは nearest revisit 0.162m でも public GT start/end endpoint が
    153.202m ずれていたので、completion 判定から外した。
  - 実public merged bag final run:
    `output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final`
    は RKO offline completion。RKO trajectory 2896 poses / 553.801s /
    path 882.955m。invalid scan drop 1121、global relocalization event 1。
  - `/map_save` 成功、Autoware verify PASS
    (`verify_autoware_map.log`: 8 PASS / 1 WARN / 0 FAIL)。
  - loop alignment analyzer は PASS:
    `output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final/mid360_robot_loop_alignment.json`
    で loop candidates 20、nearest revisit 0.162m、max loop distance 0.180m。
  - continuous completion gate を追加:
    `scripts/run_mid360_robot_public_continuous_relocalization_gate.py`。
    実artifact
    `output/mid360_public/continuous_relocalization_gate/mid360_robot_public_continuous_relocalization_gate.json`
    は PASS。public endpoint は GT start stamp 1693922461.499998 と
    GT end stamp 1693922994.700686 の最近傍poseで 2.515m (threshold 5.000m)。
    checks:
    continuous RKO trajectory complete, Autoware map verify PASS,
    loop alignment PASS, public loop endpoint relocalized, kidnap relocalization
    event present, offline node completed, tracked kidnap config matches run
    config。
  - 3D map preview/dashboard も同runで生成済み:
    `mid360_robot_3d_map_preview.html`,
    `mid360_robot_session_dashboard.html`。
- gate修正:
  - RKO quality/adoption gate に `trajectory_duration` を追加。
    map verify PASS でも trajectory が短すぎる case は gate FAIL になる。
  - `rko_sweep_loop_outdoor_kidnap_tolerant_v3` は map verify PASS だが
    trajectory 28.70s / keypoint drop 1,115 のため quality status `WARN`,
    gate pass 0。これで浅い production PASS を防げる。
- segment reset 実行状況:
  - `scripts/plan_mid360_robot_public_loop_segment_reset.py` は PASS。
    GT loop start は `segment_000`, loop end は `segment_012` に対応。
  - `segment_000` / `segment_012` はそれぞれ単体RKO-LIOで `/map_save` 成功、
    Autoware map verify PASS。RKO offline pose は 203 / 220。
  - `scripts/analyze_mid360_robot_public_segment_map_cloud_alignment.py` を追加。
    reset後の start/end segment map をICP剛体アラインし、median/p90/coverageで
    loop drift をgateできる。
  - 実データalignment gate:
    `output/mid360_public/outdoor_kidnap_segment_reset_alignment/mid360_robot_public_segment_map_cloud_alignment.json`
    は PASS。crop radius 20m、start/end analysis points 4,525 / 7,291、
    aligned median NN 0.632m、p90 2.107m、coverage within 1m 0.690。
  - dashboard は `mid360_robot_public_segment_map_cloud_alignment.json` を読み、
    `Segment Map Cloud Alignment` panel と check table に表示できる。
  - production candidate bundle は segment map alignment JSON/Markdown/PLY を
    optional artifact として同梱できる。requiredにはしないので、未生成でも
    bundle verifyは落とさない。
  - production candidate session CLI に `--segment-map-alignment <json>` を追加。
    外部alignment reportを渡すと session `artifact_paths` にJSON/Markdown/PLYが入り、
    dashboard表示、bundle export、bundle import/recheck後dashboardまで伝播する。
  - public completion gate を追加:
    `output/mid360_public/completion_gate/mid360_robot_public_completion_gate.json`
    および `.md` は PASS。`completion_ready=true`,
    scope は `public_mid360_segment_reset_loop_completion`。
    11/11 checks PASS:
    public loop cloud, segment reset plan, start/end segment RKO completion,
    start/end Autoware map verify, segment map alignment, RKO adoption gate,
    tracked config == top gate-pass config, dashboard presence,
    production candidate entrypoints presence。
    `run_release_readiness_checks.sh --public-mid360-completion` からも
    hard gate として呼べるように接続済み。
    これは「public MID-360 real-data で segment-reset loop path が完成」の判定。
    continuous RKO-LIO の完成判定は上記 continuous relocalization gate が担当する。
