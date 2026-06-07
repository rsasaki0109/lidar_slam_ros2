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

## 1.1 追加トラック（2026-04）：KITTI Odometry LO / small_gicp チューニング

2026-04 の LO ベースライン整備＋small_gicp KITTI スイープのハマりどころ（空 TUM の
QoS 不一致 / launch override 順 / 短尺 bag fallback）と再現コマンドは
[`docs/research/kitti-lo-tuning-2026-04.md`](docs/research/kitti-lo-tuning-2026-04.md)
に集約。

**現状（live）**: 公開推奨は引き続き RKO-LIO + graph_based_slam。KITTI / Velodyne-only
は評価・チューニング用の追加導線。00/05/07 のフルスイープ report は v0.4 §E
（opportunistic）として未実施。

---

## 1.2 追加トラック（2026-05）：STD/BTC 風 Triangle Descriptor 自前実装

13 PR で develop 投入した opt-in triangle descriptor stack（NTU / MID-360 / Newer の
3-dataset ablation、max_pairs=16 sweet spot、RANSAC cost isolation、≥3-run variance
discipline、PR #159-#189）の研究記録は
[`docs/research/triangle-stack-2026-05-summary.md`](docs/research/triangle-stack-2026-05-summary.md)
に集約。

**現状（live）**: 全プリセットで default-off（opt-in）。3-dataset で variance-bounded、
SOTA は狙わない honest stance。残る 2 つの open question（max_pairs=8 U-shape root
cause / RANSAC async scheduling）は v0.4 §D1 reproducibility closeout の対象。

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
- **回避策**: offline_node (RKO-LIO) でバッグを内部読み込み（既定パスはこれ）
- **根本解決**: FastDDS のシェアードメモリ設定、またはゼロコピー転送
- ユーザー向けの失敗モード解説・CycloneDDS + kernel チューニング・intra-process
  composition の状況は [`docs/dds-tuning.md`](docs/dds-tuning.md) に集約（v0.4 §F）。

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

inside-out 戦略で 10 PR（#168-#177）投入した操作員向け session pipeline
（foundation / analyzer / dashboard / record / readiness / public-RKO / session /
bundle layer）+ ament lint の罠 + 重要ファイル + 3DGS QA candidate の詳細は
[`docs/research/mid360-robot-toolkit-2026-05.md`](docs/research/mid360-robot-toolkit-2026-05.md)
に集約。

**現状（live）**: develop に landed（81 mid360 scripts + runbook smoke test +
continuous kidnap-relocalization gate #194）。残りは実 GT データセット
（v0.4 roadmap §C → v0.5）。
