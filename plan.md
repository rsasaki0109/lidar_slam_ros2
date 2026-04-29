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
| PCD ディスクキャッシュ | ✅ | OOM 対策、サブマップを逐次 PCD 保存 |
| 情報行列バグ修正 | ✅ | ループエッジを固定重み、オドメトリエッジに `adjacent_edge_info_weight` |
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
| 4 | MID-360 の精度改善 | 固体 LiDAR 対応の place recognition 手法 (BTC 等) |
| 5 | Robust kernel 導入 | 誤ループ検出への頑健性 |
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
| `scanmatcher/src/scanmatcher_component.cpp` | フロントエンド本体 |
| `scanmatcher/include/scanmatcher/voxel_hash_map.hpp` | VoxelHashMap |
| `lidarslam/launch/rko_lio_slam.launch.py` | RKO-LIO 統合ランチ |
| `scripts/verify_autoware_map.py` | Autoware 互換性検証 |
| `scripts/odom_to_tum.py` | 軌跡ロギング |
