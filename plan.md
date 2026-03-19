# lidarslam-ros2 LiDAR オドメトリ/SLAM 総合調査・改善計画

## Context

lidarslam-ros2 をオープンソース公開に耐えうる RMSE で仕上げるため、既存の LiDAR オドメトリ/SLAM 手法を網羅的にベンチマークし、改善方針を策定した。

**データセット**: Newer College math-hard (320m, Ouster OS0-128, IMU あり)
**ライセンス制約**: MIT/BSD のみ。GPL 汚染不可。

---

## 全手法ベンチマーク結果 (確定)

### LIO 系 (LiDAR-Inertial Odometry)

| 順位 | 手法 | RMSE (m) | Poses | ライセンス | 備考 |
|------|------|----------|-------|-----------|------|
| 1 | **DLIO** | **0.070** | 1896 | MIT | UCLA, NanoGICP + jerk IMU + per-point deskew |
| 2 | **RKO-LIO** | **0.082** | 1930 | MIT | PRBonn, KISS-ICP + IMU tight coupling |

### LO 系 (LiDAR-Only)

| 順位 | 手法 | RMSE (m) | Poses | ライセンス | 備考 |
|------|------|----------|-------|-----------|------|
| 1 | **KISS-ICP** | **0.440** | 1913 | MIT | VoxelHashMap + adaptive ICP + robust kernel |
| 2 | **KISS-SLAM** | **0.434** | 1930 | MIT | KISS-ICP + MapClosures (本データではループ 0) |
| 1 | **GenZ-ICP (pt=0.5)** | **0.146** | 1841 | MIT | **planarity=0.5 で KISS-ICP の 3 倍高精度** |
| 4 | lidarslam + FAST_GICP + VHM + CV | 11.522 | 1199 | BSD/MIT | 改善版 |
| 5 | lidarslam + FAST_GICP (baseline) | 15.950 | 1306 | BSD | |
| 6 | lidarslam + NDT (baseline) | 24.286 | 1887 | - | 元の baseline |

---

## 各手法の深掘り分析

### KISS-ICP — なぜ LO 系で圧倒的か

- **VoxelHashMap**: tsl::robin_map で O(1) ルックアップ。点追加時に sub-voxel 距離チェック (map_resolution = voxel_size / sqrt(max_pts_per_voxel))
- **27近傍探索**: GetClosestNeighbor で 3x3x3 ボクセルキューブを探索。KDTree 不要
- **Robust kernel**: Gaussian-Mixture weighting `w = σ² / (σ² + r²)` で外れ値自動排除
- **Adaptive threshold**: motion model error の RMS で `τ = 3σ`。手動チューニング不要
- **Constant velocity prediction**: `last_pose * last_delta` で初期値が正確→収束が速い
- **2段階ダウンサンプリング**: 0.5x (registration 用) + 1.5x (map update 用)
- **処理速度**: 20-30 fps。共分散計算なし、KDTree なし → 全スキャン処理可能

### GenZ-ICP — なぜ屋外で微妙か

KISS-ICP をベースに point-to-plane マッチングを追加。構造化環境向け。

**問題点**:
1. **planarity 閾値 (0.1-0.2)**: 固有値分解で平面分類するが、草/砂利で平面がほぼ検出されない
2. **共分散計算オーバーヘッド**: 各対応で27近傍 + 固有値分解 → KISS-ICP の数倍遅い
3. **カーネル sigma/3**: KISS-ICP の sigma より厳しい → 外れ値を過剰に除外
4. **2回ボクセル化**: adaptive voxel size 計算のため2パスダウンサンプリング
5. **結果**: 43% スキャンドロップ (840/1930)

**改善の余地**:
- `planarity_threshold` を上げる or point-to-plane を無効化
- カーネルを sigma に戻す
- `max_points_per_voxel` を増やして近傍点を確保
- rate=0.5 以上でスキャンドロップを減らす
- **config_file パラメータで newer_college.yaml を使う** (まだ未テスト)

### small_gicp — 未活用の宝の山

lidarslam では PCL ラッパー (`RegistrationPCL`) のみ使用。実は完全なオドメトリパイプラインが存在。

**使っていない機能**:
- `IncrementalVoxelMap<FlatContainerCov>`: LRU 付きボクセルマップ (KISS-ICP の VoxelHashMap 相当)
- `GaussianVoxelMap`: 分布ベースマッチング (VGICP)
- `TBB フローグラフ`: 前処理→マッチング→出力をパイプライン並列化
- `scan-to-model オドメトリ`: ベンチマークコードに完全な実装あり

**KITTI ベンチマーク** (small_gicp 公式):
- `small_gicp (OMP)`: APE=6.096±3.056
- `small_vgicp`: APE=5.956±2.725
- PCL GICP の **2.4倍高速**、fast_gicp の **1.9倍高速**

**活用可能性**:
- `IncrementalVoxelMap` + `GICPFactor` + `TBB` で KISS-ICP 相当のオドメトリを MIT で構築可能
- lidarslam の scanmatcher に統合して scan-to-model マッチングに切り替え
- ROS2 メッセージ変換ヘッダーも存在 (`small_gicp/ros/ros2.hpp`)

---

## ROS2 対応・ライセンス調査まとめ

### 使える (ROS2 + MIT/BSD + 動作確認済み)

| 手法 | 分類 | ライセンス | 備考 |
|------|------|-----------|------|
| KISS-ICP | LO | MIT | 最良の LO |
| KISS-SLAM | LO+LC | MIT | pip, ループ検出内蔵 |
| GenZ-ICP | LO | MIT | 屋外チューニング要 |
| small_gicp | 登録ライブラリ/LO | MIT | フル活用で LO パイプライン構築可能 |
| DLIO | LIO | MIT | 最良精度 |
| RKO-LIO | LIO | MIT | シンプル + 高精度 |

### ROS2 非対応だが注目

| 手法 | ライセンス | 状態 | 備考 |
|------|-----------|------|------|
| MAD-ICP | BSD-3 | 活発、ROS2 は TODO | PCA ベース kd-tree, RA-L 2024 |
| DLO | MIT | メンテ停止 | DLIO の前身、ROS1 のみ |
| CT-ICP | MIT | 実質死亡 (2022-07) | 連続時間弾性 ICP |

### ライセンス NG

| 手法 | ライセンス |
|------|-----------|
| FAST-LIO2 / Faster-LIO | GPLv2 |
| Point-LIO | "BSD" (FAST-LIO コード流用疑い) |
| LiLi-OM | GPLv3 |
| MULLS | GPL-3.0 |
| MOLA LiDAR Odometry | GPL-3.0 |

### ビルド問題で未完了

| 手法 | 問題 |
|------|------|
| SiMpLE | nanoflann/dlib 依存で実行時クラッシュ。コミュニティ小、優先度低 |
| LIO-SAM | GTSAM boost→std shared_ptr 非互換 (Jazzy) |

---

## 主要課題

### 課題 1: lidarslam の根本的な精度問題
- **PCL registration (GICP/NDT) のオーバーヘッド** でスキャンドロップが発生
- KISS-ICP のカスタム ICP は共分散計算不要で圧倒的に高速
- サブマップベースのローカルマップ管理がドリフトの原因
- VoxelHashMap を実装したが PCL registration が律速で 11.5m 止まり

### 課題 2: GenZ-ICP の屋外性能
- 構造化環境（室内）向けの最適化が屋外で裏目
- newer_college.yaml 設定の正式テストが未完了
- planarity 無効化 (alpha=0 強制) や カーネル調整でKISS-ICP相当になるか未検証

### 課題 3: small_gicp のポテンシャル未活用
- `IncrementalVoxelMap` を使えば KISS-ICP 的なオドメトリを MIT で構築可能
- だが ROS2 ノードは存在しない（ベンチマークコードのみ）
- lidarslam 内で scan-to-model に切り替える改修が必要

### 課題 4: LIO フロントエンド統合
- DLIO (0.070m) / RKO-LIO (0.082m) を lidarslam のフロントエンドとして使う方針は合意済み
- ブリッジ (Odometry → MapArray) + graph_based_slam ループクロージャーの統合を試みたが、RKO-LIO offline_node と graph_based_slam の use_sim_time 競合で TF エラー発生。RKO-LIO 内部クロックと ROS2 sim_time が干渉
- **根本問題**: graph_based_slam はリアルタイム MapArray 受信が前提。offline ノードとの統合には graph_based_slam 側の改修が必要 (課題 E-5)
- モジュール式フロントエンド設計は online_node 経由なら可能だが、RKO-LIO の online_node はリアルタイム性が必要でスキャンドロップが課題

### 課題 5: 単一データセットでの評価
- 全結果が Newer College math-hard のみ
- MID-360, NTU-VIRAL 等の別データセットでの検証が必要
- 特に GenZ-ICP は構造化環境でリベンジの余地あり

---

## 次のアクション候補

### A. GenZ-ICP チューニング [完了]
- **planarity_threshold=0.5 で RMSE 0.146m 達成** (LO 系 1 位)
- planarity=1.0 は逆効果 (26.2m)。0.5 がスイートスポット
- MID-360 でも 2566 ポーズ取得確認

### B. small_gicp ベースのオドメトリ構築 [実装完了・精度改善中]
- `IncrementalVoxelMap` + `GICPFactor` で ROS2 ノード作成 (`small_gicp_odom_node`)
- **RMSE 4.98m** (ds=0.25, voxel=1.0, corr=1.0) — シンプル版が最安定
- adaptive threshold + CV を実装したが不安定（NaN 発散問題）
- **根本問題**: 共分散計算 (`estimate_covariances_omp`) が律速でスキャンドロップ多発。KISS-ICP は共分散不要のカスタム ICP で圧倒的に速い
- **改善方向**: `ICPFactor` (共分散不要) への切替、または共分散計算のバックグラウンド化

### C. DLIO/RKO-LIO フロントエンド統合 [部分完了]
- **graph_based_slam に Odometry + PointCloud2 直接入力モード追加** (`use_odom_input`)
- **graph_based_slam に cloud-driven サブマップ生成追加** (odom/cloud の同期問題を解決)
- **graph_based_slam ループ検出改善**: ソース点群を複数サブマップ統合に改修
- **RKO-LIO + graph_based_slam**: `publish_odom_tf:=false` で同一ドメイン共存成功。50 サブマップ受信。ループ検出は fitness 1.91 > 閾値で不採用
- **DLIO**: `publish_tf` パラメータ追加済み。graph_based_slam との cloud-driven 同期も改修済みだが、DLIO 内部の TF タイミング問題が残存
- **RKO-LIO**: `publish_odom_tf` パラメータ追加、static TF 要件を文書化

### D. 別データセットでのクロス検証 [部分完了]
- **MID-360**: KISS-ICP 2760 ポーズ、GenZ-ICP 2566 ポーズで正常動作確認 (GT なし)
- NTU-VIRAL 未テスト

### 残課題
1. **small_gicp odom 処理速度**: 共分散計算が律速。`ICPFactor` への切替 or バックグラウンド共分散計算が必要
2. **DLIO + graph_based_slam**: DLIO 内部の TF タイミング問題。DLIO が CPU 速度で全フレーム処理し odom を一気に publish するため graph_based_slam との同期が困難
3. **graph_based_slam ループ検出**: NDT fitness ベースでは math-hard でループ検出困難 (fitness 1.91 > 閾値)。記述子ベース (Scan Context 等) への移行が必要
4. **GT 付きクロス検証**: MID-360 に GT がなく定量評価不可
3. **graph_based_slam ループ検出**: NDT fitness ではなく記述子ベース (Scan Context 等) への移行
4. **GT 付きデータセットでのクロス検証**: MID-360 に GT がないため定量評価不可

### E. graph_based_slam の改善 (将来)

現在の graph_based_slam はシンプルなポーズグラフ最適化 + NDT ベースのループ検出。改善余地が大きい。

#### E-1. ループ検出の改善
- **現状**: NDT スコアベースの scan-to-scan マッチングのみ。閾値 (`threshold_loop_closure_score`) の設定が難しく、Newer College では検出ゼロ
- **改善案**:
  - **Scan Context**: 記述子ベースのループ検出。**ライセンス注意: 既存実装 (irapkaist/scancontext) は GPL のため、フルスクラッチで再実装する必要がある**。アルゴリズム自体は論文公開されているので概念の利用は問題ない
  - **MapClosures** (KISS-SLAM 方式): 占有グリッドマップの局所的な overlap を計算。MIT ライセンスで pip install 可能
  - **BTC (Binary Triangle Combined)**: 幾何特徴ベースの高速ループ検出
  - **距離ベースの候補フィルタリング**: 現在の `distance_loop_closure` は直線距離のみ。走行距離との差分で候補を絞る方が効果的

#### E-2. ポーズグラフ最適化の改善
- **現状**: g2o ベースの簡易最適化
- **改善案**:
  - **GTSAM への移行**: より柔軟なファクターグラフ、iSAM2 による増分最適化 (ただし Jazzy では boost→std 互換問題あり)
  - **Robust kernel の導入**: 誤ループ検出に対する頑健性向上 (Cauchy, Huber 等)
  - **マルチセッション対応**: 複数回の走行データを統合して最適化

#### E-3. サブマップ管理の改善
- **現状**: 全サブマップを保持。メモリ使用量が無制限に増加
- **改善案**:
  - **LRU ベースのサブマップ削除**: 古い・遠いサブマップを自動削除
  - **サブマップの統合**: 近いサブマップを merge して数を減らす
  - **階層的マップ**: ローカル (高密度) + グローバル (低密度) の 2 層構造

#### E-4. マップ品質の改善
- **現状**: ループ検出後にポーズのみ修正。マップ点群は修正されない
- **改善案**:
  - **マップ点群のリファイン**: ループ閉合後にサブマップの点群も再変換
  - **占有グリッドマップの構築**: 2D/3D 占有グリッドを並行して構築
  - **セマンティック情報の活用**: 地面/壁/天井の分類でマッチング精度向上

#### E-5. LIO フロントエンドとの連携改善
- **現状**: MapArray 経由の疎結合。ブリッジノードが必要
- **改善案**:
  - **ネイティブ Odometry 入力対応**: MapArray だけでなく Odometry + PointCloud2 を直接受け取る
  - **キーフレーム選択ロジック**: フロントエンドの品質指標 (fitness, inlier数) に基づくキーフレーム選定
  - **共分散情報の伝播**: フロントエンドの推定共分散をグラフの辺の重みに反映

---

## 技術的知見まとめ

### DLIO vs RKO-LIO (1.2cm の差の理由)
| 要素 | DLIO (0.070m) | RKO-LIO (0.082m) |
|------|---------------|-------------------|
| IMU 統合 | Jerk ベース 3次連続モデル | 定加速度 + カルマンフィルタ |
| デスキュー | 各点ごとの SE(3) 補間 | フレーム境界間の補間 |
| マッチング | NanoGICP (共分散あり, k=16) | カスタム point-to-plane ICP |
| マップ | キーフレーム + 凸/凹ハル選択 | Bonxai 疎ボクセルグリッド |
| バイアス補正 | 毎フレームリアルタイム更新 | 初期キャリブのみ |
| 速度 | 7.9ms/frame | ~5ms/frame |

### lidarslam に加えた改善
1. 非単調タイムスタンプスキップ
2. VoxelHashMap (KISS-ICP 着想)
3. 全 registration method 対応の適応閾値
4. FAST_GICP / FAST_VGICP / SMALL_GICP / SMALL_VGICP 統合
5. GenZ-ICP ライブラリ名衝突修正 (`libodometry_component.so` 問題)
6. cloud_queue_depth パラメータ

### 重要ファイル
- ベンチマークレポート: `output/benchmark_report.md`
- VoxelHashMap 実装: `scanmatcher/include/scanmatcher/voxel_hash_map.hpp`
- scanmatcher 本体: `scanmatcher/src/scanmatcher_component.cpp`
- GenZ-ICP 設定: `Thirdparty/genz-icp/ros/config/newer_college.yaml`
- small_gicp オドメトリ例: `Thirdparty/small_gicp/src/benchmark/odometry_benchmark_small_gicp_model_tbb.cpp`
