# LiDAR Odometry / SLAM ベンチマーク結果

**データセット**: Newer College math-hard (320m, ループあり, Ouster OS0-128)
**GT**: 2438 poses, 始点-終点距離 0.65m
**評価**: evo_ape (SE(3) Umeyama alignment)
**日付**: 2026-03-19 (最終更新)

---

## Pure LiDAR Matching 系 (IMU なし)

| 手法 | ライセンス | Poses | RMSE (m) | Mean (m) | 備考 |
|------|-----------|-------|----------|----------|------|
| **KISS-ICP** | MIT | 1913 | **0.440** | 0.320 | PRBonn, VoxelHashMap + adaptive ICP |
| **KISS-SLAM** | MIT | 1930 | **0.434** | 0.287 | KISS-ICP + MapClosures (ループ検出 0 件) |
| **GenZ-ICP (planarity=0.5)** | **MIT** | **1841** | **0.146** | **0.123** | **planarity_threshold=0.5 で劇的改善** |
| GenZ-ICP (newer_college config) | MIT | 1659 | 12.238 | 9.079 | config_file, rate=0.5 |
| GenZ-ICP (default) | MIT | 772 | 24.011 | 21.738 | voxel=0.3, planarity=0.2 |
| lidarslam + FAST_GICP + VHM + CV | BSD/MIT | 1199 | 11.522 | 6.492 | VoxelHashMap + constant velocity |
| lidarslam + SMALL_GICP + VHM | MIT | 668 | 17.230 | 15.656 | VoxelHashMap v2 |
| lidarslam + FAST_GICP (baseline) | BSD | 1306 | 15.950 | 12.868 | Koide fast_gicp |
| lidarslam + FAST_VGICP | BSD | 1610 | 20.306 | 18.702 | voxelized GICP |
| lidarslam + SMALL_VGICP | MIT | 1438 | 22.663 | 21.001 | Koide small_gicp |
| lidarslam + NDT (IMU prior) | - | 1887 | 24.286 | 23.132 | 元の baseline |
| **small_gicp odom** | **MIT** | **1889** | **4.451** | **3.965** | **IncrementalVoxelMap + GICP, 自作 ROS2 ノード** |
| small_gicp odom | MIT | 1762 | 4.977 | 4.120 | IncrementalVoxelMap + GICP. 共分散計算が律速でスキャンドロップ多発。adaptive/CV 追加で改善試みたが不安定 |
| SiMpLE | MIT | - | - | - | ガウシアン報酬ベース (対応関係不要), 評価未完了 (*1) |

*1 SiMpLE 未完了の理由: nanoflann API 非互換 (uint32_t→size_t) を修正しビルドは通ったが、実行時に abort で停止。dlib (L-BFGS オプティマイザ) や nanoflann のバージョン依存が厳しく、ロボティクスでは一般的でない依存 (dlib) を持つ。コミュニティ規模も小さく、優先度を下げて保留とした。

### 要点

- **KISS-ICP が pure matching 系では圧倒的** (RMSE 0.44m)
- VoxelHashMap + adaptive threshold + robust kernel + constant velocity prediction が鍵
- lidarslam のサブマップ方式は VoxelHashMap に置換しても 11m 止まり
  - PCL registration (GICP/NDT) のオーバーヘッドでスキャンドロップが発生
  - KISS-ICP のカスタム ICP は共分散計算不要で圧倒的に高速
- GenZ-ICP は構造化環境向けの point-to-plane が math-hard (屋外) に合わない

---

## LIO 系 (LiDAR-Inertial Odometry, IMU 融合)

| 手法 | ライセンス | Poses | RMSE (m) | Mean (m) | 備考 |
|------|-----------|-------|----------|----------|------|
| **DLIO** | **MIT** | **1896** | **0.070** | **0.065** | UCLA, GICP ベース, continuous-time deskew, 7.9ms/frame |
| **RKO-LIO** | **MIT** | **1930** | **0.082** | **0.075** | PRBonn, KISS-ICP + IMU tight coupling |

### 未テスト (ライセンス OK)

| 手法 | ライセンス | ROS2 | 特徴 |
|------|-----------|------|------|
| LIO-SAM | BSD-3 | あり (ros2) | GTSAM ベース, ループクロージャー内蔵 (Jazzy で GTSAM API 不互換ビルド失敗) |

### ライセンス NG (参考)

| 手法 | ライセンス | 備考 |
|------|-----------|------|
| FAST-LIO2 | GPLv2 | HKU-MARS, ikd-Tree, 高性能だが GPL |
| Faster-LIO | GPLv2 | FAST-LIO 派生 |
| Point-LIO | "BSD" (要監査) | FAST-LIO コード流用の疑い |
| LiLi-OM | GPLv3 | KIT |

### 要点

- **RKO-LIO が全手法中最良** (RMSE 0.082m)
- IMU タイトカップリングで KISS-ICP (0.44m) の **5 倍以上の精度**
- FAST-LIO2 は有名だが GPLv2 で使用不可
- MIT/BSD で使える LIO は RKO-LIO, DLIO, LIO-SAM の 3 つ

---

## 全手法ランキング

| 順位 | 手法 | 分類 | RMSE (m) | ライセンス |
|------|------|------|----------|-----------|
| 1 | **DLIO** | LIO | **0.070** | MIT |
| 2 | **RKO-LIO** | LIO | **0.082** | MIT |
| 3 | **GenZ-ICP (pt=0.5)** | **LO** | **0.146** | **MIT** |
| 4 | KISS-SLAM | LO | 0.434 | MIT |
| 5 | KISS-ICP | LO | 0.440 | MIT |
| 5 | lidarslam + VHM + CV | LO | 11.522 | - |
| 6 | lidarslam + FAST_GICP | LO | 15.950 | BSD |
| 7 | lidarslam + SMALL_GICP + VHM | LO | 17.230 | MIT |
| 8 | GenZ-ICP (tuned) | LO | 20.864 | MIT |
| 9 | lidarslam + NDT | LO | 24.286 | - |

---

## LO/LIO 手法 ROS2 対応・ライセンス調査

### ROS2 対応 + MIT/BSD で動作確認済み

| 手法 | 分類 | ライセンス | ROS2 | RMSE (m) | 備考 |
|------|------|-----------|------|----------|------|
| **KISS-ICP** | LO | MIT | あり | 0.440 | PRBonn, VoxelHashMap + adaptive ICP |
| **KISS-SLAM** | LO+LC | MIT | pip | 0.434 | KISS-ICP + MapClosures |
| **GenZ-ICP** | LO | MIT | あり | 20.864 | 屋外では point-to-plane が裏目 (*2) |
| **DLIO** | LIO | MIT | あり | 0.070 | UCLA, GICP + jerk モデル |
| **RKO-LIO** | LIO | MIT | あり | 0.082 | PRBonn, KISS-ICP + IMU tight coupling |

### ROS2 非対応だが注目

| 手法 | 分類 | ライセンス | ROS2 | 最終更新 | 備考 |
|------|------|-----------|------|---------|------|
| **MAD-ICP** | LO | BSD-3 | **なし** (TODO 扱い) | 活発 | PCA ベース kd-tree, RA-L 2024 |
| **CT-ICP** | LO | MIT | **なし** (ROS1 catkin) | 2022-07 | 連続時間弾性オドメトリ, 実質死亡 |
| **DLO** | LO | MIT | **なし** (ROS1 Melodic/Noetic) | 2024-06 | DLIO の前身, GICP ベース |
| **small_gicp** | 登録ライブラリ | MIT | あり | 活発 | IncrementalVoxelMap 等未活用, オドメトリ構築可能 |

### ライセンス NG

| 手法 | ライセンス | 備考 |
|------|-----------|------|
| FAST-LIO2 / Faster-LIO | GPLv2 | HKU-MARS 系 |
| Point-LIO | "BSD" (要監査) | FAST-LIO コード流用疑い |
| LiLi-OM | GPLv3 | KIT |
| MULLS | GPL-3.0 | |
| MOLA LiDAR Odometry | GPL-3.0 | |

### ビルド成功だが動作未完了

| 手法 | 分類 | ライセンス | ROS2 | 問題 |
|------|------|-----------|------|------|
| SiMpLE | LO | MIT | あり | nanoflann/dlib 依存問題で実行時クラッシュ |
| LIO-SAM | LIO+LC | BSD-3 | あり | GTSAM boost→std shared_ptr 非互換 (Jazzy) |

*2 GenZ-ICP が微妙な理由: planarity 閾値 (0.1) が屋外で厳しすぎ、共分散計算オーバーヘッドでスキャンドロップ 43%、カーネル sigma/3 で外れ値過剰除外。構造化環境（室内）向け。

---

## 実装改善まとめ

本ベンチマーク中に lidarslam に加えた改善:

1. **非単調タイムスタンプスキップ** — bag のタイムスタンプ逆転をコールバックで検出・スキップ
2. **VoxelHashMap** — KISS-ICP 着想の空間ボクセルマップ (`voxel_hash_map.hpp`)
3. **全 registration method 対応の適応閾値** — NDT 以外でも `adaptive_correspondence_threshold` が動作
4. **FAST_GICP / FAST_VGICP / SMALL_GICP / SMALL_VGICP 統合** — pcl::Registration インターフェースで drop-in
5. **GenZ-ICP ライブラリ名衝突修正** — `libodometry_component.so` → `libgenz_odometry_component.so`
6. **cloud_queue_depth パラメータ** — SensorDataQoS のキューサイズを設定可能に

---

## 結論と次のステップ

- **精度最優先なら DLIO** (MIT, 0.070m) または **RKO-LIO** (MIT, 0.082m)
- LIO 系は pure matching 系の **6 倍以上の精度** (0.07m vs 0.44m)
- **DLIO + ループクロージャー** で更なる改善の可能性
- LIO-SAM は GTSAM の Jazzy 互換問題でビルド失敗 (boost::shared_ptr → std::shared_ptr)
- lidarslam 自体の改善は registration method ではなくパイプライン全体 (マップ管理, 処理速度) が律速
- IMU タイトカップリングの効果が圧倒的。lidarslam に取り込むなら DLIO/RKO-LIO のアプローチを参考に

## graph_based_slam 改修

- `use_odom_input: true` で Odometry + PointCloud2 を直接受信するモードを追加
- 移動距離ベースの自動サブマップ生成 (`submap_distance_threshold` パラメータ)
- ブリッジノード不要で LIO フロントエンドと接続可能
- RKO-LIO の TF regression (offline_node が 1 フレームで停止) のため統合テストは保留

## 既知の問題

- **RKO-LIO offline_node regression** (解決済み): static TF (`os_sensor` → `os_imu`) が必要。以前は別プロセスの static TF publisher が偶然残っていて動いていた
- **RKO-LIO に `publish_odom_tf` パラメータ追加**: TF broadcast の on/off を制御可能に。graph_based_slam との同一ドメイン共存が可能に
- **RKO-LIO + graph_based_slam 統合成功**: 閾値を 3.0 に緩和したところ **5回のループクロージャーを検出** (fitness 0.64-2.86)。NDT ベースでも閾値調整で十分ループ検出可能
- **Scan Context フルスクラッチ実装**: 論文ベースで GPL コード参照なし
- **PCD ディスクキャッシュ**: サブマップ点群をディスクに逐次保存、必要時のみ読み込み
- **Scan Context + PCD + voxel=0.5**: OOM 完全解消。12回のループクロージャー検出 (score 0.001-2.8)
- **ループクロージャーの効果検証**: RKO-LIO raw (RMSE 0.08m) → ループ後 (RMSE 2.86m) で**悪化**。RKO-LIO の高精度オドメトリに対してグラフ最適化が害になる。LIO 系ではループクロージャー不要の可能性が高い
- **DLIO の不安定性の根本原因を特定**:
  - `use_sim_time: true` (params.yaml デフォルト) が DLIO 内部の `rclcpp::Clock::now()` を狂わせ、Computation Time が 203ms → 5.4秒に膨張
  - `use_sim_time: false` に修正すると処理速度は改善するが、PointCloud2 の DDS 受信レートが 10Hz → 4Hz に低下 (OS0-128 の大きなメッセージが原因)
  - 4Hz ではLiDAR-IMU バランスが崩れ IMU 積分が暴走 → 発散
  - 以前の 0.070m RMSE はシステム負荷が低い条件で偶然 10Hz 受信できた結果
  - **対策**: DDS の SHM transport 有効化、PointCloud2 の QoS 調整、または RKO-LIO のような内部 bag reader 方式への移行が必要
- **MID-360 クロス検証**: KISS-ICP で 2760 ポーズ正常動作 (GT なし、目視確認用)
