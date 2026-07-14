# Repository Agent Instructions

Always prefix shell commands with `rtk`.

Examples:

```bash
rtk git status
rtk python3 -m pytest -q
rtk colcon test --packages-select lidarslam
```

---

# lidarslam-ros2

ROS 2 LiDAR SLAM パッケージ。RKO-LIO フロントエンド + graph_based_slam バックエンドで Autoware 互換の pointcloud map を生成する。

## パッケージ構成

- `lidarslam` - 統合パッケージ（launch ファイル）
- `scanmatcher` - フロントエンド（NDT/FastGICP/SmallGICP）
- `graph_based_slam` - バックエンド（ループクロージャ、ポーズグラフ最適化）
- `lidarslam_msgs` - カスタムメッセージ（MapArray, SubMap）
- `Thirdparty/` - サブモジュール群（rko_lio, ndt_omp_ros2 等）

## ビルド

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## テスト

```bash
# ローカル CI（ビルド + テスト）
bash scripts/run_default_ci_checks.sh

# パッケージ単位テスト
colcon test --packages-select lidarslam_msgs scanmatcher graph_based_slam lidarslam
colcon test-result --verbose
```

## 主要な実行コマンド

```bash
# SLAM 実行
ros2 launch lidarslam rko_lio_slam.launch.py lidar_topic:=... imu_topic:=...

# グラフベース SLAM
ros2 launch graph_based_slam graphbasedslam.launch.py map_array:=...

# マップ保存
ros2 service call /map_save std_srvs/srv/Empty
```

## コーディング規約

- C++14（graph_based_slam, lidarslam）、C++17（scanmatcher）
- clang-format: LLVM ベース、120 カラム
- clang-tidy: 厳格（WarningsAsErrors: "*"）
- ライセンス: BSD-2-Clause（デフォルトパスに GPL 不可）

## 対応 ROS 2 ディストロ

- Humble (Ubuntu 22.04)
- Jazzy (Ubuntu 24.04)

## 依存関係

ROS 2, PCL, Eigen3, G2O, OpenMP

## CI

GitHub Actions（main.yml）で Humble + Jazzy マトリクスビルド＋テスト。
リリースゲート: APE RMSE <= 0.10m + Autoware マップ検証。

## スクリプト一覧

### SLAM・ベンチマーク
| スクリプト | 用途 |
|-----------|------|
| `run_default_ci_checks.sh` | ローカル CI（ビルド＋テスト）|
| `run_release_readiness_checks.sh` | リリースゲート（APE 閾値、`--offline-determinism-bag` / `--frontend-determinism-bag` で決定論ハードゲート）|
| `run_offline_determinism_check.sh` | オフラインバックエンド決定論ゲート（N-run バイト一致 + 任意 APE レポート。`--ape-interpolate` で疎チェックポイント GT、`--save-maps` で refined マップ出力）|
| `run_frontend_determinism_check.sh` | オフラインフロントエンド決定論ゲート（scanmatcher lockstep、N-run バイト一致）|
| `run_map_quality_check.sh` | マップ品質メトリクス（MME/平面厚/coverage、N-run バイト一致 + `--profile` で閾値表照合。release gate は `--map-quality-pcd <path>[@<profile>]` / `--offline-determinism-map-quality-profile`）|
| `run_rko_lio_graph_benchmark.sh` | ベンチマークパイプライン |
| `evaluate_slam_candidate_regression.py` | Phase 7候補をMID360/HILTIのATE・RPE・地図品質・実行時間でbaseline比較 |
| `run_plane_revisit_candidate_benchmark.sh` | plane-revisit OFF/ON実行からPhase 7 manifest生成までを自動化 |
| `record_backend_input.sh` | 任意frontend実行中のbackend入力2 topicを検証付きでMCAP記録 |
| `analyze_sparse_checkpoint_errors.py` | sparse位置GTで候補別・checkpoint別のSE(3)整列誤差をJSON/Markdown化 |
| `run_autoware_quickstart.sh` | NTU VIRAL → Autoware マップ E2E |
| `download_ntu_viral_tnp01.sh` | NTU VIRAL データダウンロード |
| `download_hilti2022.sh` | HILTI 2022 DL + ROS2 変換（`--sequence exp01\|exp07`、外部 mm-GT 評価基盤、PandarXT-32）|
| `run_docker_demo.sh` | Docker ワンコマンドデモ（MID-360 bag DL → headless SLAM）|

### Autoware・ユーティリティ
| スクリプト | 用途 |
|-----------|------|
| `download_autoware_artifacts.sh` | Autoware ML モデルダウンロード |
| `simple_lanelet2_generator.py` | TUM 軌跡 → lanelet2 OSM |
| `build_autoware_map_from_slam.sh` | SLAM 出力 → Autoware マップ（PCD変換+lanelet2+projector）|
| `record_screen.sh` | 画面録画 |

## PR ガイドライン

- 狭くて明確な PR を推奨
- 再現可能なコマンド + ベンチマーク結果を添付
- ローカルチェック: `run_default_ci_checks.sh`, `run_release_readiness_checks.sh`
