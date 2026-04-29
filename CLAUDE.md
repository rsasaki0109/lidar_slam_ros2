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

## AWSIM × Autoware 自動運転パイプライン

AWSIM シミュレータで LiDAR データを取得 → lidarslam でマップ生成 → Autoware で自動運転。

```bash
# セットアップ確認
bash scripts/test_awsim_setup.sh

# サンプルマップでデモ（3ターミナル）
bash scripts/run_awsim_autoware_demo.sh awsim      # AWSIM
bash scripts/run_awsim_autoware_demo.sh autoware    # Autoware
bash scripts/run_awsim_autoware_demo.sh engage      # 自動運転開始

# 自作マップでワンコマンドデモ
bash scripts/run_awsim_selfmade_map_demo.sh

# 軌跡から lanelet2 生成
python3 scripts/simple_lanelet2_generator.py \
  --input output/.../traj_corrected.tum \
  --output map/lanelet2_map.osm \
  --lane-width 3.5 --origin-lat ... --origin-lon ... --resolution 1.0
```

### AWSIM 注意事項

- AWSIM は Docker 内で起動必須（ホスト Jazzy と衝突する）
- Autoware: `ghcr.io/autowarefoundation/autoware:universe-cuda` (Humble)
- AWSIM LiDAR にタイムスタンプなし → lidarslam は `deskew:=false`
- NDT 閾値: SLAM マップ品質次第で 2.3 → 1.5 に調整が必要な場合あり
- lanelet2: 複数 lanelet に分割 + 境界ノード共有が Autoware ルーティングに必須

詳細: `docs/awsim-autonomous-driving-tutorial.md`

## スクリプト一覧

### SLAM・ベンチマーク
| スクリプト | 用途 |
|-----------|------|
| `run_default_ci_checks.sh` | ローカル CI（ビルド＋テスト）|
| `run_release_readiness_checks.sh` | リリースゲート（APE 閾値）|
| `run_rko_lio_graph_benchmark.sh` | ベンチマークパイプライン |
| `run_autoware_quickstart.sh` | NTU VIRAL → Autoware マップ E2E |
| `download_ntu_viral_tnp01.sh` | NTU VIRAL データダウンロード |

### AWSIM・Autoware
| スクリプト | 用途 |
|-----------|------|
| `test_awsim_setup.sh` | AWSIM + Autoware セットアップ確認 |
| `run_awsim_autoware_demo.sh` | サンプルマップ AWSIM デモ |
| `run_awsim_selfmade_map_demo.sh` | 自作マップ自動運転デモ |
| `download_autoware_artifacts.sh` | Autoware ML モデルダウンロード |
| `simple_lanelet2_generator.py` | TUM 軌跡 → lanelet2 OSM |
| `build_autoware_map_from_slam.sh` | SLAM 出力 → Autoware マップ（PCD変換+lanelet2+projector）|
| `record_screen.sh` | 画面録画 |

## PR ガイドライン

- 狭くて明確な PR を推奨
- 再現可能なコマンド + ベンチマーク結果を添付
- ローカルチェック: `run_default_ci_checks.sh`, `run_release_readiness_checks.sh`
