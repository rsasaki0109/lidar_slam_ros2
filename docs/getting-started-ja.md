# 日本語クイックスタート

`lidarslam_ros2`は、rosbag2からAutowareで読み込める地図一式を作成します。
初めての場合は、手元の環境に合う入口を1つだけ選んでください。

## まず選ぶ

| 手元にあるもの | 最初のコマンド |
| --- | --- |
| インストール済みだが、状態が分からない | `lidarslam-map doctor` |
| Dockerだけで固定デモを試したい | [Dockerで固定デモ](#docker-demo) |
| sourceから構築したい | `bash scripts/source_quickstart.sh` |
| 対応する自分のrosbag2がある | `lidarslam-map start /path/to/rosbag2` |
| bagのtopicや互換性が分からない | `lidarslam-map doctor /path/to/rosbag2` |

インストール後に端末で引数なしの`lidarslam-map`を実行すると、インストール確認、
固定デモ、自分のbag、過去のsessionから安全な入口を選べます。

## 1. インストールを確認する

```bash
lidarslam-map doctor
```

この確認はネットワークへ接続せず、ファイルも書きません。ROS 2、bag reader、
製品ファイル、デモ用の空き容量を確認し、不足ごとに理由コードと次の1コマンドを
表示します。自分のbagを先に確認する場合は次を実行します。

```bash
lidarslam-map doctor /path/to/rosbag2
```

これはtopic、PointCloud2のfield、timestamp順、利用可能なprofileを確認します。

## 2. Dockerで固定デモ {#docker-demo}

ROS 2 workspaceを構築せず、固定MID-360デモから検証済み地図を作ります。

```bash
docker run --rm \
  -e LIDARSLAM_HOST_UID="$(id -u)" \
  -e LIDARSLAM_HOST_GID="$(id -g)" \
  -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble
```

このコマンドは公開済み安定版`v0.9.0-humble`に固定しています。`develop`の
移動タグを使わないため、初回導線の内容が不意に変わりません。`v0.9.1`は
まだ公開・tag付けされていない候補版なので、候補版を試す場合はsource helperを
使ってください。Ubuntu 24.04/Jazzyでは
`ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy`を使います。初回は517 MBの公開bagを
取得し、最低8 GiBの空き容量を使います。目安は約30分ですが、回線とCPUで
変わります。出力先は`lidarslam_output/mid360_demo/`です。

## 3. sourceから構築する

Ubuntu 22.04/HumbleまたはUbuntu 24.04/JazzyにROS 2を導入してから実行します。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git
cd lidar_slam_ros2
bash scripts/source_quickstart.sh
```

helperはHumble/Jazzyを検出し、このrepositoryの6 packageだけを準備・buildして、
固定デモまで実行します。実行内容だけを先に確認する場合は次を使います。

```bash
bash scripts/source_quickstart.sh --dry-run
```

buildだけなら`--build-only`、画面を開かない環境なら`--viewer none`を追加します。
完了時に表示される絶対パスの`lidarslam-map`は、新しい端末でも対応するworkspaceを
自動で有効化します。

!!! note "現在の配布境界"
    GLIMのようなPPA/package-managerの導入経路は、依存packageのrosdistro審査が
    完了するまで未対応です。現時点の正規経路はDockerまたは上記source helperです。

## 4. 自分のbagを地図にする

インストール済みの場合は、`metadata.yaml`を含むrosbag2 directoryを渡します。

```bash
lidarslam-map start /path/to/rosbag2
```

`start`は書き込み前にsensor、topic、timestamp、profile、calibrationを説明し、
確認後にmapping、地図検証、offline reviewまで進みます。長時間処理の前に確認だけ
行うには次を使います。

```bash
lidarslam-map start /path/to/rosbag2 --dry-run
```

ROS 2をhostへインストールせず、checkout済みrepositoryからDockerを使う場合は
次の1コマンドです。bagはread-onlyでmountされます。

```bash
bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2
```

## 成功と失敗の見分け方

成功時は`map_verify: PASS`とともに、次の場所が1画面に表示されます。

- Autoware用`pointcloud_map/`
- `map_projector_info.yaml`
- 自動生成された`lanelet2_map.osm`
- 検証receiptとoffline review

失敗時は`[reason-code]`と`Next:`を探してください。安全に再開できるデモの
post-processingだけが残った場合は、表示された次の形式をそのまま実行します。

```bash
lidarslam-map demo /path/to/work_dir --resume
```

`--resume`はmappingを再実行せず、安全なterminal post-processingだけを再開します。
状態が不明なbagは`lidarslam-map doctor /path/to/rosbag2`へ戻ってください。

### mapまたはviewerが空のとき: 3つの確認

初回は自分のbagやframeを変更する前に、固定公開デモをviewerなしで実行して
基準結果を確認します。

```bash
lidarslam-map demo ~/ros2_ws --viewer none
```

live入力を確認する場合は、まずtopicの型を確認します。

```bash
ros2 topic list -t
```

出力のうち`[sensor_msgs/msg/PointCloud2]`と表示された行のtopic名を選び、
`<POINTCLOUD_TOPIC>`をその名前に置き換えます。例えば
`/points [sensor_msgs/msg/PointCloud2]`なら、コマンドには`/points`だけを入れます。
PointCloud2の行がない場合は、publisherまたはlaunchのremapを直してから再確認します。
山括弧を残したまま実行せず、次の順に確認します。

1. **PointCloud2が届いているか**

   ```bash
   timeout 5s ros2 topic hz --window 5 <POINTCLOUD_TOPIC>
   ```

   `average rate:`が正の値で続けば入力は到達しています。0またはpublisherなしなら、
   topicのremapまたはpublisherを直してから再確認します。

2. **`frame_id`が空でないか**

   ```bash
   timeout 5s ros2 topic echo --once --field header.frame_id <POINTCLOUD_TOPIC>
   ```

   出力されたframe名を次のTF確認にそのまま使います。出力が空、または5秒で
   timeoutした場合は、publisherの`header.frame_id`を修正してから再確認します。
   viewerのframe名を推測して先に進めません。

3. **TFがつながっているか**

   check 2で出た空でないframe名を`<POINTCLOUD_FRAME>`に入れます。
   `<TF_TARGET_FRAME>`にはruntimeまたはviewerが基準にするtarget frameを入れます。
   例えばcheck 2の出力が`livox_frame`なら、`<POINTCLOUD_FRAME>`だけを
   `livox_frame`に置き換えます。viewerでframe名を推測したり、山括弧を残したまま
   実行したりしません。

   ```bash
   ros2 run tf2_ros tf2_echo <TF_TARGET_FRAME> <POINTCLOUD_FRAME>
   ```

   `At time ...`が繰り返し表示されることを確認します。利用できない場合は、
   この2つのframeのsource/targetの向きとstatic extrinsicを直してから、同じ実際の
   frame名で再確認します。

3つが通ってもmap messageがない場合は、viewer設定を変える前に次を確認します。

```bash
timeout 5s ros2 topic echo --once /map/pointcloud_map
lidarslam-map inspect /path/to/output --write
```

messageがなければ生成された`autoware_map_diagnosis.md`の最初のfindingに従います。
messageがあればmapは存在するため、viewerのfixed frameを`map`、topicを
`/map/pointcloud_map`に設定し、offline previewを確認します。bag、map、raw logを
uploadする必要はありません。

## 詳細

このページは最短経路だけを示します。すべてのoption、対応input、校正、復旧、
自動化contractは英語版の[Getting Started](getting-started.md)と
[Operator Workflows](workflows.md)を参照してください。正規コマンドや安全境界に
差がある場合は英語版が優先されます。
