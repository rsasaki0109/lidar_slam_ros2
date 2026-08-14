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

### JSON診断でreason-codeとNext-actionを読む

自分のbagをmappingへ渡す前に、診断結果を機械可読な形でも確認できます。
このコマンドはbagをread-onlyで読み、ネットワークへ接続せず、出力ファイルも作りません。

```bash
lidarslam-map doctor /path/to/rosbag2 --json
```

doctorのbag reportでは`findings[].code`を安定したキーとして使います。`start`の
dry-run JSONや保存済みsessionのrecovery JSONでは、全体の`reason.code`と各項目の
`findings[].code`を使います。`message`、`Details:`、viewerで見えた症状、英語の説明文を
手がかりにコードを推測したり、automationのキーにしたりしません。

診断結果の`next_action`、`Next:`、`next_command`は保持された次の操作です。自分で
pathやoptionを組み立て直さず、そのcommandを確認してからそのまま実行します。JSONには
bagのlocal pathやtopic名が含まれる場合があるため、raw JSONをissueへ貼り付けません。
支援が必要なときは、後述のサニタイズ済みsupport reportだけを内容確認後に使います。

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

### versionとsupport境界を記録する

supportや独立validationへ進む前に、実際に使ったCLIのversionとrevisionを記録します。
これはネットワークへ接続せず、mapやsessionを書き込みません。

```bash
lidarslam-map --version
```

Dockerの固定デモは公開済み安定版`v0.9.0-humble`または`v0.9.0-jazzy`を使います。
`v0.9.1`はまだ公開・tag付けされていないsource候補なので、source helperで実行した
場合はcandidateとしてそのversion/revisionを報告します。`develop`の移動tagを使ったり、
出力にないrelease identityを推測したりせず、`--version`の出力をそのままsupport report
やvalidation formへ転記してください。

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

### 自分のbagを先にdry-runで確認する

長時間のmappingやファイル作成へ進む前に、選択されるprofile、topic、frame、
calibration、実行予定のcommandをJSONで確認できます。

```bash
lidarslam-map start /path/to/rosbag2 \
  --yes \
  --dry-run \
  --json
```

`--dry-run`ではsession bundle、`sensor_setup.json`、map outputを残さず、mappingも
開始しません。`--yes`はこの確認を非対話で実行するための指定であり、dry-runを外した
実行の書き込みを許可するものではありません。readyな結果ではJSONの`status`が
`dry_run`になり、`run.command_shell`に表示された保持済みの実行commandを確認できます。

安全なprofileを選べない場合は終了コード2になり、`reason.code`、`findings[].code`、
各findingの`next_action`、`next_command`が診断を示します。sessionやmapはこの場合も
作成されません。`next_command`がdoctorを示すときはそのcommandへ戻り、viewerの症状から
mapping commandを組み立て直しません。

確認したplanで実際にmappingへ進むときは`--dry-run`を外し、端末では表示内容を確認して
から開始します。ヘッドレス環境ではbrowserを起動せず成果物を保持するため、明示的に
`--viewer none`を使います。

```bash
lidarslam-map start /path/to/rosbag2 --viewer none
```

非対話で実行する場合も、planをレビューした後だけ`--yes --viewer none`を追加します。
JSONや表示されたcommandにはlocal pathが含まれる場合があるため、確認結果はlocal-onlyで
扱い、raw outputをissueへ貼り付けません。

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

失敗時は`[reason-code]`、`Details:`、`Next:`を探してください。保持された次の操作が
安全なterminal post-processingだけを再開する`--resume`なら、表示されたcommandを
そのまま実行します。

```bash
lidarslam-map demo /path/to/work_dir --resume
```

`--resume`はmappingを再実行せず、安全なterminal post-processingだけを再開します。
bagのtopic、field、timestamp、profileが不明、または診断が入力修復を求めている場合は、
mappingやviewerを再試行せず`lidarslam-map doctor /path/to/rosbag2 --json`へ戻ってください。
`reason.code`や`findings[].code`の`Next:`が新しいoutput directoryでのretryを示す場合だけ、
保持されたcommandを使い、失敗したrunを上書きしません。

### 検証済みmapと表示できるmapを区別する

viewerで`pointcloud_map/`やpreviewが開けても、それだけではtrusted resultではありません。
表示できることはmap outputが存在することを示すだけで、Autoware verificationや証跡の整合性が
確認されたことは示しません。trusted resultとして扱えるのは、同じrunで`map_verify: PASS`が
出て、保存された`first_map_validation_receipt.json`の`status: PASS`（receipt内の全checkもPASS）を
確認できる場合だけです。

| 表示・receiptの状態 | 意味 | 次の操作 |
| --- | --- | --- |
| `map_verify: PASS` とreceiptの`status: PASS` | map、verification、manifestに結び付いた証跡が揃ったtrusted result | versionとreceiptを確認してから、必要ならsupportまたは独立validationへ進む |
| viewerだけ表示される、または`NOT VERIFIED` | map outputはあってもverificationを実行していない・完了していない | trusted evidenceとして共有せず、保存されたdiagnosisと`inspect` commandを先に読む |
| `UNAVAILABLE`、receiptの欠落・不正・`FAIL` | receiptから検証結果を確定できない | viewerの見た目からPASSを推測せず、同じsessionの証跡をinspectする |

`NOT VERIFIED`は「mapが失敗した」という推測ではなく、verificationが未実行または未完了という
境界です。`UNAVAILABLE`はreceiptがない、壊れている、または必要な証跡が揃っていない状態で、
近くにあるmapやpreviewからPASSを補いません。まず同じoutput/sessionの
`first_map_validation_receipt.json`を開き、`status`、`verification`、checkの結果を読みます。
別sessionからコピーしたreceiptを現在のmapの証拠には使いません。

`status`がPASSでない場合は、supportや独立validatorへ渡す前に、保存された
`autoware_map_diagnosis.md`/JSON、`verify_autoware_map.log`、`run_manifest.json`を読み、保持された
`Details:`と`Next:`に従います。必要なら次のread-only診断を実行します。

```bash
lidarslam-map inspect /path/to/output --write
```

`inspect`の結果が示す原因を直して同じrunを再確認するまで、表示されたmapをverified resultとして
扱いません。独立validationへ進む場合も、内容をreviewしたPASS receiptだけを使い、map、bag、
raw log、preview、local pathを添付しません。

### receiptのsessionとstatusを確認する

`first_map_validation_receipt.json`だけを別のmap directoryへコピーしても、そのmapの証拠には
なりません。receiptを読むときは、同じsessionの`session.json`にある
`artifacts.validation_receipt`のpathと`map_output`を基準にし、receiptの`run.run_id`が
同じ`run_manifest.json`の`run_id`と一致することを確認します。receiptの
`verification.manifest_sha256`は、そのsessionの`run_manifest.json`に結び付いた値です。
別runのreceipt、古いsessionのreceipt、名前だけ変更したreceiptを混ぜません。

sessionを一覧から選ぶ場合は、local-onlyのJSONを確認します。

```bash
lidarslam-map sessions ./output --json
```

独立validationやsupportへ進む前の最終ゲートは、元のsession bundleに対する次のread-only
再検証です。

```bash
lidarslam-map support /path/to/session_bundle --first-map
```

このcommandは、sessionがverifiedであること、receiptがそのsession内の通常ファイルであること、
receiptのschemaと全check、manifest・diagnosis・verification logのhashが一致することを確認します。
`READY FOR REVIEW`が出た場合だけ、表示されたreceipt pathのJSONを内容確認して共有候補にします。
自動化では`--first-map --json`を使えます。この再検証は書き込みもGitHubへの通信も行いません。

`status: PASS`のreceiptがあっても、再検証がreceipt mismatch、missing、invalid、またはnot PASSで
終了した場合は、その証拠をtrustedとして使いません。receipt、`run_manifest.json`、`session.json`を
手編集してhashを合わせたり、古いreceiptを新しいmapへコピーしたりせず、保存された`Details:`、
`Next:`、`retry.command`またはverification-enabledな新しいoutput commandへ戻ります。元の証跡は
削除・uploadせず、support reportと公開添付のprivacy境界を守ります。

### receipt再検証に失敗したときの復旧

`support --first-map`がrejectしても、元のmap、session、receipt、manifestは削除されません。
これは現在のevidenceがtrusted resultとして使えないという判定であり、mapが消えたという意味では
ありません。旧outputを削除せず、receipt、`run_manifest.json`、`session.json`の内容やhashを手で
書き換えて再検証を通そうとしません。

まずlocal-onlyのsessionと保存された復旧情報を確認します。

```bash
lidarslam-map sessions ./output --status action_required --viewer none --json
lidarslam-map inspect /path/to/output --write
```

`map_session_recovery.json`、diagnosis、`Details:`、`Next:`を順に読みます。`resume.available`
がtrueで`next_command`が`--resume`なら、表示されたcommandをそのまま実行し、mappingを再実行
しません。post-processingだけが完了すれば、同じsessionのreceiptをもう一度確認できます。

`retry.available`がtrueなら、保存された`retry.command`を編集せずに実行します。これはpinned setup
を保持したまま、通常は`map.retry`または`map.retry-2`のような新しいoutputへ書きます。旧sessionの
証跡と新しいretryの結果を混ぜず、新retryで新しく生成されたreceiptだけを再検証します。

resumeもretryも利用できない場合、または元のrunがverification offの診断だった場合は、doctorの
結果を確認してから、古いoutputとは別のpathにverificationをrequiredにした新しいrunを開始します。

```bash
lidarslam-map start /path/to/rosbag2 \
  --output-dir /path/to/output.verified \
  --verification required
```

viewerの見た目からcommandを再構成したり、古いreceiptを新しいmapにコピーしたりしません。新しい
sessionで`support --first-map`が`READY FOR REVIEW`を返し、receiptを内容確認できた場合だけ、
supportまたは独立validationの候補にします。旧証跡、bag、map、raw logは削除・uploadせず、公開時は
review済みreceiptだけを使います。

### 失敗したrunを上書きせず再試行する

失敗したrunをもう一度mappingするときも、元のdirectoryを削除したり同じpathを指定したり
しません。`start`の`--output-dir`がすでに存在する場合は`output directory already exists`
で停止し、既存のsessionやmapを上書きしません。

保存された`map_session_recovery.json`では、次の3つを分けて読みます。

- `setup_bundle`: `sensor_setup.json`、pinned parameter、session pageなどの保持されたsetup。
- `evidence`と`files_preserved: true`: 元のrunで得られたmanifest、diagnosis、log、receiptの証跡。
- `retry.available: true`: `retry.command`が同じpinned setupを使い、`retry.output_dir`
  （通常は`map.retry`のような新しいdirectory）だけへ出力する再試行。

原因を直した後は、JSONに保持された`retry.command`をpathやoptionを編集せず、そのまま
実行します。viewerの見た目から新しいmapping commandを作りません。`resume.available: true`
で`next_command`が`--resume`なら、まずpost-processingだけを再開し、mappingを再実行しません。
retryが無い場合に新しい`start`を意図して行うときだけ、古いsessionとは別の新しい
`--output-dir`を指定し、先にdoctorの診断を確認します。

recovery JSONとcommandにはbagやlocal pathが含まれるため、これらはlocal-onlyで扱います。
元のsession、map、raw logを削除・uploadせず、支援が必要な場合は後述のサニタイズ済み
support reportだけを確認して共有します。

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

### ブラウザが開かない・ヘッドレス環境の場合

offline previewはネットワーク不要のself-contained HTMLです。ブラウザが自動で
開かない場合やヘッドレス環境では、HTMLの生成とブラウザを開く操作を分けます。
`--no-open`を付けるとブラウザを起動せず、コマンドが表示する`HTML:`の絶対パスを
使えます。

```bash
lidarslam-map view /path/to/output \
  --viewer browser \
  --no-open
```

出力例の`HTML: /path/to/output/preview/mid360_robot_3d_map_preview.html`にある
実際の絶対パスを確認します。デスクトップ環境で開く場合はそのHTMLをブラウザで
開き、ヘッドレス環境で生成した場合は管理下の方法でHTMLだけをデスクトップ
環境へコピーして開きます。previewの保存先を明示したい場合は`--preview-dir`を
追加します。

```bash
lidarslam-map view /path/to/output \
  --viewer browser \
  --no-open \
  --preview-dir /path/to/preview
```

`--preview-dir`は`--viewer browser`と一緒に使います。生成されたHTMLにはmapの
表示データが含まれるため、bag、map、raw logと同じく非公開の成果物として扱い、
GitHub issueへuploadしません。共有が必要な場合は、先にサニタイズ済みのsupport
reportを確認します。

### 保存したsessionを探して復旧する

前回の実行結果をtimestamp付きdirectoryから探す代わりに、session履歴を使います。
通常は`./output`直下の保存済みbundleだけを検査し、ローカルのcatalogを作ります。

```bash
lidarslam-map sessions
```

途中で止まったsessionだけを、ブラウザを開かずに確認する場合は次を使います。

```bash
lidarslam-map sessions ./output \
  --status action_required \
  --viewer none
```

catalogや端末出力に`Details:`と`Next:`が表示されたら、保存された診断結果を読み、
`Next:`の行に表示された実際のcommandをそのまま実行します。viewerを変更する前に
`map_verify`、`autoware_map_diagnosis.md`、TF・topicのfindingを確認してください。
自動処理や確認だけなら、catalogを開かずJSONを読むこともできます。

```bash
lidarslam-map sessions ./output \
  --status action_required \
  --viewer none \
  --json
```

session履歴、catalog、診断、previewはlocal-onlyです。bag、map、raw log、preview
HTMLをGitHub issueへuploadせず、支援が必要な場合はサニタイズ済みsupport report
だけを内容確認後に共有します。

### 支援へ共有する前に確認する

まずsupport reportをJSONで確認します。これはread-onlyで、ZIPを作成せず、GitHubへ
送信もしません。

```bash
lidarslam-map support /path/to/session_bundle --json
```

ZIPが必要な場合はsession bundleの隣に作成される次の3ファイルだけを確認します。
`README.txt`、`issue-body.md`、`support-report.json`にはstatus、diagnosis、証跡の
hashが含まれますが、map geometry、bag、raw log、parameter内容、正確なlocal path、
credentialのようなcommand値は含まれません。3ファイルをすべて読んでから、必要な
部分だけをGitHub issueへ添付します。生成や添付は自動では行われません。

検証済みfirst mapを独立validatorへ渡す場合だけ、次のread-only handoffを使います。
`--first-map`はreceipt-boundのPASSを再検証し、ZIPを作らず、GitHubへ連絡しません。

```bash
lidarslam-map support /path/to/session_bundle --first-map
```

出力されるverification summaryとsafe environment hintsを確認し、公開添付には
review済みのfirst-map receiptだけを使います。`--first-map --json`のhandoff JSONは
local receipt pathを含むため、公開添付には使いません。

独立validator向けのissue formへ送る場合は、同じ出力に含まれるcanonical
independent-validation issue formを確認します。issue formには自分で実行したrunの
release/commitまたはimmutable image digest、実行command、OS・architecture・ROS環境、
statusを記入し、commandからprivate pathをredactします。receiptを添付する前に内容を
読み、PASSなら名前が示されたfirst-map receiptだけを添付します。map、bag、preview、
raw log、trajectory、parameter、スクリーンショットは添付しません。

`--first-map --json`のhandoff JSONとlocal receipt pathはpublic attachmentではありません。
このcommandはuploadしないため、自分でissue formへ入力して共有します。独立validationは
公開手順を自分で実行した結果だけを対象にし、maintainerのlive step-by-step guidanceは
validationとして扱いません。詳しい判定条件は[Independent First-map Validation](external-first-map-validation.md)
を参照してください。

### 公開共有前の5項目チェック

`READY FOR REVIEW`が表示されても、session bundle全体を公開するわけではありません。同じ
sessionについて次の5項目を確認し、1つでも不明なら共有を止めます。5つすべてを確認できた
場合に限り、最後のreceiptだけを公開添付の候補にします。

```text
- [ ] 1. version/revision: `lidarslam-map --version`を実行し、receiptの
      `run.product_version`と`run.git_commit`（値がある場合）を記録した。
- [ ] 2. same session/output: `run.run_id`、`map_output`、receipt path、
      `verification.manifest_sha256`が同じsessionの証跡に結び付いている。
- [ ] 3. revalidation: `support --first-map`が`READY FOR REVIEW`を返し、
      receiptの`status: PASS`と全checkのPASSを内容確認した。
- [ ] 4. privacy: receiptの`shareability`を読み、issue formへ貼るcommandから
      private pathをredactした。handoff JSONとlocal receipt pathはlocal-onlyのままにした。
- [ ] 5. attachment: 公開添付は内容確認済みの
      `first_map_validation_receipt.json`だけにした。
```

1では移動する`develop` tagやviewerの表示からversionを推測せず、公開release、commit、または
immutable image digestを使います。2では別runのreceiptをコピーせず、`session.json`の
`map_output`とreceiptの`run.run_id`、manifest hashを同じsessionで照合します。3がrejectしたら、
receiptを編集せず、前述の「receipt再検証に失敗したときの復旧」へ戻ります。

4のhandoff JSON、receipt path、session JSON、support reportのlocal pathは、共有用の資料では
ありません。5以外に、map、bag、raw log、preview HTML、trajectory、parameter、screenshot、
session bundle全体も添付しません。issue formのversion、environment、redacted command、
verification summaryは自分のrunから転記し、receiptのJSONだけをレビュー済みの添付として使います。

## 詳細

このページは最短経路だけを示します。すべてのoption、対応input、校正、復旧、
自動化contractは英語版の[Getting Started](getting-started.md)と
[Operator Workflows](workflows.md)を参照してください。正規コマンドや安全境界に
差がある場合は英語版が優先されます。
