# 3DGS 後処理マップ成果物 — 設計ドキュメント (draft, 2026-06-05)

> Status: **設計 draft（実装前）**。`まず設計ドキュメント` の合意のもとで作成。
> v0.4 roadmap (`docs/roadmap/v0.4.md`) には現状 3DGS は含まれておらず、
> これは新規ワークストリームの提案。ライセンス・センサ調査・PoC 計画を確定し、
> 実装着手の Go/No-Go をユーザー判断に委ねるための土台。

関連メモリ: `[[project_v0_3_positioning]]` `[[project_awsim_pipeline]]`
`[[project_mid360_robot_toolkit_stack]]` `[[project_v0_4_roadmap_draft]]`

---

## 1. スコープ確定（このセッションの合意）

| 項目 | 決定 |
|---|---|
| **役割** | マップ成果物（後処理）— SLAM 軌跡を入力に、pointcloud_map と並ぶ photorealistic map / novel-view 成果物として 3DGS を再構成。**SLAM 本体（RKO-LIO + graph_based_slam）は一切触らない**。 |
| **入力センサ** | 調査の結果、production の MID-360 はカメラ無しだが、複数ベンチマーク bag が LiDAR+カメラ同期で**既にローカルに存在**（§3）。 |
| **導入スコープ** | まず本設計ドキュメント。実装は本ドキュメント承認後。 |

### 明示的に「やらないこと」(out of scope)

- **3DGS-SLAM**（online Gaussian SLAM、RKO-LIO 置換）— 別物。今回はやらない。
- **3DGS マップでの自己位置推定** — Autoware は PCD/NDT で localize する。
  3DGS は localization マップを**置き換えない**。あくまで人間向け検査 /
  digital-twin / NVS 成果物。この境界を曖昧にしない。
- **pointcloud_map の置換** — 3DGS は pointcloud_map と**並ぶ**追加成果物であって
  代替ではない。Autoware 入力は従来どおり PCD。

この境界設定は v0.3 のポジショニング（"SOTA を狙わず、正直な map authoring
stack"、`[[project_v0_3_positioning]]`）と整合する。3DGS は「精度 claim」ではなく
「成果物の説得力・検査性」を上げる方向の投資。

---

## 2. ライセンス分析（最重要・先に確定）

このリポジトリは BSD-2/MIT の商用フリーを 1st claim にしている
（`[[project_v0_3_positioning]]`、デフォルトパスに GPL 不可）。3DGS 実装の選定は
ここが律速。

| 実装 | ライセンス | 商用可 | 採否 |
|---|---|---|---|
| INRIA `graphdeco-inria/gaussian-splatting`(本家) | **non-commercial 研究ライセンス** | ❌ | **不可**。本家 CUDA rasterizer は商用フリー方針と非互換。 |
| **gsplat** (`nerfstudio-project/gsplat`) | **Apache-2.0** | ✅ | **採用候補**。CUDA rasterizer を独自実装。Nerfstudio エコシステム。 |
| Nerfstudio | Apache-2.0 | ✅ | 学習フレームワークとして利用可（重量級なら gsplat 直叩きでも可）。 |
| viewer: SuperSplat / antimatter15 splat | MIT | ✅ | `.ply` 成果物の web 可視化に利用可。 |
| Taichi 3DGS, 各種 PyTorch 実装 | MIT/Apache 混在 | 要確認 | 個別確認が必要。 |

**推奨**: **gsplat (Apache-2.0)** を rasterizer/学習コアに採用。本家 INRIA コードは
一切取り込まない。

**正直な注意書き（doc に残す）**: 3DGS のラスタライズ手法自体に関する特許懸念が
業界で議論されている。gsplat は独立実装だが、商用展開時はライセンス + 特許の
両面で最新状況を再確認すること。本ツールは「成果物生成のオフライン後処理ツール」で
あり、ランタイム localization パスには載らないため、リスク面は限定的。

---

## 3. RGB センサ・インベントリ（調査結果）

`demo_data/` 内の bag メタデータ実査結果。**新規データ取得なしで PoC 可能**。

| ソース | RGB | トピック / calib | 同期 LiDAR | first-light 適性 |
|---|---|---|---|---|
| **koide_lidar_camera_calib**(ローカル) | 単眼 | `/image` (`sensor_msgs/Image`) + `/camera_info` | `/livox/points` + `/livox/imu` | ★最有力。intrinsics 同梱、データ最小。 |
| **NTU VIRAL tnp_01**(ローカル) | ステレオ | `/left/image_raw`,`/right/image_raw` + `camera_left.yaml`,`camera_right.yaml` | OS1 + horz/vert | release-track + Leica GT。屋外検証用。 |
| **autoware_leo_drive_isuzu** bag1-6(ローカル) | 3カメラ | `/lucid_vision/camera_{0,1,2}/raw_image` + camera_info | あり | 実車 AD マルチカメラ。large。 |
| **KITTI Odometry** | カラー(別DL) | `calib.txt` P0-P3 + `Tr`(velo→cam) | velodyne | LO ベースライン連動。画像は別 DL 必要。 |
| **AWSIM** | △ | シーン側でカメラセンサ有効化が必要 | あり | sim photoreal twin。AWSIM demo 連動。 |
| **MID-360 実機 / mid360_public** | **なし** | `/livox/lidar` + `/livox/imu` のみ | — | **production のカメラ欠落（§7 課題1）**。 |
| Newer College math_hard(ローカル) | なし | LiDAR + IMU のみ（local subset） | — | このサブセットは画像無し。 |

**結論**: production センサ（MID-360）はカメラ無しだが、PoC〜検証は
**ローカルの koide / NTU VIRAL bag で完結**できる。MID-360 production path への
3DGS 載せは別途センサ追加判断が必要（§7）。

---

## 4. アーキテクチャ（後処理データフロー）

SLAM が既に出力しているもの:
- 最適化軌跡（TUM）: `traj_corrected.tum` 等
- `pose_graph.g2o`, `MapArray`
- `pointcloud_map`（`map.pcd`）

3DGS 後処理は**これらの純粋な消費者**として動く:

```
                         ┌─────────────── 既存 SLAM（変更なし）──────────────┐
  rosbag2 ──► RKO-LIO ──► graph_based_slam ──► traj_corrected.tum + map.pcd
     │                                                    │          │
     │ (同じ bag を後処理で読み直す)                       │          │
     ▼                                                    ▼          ▼
 ┌──────────────────── tools/gaussian_splatting/ (新規・out-of-tree) ─────────────┐
 │ 1. 画像抽出   bag から (t, image, camera_info) を取り出す                       │
 │ 2. ポーズ補間 各画像 t における SLAM ポーズを補間 → world←base                 │
 │ 3. 外部標定   static TF / calib で base←camera を合成 → world←camera           │
 │ 4. 初期化     LiDAR pointcloud_map から Gaussian を初期化（COLMAP 不要）        │
 │              色は posed 画像の投影 or intensity                                │
 │ 5. 学習       gsplat で photometric loss 最適化（任意で LiDAR depth 正則化）     │
 │ 6. エクスポート .ply（3DGS 標準）+ web viewer 成果物                            │
 └────────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼
                       output/<run>/gsplat/ : point_cloud.ply + viewer.html
```

### 設計上の核（なぜ LiDAR-SLAM × 3DGS が強いか）

通常の 3DGS は **COLMAP の SfM** でカメラポーズと初期点群を起こすが、これは
遅く・スケール曖昧・テクスチャ薄い屋外で破綻しやすい。本構成では:

- **メトリックなポーズと点群を LiDAR-SLAM が既に持っている** → COLMAP 不要。
- スケールが正しい（メートル単位）→ Autoware の PCD と座標系が一致。
- LiDAR 点群を Gaussian 初期化に使える → 収束が速く floater が減る。

これが「LiDAR-primed 3DGS」の売り。既存パイプラインの出力をそのまま活かせる。

### 既存資産とのシナジー

- **dynamic-object filter**（既存）: 動的物体（車・人）は 3DGS で floater 化する。
  既存の動的物体フィルタで posed 画像/点群をマスクすれば品質が上がる。
- **AWSIM demo**: AWSIM シーンでカメラを有効化すれば、自作マップの
  photoreal twin を AWSIM × Autoware tutorial に追加できる。

---

## 5. 統合方針（リポジトリへの載せ方）

| 論点 | 方針 |
|---|---|
| 配置 | **`tools/gaussian_splatting/`**（新規 Python ディレクトリ）。**colcon パッケージにしない**（CUDA を C++ ビルド/CI に持ち込まない）。 |
| 言語 | Python + gsplat(CUDA)。C++ SLAM ツリーには触れない → BSD-2 ソース制約・clang-tidy 厳格ルールの影響を受けない。 |
| 実行環境 | **Docker(CUDA)**。既存の GLIM/Autoware Docker パターン（`compare_with_glim.sh` の `koide3/glim_ros2` 等）を踏襲。 |
| CLI | `python3 tools/gaussian_splatting/build_3dgs_map.py --bag <dir> --traj <tum> --camera-topic /image --camera-info-topic /camera_info --pointcloud-map map.pcd --out output/<run>/gsplat/` |
| CI | GPU 必須なので標準 runner では走らせない。**opt-in**。CPU 側は引数パース/抽出/ポーズ補間の単体テストのみ（gsplat 学習は GPU 環境でのみ）。 |
| ROS 2 ラッパ | 当面不要。将来 launch から呼ぶ薄いラッパは M3 以降で検討。 |

この配置なら既存の `run_default_ci_checks.sh` / release gate を一切壊さず、
3DGS を完全に opt-in な後処理ツールとして足せる。

---

## 6. PoC マイルストーン

| M | 内容 | データ | 完了条件 |
|---|---|---|---|
| **M0** | 本設計 doc（ライセンス・センサ・アーキ確定） | — | 本ドキュメント承認 |
| **M1 first light** | koide bag → posed 画像 → LiDAR-primed gsplat → `.ply` + viewer | `koide_lidar_camera_calib`（ローカル） | 1 シーンで認識できる 3DGS が出る。スクショ 1 枚。 |
| **M2 GT 検証** | NTU VIRAL ステレオで品質確認、ポーズ品質と 3DGS 品質の相関を見る | `ntu_viral/tnp_01`（ローカル, GT 付） | PSNR/見た目を記録。SLAM ドリフトが ghosting に出るか確認。 |
| **M3 AWSIM 連動** | AWSIM シーンでカメラ有効化 → 自作マップの photoreal twin | AWSIM | AWSIM tutorial に 3DGS セクション追加 |
| **M4 production 化(stretch)** | ドキュメント化された成果物、CI smoke（極小 fixture） | fixture | release 成果物として doc 化 |

M1 は**全データがローカル**にあり、外部依存は gsplat の Docker のみ。最小リスクで
着手できる。

---

## 7. 未解決の課題 / ユーザー判断待ち

1. **MID-360 production のカメラ欠落** — production センサ（MID-360）はカメラ無し。
   3DGS を production path に載せるなら選択肢:
   - (a) MID-360 リグに USB カメラ追加 + 外部標定（toolkit 拡張、`[[project_mid360_robot_toolkit_stack]]`）
   - (b) LiDAR-only の intensity/geometry 3DGS（単色 or intensity 着色、低 fidelity）
   - (c) 3DGS は当面ベンチマーク/AWSIM 成果物に限定し、MID-360 production には載せない
   → **どれを取るか要判断**。

2. **3DGS をどの claim に紐づけるか** — 「検査用の見栄え成果物」止まりか、
   「digital-twin / sim アセット生成」まで狙うか。後者なら AWSIM 連動(M3)を前倒し。

3. **v0.4 roadmap への組み込み** — 現 roadmap (A〜F) は 3DGS を含まない。
   新ワークストリーム G として差し込むか、v0.4 とは独立した research track にするか。

4. **GPU 前提の CI 戦略** — GPU runner を用意するか、3DGS は永続的に opt-in
   ローカル/Docker 専用にするか。

---

## 8. リスク

- **ポーズ品質依存**: 3DGS はポーズ誤差に敏感で、SLAM ドリフトが直接 ghosting/
  floater になる。裏を返せば SLAM 品質の良い可視化テストにもなる。
- **GPU 必須**: CUDA 必須で標準 CI に載らない → opt-in + Docker で隔離。
- **Rolling shutter / motion blur / 時刻同期誤差**: 画像と LiDAR ポーズの
  時刻ずれが品質を直撃。`camera_info` と TF の正確さが前提。
- **動的物体**: floater 化 → 既存 dynamic-object filter でマスク（§4 シナジー）。
- **ライセンス/特許**: §2 の注意書きを成果物 doc に残す。

---

## 9. 次アクション（承認後）

1. `tools/gaussian_splatting/` 雛形 + gsplat Docker（Apache-2.0 のみ）。
2. `extract_posed_images.py`: bag → (image, intrinsics, world←camera) の単体テスト
   付き抽出器（GPU 不要、ここを先に固める）。
3. M1 first light を koide bag で実行、`.ply` + スクショを取得。
4. 結果を `docs/research/` に追記し、v0.4 roadmap への組み込み是非を再協議。
