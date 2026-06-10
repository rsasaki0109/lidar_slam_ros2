# 3DGS trajectory flythrough — multi-session negative / RTK-SLAM walk positive (2026-06-10)

README の 3DGS フライスルー GIF を「推定軌跡に沿ってカメラが移動する」ものに差し替える
ための検証記録。**結論: RTK-SLAM construction_seq1（CC-BY 4.0、リリースゲートと同一
データ）の歩行 60m 窓で、軌跡沿いフライスルー（採用区間 平均 21.2dB）が成立。**
鍵は 2 つの本体バグ修正（means LR、時刻オフセット）で、過去の「広域シーンだけ低品質」
（isuzu 14.5dB / NTU 10dB）の主因の一つも同じ LR バグだったと考えられる。

## 背景

koide first light（`3dgs-koide-first-light.md`）の GIF はカメラパスが軌跡通りでも
**bag 自体がほぼ静止**（総移動 0.91m/15s）で、定点首振りにしか見えない。動きのある
カラー+LiDAR データが必要だった。

## 試行 1: koide キャリブ bag 5 本のマルチセッション統合（負け筋）

direct_visual_lidar_calibration のサンプル 5 bags を「同一シーンの 5 視点」と仮定して
点群レジストレーションで統合 → 失敗。学び:

- **5 bags は同一シーンではない**。同一広場は 13_42_46 + 13_46_10（6.4m 離れ）のみ。
  13_44_10（レンガ建物・駐輪場）と 13_46_54 は別サイト、13_44_54 は近いが光度整合せず。
- **幾何 ICP は平面で滑る**: fitness/rmse 良好でも接線方向にメートル級の誤整合
  （colored ICP が 1.5-4m 動かした）。キャリブ用キャプチャはそもそも重なりを保証しない。
- **静止標点の単一セッション 3DGS は 6m 離れた新視点で崩壊**（標点に過学習）。
- 対策として `train_gsplat.py --optimize-pose-groups`（セッション単位の world 側 SE(3)
  自己校正、group 0 がゲージ）と `--optimize-exposure`（グループ別アフィン色補償、
  L2 正則化 0.1）を実装。2 標点構成で 12.7 → **19.2dB** まで回復したが頭打ち
  （ガラス反射等、真の視点依存外観が残差）。露出 gain は 0.998 = 露出は犯人でなかった。
- マルチセッション統合した静止標点間のドリーは「動き」も中途半端 → 不採用。

## 試行 2: 公開データセット調査

ハンドヘルド カラー+LiDAR の定番（R3LIVE / FAST-LIVO2 / MCD / Newer College / KITTI）
は**ほぼ全部 CC BY-NC-SA（非商用）**で README 素材として不適。Newer College は加えて
mono。→ **RTK-SLAM dataset（CC-BY 4.0）に 2MP カラー 20Hz カメラが載っている**ことに
気づく（`/camera/image_raw/compressed`、リリースゲート採用データそのもの）。
キャリブは rtk-slam-eval の Kalibr `calib/calib.yaml` から
`configs/gaussian_splatting/rtk_slam_cam0_{intrinsics,extrinsic}.yaml` に取り込み。

## 試行 3: RTK-SLAM construction_seq1 の歩行窓（採用）

窓 480–545 s = 機械ホール内を 60m 周回（bbox 10×13m、視点重複大）。RKO-LIO 軌跡
（リリースゲートと同一 run の `traj_raw.tum`、~10Hz）+ stride 5 → 260 views。
半解像度 + ビネット切り（周辺の口径食はカメラと一緒に動き 3DGS では表現不能）。

ハマりどころと修正（再現実験つき）:

1. **means 学習率バグ（本体修正、最重要）**: 旧 `lrs['means'] = lr * extent` は
   INRIA/gsplat の `1.6e-4 * extent` の **~60 倍**で、しかも減衰なし。extent は
   カメラ広がりなので、koide（静止クラスタ、extent ≈ 0.3m）では偶然無害、歩行窓
   （extent ≈ 8m）では位置が永遠に churn して全ビュー霧化（max 16.8dB に均一化）。
   `lr * 0.016 * extent` + ExponentialLR（1% まで指数減衰）に修正 → 同条件で
   median 16.6 / p75 21.7 / max 27.8dB、SSIM 0.54 → 0.64。isuzu/NTU の過去の
   ネガティブ結果もこのバグの影響を受けていたはず（再評価は未実施）。
   koide first light の回帰: 24.3 → 23.95dB（SSIM 0.81 → 0.846）で劣化なし。
2. **時刻オフセットの過剰適用**: Kalibr の timeshift (-20.6ms) を `--time-offset` に
   渡したら品質劣化。LiDAR 単一スキャンを連続 2 画像に投影する光度整合スイープで
   **bag のカメラスタンプは補正済み（最適オフセット 0ms）**と実証。盲目的に
   キャリブの timeshift を足さないこと。
3. **`--optimize-extrinsic`/`--optimize-pose-groups` は長期で漂流**: 27k iters で
   tau が 0.47m まで歩き PSNR 悪化（11.6dB）。係数の良いキャリブがあるなら固定が正解。
   per-view ポーズ精錬（フレーム毎にユニーク `bag` を振る）も +0.5dB 止まりで、
   LR バグ修正後は不要だった。
4. **ハンドヘルド LiDAR はオペレータ自身を毎スキャン至近で写す** →
   `build_lidar_init.py --min-range`（1.5m）を追加。カメラから一度も見えない
   ゴースト点は不透明度が初期値のまま残り、経路上の霧チューブになる。
5. **CompressedImage 対応**: `extract_posed_images.py` が
   `sensor_msgs/CompressedImage`（jpeg、cv_bridge の bgr タグ対応）を自動判別。

フライスルーは per-view PSNR のローリング窓で**最良の連続区間**（views 116–208、
~23 s、移動 6.7m、平均 21.2dB）を選択。窓・白飛び方向を見るビューは諦める
（PSNR 平均はそれらで沈むが、採用区間の見栄えが基準）。

## 再現

```bash
python3 scripts/download_rtk_slam_dataset.py --sequence construction_seq1
BAG=... TRAJ=... bash scripts/run_rtkslam_3dgs_flythrough.sh
```

成果物: `lidarslam/images/3dgs_rtkslam_walk_sidebyside.gif` / `.mp4`（README 掲載）。
README の GIF は「LiDAR SLAM × 3DGS」が一目で伝わるよう、左 = SLAM 点群地図
（高さカラー）+ 推定軌跡（マゼンタ線、床側に -0.4m オフセット）、右 = 3DGS を
同一カメラパスで同期描画する左右 2 分割
（`tools/gaussian_splatting/render_slam_3dgs_sidebyside.py`）。点群・軌跡も極小等方
ガウシアンとして同じ gsplat ラスタライザで描く（追加依存なし）。カメラは軌跡上を
飛ぶため、軌跡線はフレーム毎にカメラ近傍 1m をカリングしないと巨大ブロブが画面を
覆う。GIF は imageio 直書きだと 15MB 級になるので ffmpeg の palettegen/paletteuse
（96 色、bayer dither）で 8.5MB に圧縮。
データ出典: RTK-SLAM Dataset（Zhang, Ress, Skuddis, Soergel, Haala, Univ. Stuttgart、
arXiv:2604.07151、CC-BY 4.0）。

## 追補 (2026-06-11): 「本当に歩いている区間」の photoreal 化は 5 戦略全敗

「もっと移動してる感を」の要望で再調査して判明した重要事実: **採用区間 views
116-208 はオペレータがほぼ立ち止まっている**(net 0.3m。"6.7m" は手ブレの積算)。
PSNR が高いのは静止視点だから。本当に歩いている区間(views 25-110 / 200-259、
net 4-6m/窓)は s5 モデルで 12-15dB の霧。以下すべて失敗:

1. stride 2(650 views)15k iters — view あたり学習回数半減で全域劣化(median 14.7)。
2. 同 40k iters — 静止区間は 18→24dB に伸びたが**歩行区間は 12dB のまま**
   (巨大緑フローター)。view 密度と学習量は律速でない。
3. 歩行ウィング専用モデル(483-514s、stride 1 = 620 views、20k)— 全域 11-13dB に
   総崩れ。歩行データの不整合がモデル全体を汚染する。
4. 同 + per-view ポーズ自己校正(`--optimize-pose-groups`、フレーム毎グループ)—
   tau 補正は 5-30mm 止まりで median 11.6。10Hz 軌跡補間の並進誤差は主因でない。
5. 同 + per-view 露出補償(`--optimize-exposure`)— gain 0.84-1.07 と実際の
   自動露出変動は検出されたが median 12.2。露出も律速でない。

消去法で残る律速候補: **歩行中のモーションブラー / ローリングシャッター**
(パイプラインのどちらも未モデル化)。歩行 photoreal はこの bag では不成立。

採用した代替: 良品質区間のまま **片道再生**(ping-pong 廃止で体感 2 倍速)+
**ループフェード**(`--loop-fade`)+ **俯瞰ミニマップ**(全軌跡 + 現在地ドット +
ライド開始点からの進捗線)。軌跡線の進捗 2 色塗りは一人称では逆効果
(見えるのは常に「未来」側なので線が暗くなるだけ)でミニマップ側のみに採用。

## 今後

- isuzu / NTU VIRAL を LR 修正後の trainer で再評価（過去ノートの数字は旧 LR のもの）。
- 歩行 photoreal の再挑戦はデータ側から: グローバルシャッター or 高速シャッターの
  カメラ + 高レート姿勢（IMU 統合）で自前撮影するか、静止スタンドポイントが
  歩行路に沿って密に並ぶデータを選ぶ。
- 窓・白飛びビュー対策（露出の per-view 推定 or HDR 風重み）と動的物体（作業者）の
  マスクで、捨てている区間も使えるはず。
- `output/koide_3dgs_firstlight/gsplat/point_cloud.ply` は後続実験で random-init の
  残骸に上書きされている（正: `pc_sh1_9k.ply`）。flythrough スクリプトを koide で
  回し直す場合は要再学習。
