# 3DGS sim2real gap — Phase 0 外挿安定性測定 (2026-06-15)

3DGS を **sim2real シミュレーション基盤**として開発するトラックの Phase 0。
全下流用途（open-loop 知覚データ生成 / closed-loop Autoware-in-the-loop /
RL 方策学習）の前提となる問いを、丸ごとシミュレータを作る前に安く測る:

> 実データで学習済みのモデルは 3DGS レンダにそのまま通用するか。
> ego が記録軌跡から横ずれしたとき、どこまで視点を外挿してよいか（**有効視点範囲**）。

ハーネス: `tools/gaussian_splatting/sim2real_gap.py`
（pure 部 18 ケースを `graph_based_slam/test/test_gaussian_splatting_sim2real_gap.py`
で CPU テスト、ament_flake8 clean）。
再現: `PLY=... TRANSFORMS=... OUT_DIR=... bash scripts/run_sim2real_gap.sh`

## 測定内容

各学習視点について 2 つを測る:

1. **再構成忠実度（offset 0）** — 記録 pose でレンダ vs 実画像: PSNR / SSIM、
   および任意で物体検出器の一致（実画像の検出が render 上で IoU マッチで何割残るか）。
   これが記録軌跡上での文字どおりの sim2real gap。
2. **外挿安定性（横オフセットスイープ）** — カメラを camera-local x（右）に
   ±0.25/0.5/1.0 m スライド（heading 固定 = ego の横ずれ）。各 offset を
   offset-0 render と比較: 見た目安定性 SSIM、sharpness 比、bright-floater 率
   （3DGS 特有の破綻 = 白飛び floater の proxy）、検出保持率。
   これらが崖になる offset が closed-loop でこのシーンを使える**有効視点範囲**。

## 結果（有効視点範囲はシーンスケールで決まる）

GPU / gsplat 1.5.3 / scale 0.25–0.5 / 横オフセット x 軸。

| シーン | モデル | recon PSNR/SSIM | ssim_vs_base @±0.25 / ±0.5 / ±1.0 m | floater @±1.0 m |
|---|---|---|---|---|
| **koide**（近接ハンドヘルド屋内） | pc_sh1_15k | 22.5 dB / 0.82 | 0.34 / 0.30 / 0.25 | ~0.10 |
| **isuzu**（屋外・走行スケール） | first150 | 17.3 dB / 0.50 | 0.95 / 0.91 / 0.88 | **~0.000** |

- **走行スケールは外挿に強い（本命の追い風）**: isuzu は ±1.0 m 横ずれでも
  見た目安定性 0.88、floater ほぼ 0。LiDAR-primed の正しい幾何が深い被写体距離で
  視差を正しく出すため、±1 m の ego 横ずれが render をほとんど乱さない。
  これは closed-loop Autoware-in-the-loop が必要とする性質そのもの。
- **近接シーンは速く崩れる**: koide は ±0.25 m で見た目安定性 0.34 まで落ちる。
  ただし floater は ~0.10 で穏当なので、低下の主因は破綻でなく**近接被写体の正当な
  視差**（offset-0 比 SSIM は破綻と視差を区別しない）。被写体距離が短いシーンは
  有効横範囲 < 0.25 m。先行の `3dgs-trajectory-flythrough-notes.md`（koide は
  0.4 m 外挿で floater 崩壊）と整合。

## 負の結果 / 限界

- **検出器ベースの知覚 gap は手元データで exercise できなかった**。YOLO-COCO は
  3 シーンすべてで real 画像でも検出ゼロ: koide=屋内（COCO 物体なし）、isuzu
  first150=空の試験路（草地・舗装のみ、車/人なし）、construction=データ不整合。
  layer は実装・動作済み（`--detector yolov8n.pt`）だが、**車両/歩行者が実際に
  frame に映る走行シーンが必要**。これはコードでなくデータの不足。
- **construction_seq1 はクリーンに測れなかった**。ディスク上の 3DGS 成果物
  （`output/rtkslam_3dgs/`）は flythrough 実験で images/ と transforms が何度も
  再生成され不整合化しており、現行 `transforms_crop.json` から再学習しても
  **train PSNR 12 dB・MSE 発散気味（収束せず）**。raw rosbag はローカルに無く
  （`scripts/download_rtk_slam_dataset.py` で都度 DL する設計）、クリーンな数値には
  extract パイプラインの再実行が必要。Phase 0 の配線検証スコープ外として保留。

## sim2real ロードマップ上の位置づけ

```
Phase 0  外挿安定性測定         ← 本ノート（走行スケールで ±1m OK を確認）
   ▼
Phase 1  open-loop データ生成   ← 有効範囲内で RGB/depth 量産（depth は LiDAR-primed）
   ▼
Phase 2  closed-loop sensor-sim ROS 2 node（本命）← ego pose 購読→レンダ→Image 配信
   ▼
Phase 3  dynamic actors + RL
```

## 次アクション

- 検出器 gap を本当に測るには、車両/歩行者が映る走行シーンを用意する
  （公開運転データセット、または後半 frame に物体が映る isuzu 区間を full-res・
  低 conf で）。
- construction の数値が要るなら `download_rtk_slam_dataset.py` → SLAM traj →
  extract を回してクリーンな `point_cloud.ply` × `transforms` を作り直す。
- 走行スケールで ±1 m が通ったので、Phase 2 の closed-loop sensor-sim node の
  レンダリングリアルタイム性検証に進むのが妥当。
