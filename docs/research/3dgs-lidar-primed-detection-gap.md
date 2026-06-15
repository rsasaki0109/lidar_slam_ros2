# LiDAR-primed sim2real 検出 gap — 実屋外シーン (2026-06-16)

sim2real トラックの知覚スレッドを、**自前の LiDAR-primed シーン**で締める。
Phase 0 は検出器レイヤ（`sim2real_gap.py --detector`、実画像の検出が 3DGS render 上で
何割生き残るか = recon_det_agree）を**手元シーンに COCO 物体が皆無で exercise できなかった**。
RTK-SLAM の **stadtgarten_seq2**（屋外・市民公園、CC-BY 4.0）は駐輪自転車・歩行者が
映るので、ここでついに直接測れる。

パイプライン（既存ツールのみ、新規コードなし）:
rosbag2 + RKO-LIO 軌跡 → posed images → **LiDAR-primed gsplat** →
`sim2real_gap.py --detector`。再現: `scripts/run_stadtgarten_detection_gap.sh`。
（カメラ timeshift t_imu = t_cam − 0.0206s → `--time-offset -0.020638`。frontend raw
軌跡を dense pose に使用。窓 225–290s に**駐輪自転車**＝静的が含まれる。）

## 結果（stadtgarten_seq2, 窓 225–290s, yolov8n, 30 view）

```
recon PSNR 18.9 dB, det-agree 0.27
offset sweep: ±0.25m で det-retain ~0（sharp 比 70–1800 = floater 爆発）
```

集計 det-agree 0.27 は低いが、**静的/動的で完全に分かれる**のが本質:

| 物体 | 種別 | 実画像 | 3DGS render | agree |
|---|---|---|---|---|
| 駐輪自転車 (view 18–80) | **静的** | bicycle 0.72–0.77 | **bicycle 0.75–0.78** | **~1.0** |
| 歩行者 (view 9, 89, 98, 107) | **動的** | person 0.77 | **(なし)** | **~0.0** |

- **静的 COCO 物体は LiDAR-primed 3DGS render 上でほぼそのまま検出される**。駐輪自転車は
  実画像 conf 0.77 → render 0.75 と**ほぼ同一**（render の方が高い view もある）。連続 view
  18–80 で agree 1.0 を維持＝静的物体の sim2real 検出 gap はほぼゼロ。
- **動的 COCO 物体は静的前提の 3DGS で ghosting し、render から消える**（agree 0）。歩行者は
  実画像で検出されても render では smear して未検出。
- 集計 0.27 は窓内の静的/動的検出の混在比を反映しているだけで、gap の本質は**物体の運動**。

これが Phase 0 で data 不足だった問いへの答え: **実画像の検出器は LiDAR-primed 3DGS
render 上の実 COCO 物体に発火する — 静的物体には高忠実に（gap ≈ 0）、動的物体には
dynamic-3DGS 手法が要る**。先行の Truck シーン（SfM, orbit/dolly, class-present 0.47–0.75,
検出レンジ 15m）の所見—「検出可能だが viewpoint/距離依存」—と整合し、初期化が LiDAR でも
SfM でも検出器側の挙動は一貫することを示す。

## 限界 / 次

- recon 18.9 dB と低めなのは屋外＋frontend raw 軌跡のドリフト＋動的物体の不整合。backend
  corrected 軌跡は 278 pose と疎で camera 補間に不足。**dense かつ backend 精度の軌跡**が
  あれば recon は上がる（construction clean は 28.9 dB）。
- 有効視点範囲は近接屋外＋ドリフトで < 0.25m（offset で floater 爆発）。Phase 0 の
  「有効範囲はシーンスケール依存」と整合。
- **動的物体の検出 gap には dynamic/4D-3DGS**（時間変形 or per-frame actor）が必要。
  本リポジトリの actor compositing（`3dgs-actor-compositing-phase3.md`）は静的シーンに
  動的 actor を別途差し込む補完経路。

## ロードマップ上の位置づけ

```
Phase 0  外挿安定性 / 有効視点範囲                         ← 完了
Phase 1  open-loop RGB-D データ生成                        ← 完了
Phase 2  closed-loop sensor-sim ROS 2 node                 ← 完了
Phase 3  dynamic actors (box/sprite/ply, 検出 gap)         ← 完了
  └ COCO actor 調達 (Tanks&Temples truck)                  ← 完了
  └ 実物体 × novel view / 検出レンジ (orbit/dolly)          ← 完了
  └ LiDAR-primed 実屋外シーンの検出 gap (静的≈1.0/動的≈0)  ← 本ノート
```
