# 3DGS 検出 gap — 実 COCO 物体 × novel view (2026-06-16)

sim2real トラックの知覚スレッドの締め。問い:

> 実画像で学習した検出器は、**実在の物体を 3DGS で再レンダした像**に発火するか。
> novel な視点（記録軌跡から外れた視点）を横断してどれだけ保つか。

Phase 0 はこれを測れなかった（手元 LiDAR-primed シーンに COCO 物体が皆無）。
Phase 3 の compositing は隣接する問い（貼り込んだ actor）に答えたが合成
アーティファクトが乗る。本ノートは**合成なし**で直接測る。

ツール: `tools/gaussian_splatting/detect_in_scene.py`
（pure 部 10 ケースを `graph_based_slam/test/test_gaussian_splatting_detect.py`
で CPU テスト、ament_flake8 clean）。

## 方法（合成なし）

実物体を既に含む 3DGS シーン（調達した Apache-2.0 の Tanks&Temples **Truck**、
`3dgs-coco-actor-procurement.md`）に対し、物体の AABB を与えると:

1. 物体周りに**合成カメラを orbit**（`orbit_viewmats`、up 軸指定。T&T は y-up）。
2. 各 novel view をレンダ（`rasterize_rgbda`）。
3. 検出器（yolov8n）をかける。
4. 物体の Gaussian 平均を投影して得た**tight GT box** とクラス一致で照合。

合成も向き整合も不要で、純粋に「3DGS 再レンダ上の実物体に検出器が発火するか」を測る。

## 結果（Truck シーン、yolov8n、360° orbit 36 view、640x480）

```
--center=0.75,-2.0 --half-extent 3.0 --z-range=-3,2 --class-id 7 (truck)
--radius 11 --elevation -2 --up-axis y
→ class-present rate 0.47, recall@IoU0.5 0.06, mean best IoU 0.16
```

- **実トラックは novel view の約半数（class-present 0.47）で "truck" と認識される**。
  検出は**正面・3/4 視点で確実**（standalone クロップは truck conf 0.73–0.78、
  `3dgs-coco-actor-procurement.md`）、**後方・斜め視点で低下**する。viewpoint 依存の
  gap という所見は、closed-loop の ego が車両を主に canonical な角度から見る運用に
  とって追い風（不利な後方視点は稀）。
- **headline は class-present rate**（GT box の tightness に依存しない頑健な指標）。
  recall@IoU0.5 が 0.06 と低いのは、GT box が 3D crop の投影でインスタンス mask が
  ないため物体シルエットよりやや緩く、検出 box との IoU が 0.5 に届きにくいため。
  代表 hit view（IoU 0.536）では GT box がトラックをよく囲む。

## 限界 / 次

- tight な IoU/recall には 3D インスタンス mask（学習済み actor の alpha or
  per-gaussian ラベル）が要る。クラス検出率（class-present）は GT 非依存で頑健。
- venue は SfM ベースの Tanks&Temples で **LiDAR-primed ではない**。LiDAR-primed
  屋外シーンに COCO 物体が入る素材が用意できれば、同じツールで「LiDAR-primed 3DGS の
  検出 gap」を直接測れる（`--ply` を差し替えるだけ。data gap であってコードでない）。
- 軌跡沿い（orbit でなく前進）の view 系列を測るには `orbit_viewmats` を実軌跡 pose に
  差し替えればよい（closed-loop の sensor_sim_node 視点に一致）。

## ロードマップ上の位置づけ

```
Phase 0  外挿安定性 (有効視点範囲)                    ← 完了
Phase 1  open-loop RGB-D データ生成                   ← 完了
Phase 2  closed-loop sensor-sim ROS 2 node            ← 完了
Phase 3  dynamic actors (box/sprite/ply, 検出 gap)    ← 完了
  └ COCO actor 調達 (Tanks&Temples truck, Apache-2.0) ← 完了
  └ 実物体 × novel view 検出 gap                       ← 本ノート (class-present 0.47)
```
