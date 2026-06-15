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

## 追記: 検出レンジ（dolly = ego 前進接近） (2026-06-16)

orbit は全方位だが、closed-loop で効くのは「ego が路側の物体にどこまで近づけば
検出できるか（**検出レンジ**）」。`--path dolly` は固定方位で far→near にカメラを
寄せる（ego が前方の物体へ接近）パスで、距離ごとの検出を測る。

Truck シーン、正面方位（azimuth 125°、orbit で hit した向き）、18m→4m、24 step:

```
--path dolly --azimuth 125 --near 4 --far 18 --frames 24
→ class-present rate 0.75, recall@IoU0.5 0.42, mean best IoU 0.34
→ max detection range 15.0 m
```

距離別（present = truck として検出）:

| 距離 | 検出 | 最良 IoU |
|---|---|---|
| >15 m | ✗ | 0（小さすぎ） |
| 15 → 6.4 m | ✓ | 0.47–**0.63**（9–9.5m がピーク） |
| <6 m | △ | 0.0–0.01（画面を溢れ文脈喪失） |
| 4 m | ✗ | 0 |

- **3DGS 再レンダの実トラックは正面接近で 15m から検出でき、7–13m が sweet spot**。
  近すぎ（<6m、物体が画面を溢れる）と遠すぎ（>15m、小さい）で落ちる典型的な
  検出器特性が、3DGS render 上でもそのまま出る。
- **正面方位では class-present 0.75**（全方位 orbit の 0.47 より高い）= viewpoint
  依存の再確認。closed-loop の ego は前方の車両を canonical 角度で見るので、
  この sweet-spot レンジが実効的に効く。
- dolly は recall@IoU0.5 も 0.42 と orbit（0.06）より高い。物体が常に良く framed され
  GT box と整合するため。

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
