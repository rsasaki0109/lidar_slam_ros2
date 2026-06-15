# 3DGS dynamic actor compositing + 検出器 gap — Phase 3 (2026-06-15)

3DGS を **sim2real シミュレーション基盤**として開発するトラックの Phase 3
（actor 挿入の部分。RL はスコープ外）。静的 3DGS シーンには動的交通が無く、
Phase 0 で手元 3 シーンに COCO クラス物体が皆無 → 実画像検出器が render 上で
一度も発火しなかった（負の結果）。本 Phase はシーンに**動く actor を挿入**し、
その検出器 gap をようやく exercise する。

ツール: `tools/gaussian_splatting/actor_compositing.py`
（pure 部 19 ケースを `graph_based_slam/test/test_gaussian_splatting_actor.py`
で CPU テスト、ament_flake8 clean）。
再現: `PLY=... TRANSFORMS=... OUT_DIR=... MODE=... bash scripts/run_actor_compositing.sh`

## 仕組み

actor をシーンカメラの前方にステージし、視野を横断させる。各フレームで
**per-pixel depth test** によりシーン render と actor を合成（actor はシーンより
手前のピクセルにだけ描かれる）→ 幾何的に正しいオクルージョン。各フレームの
**ground-truth 2D box** も出力（actor の投影範囲から）。

- **box actor** — Gaussian の合成立体。シーンカメラからラスタライズして
  depth test 合成（`composite_depth`）。アセット不要、決定論的、移動物体の
  photoreal-geometry デモ。
- **sprite actor** — 実写 RGBA カットアウトを billboard 挿入。depth でスケールし
  シーン depth でオクルージョン（`paste_sprite`）。sprite は実物体なので COCO
  検出器が発火し得る → これで**検出 gap を実測**する。

## 検証（construction クリーン再構築シーン、recon 28.9 dB）

RTX 4070 Ti SUPER / gsplat 1.5.3 / yolov8n。

### box actor（オクルージョン正当性）

```
VIEW=40 FRAMES=36 MODE=box BOX_SIZE=0.6,0.6,1.7 DISTANCE=4 LATERAL=2 SCALE=0.5
→ 36/36 frames に可視 actor ラベル。GT box が x1: 0→201 と視野を横断。
```

赤い人型 box が床に立ち、**前景の機材に正しく下半身を遮蔽される**（depth test
が機能）。GT box は移動と遮蔽に追従。

### sprite actor（検出 gap 実測）

bus.jpg の縦長歩行者（504x197、素のクロップは person conf 0.87 で検出）を挿入:

| render scale | 解像度 | detection recall | mean best IoU |
|---|---|---|---|
| 1.0 | 600x440 | **0.28** | 0.28 |
| 0.5 | 300x220 | 0.06 | 0.21 |

**検出 gap は render 解像度とオクルージョンが主因**（物体の見え方ではない）:
素のクロップは 0.87 で確実に検出されるのに、3DGS render に埋め込むと
フル解像度でも recall 0.28、半解像度で 0.06 に落ちる。フレーム別に見ると
**開けた床では検出成立・前景機材の背後を歩くと下半身が遮蔽されて検出失敗**
（hit frame は視野端、miss frame は遮蔽の濃い中央）と、recall がオクルージョンに
明瞭に連動する。これが「3DGS をデータ生成・closed-loop に使うときの知覚 gap」の
最初の定量値。

## 限界 / 次アクション

- **sprite は矩形カットアウト**（セグメンテーション alpha でない）ので GT box に
  背景マージンが入り、検出が成立しても IoU が伸びにくい（mean ~0.28）。tight な
  検出 gap には人物セグメント alpha か、actor 自体の 3DGS モデルが要る。
- **真に photoreal な actor** は実物体の 3DGS モデル（車・人の learned Gaussian）を
  挿入するのが本命。billboard は単一 depth なので斜めビューで平面感が出る。
- 検出 gap の改善余地: 解像度を上げる / closed-loop では sensor_sim_node
  （Phase 2）の出力解像度を上げる。低解像度ほど gap が急拡大するのは
  運用上の重要知見。
- RL（動的 actor 群 + 方策学習）は本 Phase のスコープ外。compositing と GT label の
  基盤は整ったので、actor 軌跡を増やせば multi-agent シナリオに拡張できる。

## ロードマップ上の位置づけ

```
Phase 0  外挿安定性測定 (有効視点範囲)            ← 完了
Phase 1  open-loop RGB-D データ生成               ← 完了 (3dgs-dataset-gen-phase1.md)
Phase 2  closed-loop sensor-sim ROS 2 node        ← 完了 (3dgs-sensor-sim-phase2.md)
Phase 3  dynamic actors (compositing + 検出 gap)  ← 本ノート（RL は将来）
```
