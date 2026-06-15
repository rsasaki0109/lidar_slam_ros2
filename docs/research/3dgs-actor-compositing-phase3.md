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

GPU / gsplat 1.5.3 / yolov8n。

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

## 追記: セグメント alpha sprite で gap を tight に再測 (2026-06-16)

初回の矩形クロップ sprite は背景マージンを抱え（alpha coverage ~1.0）、
合成にハローが出て GT box も緩く、検出が成立しても IoU が伸びなかった。
インスタンスセグメンテーション（`tools/gaussian_splatting/make_actor_sprite.py`、
ultralytics `*-seg` + `retina_masks`、pure 部 8 ケースを
`test_gaussian_splatting_sprite.py` で CPU テスト）で**人物シルエットの alpha matte**
に置き換えると、合成は背景ハローなし・GT box は tight になり、検出 gap が改善した:

| sprite | render scale | recall | mean best IoU |
|---|---|---|---|
| 矩形クロップ | 1.0 (600x440) | 0.28 | 0.28 |
| **セグメント alpha** | 1.0 | **0.50** | **0.38** |
| 矩形クロップ | 0.5 (300x220) | 0.06 | 0.21 |
| **セグメント alpha** | 0.5 | **0.33** | **0.31** |

bus.jpg の歩行者 sprite は 194x507・alpha coverage 0.52（= 矩形の約半分が背景だった）。
背景除去でフル解像度 recall がほぼ倍増（0.28→0.50）。`paste_sprite` は元から
per-pixel alpha を尊重するので合成側の変更は不要、sprite 生成だけで gap が締まる。
**それでも recall 0.50 止まり**なのは依然オクルージョン（前景機材の背後で失敗）と
低解像度が主因で、Phase 3 本体の所見は変わらない。

## 追記: volumetric photoreal actor (ply モード) (2026-06-16)

billboard sprite は単一 depth なので視点が振れると平面に見える。これを解消する
**volumetric actor**（任意の学習済み 3DGS .ply を actor として読み込む `--mode ply`）を
追加した。box actor と同じ depth-test ラスタライズ経路に乗り、毎フレーム
`transform_gaussians` で world pose に置いて `rasterize_rgbda` → `composite_depth`。
真の Gaussian 幾何なので**正しい視差**を持ち、**per-pixel で前景に部分遮蔽される**
（単一平面の billboard では原理的に不可能）。

photoreal なデモ素材は外部 COCO アセット（データ依存）に頼らず、
**シーン自身の Gaussian を AABB で切り出して可動オブジェクト化**する
（`tools/gaussian_splatting/crop_actor_ply.py`、pure 部 `crop_gaussians`/
`recenter_gaussians` を CPU テスト）。検証（construction, view40）: シーンから
10265 Gaussian（1.4x1.4x3.4m）を切り出し → actor として横断 → 実写の塊が床に立ち
前景機材に部分遮蔽されて合成される（600x440, 36/36 frame ラベル）。

残る素材課題は「**車・人クラスの learned 3DGS モデル**」の調達（コードでなくアセット）。
machinery（ply 読み込み・配置・parallax・occlusion・GT label）は完成しているので、
そうした actor モデルを `--actor-ply` に渡せばそのまま COCO 検出 gap も締まる。

## 限界 / 次アクション

- **COCO クラスの learned 3DGS actor モデル**（車・歩行者）の調達がデータ依存で残る。
  公開 object-3DGS データセット or 小規模学習が経路。machinery は ply モードで完成済み。
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
