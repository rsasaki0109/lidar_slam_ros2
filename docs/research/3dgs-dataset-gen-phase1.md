# 3DGS open-loop データ生成 — Phase 1 (2026-06-15)

3DGS を **sim2real シミュレーション基盤**として開発するトラックの Phase 1。
Phase 0（`3dgs-sim2real-gap-phase0.md`）で測った**有効視点範囲**の内側で、
学習済み LiDAR-primed シーンを**ラベル付き RGB-D データセット**に変換する。
オフラインの知覚モデル学習・評価にそのまま投入できる形が成果物。

ツール: `tools/gaussian_splatting/generate_dataset.py`
（pure 部 14 ケースを `graph_based_slam/test/test_gaussian_splatting_dataset.py`
で CPU テスト、ament_flake8 clean）。
再現: `PLY=... TRANSFORMS=... OUT_DIR=... bash scripts/run_generate_dataset.sh`

## 何を出すか

各参照ビューについて、記録 pose に加えて **jitter pose**（カメラ右 x・下 y
方向の横ずれ）を `--max-lateral` / `--max-vertical` の箱内から決定論的に
サンプル（seed 固定 → データセットはバイト再現可能）。各フレームを書き出す:

- `rgb/<stem>.png` — 8-bit RGB レンダ
- `depth/<stem>.png` — **16-bit metric depth**（既定でミリメートル。
  gsplat の expected depth = 不透明度重み付きレイ距離。LiDAR-primed なので単位は
  メートル → 真の range ラベル）
- `transforms.json` — nerfstudio 互換の intrinsics + 各フレーム camera-to-world

depth は `depth_scale`（uint16 1 単位あたりのメートル、既定 0.001）で量子化し、
無効サンプル（非有限・負）は 0（= no return）、`--max-depth` と uint16 上限で
クランプ。`train_gsplat.load_transforms` で読み戻せる convention（OpenCV w2c /
OpenGL c2w フリップ）で出力するので、生成データを再学習にもそのまま使える。

## なぜ jitter が効くか（Phase 0 との接続）

記録軌跡だけだとデータは 1 本の視点列に縛られる。Phase 0 で「有効視点範囲内なら
render は破綻せず、LiDAR-primed の正しい幾何が横ずれに対し正しい視差を出す」ことを
確認済みなので、その範囲内で pose を散らせば**幾何的に妥当な新規視点**を量産できる。
これが 3DGS をデータ拡張基盤にする差別化点（NeRF/画像合成と違い depth が metric）。

範囲はシーンスケール依存（Phase 0 の結論）。近接屋内ウォークは ~0.2 m、
開けた走行スケールは ~1.0 m まで。`--max-lateral` をその範囲に収めること。

## 検証（construction クリーン再構築シーン）

GPU / gsplat 1.5.3。クリーン再構築済み
`output/rtkslam_3dgs_clean/gsplat/point_cloud_good.ply`（recon 28.9 dB）。

```
VIEWS=8 AUG=2 MAX_LATERAL=0.2 MAX_VERTICAL=0.05 SCALE=0.5 MAX_DEPTH=30
→ 24 RGB-D frames (8 views x 3), 300x220
→ depth coverage 1.00, median 2.57 m, p95 6.00 m
```

- **depth は実シーンスケールで妥当**: 屋内マシンホール近接ウォークの被写体距離
  （中央値 2.5 m、p95 6 m）と一致。uint16 PNG を読み戻すと metric（p50 2.47 m）。
- **transforms.json は round-trip 健全**: `load_transforms` で 24 frame・
  300x220 を読み戻せることを確認（再学習・別ツール消費が可能）。
- データセットは seed 固定で**バイト再現可能**。

## ロードマップ上の位置づけ

```
Phase 0  外挿安定性測定 (有効視点範囲)         ← 完了
   ▼
Phase 1  open-loop RGB-D データ生成            ← 本ノート（有効範囲内で量産、depth metric）
   ▼
Phase 2  closed-loop sensor-sim ROS 2 node     ← 完了 (3dgs-sensor-sim-phase2.md)
   ▼
Phase 3  dynamic actors + RL
```

## 限界 / 次アクション

- jitter は静的シーンの視点拡張のみ。動的物体（車両・歩行者）は映らない
  → Phase 3 の actor compositing で供給し、そこで初めて検出器 gap を exercise できる
  （Phase 0 で手元 3 シーンとも real 画像で COCO 検出ゼロだった負の結果の解消経路）。
- depth は expected depth（不透明度重み付き）なので、半透明 Gaussian が多い領域では
  surface depth よりやや手前寄りに出ることがある。固い面（LiDAR-primed の主成分）では
  問題にならないが、薄い構造の多いシーンでは median depth で健全性を確認すること。
