# COCO クラス volumetric actor の調達 (2026-06-16)

Phase 3（`3dgs-actor-compositing-phase3.md`）で残った「真に photoreal な actor には
actor 自体の 3DGS モデルが要る（学習アセット待ち）」を埋めるための調達ノート。
**実在の learned 3DGS COCO クラス物体**を入手し、`--mode ply` の volumetric actor
として検出可能なことを検証した。

## 調達したアセット

| 項目 | 内容 |
|---|---|
| ソース | `Voxel51/gaussian_splatting`（HuggingFace dataset） |
| ライセンス | **Apache-2.0**（permissive、ローカル評価に支障なし） |
| シーン | Tanks&Temples **Truck**（旧型ピックアップを 360° 撮影） |
| ファイル | `FO_dataset/truck/point_cloud/iteration_30000/point_cloud.ply`（2,056,645 Gaussian, sh_degree 3） |

入手:
```
curl -sL "https://huggingface.co/datasets/Voxel51/gaussian_splatting/resolve/main/\
FO_dataset/truck/point_cloud/iteration_30000/point_cloud.ply" -o output/assets/truck_scene.ply
```

## actor 化（シーン → standalone actor）

`crop_actor_ply.py` でトラックを AABB 切り出し。Tanks&Temples は **y-up** なので
`--up-axis y` で z-up（actor 規約）に reorient:
```
python3 tools/gaussian_splatting/crop_actor_ply.py \
  --ply output/assets/truck_scene.ply --out output/assets/truck_actor.ply \
  --center=1.0,-2.5 --half-extent 4.0 --z-range=-4,3 --up-axis y
# -> 538170 gaussians, extent 7.99 x 7.00 x 3.93 m
```

## 検証: COCO 検出器が "truck" として発火する

truck_actor を単体レンダ（z-up・接地）して yolov8n にかけると、複数視点で
**truck として確実に検出**:

| view | 検出 |
|---|---|
| 30° | truck **0.78** |
| 70° | truck 0.49（+ stop sign 誤検出） |
| 110° | truck **0.73** |

→ 調達物は「3DGS render 上で実画像検出器が発火する COCO クラス volumetric actor」
として機能する。Phase 3 までは手元 3 シーンに COCO 物体が皆無で exercise 不能
だった検出 gap の、**物体側ブロッカーが解消**された。

## 性能上の必須修正（million-Gaussian actor）

実シーン由来の actor は ~10^6 Gaussian になる。Phase 3 の `_rotate_quats_wxyz` は
Python ループで、538k × フレーム数では実質ハングした。
(1) quat 回転を **Hamilton 積でベクトル化**、(2) heading は sweep 中一定なので
**回転は 1 回だけ・フレームごとは並進のみ**、に修正して実用速度にした。

## 残課題: シーン規模・品質の整合（data gap）

調達物（実寸 8m トラック）を手元 SLAM シーンに合成する所で**シーン側**が律速:

- **construction**（recon 28.9 dB と高品質だが狭い屋内）: 全 view で前方 depth
  ~1–4m。8m トラックは壁の奥に置かれ depth-test で完全オクルージョン、近づけると
  巨大クリップ。**屋内の狭小空間に実寸車両は物理的に入らない**。
- **isuzu**（屋外・走行スケールで広いが recon 17.3 dB と低品質）: 空間は足りるが
  washed-out で合成が成立しない。

→ 実寸車両を活かすには **大規模かつ高品質な屋外路上シーン**が要る。これは
machinery（ply 読み込み・配置・視差・オクルージョン・GT label）でなく、
**シーン素材の data gap**。machinery と検出可能な COCO actor は揃ったので、
そうしたシーンが用意できれば `--actor-ply output/assets/truck_actor.ply` を
渡すだけで検出 gap が締まる。

## 関連
- 機構: `3dgs-actor-compositing-phase3.md`（box/sprite/ply モード、depth-test 合成）
- Phase 0 の有効視点範囲・シーンスケール依存の所見と一貫（走行スケール屋外が本命）
