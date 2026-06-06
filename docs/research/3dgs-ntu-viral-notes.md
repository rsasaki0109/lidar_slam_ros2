# 3DGS on NTU VIRAL tnp_01 — multi-view attempt (2026-06-06)

koide first light（`docs/research/3dgs-koide-first-light.md`）の上限要因が「視点数 30 /
単一短セグメント」だったため、**多視点**の NTU VIRAL tnp_01（地上トラバース、
mono カメラ 10Hz・580s）で評価を試みた記録。**正直な結論: パイプラインは
end-to-end で通るが、3DGS 品質は低い（~10–11 dB）**。koide（~25dB）との差は
シーン性質の差。

## やったこと
- RKO-LIO + graph backend で SLAM 軌跡生成（`run_rko_lio_graph_benchmark.sh`、
  restamped bag）。**離陸前静止で quiescence が誤発火**するため
  `--quiescence-secs 150` が必要だった。
- extrinsic 連鎖: `base_T_lidar ∘ T_Body2Lidar == I` より **base_link ≡ NTU Body**、
  よって `base_link_T_cam = inv(T_Body2Cam)`
  （`configs/gaussian_splatting/ntu_viral_tnp01_left_extrinsic.yaml`）。
- 時間窓 [60,120]s を `--start-time/--end-time` で切り出し、`--intrinsics-yaml`
  （camera_left.yaml）+ `--undistort`（k1=-0.288 を pinhole 化）で 300 views 抽出。
- LiDAR-primed init（600 scans → 250k 点）+ densify で学習。

## 結果と診断
- PSNR ~10–11 dB、render は霧状。
- **診断: カメラ中心は LiDAR 雲の重心とほぼ一致**（cam mean ≈ cloud mean、
  path 51.8m）→ extrinsic/pose は概ね正しい。
- 問題はシーン性質: LiDAR 雲が **75m 級の広域**で、カメラが見る構造（10–30m 先の
  建物・樹木）に対応する点が**疎**。さらに **mono 低テクスチャ**。広域・疎・単色は
  近接密シーン（koide）より 3DGS に本質的に不利で、最小トレーナ（MSE のみ、
  基本 scale init）では収束しない。

## 学び（汎用ツールとして残した機能）
- `extract_posed_images.py`: `--intrinsics-yaml`（bag に camera_info 無い場合）、
  `--undistort`（gsplat は pinhole）、`--start-time/--end-time`（広域から窓切り出し）、
  **storage filter + 窓終端 break**（大型 bag の高速読み）。
- `build_lidar_init.py`: 同じ time-window + storage filter。
- これらは別データセットでも有用。

## もし NTU で品質を狙うなら（未実施）
- より小さい時間窓（~20s、視点重複大）で局所シーンに絞る。
- restamped 軌跡と画像 bag の**クロック対応**を厳密化（今回は同一エポック前提）。
- SSIM/LPIPS 損失、scene-scale を考慮した scale init、長 run。
- ただし mono 広域という素性上、koide 級の見栄えは期待しにくい。

→ 当面は **近接・密・カラーのシーン**（koide 型）が 3DGS 成果物に向く、という
position を支持する結果。
