# 3DGS closed-loop RL — Phase 4 環境基盤 (2026-06-16)

sim2real トラックの Phase 4（RL）の第一段 = **学習環境**。closed-loop の部品は
既に揃っている（GPU レンダラ、sensor-sim node、depth-test actor 合成、検出
スコアリング）ので、それらを標準の `gymnasium.Env` として包み、エージェントが
学習できる基盤を作った。

ツール: `tools/gaussian_splatting/drive_env.py`（pure 部 12 ケースを
`graph_based_slam/test/test_gaussian_splatting_drive_env.py` で CPU テスト、
gymnasium env_checker 通過、ament_flake8 clean）。
学習: `tools/gaussian_splatting/train_drive_policy.py` / `scripts/run_drive_rl.sh`。

## タスクと環境

- **ego**: 平面 unicycle `(x, y, yaw)`。action = `[v, omega]`（正規化）。
- **タスク**: goal pose に到達。ただし **valid viewpoint corridor**（Phase 0:
  3DGS render が信頼できるのは記録軌跡から `max_dev` 以内）に留まる。
- **報酬**: goal への前進 − step cost − corridor 逸脱ペナルティ（+ 到達ボーナス）。
- **終了**: goal 到達 / 場外 / corridor を `hard_dev` 超で逸脱 / timeout。
- **観測 2 モード**:
  - `state` — 低次元ベクトル `[goal_dist, cos(bearing), sin(bearing), corridor_dev]`。
    GPU 不要、CPU で数秒学習・単体テスト可能。
  - `camera` — ego カメラの 3DGS render（注入する `render_fn` = `GaussianRenderer`
    クロージャ）。これが **sim2real 観測**で、closed-loop sensor-sim が配信する像と一致。

運動学・報酬・終了は pure numpy ヘルパー（単体テスト済み）で、`DriveEnv` は
それらを Gymnasium API と任意のレンダラに配線するだけ。

## 学習結果（state 観測、PPO、合成 arc コリドー）

stable-baselines3 PPO / MlpPolicy / 60k step（CPU）:

```
random policy : mean return -26.07, success  0%
PPO (60k step): mean return  23.78, success 100%
```

- **RL ループが収束**: ランダム方策は corridor を外れて 0% 成功なのに対し、PPO は
  60k step で **goal 到達 100%**・return が大きく改善。環境が end-to-end で学習可能
  であることを実証（= Phase 4 RL 着手）。
- 報酬が「前進 − corridor 逸脱」なので、学習方策は **Phase 0 の有効視点範囲内に
  留まりながら goal へ向かう**走り方を獲得する（sim2real の前提と一貫）。

## 追記: dynamic actor 回避タスク (2026-06-16)

closed-loop の dynamic actor（`3dgs-actor-compositing-phase3.md`）を RL に取り込んだ。
env に**横断歩行者**（`crossing_actor`、step→xy のスクリプト actor）を追加し、
state 観測に actor の ego 相対 range/bearing（+3 次元）、報酬に近接 yield コスト、
衝突（`actor_radius` 内）で終了＋ペナルティを加えた（後方互換: actor 無しは現状維持）。

直線コリドー＋「フル速で走ると到達タイミングで歩行者が経路中心に来る」横断を設定
（= **直進フル速は step 18 で衝突**する経路）。stable-baselines3 PPO / state 観測 / CPU:

```
random policy : mean return -24.17, success   0%, collision 0%
PPO (120k)    : mean return  24.05, success 100%, collision 0%
```

- **PPO は衝突 0% で goal 到達 100%** を獲得。rollout では最小 ego–歩行者距離 2.86 m
  （衝突半径 1.0 m に対し安全マージン）を保ち、安全な接近速度（throttle ~0.74、
  フル 1.0 でない）で歩行者が抜けた後に通過する。**直進フル速が衝突する経路で、
  actor 観測を使って衝突回避方策を学習**したことを示す。
- これで RL は「ナビ（goal 到達）」だけでなく「**動的物体の知覚-回避**」まで学習可能と
  実証。検出スコアリング（`detect_in_scene`/`sim2real_gap`）を報酬項に差し替えれば、
  pixel 観測下の知覚駆動方策へ直結する。

## sim2real ブリッジ（pixel 観測）と次

- `obs_mode='camera'` + `render_fn`（`GaussianRenderer.render(pose→viewmat)`）で、
  **エージェントが 3DGS render を直接観測**して学習できる。これは GPU rollout で
  重いので本検証（state 観測）から分離した、明確な次ステップ。
- corridor anchors は記録 SLAM 軌跡（TUM）をそのまま使える（`--traj`）。stadtgarten /
  construction の実コリドーで「有効視点範囲内を走る」方策を学べる。
- dynamic actor（`3dgs-actor-compositing-phase3.md` の box/sprite/ply）を毎 step
  シーンに合成すれば、**動的交通下の知覚-行動方策**（回避・追従）に拡張できる。検出
  スコアリング（`detect_in_scene` / `sim2real_gap`）はそのまま報酬項に使える。

## ロードマップ上の位置づけ

```
Phase 0  外挿安定性 / 有効視点範囲                  ← 完了
Phase 1  open-loop RGB-D データ生成                 ← 完了
Phase 2  closed-loop sensor-sim ROS 2 node          ← 完了
Phase 3  dynamic actors + 検出 gap (orbit/dolly/LiDAR-primed) ← 完了
Phase 4  closed-loop RL                              ← 本ノート（env + PPO 収束。pixel/dynamic は次）
```
