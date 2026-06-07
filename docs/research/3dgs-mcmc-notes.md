# 3DGS densification strategy: MCMC vs clone/split on koide (2026-06-07)

gsplat には密度制御の戦略が2つある: `DefaultStrategy`（INRIA 流の clone/split/prune）と
`MCMCStrategy`（固定予算 `cap_max` で低 opacity の splat を Metropolis-Hastings で
**再配置**、opacity/scale の L1 正則化が必要）。MCMC は論文で「同予算なら高品質」と
報告されるため、`--mcmc` を追加して koide で比較した。
**結論: LiDAR-primed init が効いている koide では DefaultStrategy が一貫して優位**
（MCMC は -1.3〜-1.5dB）。

## 実装

- `--mcmc`（opt-in、implies densify）+ `--mcmc-cap`（既定 50万）。
- 損失に MCMC 正則化（gsplat 既定）を加算: `0.01*|sigmoid(opacity)|_1 +
  0.01*|exp(scale)|_1`。`step_post_backward` は means LR を渡してオプティマイザ step 後に
  呼ぶ（relocation がモーメントをリセットするため）。SH 係数 `sh0`/`shN` も追加キーとして
  relocation 対象になる。

## 結果（koide, SH deg1, LiDAR-primed init, SSIM densify, 学習ビュー評価）

| strategy | iter | PSNR | SSIM |
|----------|------|------|------|
| **DefaultStrategy** | 9000 | **25.18 dB** | **0.8569** |
| MCMCStrategy (cap 400k) | 9000 | 23.66 dB | 0.8319 |
| **DefaultStrategy** | 15000 | **25.47 dB** | **0.8617** |
| MCMCStrategy (cap 400k) | 15000 | 24.12 dB | 0.8367 |

## 解釈

- **MCMC は全 iter で -1.3〜-1.5dB**。長く回しても（15k）DefaultStrategy の 9k にすら届かない。
- 理由: **koide は LiDAR-primed init で既にメトリックな幾何事前を持つ**。DefaultStrategy の
  clone/split は「正しい場所にある Gaussian を増やして精緻化」する一方、MCMC は固定予算で
  Gaussian を**再配置**し、opacity/scale 正則化で間引くため、せっかくの幾何事前を一部捨てる
  方向に働く。MCMC が活きるのは random/COLMAP-SfM init で**構造をゼロから発見**する場面。
- → **LiDAR-primed パイプラインでは DefaultStrategy が既定で正解**。`--mcmc` は非 LiDAR-init
  / 幾何事前が弱いデータ用に残すが、koide 型では使わない（既定 OFF）。

これで密度制御戦略も「LiDAR 幾何事前があるなら clone/split」という形で決着。3DGS 品質の
ベスト構成は引き続き **SH deg1 + classic DefaultStrategy + iter 9000〜15000 = 25.2〜25.5dB**
（`3dgs-sh-degree-notes.md`）。
