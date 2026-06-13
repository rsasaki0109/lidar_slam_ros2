# Vendored WebGL 3DGS viewer

`index.html` + `main.js` are vendored from
[antimatter15/splat](https://github.com/antimatter15/splat) (MIT License, see
`LICENSE`). Local modifications, all in `main.js` (each tagged with a comment):

1. The `?url=` splat is resolved relative to this page (so the bundled map
   loads from the docs site) instead of the upstream Hugging Face base, and the
   default model points at `koide_lidar_primed.splat`.
2. `defaultViewMatrix` is set to one of the koide map's training views, so the
   first frame is photoreal. Free-orbiting away from the capture path degrades
   (single-bag view-dependent overfit), so we open at a known-good pose.
3. The auto-fly `carousel` starts disabled (its preset cameras are from a
   different upstream scene); the view stays put until the user drags.

`index.html` additionally swaps the upstream credit line for a lidarslam_ros2
attribution (keeping the antimatter15 + MIT credit).

`koide_lidar_primed.splat` is a 300k-Gaussian, web-sized export of the koide
LiDAR-primed map (SH deg-1, 15k iters, ~25.5 dB PSNR). Regenerate it from a
trained INRIA-layout `.ply` with the converter in this repo:

```bash
python3 tools/gaussian_splatting/ply_to_splat.py \
  output/koide_3dgs_firstlight/gsplat/pc_sh1_15k.ply \
  -o docs/assets/3dgs-viewer/koide_lidar_primed.splat \
  --max-points 300000 --min-opacity 0.04 --max-scale 0.5
```

Drop any other `.ply` onto the viewer to convert + view it in-browser, or pass
`?url=<your>.splat` to load a different bundled map.
