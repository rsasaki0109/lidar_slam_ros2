# Interactive 3DGS Map Viewer

Spin the LiDAR-primed 3D Gaussian Splatting map in your browser — no install,
no GPU. This is the same photoreal map produced by the
[3DGS Photoreal Map](3dgs-map-tutorial.md) pipeline, exported to a web-sized
`.splat` and rendered with WebGL.

<iframe
  src="assets/3dgs-viewer/index.html"
  title="Interactive 3DGS map viewer"
  width="100%"
  height="520"
  style="border: 1px solid #ccc; border-radius: 8px;"
  loading="lazy"
  allowfullscreen></iframe>

!!! tip "Controls"
    The viewer opens at one of the map's capture views, so the first frame is
    photoreal. **Drag** to orbit · **scroll / pinch** to dolly ·
    **right-click drag** (or shift + scroll) to pan. Nudge gently — this is a
    single-bag, close-range capture, so orbiting far from the recorded path
    shows the usual view-dependent Gaussian smear. Drop any INRIA-layout `.ply`
    onto the viewer to convert and inspect it in-browser.

## What you are looking at

The bundled map is the koide handheld scene (Livox MID-360 + RGB camera): a
single-bag, close-range capture reconstructed at SH degree 1 over 15k
iterations (~25.5 dB PSNR). The geometry is seeded from the SLAM point cloud
(LiDAR-primed init), so the Gaussians sit on the metric surface rather than a
photometric guess.

## Export your own

The viewer reads the compact `.splat` format. Convert any map trained with
[`train_gsplat.py`](3dgs-map-tutorial.md) using the numpy-only converter:

```bash
python3 tools/gaussian_splatting/ply_to_splat.py \
  output/<run>/gsplat/point_cloud.ply \
  -o docs/assets/3dgs-viewer/my_map.splat \
  --max-points 300000 --min-opacity 0.04 --max-scale 0.5
```

`--max-points` keeps the most significant Gaussians first (volume × opacity),
trading file size for detail — 300k splats is ~9 MB, a good balance for a fast
web load. `--max-scale` drops the handful of giant floater Gaussians that
otherwise streak across off-axis angles. Point the viewer at your export with
`assets/3dgs-viewer/index.html?url=my_map.splat`.

The renderer is vendored from [antimatter15/splat](https://github.com/antimatter15/splat)
(MIT); see `docs/assets/3dgs-viewer/README.txt` for the one-line local change.
