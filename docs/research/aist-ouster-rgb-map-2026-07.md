# AIST Ouster real-RGB map benchmark (2026-07-13)

## Outcome

The AIST public Ouster captures now validate a coloured **map**, not only one
coloured scan. The CPU-only workflow runs deterministic LiDAR-only odometry,
poses every RGB image on that trajectory, accumulates every Ouster scan, and
uses occlusion-aware multi-view RGB medoids. Both independent captures produce
real chromatic maps with about 35% whole-map camera coverage.

No trajectory-accuracy claim is made: these calibration bags have no external
pose ground truth. The valid claims are map geometry, real-RGB coverage, and
held-out camera colour consistency.

## Frozen workflow

- official direct-visual-LiDAR `calib.json`
- `/points`, `/image`, `/camera_info`
- deterministic `scan_matcher_offline_runner` with `lidarslam_lo.yaml`
- automatic camera-clock to LiDAR-clock offset and drift estimation
- all RGB frames, all scans, 0.15 m map voxel, range 1–80 m
- weak density guard: one supporting point (`min_neighbors=1`)
- robust 1-pixel z-buffer + edge-aware sampling + RGB medoid fusion
- rigid scan accumulation (`--no-deskew`), explicitly frozen below
- held-out protocol: even images colour, odd images score

Reproduce one capture:

```bash
bash scripts/run_aist_ouster_rgb_map_benchmark.sh \
  --bag /path/to/rosbag2_2023_03_28-16_25_54 \
  --extrinsic /path/to/calib.json \
  --localization-zoo /path/to/localization_zoo \
  --output-dir /path/to/aist_162554
```

## Public-data evidence

| metric | 16:25:54 | 16:26:51 |
|---|---:|---:|
| scans / RGB views | 291 / 58 | 265 / 53 |
| map points | 176,709 | 137,477 |
| RGB-coloured points | 64,337 | 47,764 |
| whole-map coloured fraction | **36.41%** | **34.74%** |
| chromatic fraction (channel range ≥10) | **59.52%** | **64.14%** |
| unique observed colours | 36,416 | 29,072 |
| held-out RGB L2 median ↓ | **21.02** | **16.90** |
| held-out RGB L2 inlier ≤20 ↑ | **48.34%** | **56.23%** |
| held-out scored fraction | 99.88% | 100.00% |
| plane thickness mean ↓ | 0.0823 m | 0.0744 m |
| plane thickness p95 ↓ | 0.1230 m | 0.1193 m |
| planar coverage ↑ | 47.44% | 53.95% |
| colour-map wall time | 174.6 s | 137.2 s |
| processing / bag duration ↓ | 6.02× | 5.19× |
| colour-map peak RSS | 1,088.1 MiB | 1,003.6 MiB |

The former one-scan check covered only 9.55% / 10.57% of each 32,768-point
scan. It remains useful as a fast calibration smoke test, but it is no longer
the evidence for a camera-coloured point-cloud **map**.

The 16:25 resource row is a clean end-to-end rerun of the frozen rigid-scan
runner. Its PLY SHA-256 (`79589bd4...e3e0e5`) and all quality metrics match the
earlier rigid artifact; unlike the preliminary table, its timing comes directly
from that run rather than a first-pass deskew ablation.

## Ablations and rejected shortcuts

### Image sampling

Nearest-pixel sampling raised the single-scan chromatic fraction on both
captures (57.74→58.44%, 59.56→60.60%), but it quantizes smooth gradients and
lacked an independent held-out map win. Edge-aware threshold 48 remains the
default. The single-frame CLI now records and exposes interpolation, edge,
z-buffer, and depth-tolerance knobs so future ablations are auditable.

### Pair search and timestamp offset

Searching every image raised single-scan coverage slightly, but can select an
unrepresentative bag edge. A synthetic −50 ms scan-midpoint pairing reduced the
reported timestamp residual while lowering chromatic fraction on both captures.
Neither was adopted.

### Exposure normalization

Disabling the robust colourizer's per-view median-luminance gains was tested
first on the frozen 16:25:54 capture. The held-out evaluator applied the same
choice independently to its train and held-out image folds; it did not score
RGB values produced from held-out views.

| 16:25:54 | normalized (adopted) | raw exposure (rejected) |
|---|---:|---:|
| chromatic fraction ↑ | 59.52% | **61.66%** |
| held-out RGB L2 median ↓ | **21.02** | 21.24 |
| held-out RGB L2 p90 ↓ | **89.83** | 94.00 |
| held-out RGB L2 inlier ≤20 ↑ | **48.34%** | 47.91% |

The extra saturation did not represent more consistent colour: all held-out
quality measures regressed. The candidate therefore failed the first-dataset
gate and was not spent on the second capture. Exposure normalization remains
enabled by default. The build and held-out CLIs retain explicit opt-out and
gain-limit controls so this decision is reproducible rather than hard-coded.
The rejected artifact is
`aist_rgb_map/exposure_ablation_20260713/162554_no_normalize`.

### Per-point Ouster deskew

Ouster stores `t=0..99.96 ms`, but the scanmatcher trajectory is a sequence of
whole-scan registration results, not a continuous-time body state. Its adjacent
poses contain 4.2 cm / 1.51° mean registration changes (max 16.9 cm / 4.44°).
Interpolating those corrections through a scan looked superficially better in
held-out RGB (median 21.02→20.90) but severely damaged geometry:

| 16:25:54 | rigid scan (adopted) | interpolated per-point (rejected) |
|---|---:|---:|
| thickness mean ↓ | **0.0823 m** | 0.0891 m |
| planar coverage ↑ | **47.44%** | 25.49% |
| coloured fraction ↑ | **36.41%** | 35.58% |
| chromatic fraction ↑ | **59.52%** | 58.77% |

The runner therefore passes `--no-deskew` explicitly. Per-point deskew may be
revisited only with continuous-time IMU/odometry states and must pass geometry
and held-out RGB together.

## Artifacts

Each output contains:

- `frontend/run*/trajectory_frontend.tum` and byte-determinism evidence
- `colored_map/posed_images/transforms.json`
- `colored_map/colored_map.ply`
- `colored_map/colored_map_report.json`
- `colored_map/heldout_point_colors.json`
- `colored_map/map_quality/run1/map_quality_report.yaml`
- `colored_map/process_time.txt`
- `colored_map/runtime.json` (wall time, realtime factor, peak RSS, CPU, exit)
- `shared_evidence/cross_repo_benchmark.json` (the complete public-suite contract)

The shared manifest hashes the complete source rosbag directory, official
extrinsic, frontend trajectory, coloured PLY, profile, and every metric report.
The suite gate can therefore detect missing inputs or later artifact drift.

`analyze_colored_point_cloud.py` hashes the PLY and excludes the known default
fill colour before computing coverage/chroma, preventing unobserved grey points
from being misreported as camera-coloured.
