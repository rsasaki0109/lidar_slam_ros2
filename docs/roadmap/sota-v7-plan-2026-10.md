# SOTA v7 Plan (2026-10)

This plan picks up from the [SOTA Product Development Plan](sota-product-development-2026-08.md)
and the [SOTA-v6 recovery plan](../research/sota-v6-recovery-plan-2026-08.md). Their
measurement policy, the fresh-holdout rules, and the SOTA-v6 runtime and memory gates
still apply. What changes is the order of the work: the next bottleneck is
generalisation (one configuration across datasets), not any single algorithm.

## 1. Where we stand (2026-10-26)

| Result | Evidence | Why it is not yet a SOTA claim |
| --- | --- | --- |
| HILTI 2022 `exp04` vs GLIM CPU | APE 0.048 vs 0.086 m, RTF 0.194 vs 0.218, peak RSS 369 vs 696 MB (three alternated runs, same VM) | One sequence, one rival; FAST-LIVO2 (historical 0.053 m all-seven) is comparable |
| MapsHD humanoid MID-360, 10-sequence set | Geometric mean 0.091 → 0.036 m over report chapters 15–25 | Every setting was chosen on these sequences; the MapsHD frontend profile makes Oxford worse |
| ENWIDE (10 sequences) | Lowest median ATE of five methods (0.29 m), no failure | Three of ten sequences were used to build the configuration |
| GEODE Metro Tunneling (13 held-out bags) | No failure | BIEVR-LIO and FAST-LIO2 are more accurate (0.35–0.36 vs 0.84 m on tunnel1–3) |
| SOTA-v5 blind evaluation | Geometric mean APE 24.99 m vs Voxel-SLAM 2.72 m; peak RSS 3.1× Point-LIO | Failed |

Every recent win depends on a configuration chosen on the same data, and the only blind
evaluation failed. The fresh-holdout acquisition of 2026-08 also stalled: M2DGR's
transfer endpoint returned 403, and UrbanLoco and MulRan need a request form.

## 2. What a claim will mean

**One global configuration (G).** A single frontend and backend parameter set for every
sequence. Per-rig inputs are allowed: topics, frames, extrinsics, IMU acceleration unit,
calibrated clock offsets, and the LiDAR beam model. The gravity-window alignment is also
a per-rig input (decided 2026-10-26, report chapter 30.9). It is on only for a rig whose IMU
calibration (for example static multi-orientation data, never a reference trajectory)
shows a small horizontal accelerometer bias. It is off for public data without such a
calibration. Nothing else may differ by dataset or
sequence. Self-adaptation inside the algorithm is allowed when it uses only sensor data
and its rule and thresholds are frozen before any holdout is opened.

**Claim tiers.** Each tier comes from the same G and the same blind run, scored separately:

| Tier | Platforms | Primary rivals |
| --- | --- | --- |
| C0 overall | every holdout sequence | all pinned rivals |
| C1 walking and handheld | humanoid, quadruped, handheld | GLIM, FAST-LIO2, Point-LIO, BIEVR-LIO |
| C2 vehicle | UGV, car | GLIM, FAST-LIO2, Point-LIO, Voxel-SLAM |
| C3 degenerate | tunnels, long corridors, open fields | BIEVR-LIO, COIN-LIO, FAST-LIO2 |
| C4 external | public hidden-ground-truth leaderboards | the leaderboard |

**Win condition** for C0–C3, unchanged from SOTA-v6: lowest geometric-mean APE among the
pinned rivals; no sequence more than 10% worse than its best rival; 100% completion; the
SOTA-v6 runtime and memory gates. Rivals run their own published configuration with only
the sensor settings changed, at revisions pinned before the run. The upstream PRBonn
RKO-LIO is added as a rival, so that the claim also measures this repository's changes.

## 3. Data

### Development data (exposed; may be tuned on)

MapsHD (8 trials), Oxford Spires Keble 05, HILTI 2022 `exp01`/`exp04`/`exp07`,
NTU VIRAL, MID-360 RTK-SLAM, Koide handheld outdoor, ENWIDE (10), GEODE (Hard, stairs,
Metro), NavINST, UrbanNav, KITTI 00/05/07. Together with the families that the SOTA-v6
recovery plan marks as consumed, none of these may appear in a later blind claim.

### Holdouts (unopened)

| Slot | Family | Tier | Status (2026-10-26) |
| --- | --- | --- | --- |
| H1 | UrbanLoco | C2 | Not available: the request form requires an institutional email address |
| H2 | MulRan | C2 | Not available: the request form requires an institutional email address |
| H3 | M2DGR | C2 | Blocked: the official transfer endpoint returned 403 |
| H4 | HILTI 2022 `exp14`, `exp16`, `exp18` | C0 (same family) | Frozen unopened since 2026-08-21 ([selection](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml)); a within-family holdout, weaker than a fresh family |
| H5 | GrandTour (ETH, ANYmal legged robot) | C1, possibly C3 | Candidate (2026-10-26). Public on Hugging Face (`leggedrobotics/grand_tour_dataset`, MIT, not gated), about 50 missions, absent from this repository's records. The Livox MID-360 and Hesai XT-32 point clouds and several IMUs are per-topic files of 0.5–1.7 GB per mission. Reference trajectories are RTK-GNSS/INS (`cpt7_*`) and a Leica total station (`prism_position`). The reference topics and the `dlio_*` outputs of another SLAM system stay unread until the outputs are sealed. No algorithm is run on it before G is frozen. **In use for C4 (2026-10-26):** its COMFORT Localization Benchmark hides the references of six test missions. G was preregistered for it before any test mission was run (`bench_report/data/grandtour/preregistration_G_comfort.txt`); the two validation missions (HEAP-1, ARC-3) were used only to check the frame chain and are no longer holdouts. Fourteen train missions became development data on 2026-10-27 by the maintainer's decisions (`bench_report/data/grandtour/dev_missions_2026-10-27.txt`, `dev_missions_2026-10-27b.txt`). The other missions' public references stay unread, so they remain a C1 holdout |
| H6+ | to be audited | C2, C3 | Vehicle and degenerate tiers still need families that can be downloaded without an institutional request |

A tier is not claimed until it has at least two fresh families.

### Cloud-VM budget

The work runs on one 4-vCPU, 15 GiB cloud VM with about 30 GB of free disk. Datasets are
fetched one at a time, converted, run, and deleted. Only trajectories, logs, and hashes
are kept. Ground truth is moved to a sealed directory on download and opened only after
the outputs are hashed, as in report chapter 15.

## 4. Phases

### Phase 1 — Collapse the per-dataset profiles into G (now)

The [profile inventory](#appendix-profile-inventory) shows 36 algorithm keys that differ
from the RKO-LIO defaults across 15 representative profiles. They fall into five groups:

1. **Competitive core**, shared by 5–12 families: `voxel_size: 0.25`, `min_range: 0.7`,
   `initialization_phase: true`, `max_iterations: 50`, `convergence_criterion: 3e-5`,
   `icp_keypoint_voxel_multiplier: 2.25`. This is the starting point for G.
2. **Gravity and attitude**: `max_expected_jerk: 1000` (4 families), `min_beta: 1e9`
   (4, GEODE), `initialization_max_gyro_rad_s: 0.02` (7), `gravity_window_alignment` with
   map relevelling (4), `max_tilt_correction_deg` (1). These five settings all control how
   much the accelerometer is trusted to correct tilt. The goal is one rule driven by
   measured specific-force consistency instead of five switches.
3. **Degeneracy and robustness**: `bump_image_registration`, `velocity_window_sec`,
   `skip_registration_after_gap_sec`, `icp_levenberg_marquardt` (4–5 families, all
   ENWIDE/GEODE), `vehicle_stall_guard` (3), `photometric` (1). These are gated features.
   The test is that they stay inactive (byte-identical, or under 1% APE change) on
   non-degenerate data.
4. **Walking**: the `gait_*` settings (GEODE handheld only). They must become self-gated
   by a GT-blind gait detector before they can enter G.
5. **Structure-specific**: `localizability_normal_only` with `max_correspondence_distance:
   0.25` (MapsHD and the walking profile). They help MapsHD and hurt Oxford. The fixed
   value has to be replaced by one derived from the scan, for example from the point
   spacing or the motion-model residual.

Procedure:

- Build G step by step (core → gravity → gated features → walking → structure). Each step
  runs frontend-only on every development family once (`n=1`).
- Choose every self-adaptation threshold by leaving one dataset family out: choose on the
  others and check on the one left out. Overfitting must show up there before a holdout
  is spent.
- **Gate:** with G, every development family is within 1.10× of its own best profile's
  APE, and the geometric mean is within 1.05×.

Status (2026-10-26, [report chapter 30](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/bench_report/REPORT.md)):

- The first candidate was the shared profile plus a median-scaled Geman-McClure kernel. It
  roughly halves the error of the indoor MapsHD trials. It was rejected because it loses the
  vertical constraint outdoors: MapsHD outdoor-urk was 1.43× its best profile, and Oxford
  1.13×.
- Four other variants were also rejected.
- Separate kernel scales for horizontal and other surfaces were rejected too: MapsHD
  outdoor-urk was 1.32× its best profile, and garage 1 1.14×.
- The two outdoor development sequences point in opposite directions, so more outdoor
  development data comes first: Koide outdoor_hard, then ENWIDE.
- Koide outdoor_hard needs the gravity-window keys. The shared profile is 1.51× worse there.
  Adding only those keys (G3) matches the Koide profile on Koide but breaks the height
  everywhere else: Oxford 0.88 m, MapsHD calibration-room 1 0.29 m.
- Before any gravity key enters G, a GT-free test is needed of when the window's gravity
  direction can be trusted.
- Ground-truth attitudes show why G3 fails (report chapter 30.8).
  - On Oxford and MapsHD, the window tilt turns with the heading. It comes from a body-frame
    accelerometer bias of 0.7–1.0°, which is larger than RKO-LIO's own attitude drift.
  - Koide's bias is 0.3°.
  - Separating the bias inside the window (G4) and raising the re-level threshold to 1°
    (G5) both fail.
- No GT-free test was found for when the window may be used (chapter 30.9). The gravity
  window is now a per-rig input, and long handheld outdoor walks like Koide's are a known
  weakness of G for now.
- **G adopted (2026-10-26, report chapter 30.10):** the shared profile, plus the MapsHD
  profile's two keys (cap 0.25 m, localizability boost 10, normal only) on scans whose
  keypoints' median range is below 12 m.
  - A close scan that the tight cap explains poorly (under 60% of keypoints within 0.25 m)
    is re-registered coarse-to-fine.
  - Over the ten development sequences, G is within 1.5% of the per-dataset best profiles
    (geometric mean 0.0353 against 0.0348; the shared profile alone 0.0635).
  - Configuration: `configs/sota_v7/rko_lio_global_G_2026-10-26.yaml`.
  - Both thresholds come from development data, so a holdout must confirm them.
- Still open:
  - Koide-style long outdoor walks (no gravity window).
  - G now includes `min_icp_keypoints: 100` (chapter 30.11), adopted by the maintainer's decision
    despite a +1.0% rule miss on Koide 02a. It fixes Koide 01a (3.23 to 0.98 m) and leaves the
    ten development sequences byte-identical.
  - More development families (ENWIDE, GEODE, NTU VIRAL) to test G before a holdout is
    spent.
- **G12 adopted (2026-10-26, report chapter 31.3):** the GrandTour validation mission ARC-3
  (multi-floor building) broke G at 4.6 m. Its stairwell scans have 75–99 keypoints, under
  `min_icp_keypoints: 100`, and were not registered. G12 adds
  `keypoint_shortfall_voxel_multiplier: 1.0`: such a scan takes its keypoints again at the
  0.25 m voxel before it is dropped.
  - ARC-3 drops to 0.28 m, or 0.072 m without one 16 m jump in the reference.
  - The checked development sequences (flat-room 1, Oxford, Koide ×4), HEAP-1, and the
    COMFORT test trajectories are byte-identical to G.
  - Configuration: `configs/sota_v7/rko_lio_global_G12_2026-10-26.yaml`.

### Phase 2 — Frontend accuracy, driven by the remaining development error

- The garage scale stretch (+0.09–0.11%, specific to RKO-LIO; GLIM does not show it).
  The remaining hypotheses are voxels filled by their first points and point-order bias
  within a scan. Report chapter 29 could not separate them without per-scan comparison
  to the TLS map. On this VM, do that comparison in chunks streamed from the archive.
- Long vehicle sequences are the weakest part of the record (SOTA-v5 NavINST and
  UrbanNav). Add one long development sequence of this kind before Phase 5.

### Phase 3 — Backend: loops that never hurt, and that help on long trajectories

Order set by the maintainer (2026-10-27): the MID-360 scale stretch (Phase 2) first, then
robust loops (this phase), then a multi-session mode that joins several runs of one site
by place recognition (Scan Context, loop verification and candidate aggregation already
exist in graph_based_slam). A wrong cross-session join breaks the whole map, so the
robust loop machinery comes first. GrandTour has several runs per site (ARCHE 7,
Jungfraujoch 9, Construction Halter 4); whether their total-station references share
one frame is still to be checked.

- On the short development sequences, even ground-truth loops improve APE by at most 4%
  (report chapters 19, 24, 27). Loops matter on long trajectories.
- Register loop edges with GICP (the existing small_gicp plugin) instead of NDT. The NDT
  edges are 5–24 cm off where the frontend is 1–3 cm.
- Derive the edge information from the registration Hessian, not from fitness (chapter 19:
  fitness does not predict accuracy).
- Replace the hand-set loop rejection thresholds with a robust solve (GNC or switchable
  constraints).

### Phase 4 — Resources

The Oxford pipeline still peaks at 1.4–1.8 GiB, against the SOTA-v6 gate of 330 MB. The
main item is streaming the saved map instead of holding the full cloud (chapter 16:
685 MiB for the Oxford map alone).

### Phase 5 — Blind evaluation and publication

- Preregister G, the rival revisions, the thresholds, the machine, and the run order. Run
  every holdout, hash-seal the outputs, then open the ground truth.
- Publish only through the generated claim block in the README, one row per tier, with
  failed tiers shown.
- C4: KITTI Odometry (LiDAR-only track) is the canonical hidden-ground-truth LiDAR
  benchmark. The HILTI 2022 ground truth is now public on the official Hugging Face
  dataset, so it no longer works as a hidden test. Whether the HILTI 2023 leaderboard
  still accepts submissions has not been confirmed.
- C4, first entry: the GrandTour COMFORT Localization Benchmark on Codabench (LiDAR + IMU
  category, six test missions with hidden total-station references, SE(3)-aligned ATE,
  real-time methods only; test phase until 2026-10-30 23:59 UTC). G is submitted once as
  preregistered (report chapter 31). The maintainer uploads it, because submissions need
  the maintainer's Codabench account.

## Appendix: profile inventory

Regenerate the inventory with:

```bash
python3 scripts/rko_profile_inventory.py          # Markdown matrix
python3 scripts/rko_profile_inventory.py --json   # includes rig keys
```

The defaults are read from the vendored RKO-LIO sources (`LIO::Config`, the photometric
sub-configurations, and the ROS node). Rig keys (topics, frames, extrinsics, units, clock
offsets, beam model, output pacing) are listed separately in the JSON output. The
2026-10-26 matrix is in [RKO-LIO profile inventory](../research/rko-profile-inventory-2026-10.md).
