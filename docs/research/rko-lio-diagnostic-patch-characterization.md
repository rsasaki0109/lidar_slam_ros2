# RKO-LIO diagnostic patch — characterization record (v0.8 Phase 1)

Status: **frozen record, 2026-07-06.** Companion to
[`docs/roadmap/v0.8.md`](../roadmap/v0.8.md) §5 Phase 1 and
[`hilti-degeneracy-baseline.md`](hilti-degeneracy-baseline.md).

## What the patch is

`Thirdparty/rko_lio` fork commit `d6c767d`
(rsasaki0109/rko_lio PR #1, `feat/diagnostic-icp-hessian-exposure`,
+207/−9 across 3 files):

- `core/lio.cpp` — `icp()` returns `IcpResult{pose, H, b}`: the final
  iteration's 6×6 Gauss-Newton normal-equations matrix and residual
  vector, previously discarded. The solve path is untouched.
- `core/util.hpp` — `LocalizabilitySummary` (ascending eigenvalues +
  sign-canonicalized eigenvectors) and `IcpDiagnostics`;
  `State::icp_diagnostics` is `std::optional`, cleared on every
  no-solve path (first frame, dropped scan, kidnap recovery, local
  reset).
- `ros/node.cpp` — `publish_odometry()` fills `pose.covariance`
  anisotropically from `Cov = V·diag(1/λ)·Vᵀ` (H is the
  correspondence-averaged Gauss-Newton information matrix at the
  optimum, per Zhang & Singh ICRA 2016). Eigenvalues are floored at
  `max(1e-9, 1e-6·λ_max)` so exactly-degenerate directions publish
  large-but-finite uncertainty; scans without diagnostics fall back to
  a 1e6 diagonal. `pose.pose` / `twist.twist` assignments unchanged.

## Byte-identity gate result: green

| substrate | comparison | result |
|---|---|---|
| HILTI exp04 | full registered pose TUM (`dump_results`), 1258 poses | `cmp` exit 0 — byte-identical before/after patch |
| HILTI exp07 | full registered pose TUM, 1322 poses | `cmp` exit 0 — byte-identical before/after patch |

Covariance verified anisotropic on the wire (`/rko_lio/odometry` echo
during exp04 replay: e.g. tz variance 1.885 vs tx 1.103, rotation
block ~1e-3 rad², symmetric with off-diagonal terms).

Config: `configs/hilti2022/rko_lio_hilti2022_pandar.yaml`, bags under
`/media/sasaki/aiueo/datasets/hilti2022/`, artifacts under
`/media/sasaki/aiueo/lidarslam_work/output/v0.8_phase1_characterization/`.

## Methodology note: byte-identity must be judged single-core

Under the default TBB threading, **two runs of the same binary** differ
in `traj_raw.tum` from the first scans (last-digit float differences,
~1e-19 order): `build_icp_linear_system`'s `tbb::parallel_reduce`
reduction order is nondeterministic. This is the known upstream
property that made v0.6 scope RKO-LIO raw odometry **out** of the
byte-identical-map release gate (the gate holds for the backend and
scanmatcher offline runners, which are deterministic).

Therefore the before/after characterization is judged with
`taskset -c 0` (single-core), which makes the reduction order — and
the full pose stream — exactly reproducible. This adds **no new
determinism obligation**: the v0.6 gate scope is unchanged; single-core
execution is a measurement instrument for patch-neutrality, not a
supported production mode.

Secondary observation: live-subscriber TUM logs (`odom_to_tum.py`,
queue 10) can drop messages under load, so line counts differ run to
run even when the underlying pose stream is identical. Byte comparison
must therefore use the LIO's own `dump_results` TUM (complete pose
list), not the subscriber capture.

## Known limitations

- The fork's unit-test surface is Python/pybind only;
  `State::icp_diagnostics` is deliberately not exposed to pybind (would
  grow the upstream diff for no Phase 1 benefit). Eigen-analysis
  correctness is unit-tested in the main repo against the Phase 0
  synthetic fixtures (`localizability_analysis.hpp`, which consumes the
  same H).
- Characterization covers exp04/exp07 (the two locally-available HILTI
  substrates). Extension to NTU / Newer College / MID-360 substrates
  rides along with the Phase 1 release-readiness wiring.
