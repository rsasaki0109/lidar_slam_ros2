# GNSS constraint — first real-data validation (2026-06-12)

The GNSS pose-graph constraint and the `LocalCartesian` projector output had
been documented as untested ⚠️ since they landed. #242 fixed the constraint's
information-matrix block order (it had never pulled translation), which made a
real validation meaningful for the first time. Substrate: RTK-SLAM
`stadtgarten_seq2` (outdoor park, `/gnss/fix` NavSatFix at ~136 Hz, RTK-grade,
total-station GT available), the v0.5 outdoor preset, A/B with
`use_gnss: false/true`. Raw artifacts: `output/gnss_ab/`.

## Validated ✅

1. **The fixed constraint works mechanically on real data**: with the stamp
   skew override (below), the final pose adjustment reports
   `Added 163 GNSS position constraint edges (163 RTK-like by covariance)` —
   origin initialization (3-sample + 20 m consistency), geodetic→ENU
   conversion, covariance weighting and RTK classification all behave.
2. **`LocalCartesian` projector output, first ever**:
   `map_projector_info.yaml` came out as `LocalCartesian` + `WGS84` with
   `map_origin` lat 48.7819948 / lon 9.1729486 — the actual Stuttgart
   Stadtgarten. The `Local`-fallback path (GNSS off) is unchanged.
3. **No-regression sanity**: the GNSS-off arm reproduces the v0.5 result
   exactly (raw 0.835 m RMSE, 19/19 checkpoints, median 0.327) — also strong
   end-to-end evidence that the Phase 1 changes (#238–#242) did not move the
   benchmark path. GNSS-on raw is identical (GNSS only affects the graph), as
   it must be.

## The remaining gap ❌ — no odom→ENU yaw alignment

The GNSS-on corrected trajectory diverges from GNSS-off by a pattern that is
zero at the fixed vertex and grows linearly with distance from it:

| pose index | 0 | 10 | 50 | 270 |
|---|---|---|---|---|
| on/off displacement [m] | 0.00 | 24.1 | 94.1 | 337.1 |

That is a pure **frame-rotation mismatch**: the odometry frame's x axis is
the robot's initial heading, the ENU anchors' x axis is east, and nothing in
the pipeline estimates the yaw between them. With vertex 0 fixed (position
*and* orientation), the optimizer cannot rotate the graph into the ENU frame,
so 163 strong anchors shear the map instead of georeferencing it. The effect
was unobservable before #242 because the anchors never pulled translation at
all.

**Conclusion: `use_gnss: true` remains unusable for accuracy on real data
until a yaw alignment lands.** Design sketch for the follow-up:

- estimate the odom→ENU yaw online once the GNSS track baseline exceeds a
  few meters (2-D least squares between odometry positions at fix stamps and
  the ENU track), then
- either rotate the ENU anchors into the odometry frame (map stays in the
  odometry frame; record the yaw in the projector metadata), or release the
  vertex-0 gauge under GNSS so the graph itself rotates into ENU (map becomes
  truly georeferenced; matches what `LocalCartesian` consumers expect).
  The second is the production-correct target; it needs a minimum-anchor
  guard before un-fixing the gauge.

## Replay pitfalls worth keeping (cost a run each)

- The RKO-LIO offline node reads the bag internally — `/gnss/fix` never
  enters the ROS graph. Replay it alongside
  (`ros2 bag play <bag> --topics /gnss/fix --rate 5`); matching is by message
  stamp so the rate is free.
- `gnss_header_stamp_max_skew_sec` (default 30 s) re-stamps fixes with the
  receive time when the header is "too old" — on a recorded bag that is
  *every* fix, silently producing 0 edges. Override it to a huge value for
  replays (the experiment used 1e12).
- A benchmark `--output-dir` whose basename is `on`/`off` becomes a boolean
  `run_name` parameter via YAML and aborts RKO-LIO; name arms `gnss_on`-style.
- The corrected (submap-node) trajectory still cannot be scored against the
  total-station checkpoints (dwell-gap pairing, the known v0.5 limitation), so
  the corrected-level A/B metric stays blocked until denser export exists.

## Reproduce

```bash
# off arm = v0.5 outdoor preset; on arm adds use_gnss + the skew override
bash output/gnss_ab/run_ab.sh   # see the file for the exact invocations
```
