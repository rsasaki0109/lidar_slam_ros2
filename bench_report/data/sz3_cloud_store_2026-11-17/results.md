# SZ3 stage 1 results (plan.txt, preregistered 2026-11-17)

Harness: `tools/harness.cpp` (graph_based_slam's own `compressPacked` for A and for the intensity
plane of Q and S; SZ3 db2eca3e header-only with the system zstd; g++ 13 -O2, one thread) fed by
`tools/feed.py` (rosbags, every 5th `/rko_lio/frame`). Per-dataset JSON: `summaries/`.

## C1

A restored every cloud bit-exactly on all six datasets. Max |error| per axis: S1 1.0000 mm,
S5 5.0000 mm, Q1 0.9995 mm, Q5 4.997 mm, all within their bounds. No non-finite coordinates.

Two harness faults were found by C1 and fixed before the decision was read: (1) Q's decoder did
not sign-extend the 32-bit zigzag value (garbage errors; sizes unaffected); (2) with a step of
exactly 2b, float32 rounding of the restored value put Q's error 0.4 um over the bound
(1.0004 mm), so the step is 2b x 0.999. The sizes of a first pass on all six datasets with the
exact 2b step were within 0.1 % of the final ones and gave the same decision.

## Sizes and speed

Ratios are of stored bytes; S/Q compares the x, y, z part (intensity is stored the same way in
both). Speed is raw x, y, z megabytes per second of compress + decompress.

| Dataset | clouds | points | A MB | Q1/A | S1/Q1 | S1 MB/s | Q5/A | S5/Q5 | S5 MB/s | Q1 MB/s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| D1 MapsHD calibration-room 2 | 461 | 2.2 M | 20.8 | 0.385 | 1.058 | 25 | 0.289 | 1.038 | 34 | 153 |
| D2 MapsHD underground-garage 1 | 722 | 4.5 M | 43.0 | 0.398 | 1.039 | 28 | 0.299 | 1.025 | 39 | 163 |
| D3 HILTI exp04 | 252 | 2.9 M | 27.2 | 0.441 | 1.076 | 25 | 0.347 | 1.043 | 38 | 153 |
| D4 Stadtgarten seq 2 | 1727 | 14.4 M | 136.0 | 0.425 | 1.041 | 25 | 0.324 | 1.021 | 36 | 148 |
| D5 Construction seq 1 | 1457 | 15.7 M | 147.2 | 0.429 | 1.021 | 29 | 0.325 | 1.012 | 43 | 157 |
| D6 GrandTour CON-1 | 776 | 15.7 M | 143.0 | 0.427 | 1.038 | 37 | 0.327 | 1.023 | 59 | 171 |

A stores 16 bytes per point at a 1.69-1.76x ratio. The intensity of these backend inputs is
constant (RKO-LIO's frame carries none), so its plane costs almost nothing.

## Decision

- SZ3: S_b's x, y, z bytes are 1.2-7.6 % larger than Q_b's on every dataset (rule: <= 0.8x),
  and it runs at 25-59 MB/s (rule: >= 50 MB/s on all). **SZ3 is not the candidate** at either
  bound.
- Quantization: Q1 stores 0.385-0.441x of A and Q5 0.289-0.347x on every dataset (rule:
  <= 0.6x). **Q is the stage-2 candidate** at both bounds, with no new dependency (zstd is
  already used) and faster than SZ3 (Q1 at 148-171 MB/s here).

Why SZ3 does not win here: its predictors pay off on smooth gridded fields. A scan in point order
is a sequence of nearly independent ranges, so after quantization there is little left to
predict, and plain deltas + zstd do as well or better.

## Report only

Bytes a run's scans would take less if every scan were stored (A minus Q, sampled clouds x 5).
The backend keeps only each submap's scan, so the memory it would save is this times the share
of scans that become submaps. The runs that recorded these inputs did not log a backend peak
RSS, so no ratio to peak memory is given.

| Dataset | A - Q1 | A - Q5 |
| --- | ---: | ---: |
| D1 MapsHD calibration-room 2 | 64 MB | 74 MB |
| D2 MapsHD underground-garage 1 | 129 MB | 151 MB |
| D3 HILTI exp04 | 76 MB | 89 MB |
| D4 Stadtgarten seq 2 | 391 MB | 460 MB |
| D5 Construction seq 1 | 420 MB | 497 MB |
| D6 GrandTour CON-1 | 409 MB | 481 MB |

Stage 2 (below) was approved by the maintainer and pre-registered in plan_stage2.txt.


## Stage 2: opt-in quantized store in the offline backend (plan_stage2.txt + addendum)

Code: `submap_cloud_max_error_m` (offline_pipeline.cpp; default 0.0), `cloud_compression`'s
`store(message, max_error)` / `compressQuantized`. Runs: `python -m lidarslam backend` with the
package built from these sources (`tools/run_stage2.sh`, `tools/compare_stage2.py`,
`summaries/stage2.json`).

### T1 default unchanged

With the parameter absent and with 0.0, all 6 output files on D1 and D3 are byte-identical to the
build before the change. The cloud_compression unit tests pass (4/4, including the quantized
round trip within the bound, the lossless fallback for non-finite or very large coordinates, and
0.0 giving the lossless bytes).

### T2 against the lossless run of the same build

Loops lossless / quantized; trajectory_optimized position difference RMS / max and rotation max;
stored submap-cloud bytes as a fraction of lossless. Report only: trajectory_refined max
difference, peak RSS (MB) and wall time (s), lossless / quantized.

| Dataset | bound (m) | loops | opt pos RMS / max | opt rot max (deg) | bytes | T2 | refined max | RSS | wall |
| --- | --- | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: |
| D1 MapsHD calib-room 2 | 0.001 | 0 / 0 | 0.0 / 0.0 mm | 0.000 | 0.388 | pass | 2.3 mm | 72 / 72 | 2 / 2 |
| D1 MapsHD calib-room 2 | 0.005 | 0 / 0 | 0.0 / 0.0 mm | 0.000 | 0.290 | pass | 3.3 mm | 72 / 71 | 2 / 2 |
| D2 MapsHD garage 1 | 0.001 | 0 / 0 | 0.0 / 0.0 mm | 0.000 | 0.399 | pass | 3.2 mm | 99 / 98 | 12 / 11 |
| D2 MapsHD garage 1 | 0.005 | 0 / 0 | 0.0 / 0.0 mm | 0.000 | 0.300 | pass | 8.5 mm | 99 / 96 | 12 / 11 |
| D3 HILTI exp04 | 0.001 | 0 / 0 | 0.0 / 0.0 mm | 0.000 | 0.439 | pass | 75.2 mm | 105 / 104 | 2 / 2 |
| D3 HILTI exp04 | 0.005 | 0 / 0 | 0.0 / 0.0 mm | 0.000 | 0.345 | pass | 92.5 mm | 105 / 103 | 2 / 2 |
| D4 Stadtgarten 2 | 0.001 | 0 / 0 | 0.0 / 0.0 mm | 0.000 | 0.429 | pass | 17.0 mm | 181 / 180 | 22 / 26 |
| D4 Stadtgarten 2 | 0.005 | 0 / 0 | 0.0 / 0.0 mm | 0.000 | 0.327 | pass | 20.1 mm | 181 / 176 | 22 / 23 |
| D5 Construction 1 | 0.001 | 9 / 10 | 0.9 / 1.6 mm | 0.003 | 0.427 | pass | 7.5 mm | 190 / 183 | 111 / 111 |
| D5 Construction 1 | 0.005 | 9 / 9 | 0.7 / 2.3 mm | 0.006 | 0.324 | pass | 18.3 mm | 190 / 182 | 111 / 109 |
| D6 GrandTour CON-1 | 0.001 | 1 / 1 | 1.0 / 1.6 mm | 0.002 | 0.426 | pass | 1303.3 mm | 245 / 236 | 130 / 127 |
| D6 GrandTour CON-1 | 0.005 | 1 / 1 | 0.4 / 0.7 mm | 0.001 | 0.325 | pass | 1018.0 mm | 245 / 229 | 130 / 125 |
| D7 MapsHD calib-room 1 (walking) | 0.001 | 18 / 18 | 0.4 / 1.0 mm | 0.019 | 0.376 | pass | 4.7 mm | 105 / 104 | 23 / 20 |
| D7 MapsHD calib-room 1 (walking) | 0.005 | 18 / 18 | 0.5 / 1.4 mm | 0.015 | 0.280 | pass | 8.6 mm | 105 / 104 | 23 / 18 |
| D8 calib-room 2 (walking) | 0.001 | 1 / 1 | 0.2 / 0.4 mm | 0.002 | 0.388 | pass | 2.4 mm | 81 / 79 | 4 / 3 |
| D8 calib-room 2 (walking) | 0.005 | 1 / 1 | 0.2 / 0.4 mm | 0.003 | 0.290 | pass | 3.0 mm | 81 / 79 | 4 / 4 |

**Decision (pre-registered rule): both bounds pass T2 on all eight datasets; the parameter ships,
default 0.0, with 0.005 named as the tested setting.** D1-D4 accept no loop either way, so only
D5-D8 test loop closure (9, 1, 18 and 1 loop edges; 18 of 18 kept on D7).

### What the numbers do not show: the refinement's sensitivity (report only)

The pose-graph result (trajectory_optimized) moves by at most 2.3 mm. The plane refinement that
runs after it reads the restored clouds of every submap and can amplify a small change:
trajectory_refined moves by 2-20 mm on six datasets, by up to 92 mm on D3 and by up to 1.3 m on
D6. A diagnostic run with a 1 um bound (not pre-registered) separates the two cases:

| Dataset | refined max, 1 um | 1 mm | 5 mm | lossless refined vs optimized, max |
| --- | ---: | ---: | ---: | ---: |
| D3 HILTI exp04 | 0 | 75 mm | 92 mm | 168 mm |
| D6 GrandTour CON-1 | 0.77 m | 1.30 m | 1.02 m | 0.87 m |

On D6 the refinement itself is unstable: a 1 um change of the stored scans already moves its
output by 0.77 m, so any input change (another build, another parameter) would; this is a
finding about the refinement on this sequence, not about quantization. On D3 the refinement is
stable at 1 um and amplifies the quantization (1 mm -> up to 75 mm, within the 168 mm the
refinement moves the poses anyway). Users who keep the refined map should take this into account;
the default stays lossless.

### Memory

The store shrinks to 0.28-0.44x of the lossless bytes (D5: 30.8 -> 13.2 MiB at 1 mm), but on these
runs (2-130 s, at most 282 submaps) the stored scans are a small part of the backend's memory:
peak RSS drops by 0-16 MB (D6: 245 -> 229 MB at 5 mm). The saving grows with the number of
submaps; a run whose peak is dominated by stored scans gains most.
