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

Stage 2 (not started; needs the maintainer's approval): an opt-in, default-off quantized store
(`submap_cloud_quantization_m`, e.g. 0.001) with its own preregistered check that maps and
trajectories stay within tolerance of the lossless store, and the memory it saves measured as
peak RSS. The default stays lossless, so every byte-identical check stays valid.
