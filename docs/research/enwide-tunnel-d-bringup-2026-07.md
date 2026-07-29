# ENWIDE TunnelD public benchmark bring-up (2026-07-29)

## Decision

The first public ENWIDE TunnelD run is a successful pipeline bring-up and a
failed accuracy baseline. It is not valid evidence for a SOTA claim.

Keep the preregistered geometry/intensity v1 configuration as the reproducible
baseline. Reject the exploratory v2 intensity-disagreement candidate: it
improves global ATE but worsens local RTE, runtime, memory, and estimated path
length.

The first alias-aware reflectivity milestone is implemented in RKO-LIO commits
`b4a8937ab13bbb3dfbffe76365752c77bcdca678` and
`415375106ef0bc706e307ae507d47d9970b6d0ba`. Raw peak observations are exported
by `f285c9e97e6e4425a1b8cf2d5891624448922d8e`. The one-dimensional NCC result
now reports a best-versus-second-peak margin, can reject ambiguous peaks, and
records aggregate margin diagnostics. Its default margin is zero, preserving
the historic result until a threshold is selected from diagnostics rather
than TunnelD accuracy.

Peak selection is a sensor- and dimensionality-independent core API:
correlation implementations emit scored offset candidates, while a reusable
policy selects the best supported candidate, excludes the configurable local
peak radius, and reports an explicit rejection reason. The current 1D profile
is an adapter over that API. A later local oriented reflectivity/height
matcher can reuse the selector without adding another gate directly to
`LIO::register_scan`.

Every attempted correlation now emits timestamp, source, best and second-best
score, margin, overlap support, base qualification, ambiguity, and acceptance
to `intensity_peak_diagnostics.csv`. Base qualification records whether score
and support passed before the alias-margin policy is applied, so low-quality
candidates cannot contaminate the alias distribution.

The selection-independent summarizer consumes one or more of these CSVs and
writes input paths and SHA-256 hashes, qualified row/source counts, fixed
quantiles, and fixed threshold counts:

```bash
python3 scripts/summarize_intensity_peak_diagnostics.py \
  run-a/intensity_peak_diagnostics.csv \
  run-b/intensity_peak_diagnostics.csv \
  --output intensity_peak_summary.json
```

It has no accuracy-metric input. Benchmark analysis can therefore aggregate
repetitions or holdouts and test a future selection policy from raw evidence
without replaying the multi-gigabyte bags or selecting against TunnelD ATE.

The 20-second Tunnel smoke test produced 83 raw observations and 25
base-qualified observations. Its qualified median margin was 0.1465; 20% were
below 0.05. This validates the dump and summarizer only. The sample is too
short and too sequence-specific to select a production threshold.

The threshold-selection diagnostic set then ran the full NTNU tunnel once and
the distinct fog sequence three times, without computing accuracy:

| sequence | runs | qualified | p01 | p50 | below 0.005 |
| --- | ---: | ---: | ---: | ---: | ---: |
| tunnel | 1 | 1,322 | 0.00628 | 0.1756 | 0.91% |
| fog | 3 | 732/run | 0.05459 | 0.4318 | 0.00% |

All three fog diagnostic CSVs were byte-identical, with SHA-256
`4958c9dbce9a1d3b9726ff4765ddd9f38c80f175adb73eb431b6fe5741479e64`.
The tunnel CSV SHA-256 was
`b8ab4bb3e56ed0d58e03da478422b8b69b57216cdbe51a696570f52cf143aa40`.

The first nonzero candidate margin is frozen at `0.005`: it is below the first
percentile on each sequence and retains at least 99% of base-qualified
observations per sequence. This is intentionally a conservative extreme-alias
filter, not an accuracy-tuned operating point. The frozen candidate is
`configs/enwide/rko_lio_os0_intensity_alias_v3.yaml`; only its peak margin
differs from exploratory v2. Accuracy evaluation happens after this freeze.

## Frozen public input

- sequence: ENWIDE `tunnel_d`
- source bag bytes: `7485669675`
- source bag SHA-256:
  `afa448cd2ee32921cd514bb7d4c2e139f642bb164f66b1d556cf48c0c798406e`
- ground-truth SHA-256:
  `25c7a20513b3c41e7a5f517119ff41bcf07329b6d87f3aeb8f5ed7725f5c922e`
- converted rosbag2 tree SHA-256:
  `4faf394304f087af616debd86140fc8bcadeb06426f3d8e612302286ddec34ec`
- duration: `119.013327769 s`
- messages: 1,189 PointCloud2 and 11,861 IMU
- position ground-truth path length: `179.707055 m`

The source PointCloud2 layout matches the runner contract: relative `t` is
`uint32` nanoseconds, `reflectivity` is `uint16`, and the frames are
`os_sensor` and `os_imu`.

## Results

These are single-run exploratory results on the same machine and are not
valid three-repetition sequence comparisons.

| candidate | ATE RMSE (m) | 10 m RTE (%) | matched GT | RTF | peak RSS (MiB) |
| --- | ---: | ---: | ---: | ---: | ---: |
| preregistered v1 | 23.7841 | 64.5694 | 99.579% | 2.421 | 454.7 |
| intensity disagreement v2 | 22.0945 | 65.2899 | 99.579% | 2.780 | 642.8 |
| v2 + alias diagnostics, margin disabled | 22.5956 | 64.6661 | 99.579% | 2.765 | 513.4 |

The published COIN-LIO TunnelD reference is 0.487 m ATE and 1.59% RTE. These
numbers are only an external reference because the local scorer has not yet
reproduced COIN-LIO on the identical converted input.

## Failure analysis

The source timestamps and calibration path are internally consistent. The v1
degeneracy intervention did not fire:

- persistent-prior attempts: 0
- persistent-prior applications: 0
- confirmed persistent weak directions: 0

The estimated v1 path is 261.51 m and its per-step speed p95 is 5.40 m/s,
versus 179.71 m and 2.79 m/s for the position ground truth. This is not a
single constant scale error; it contains local correspondence spikes and ends
36.29 m from its own starting point even though the reference is a return
trajectory.

The exploratory v2 changes only four existing reflectivity-gate parameters.
It makes 998 correlation attempts, accepts 897 shifts, and corrects 721 scans.
Its endpoint separation improves to 8.66 m, but its total path grows to
321.70 m and speed p95 to 7.22 m/s. The better global ATE therefore hides
worse local motion. The 10 m RTE correctly rejects it.

The margin-disabled diagnostic run records 913 peak-margin samples with mean
0.0942 and minimum 0.0. Its correction acceptance logic is identical to v2,
but its trajectory SHA-256 differs and its score moves by about 0.5 m ATE.
RKO's parallel scan processing is therefore not byte-deterministic across
runs. This is further evidence that the required three repetitions are a
measurement requirement, not just a reporting convention. No peak-margin
threshold may be selected from either single-run TunnelD score.

The validated open-tunnel inertial preset is also inapplicable without a new
classifier. On TunnelD, the median fraction of points below 3 m is 0.780 and
the median fraction above 10 m is 0.0075, so every scan fails that preset's
open-scene gate. Its thresholds must not be relaxed specifically for this
sequence.

## Evaluation infrastructure correction

The first score attempt exposed a benchmark bug rather than a SLAM failure.
The 10 Hz estimate has normal timestamp jitter up to roughly 0.106 s, while
the scorer used a 0.100 s interpolation bracket. That fragmented the matched
ground truth into 276 short blocks and left no complete 10 m segment.

The frozen ENWIDE bracket is now 0.11 s. It matches 23,202 of 23,300
ground-truth poses in six blocks while still splitting the real larger gaps.
The runner also emits its summary receipt when position scoring fails, so a
metric failure cannot erase the process result.
