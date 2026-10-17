| # | system | round | exit | frontend exit s | results saved s | RTF | peak RSS MB | CPU s | took s | save s | result md5 |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | ours_off | 2 | 0 | 28.038 | 28.506 | 0.2266 | 455.7 | 92.2 | 27.5143 | 0.467 | 31467b8b |
| 2 | ours_loop | 2 | 0 | 31.424 | 33.438 | 0.2658 | 451.1 | 109.9 | 30.8494 | 2.013 | ce7c82a5 |
| 3 | glim | 2 | 0 | – | 37.230 | 0.2959 | 712.8 | 107.9 | – | – | 9110babe |
| 4 | ours_loop | 3 | 0 | 30.776 | 32.401 | 0.2575 | 453.5 | 108.0 | 30.2268 | 1.624 | ce7c82a5 |
| 5 | glim | 3 | 0 | – | 37.010 | 0.2942 | 705.1 | 106.5 | – | – | b12efd06 |
| 6 | ours_off | 3 | 0 | 28.249 | 28.783 | 0.2288 | 443.4 | 92.5 | 27.7193 | 0.533 | 31467b8b |
| 7 | glim | 4 | 0 | – | 35.980 | 0.2860 | 716.2 | 105.0 | – | – | 89bd483a |
| 8 | ours_off | 4 | 0 | 27.829 | 28.285 | 0.2248 | 448.5 | 91.3 | 27.3028 | 0.456 | 31467b8b |
| 9 | ours_loop | 4 | 0 | 30.777 | 32.709 | 0.2600 | 448.1 | 107.0 | 30.2244 | 1.931 | ce7c82a5 |
| 10 | ours_off | 5 | 0 | 28.010 | 28.460 | 0.2262 | 457.4 | 92.2 | 27.4985 | 0.449 | 31467b8b |
| 11 | ours_loop | 5 | 0 | 30.853 | 32.971 | 0.2621 | 454.5 | 108.2 | 30.3066 | 2.118 | ce7c82a5 |
| 12 | glim | 5 | 0 | – | 36.570 | 0.2907 | 687.5 | 106.6 | – | – | ae5432ce |
| 13 | ours_loop | 6 | 0 | 30.127 | 32.179 | 0.2558 | 451.3 | 106.3 | 29.6249 | 2.052 | ce7c82a5 |
| 14 | glim | 6 | 0 | – | 38.530 | 0.3062 | 712.5 | 110.6 | – | – | ffa58884 |
| 15 | ours_off | 6 | 0 | 28.055 | 28.521 | 0.2267 | 448.8 | 92.3 | 27.5043 | 0.465 | 31467b8b |

| system | n | results saved s (median) | RTF | vs GLIM | peak RSS MB (max) | CPU s (median) | APE 6 pts m (median) | APE 7 pts m | map thickness mean / p95 m | planar coverage | map points |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| glim | 5 | 37.01 | 0.2942 | 1.000 | 716.2 | 106.6 | 0.0874 (0.0822–0.0892) | 0.0874 (pairs 6) | 0.0789 / 0.1222 | 0.434 | 1307317 |
| ours_off | 5 | 28.51 | 0.2266 | 0.770 | 457.4 | 92.2 | 0.0482 | 0.0520 (pairs 7) | 0.0391 / 0.0952 | 0.671 | 309170 |
| ours_loop | 5 | 32.71 | 0.2600 | 0.884 | 454.5 | 108.0 | 0.0495 | 0.0488 (pairs 7) | 0.0408 / 0.0969 | 0.669 | 313788 |

- glim: exits ['0'], distinct result trajectories 5, frontend exit median nan s, offline node took median nan s, map_save median nan s
- ours_off: exits ['0'], distinct result trajectories 1, frontend exit median 28.04 s, offline node took median 27.50 s, map_save median 0.465 s
- ours_loop: exits ['0'], distinct result trajectories 1, frontend exit median 30.78 s, offline node took median 30.23 s, map_save median 2.013 s
- GLIM odometry only (odom_imu) APE 6 pts median 0.0876
- ours_off/glim: time 0.770, RSS 0.639, CPU 0.865, APE6 0.552 (44.8% lower), thickness mean 0.495, p95 0.779, coverage 1.547
- ours_loop/glim: time 0.884, RSS 0.635, CPU 1.013, APE6 0.566 (43.4% lower), thickness mean 0.517, p95 0.793, coverage 1.542
