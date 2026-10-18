| # | system | round | exit | frontend exit s | results saved s | RTF | peak RSS MB | CPU s | took s | save s | result md5 |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | glim | 1 | 0 | – | 37.190 | 0.2956 | 694.0 | 107.9 | – | – | 9e362e51 |
| 2 | ours_off | 1 | 0 | 39.960 | 40.413 | 0.3212 | 461.2 | 139.2 | 39.4438 | 0.452 | 31467b8b |
| 3 | ours_loop | 1 | 0 | 44.525 | 44.939 | 0.3572 | 457.6 | 158.8 | 43.9996 | 0.413 | ce7c82a5 |
| 4 | ours_off | 2 | 0 | 41.584 | 42.070 | 0.3344 | 454.8 | 144.9 | 41.0467 | 0.485 | 31467b8b |
| 5 | ours_loop | 2 | 0 | 45.006 | 45.444 | 0.3612 | 449.7 | 160.3 | 44.4436 | 0.437 | ce7c82a5 |
| 6 | glim | 2 | 0 | – | 38.030 | 0.3023 | 692.3 | 109.9 | – | – | 87ed0480 |
| 7 | ours_loop | 3 | 0 | 44.609 | 45.049 | 0.3581 | 456.2 | 158.4 | 44.0636 | 0.439 | ce7c82a5 |
| 8 | glim | 3 | 0 | – | 38.100 | 0.3028 | 673.3 | 108.7 | – | – | a6665eeb |
| 9 | ours_off | 3 | 0 | 39.554 | 40.003 | 0.3180 | 459.5 | 137.2 | 39.045 | 0.449 | 31467b8b |
| 10 | glim | 4 | 0 | – | 37.500 | 0.2981 | 713.4 | 108.0 | – | – | caee30b2 |
| 11 | ours_off | 4 | 0 | 40.559 | 41.029 | 0.3261 | 453.1 | 140.9 | 40.0541 | 0.469 | 31467b8b |
| 12 | ours_loop | 4 | 0 | 44.400 | 44.837 | 0.3564 | 450.2 | 158.4 | 43.8089 | 0.437 | ce7c82a5 |
| 13 | ours_off | 5 | 0 | 42.448 | 42.895 | 0.3409 | 457.6 | 144.4 | 41.8914 | 0.447 | 31467b8b |
| 14 | ours_loop | 5 | 0 | 45.155 | 45.605 | 0.3625 | 452.7 | 160.6 | 44.5881 | 0.449 | ce7c82a5 |
| 15 | glim | 5 | 0 | – | 37.840 | 0.3008 | 681.7 | 110.2 | – | – | b0d57f1d |

| system | n | results saved s (median) | RTF | vs GLIM | peak RSS MB (max) | CPU s (median) | APE 6 pts m (median) | APE 7 pts m | map thickness mean / p95 m | planar coverage | map points |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| glim | 5 | 37.84 | 0.3008 | 1.000 | 713.4 | 108.7 | 0.0890 (0.0863–0.0898) | 0.0890 (pairs 6) | 0.0794 / 0.1223 | 0.428 | 1306366 |
| ours_off | 5 | 41.03 | 0.3261 | 1.084 | 461.2 | 140.9 | 0.0482 | 0.0520 (pairs 7) | 0.0391 / 0.0952 | 0.671 | 309170 |
| ours_loop | 5 | 45.05 | 0.3581 | 1.191 | 457.6 | 158.8 | 0.0495 | 0.0488 (pairs 7) | 0.0408 / 0.0969 | 0.669 | 313788 |

- glim: exits ['0'], distinct result trajectories 5, frontend exit median nan s, offline node took median nan s, map_save median nan s
- ours_off: exits ['0'], distinct result trajectories 1, frontend exit median 40.56 s, offline node took median 40.05 s, map_save median 0.452 s
- ours_loop: exits ['0'], distinct result trajectories 1, frontend exit median 44.61 s, offline node took median 44.06 s, map_save median 0.437 s
- GLIM odometry only (odom_imu) APE 6 pts median 0.0907
- ours_off/glim: time 1.084, RSS 0.646, CPU 1.296, APE6 0.542 (45.8% lower), thickness mean 0.492, p95 0.779, coverage 1.568
- ours_loop/glim: time 1.191, RSS 0.641, CPU 1.461, APE6 0.555 (44.5% lower), thickness mean 0.514, p95 0.792, coverage 1.564
