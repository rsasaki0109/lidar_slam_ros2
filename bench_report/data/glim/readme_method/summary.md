| # | system | round | exit | time s | time incl. save s | RTF | RTF incl. save | peak RSS MB | CPU s | FS inputs | cached GB before | poses | frontend md5 |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | glim | 1 | 0 | 38.90 | 38.90 | 0.3092 | 0.3092 | 722.9 | 111.6 | 3712 | 12.84 | 1228 | NA |
| 2 | ours_readme | 1 | 0 | 45.65 | 45.65 | 0.3628 | 0.3628 | 446.8 | 137.8 | 34672 | 12.81 | 1258 | bc231992 |
| 3 | ours_loop | 1 | 0 | 44.0 | 45.34 | 0.3497 | 0.3604 | 461.8 | 159.0 | 7856 | 12.84 | 40 | bc231992 |
| 4 | ours_readme | 2 | 0 | 43.06 | 43.06 | 0.3423 | 0.3423 | 460.4 | 139.3 | 0 | 12.75 | 1258 | bc231992 |
| 5 | ours_loop | 2 | 0 | 43.0 | 44.33 | 0.3418 | 0.3523 | 455.7 | 157.7 | 0 | 12.76 | 40 | bc231992 |
| 6 | glim | 2 | 0 | 36.73 | 36.73 | 0.2919 | 0.2919 | 688.9 | 106.4 | 0 | 12.76 | 1228 | NA |
| 7 | ours_loop | 3 | 0 | 44.0 | 45.32 | 0.3497 | 0.3602 | 474.5 | 161.3 | 8 | 12.83 | 40 | bc231992 |
| 8 | glim | 3 | 0 | 39.31 | 39.31 | 0.3124 | 0.3124 | 701.4 | 112.1 | 0 | 12.75 | 1228 | NA |
| 9 | ours_readme | 3 | 0 | 43.05 | 43.05 | 0.3422 | 0.3422 | 452.4 | 139.4 | 0 | 12.82 | 1258 | bc231992 |
| 10 | glim | 4 | 0 | 36.53 | 36.53 | 0.2903 | 0.2903 | 679.2 | 105.7 | 0 | 12.82 | 1228 | NA |
| 11 | ours_readme | 4 | 0 | 42.88 | 42.88 | 0.3408 | 0.3408 | 462.8 | 141.1 | 0 | 12.84 | 1258 | bc231992 |
| 12 | ours_loop | 4 | 0 | 45.0 | 46.44 | 0.3577 | 0.3691 | 459.9 | 163.1 | 0 | 12.82 | 40 | bc231992 |
| 13 | ours_readme | 5 | 0 | 44.88 | 44.88 | 0.3567 | 0.3567 | 454.2 | 142.8 | 0 | 12.77 | 1258 | bc231992 |
| 14 | ours_loop | 5 | 0 | 45.0 | 46.39 | 0.3577 | 0.3687 | 459.4 | 163.3 | 0 | 12.77 | 40 | bc231992 |
| 15 | glim | 5 | 0 | 37.56 | 37.56 | 0.2985 | 0.2985 | 667.6 | 108.2 | 0 | 12.77 | 1228 | NA |

- glim: n=5 exits=['0'] time median 37.56 s (RTF 0.2985) runs ['38.90', '36.73', '39.31', '36.53', '37.56']; incl. save 37.56 s (RTF 0.2985) runs ['38.90', '36.73', '39.31', '36.53', '37.56']; peak RSS max 722.9 MB; CPU median 108.2 s; poses ['1228']; frontend md5 ['NA']
    - APE odom_imu: common6 median 0.086952 (range 0.085020..0.093158), all7 median 0.086952 (pairs ['6']), distinct trajectories 5
    - APE traj_imu: common6 median 0.085601 (range 0.083734..0.090349), all7 median 0.085601 (pairs ['6']), distinct trajectories 5
- ours_readme: n=5 exits=['0'] time median 43.06 s (RTF 0.3423) runs ['45.65', '43.06', '43.05', '42.88', '44.88']; incl. save 43.06 s (RTF 0.3423) runs ['45.65', '43.06', '43.05', '42.88', '44.88']; peak RSS max 462.8 MB; CPU median 139.4 s; poses ['1258']; frontend md5 ['bc23199243971466e1645e4eed7640ca']
    - APE frontend: common6 median 0.048218 (range 0.048218..0.048218), all7 median 0.051967 (pairs ['7']), distinct trajectories 1
- ours_loop: n=5 exits=['0'] time median 44.00 s (RTF 0.3497) runs ['44.0', '43.0', '44.0', '45.0', '45.0']; incl. save 45.34 s (RTF 0.3604) runs ['45.34', '44.33', '45.32', '46.44', '46.39']; peak RSS max 474.5 MB; CPU median 161.3 s; poses ['40']; frontend md5 ['bc23199243971466e1645e4eed7640ca']
    - APE corrected_dense: common6 median 0.049459 (range 0.049459..0.049459), all7 median 0.048755 (pairs ['7']), distinct trajectories 1
    - APE frontend: common6 median 0.048218 (range 0.048218..0.048218), all7 median 0.051967 (pairs ['7']), distinct trajectories 1

- ours_readme/glim: time 1.146, incl. save 1.146, peak RSS 0.640, CPU 1.288, APE common6 0.563 (43.7% lower), APE all7 0.607
- ours_loop/glim: time 1.171, incl. save 1.207, peak RSS 0.656, CPU 1.491, APE common6 0.578 (42.2% lower), APE all7 0.570
