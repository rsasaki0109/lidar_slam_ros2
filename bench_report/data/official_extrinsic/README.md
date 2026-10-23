# Official RKO-LIO 0.4.2 with a non-identity LiDAR extrinsic (HILTI exp04)

HILTI 2022 exp04, whose PandarXT-32 is mounted upside down: `extrinsic_lidar2base_quat_xyzw_xyz` is a
half turn about (1, -1, 0) / sqrt(2) plus a 5.6 cm offset (`/home/user/bench/params/rko_params.ros.yaml`).
Each run is `scripts/official_extrinsic_run.sh` (`pipe_pipeline.sh`, BARRIER=1, chapter 25's C backend).

| Run | Frontend | Backend | Expected scan handling |
| --- | --- | --- | --- |
| `fork` | dev_c source rko_lio (scans stamped `PandarXT-32`) | develop 99ab052 | extrinsic applied once, in the backend |
| `official` | official `ros-jazzy-rko-lio 0.4.2-1noble.20260930.090637` (scans stamped `base_link`) | develop 99ab052 | extrinsic applied once, in the frontend; the backend skips it |
| `official_double` | the same official binary | develop with the `scanIsInBaseFrame` skip removed, the behaviour before #561 (local build, not committed) | extrinsic applied twice |

- `runs.csv`: `pipe_pipeline.sh` columns (label, frontend_s, complete_s, save_s, offline_node_took_s, submaps,
  loop_edges, front_md5, traj_md5, g2o_md5, loops_md5).
- `ape.csv`: `ape_runs.py` (frontend_all7, frontend_common6, corrected_all7, corrected_common6; m).
- `map_quality.csv`: `map_quality_cmp.sh` (run, map_points, map_md5, evaluated_points, thickness_rms_mean_m,
  thickness_rms_p95_m, planar_coverage, patch_count, mean_map_entropy_nats).
- `*_frontend.txt`: the frontend's extrinsic and topic log lines.

The fork's front_md5 `bc231992…` equals earlier exp04 runs with these parameters. Its 871 s frontend time is
the cold page cache: the fork first reads the whole 13 GB bag for `/tf_static` (632 s before the first scan);
earlier runs with a warm cache took 22 to 52 s.
