# M7 step b results (plan.txt, preregistered 2026-11-17)

Carrier: the M7b commits on develop b499ce88 (local build: graph_based_slam in a Jazzy workspace;
the `lidarslam` package built with pip, run with no ROS environment).

## Checks without ground truth

| Check | Result |
| --- | --- |
| P1 GNSS off: `graph_slam_offline_runner`, MapsHD calibration-room 2, walking preset | 7/7 files identical to the runner before the change |
| P1 GNSS off: `lidarslam-map run` (core engine), HILTI exp14 | 24 data files identical to M5 (the 6 reports differ in paths and times, as in M6) |
| P2 GNSS on (`lidarslam_rtk_gnss.yaml`, map bundle on): ROS 2 runner vs package, Construction seq1 / seq2 | 51/51 and 44/44 files identical; 98 and 59 RTK anchors, yaw alignment applied, `LocalCartesian` projector |
| P3 live node: `test_gnss_anchor_store` (the store vs the previous inline code, three configurations) | samples, anchors, yaw alignment identical |

## Accuracy acceptance (RTK-SLAM offline protocol, no alignment)

Scored as the references were: the corrected submap poses densified with the odometry
(`densify_corrected_trajectory.py`), ENU origin = the backend's GNSS origin, lever arm
[-0.062, 0.0, -0.216]. Re-scoring the live reference run of
docs/research/gnss-rtk-slam-absolute-accuracy.md with the same tool gives its 0.1000 m.

| Run | absolute 3D | horizontal | vertical | SE3 | checkpoints | reference (live ROS 2) | limit | |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| A stadtgarten_seq2, Stadtgarten 2 preset | 0.1000 m | 0.052 m | 0.086 m | 0.074 m | 19/19 | 0.100 m | 0.130 m | PASS |
| B stadtgarten_seq1, shared preset | 0.0670 m | 0.041 m | 0.053 m | 0.057 m | 36/36 | 0.068 m | 0.098 m | PASS |

**ACCEPT.** A: 165 RTK anchors on 266 submaps, 0 loops (none on the live path either); 77 s,
608 MB peak. B: 448 RTK anchors on 635
submaps, 6 loops; 205 s, 1.21 GB peak (one process: RKO-LIO and the backend). The expected
difference (loops searched on odometry poses, anchoring once at the end) did not show: B accepted
the same 6 loops the live path accepted.

Report only, Construction seq1 / seq2 backend inputs (walking-preset frontend,
`lidarslam_rtk_gnss.yaml`): 0.248 m / 0.166 m absolute 3D (outdoor checkpoints 0.052 / 0.063 m;
indoors there is no GNSS).

Inputs: the 3-topic extracts (`/livox/points`, `/livox/imu`, `/gnss/fix`) read once from the
dataset's `.db3` with `scripts/extract_rosbag2_sqlite_topics.py` (whole-file sha256 as in
`scripts/download_rtk_slam_dataset.py`); its topics-table bug on Stadtgarten 2 (message_definitions
rows taken for topics) was fixed first.
