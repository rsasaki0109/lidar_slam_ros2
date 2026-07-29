# Leo Drive packet benchmark evidence — 2026-07-30

## Scope

Clean candidate commit `7a474c931f5fbd7e66670e3df23bba857df466ea`
was built on ROS 2 Jazzy and run against the public Leo Drive
`all-sensors-bag1`. The command used only the benchmark defaults for packet
topic and playback rate. It therefore exercises the product behavior added in
this revision: deterministic front-LiDAR preference and correctness-first
`1.0x` playback.

The Applanix GSOF49 trajectory is a cross-validation reference from the same
recording, not independent ground truth. This evidence closes the named
`leo_drive_applanix_velodyne_cross` regression profile only.

## Reproduction

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix

bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag /path/to/all-sensors-bag1 \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --output-dir output/leo_drive_all_sensors_bag1_auto \
  --verify-map
```

The selector reported all three packet streams and chose
`/sensing/lidar/front/velodyne_packets`; the benchmark reported `rate: 1.0`.
No `--packet-topic` or `--rate` override was used.

## Provenance

Input identity:

- `metadata.yaml` SHA-256:
  `2bee353e20f73b3fe078f24a1e18426655a25752138abc2f03a7be6afe5cb014`;
- sqlite3 storage SHA-256:
  `74e5915719a7b7b4820b5339207eeade0c656deaa38b8e5b5e8d18787a58ac22`;
- sqlite3 storage size: 2,506,907,648 bytes;
- bag duration: 36.513196011 seconds.

Software identity:

- candidate commit:
  `7a474c931f5fbd7e66670e3df23bba857df466ea`;
- worktree state at build and run: clean;
- Velodyne upstream:
  `56fc178d2dad4b6d38c6a69aeb2435ff75503e52`;
- diagnostics upstream:
  `81b53fad4dfdd5169d7db4aa77b60602e4b46d86`;
- angles upstream:
  `a96224f9ab3ac51fe8fd981c1e1554528dc4345a`;
- `velodyne_transform_node` SHA-256:
  `e75e0c6f5f479dbbae0534234907615ea532a46d291038674ba0df97aa37245a`;
- `scanmatcher_node` SHA-256:
  `e07885ee6db88ad0f1f636832b93d0a29a5caf0318d2710f84bfac2ca9b1b961`;
- `graph_based_slam_node` SHA-256:
  `474b162fb021b33f34ce1eca6cc57022574c5f779cc522f42f67d79ae78dd85a`.

Machine identity:

- Ubuntu 24.04.4 LTS, x86_64, ROS 2 Jazzy;
- Intel Core i5-1145G7, 8 logical CPUs, 30 GiB RAM.

Output identity:

- `metrics.json` SHA-256:
  `d0a039facfeb7bd5c5c3a6de019d884dfae744192d090fea67762379daa2fc66`;
- `map.pcd` SHA-256:
  `cf54234f0a75b341ea03c0e079559db281510a03e4251ebc9c25780063d2b3ab`;
- extracted reference TUM SHA-256:
  `98ea2825378036b2bad838cdff921540bcd4abd050f33b80ba35c5b9db4dba26`.

## Result

The release profile reported `TARGET_MET`:

- corrected APE RMSE: **0.139152 m**;
- profile pass threshold: 1.500 m;
- profile target threshold: 0.500 m;
- matched corrected/reference poses: 571, above the 200-pair coverage floor;
- corrected path length: 95.440 m;
- raw path: 365 poses, 3,640 matched reference poses, 0.269712 m RMSE;
- wall time: 54.381 seconds; processing RTF: 1.489.

The generated map passed all eight Autoware compatibility checks. The verifier
reported one informational warning because grid-map metadata lives in the
`pointcloud_map/` subdirectory.

The full release-profile command still exited `2`, as intended, because four
other blocking profiles were not present in this one-run evidence directory.
The Leo Drive row itself was `TARGET_MET`; missing unrelated evidence was not
treated as success.

## Defect comparison

Before the selector fix, automatic detection chose the right-side packet
stream because it contained one extra message. At `1.0x`, that run produced
5.576551 m corrected APE RMSE and failed the profile. Explicit front-LiDAR
selection produced 0.138869 m. The approximately 40x difference identified
topic selection, rather than SLAM tuning, as the dominant defect.
