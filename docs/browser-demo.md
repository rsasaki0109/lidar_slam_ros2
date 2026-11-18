# Try It in Your Browser

RKO-LIO, the LiDAR-inertial odometry behind `lidarslam-map start`, runs here as
WebAssembly. Press **Start**: the page downloads a 60-second clip of a public
Livox MID-360 drive (7.4 MB) and builds the map on your device, in a worker
thread. Nothing is uploaded.

<iframe src="demo/index.html" title="LiDAR SLAM in your browser"
        style="width: 100%; height: 70vh; border: 1px solid rgba(127,127,127,0.3); border-radius: 8px;"
        loading="lazy"></iframe>

[Open the demo on its own page](demo/index.html).

## What runs

- The odometry is `rko_lio::driver::OfflineDriver` with the RKO-LIO core, the
  same C++ the ROS 2 node and the `lidarslam` Python package run, compiled with
  Emscripten. It is single-threaded: a serial stand-in for oneTBB
  (`web/tbb_shim`) keeps oneTBB's splitting, so deterministic sums are formed in
  the same order.
- On the clip, the WebAssembly build gives the native build's trajectory: all
  599 poses agree to the 9 printed decimals (`web/replay_node.js` checks this
  against `web/demo_reference.tum`, written by the native build).
- The map is the accumulated deskewed scans at the odometry poses. The pose
  graph, loop closure and the Autoware map bundle of the full pipeline are not
  part of this page; use `pip install lidarslam` or the ROS 2 packages for those.

## The clip

`docs/demo/clip_mid360_60s.lswc2.gz` holds the first 60 s of
`rosbag2_2024_04_16-14_17_01` from the public Livox MID-360 driving recording
([Zenodo 14841855](https://zenodo.org/records/14841855), CC BY 4.0): the IMU
samples as recorded, and every fourth LiDAR point with coordinates rounded to
1 cm and point times to 2 µs. `web/make_clip.py` writes it from the bag. On
this thinned clip the odometry stays within 2 m of the raw trajectory of the
product demo (`lidarslam-map demo`, all points) over the same 60 s: 0.8 m on
average over 178 m.

Build the module yourself with `web/build_wasm.sh`; see `web/README.md`.
