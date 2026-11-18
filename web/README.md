# Browser demo (WebAssembly)

The docs page [Try It in Your Browser](../docs/browser-demo.md) runs RKO-LIO odometry in
the browser. This directory holds what it is built from:

| File | Purpose |
| --- | --- |
| `slam_web.cpp` | `rko_lio::driver::OfflineDriver` behind a small C API for the page; built natively, `main()` replays a clip and writes a TUM trajectory |
| `tbb_shim/tbb/` | single-threaded stand-in for the oneTBB headers RKO-LIO uses; splitting follows oneTBB, so deterministic sums keep their order |
| `build_wasm.sh` | builds the module with Emscripten (`--target web` for the page, `--target node` for the check below) |
| `make_clip.py` | writes the page's compact LSWC2 clip from a rosbag2 |
| `replay_node.js` | replays a clip through the module in Node.js, optionally against a reference trajectory |
| `demo_reference.tum` | the native build's trajectory on `docs/demo/clip_mid360_60s.lswc2.gz` |

The page itself (`docs/demo/index.html`, `slam_worker.js`, `params.json`, the clip) lives under
`docs/demo/`. The docs workflow builds `slam_web.js` and `slam_web.wasm` there before MkDocs runs;
they are not committed.

## Build

Emscripten (Ubuntu 24.04: `apt install emscripten`, 3.1.6) and the header-only dependencies,
gathered in one include directory:

```bash
sudo apt install emscripten libeigen3-dev robin-map-dev nlohmann-json3-dev
git clone --depth 1 -b 1.22.10 https://github.com/strasdat/Sophus.git /tmp/Sophus
mkdir -p /tmp/wasm-include
ln -s /usr/include/eigen3/Eigen /tmp/wasm-include/Eigen
ln -s /tmp/Sophus/sophus /tmp/wasm-include/sophus
ln -s /usr/include/tsl /tmp/wasm-include/tsl
ln -s /usr/include/nlohmann /tmp/wasm-include/nlohmann

web/build_wasm.sh --include-dir /tmp/wasm-include --out-dir docs/demo
python3 -m http.server -d docs/demo 8000   # then open http://localhost:8000
```

## Check against the native build

```bash
web/build_wasm.sh --include-dir /tmp/wasm-include --out-dir /tmp/wasm-node --target node
node web/replay_node.js /tmp/wasm-node/slam_web_node.js docs/demo/clip_mid360_60s.lswc2.gz \
  docs/demo/params.json /tmp/wasm.tum --reference web/demo_reference.tum
```

All 599 poses match the native trajectory to the 9 printed decimals. `demo_reference.tum`
comes from the native build of `slam_web.cpp` (g++ with oneTBB, same Eigen and Sophus):

```bash
R=Thirdparty/rko_lio
g++ -std=c++20 -O3 -DNDEBUG -DSOPHUS_USE_BASIC_LOGGING -I$R -I/tmp/wasm-include \
  $R/rko_lio/core/{lio,bump_image_map,direct_visual_odometry,photometric_features,photometric_image,voxel_hash_map,voxel_down_sample,process_timestamps,preprocess_scan}.cpp \
  $R/rko_lio/driver/{driver_config,offline_driver,point_cloud}.cpp web/slam_web.cpp -ltbb -o /tmp/slam_web
gunzip -c docs/demo/clip_mid360_60s.lswc2.gz > /tmp/clip.lswc2
/tmp/slam_web /tmp/clip.lswc2 docs/demo/params.json web/demo_reference.tum
```

The native build with the serial `tbb_shim` instead of oneTBB writes the same file byte for byte.

## The clip

```bash
python3 web/make_clip.py <rosbag2_2024_04_16-14_17_01> docs/demo/clip_mid360_60s.lswc2.gz \
  --duration 60 --keep-every 4 \
  --meta '{"dataset": "Livox MID-360 public driving SLAM recording rosbag2_2024_04_16-14_17_01", "source": "https://zenodo.org/records/14841855", "license": "CC BY 4.0"}'
```

The bag is the public MID-360 recording `lidarslam-map demo` downloads
([Zenodo 14841855](https://zenodo.org/records/14841855), CC BY 4.0).
