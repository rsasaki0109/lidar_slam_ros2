# Humble clean-source first-map validation — 2026-07-28

## Result

A fresh ROS 2 Humble container cloned the product branch recursively, installed
all declared dependencies, built all six packages, and used only the installed
`lidarslam-map` command to produce and verify an Autoware-compatible map from
the tracked public MID-360 bag.

The run passed. Most importantly for the installed-provenance contract, the
final `run_manifest.json` retained the exact clean source revision after the
installed command was invoked:

```json
{
  "git_commit": "41cdfaadfa89b92b764079d1997fdf40e6fe78d7",
  "git_dirty": false,
  "ros_distro": "humble"
}
```

This closes the maintainer-operated Humble source-workspace trial requested by
the Phase 1 gate. It does not count as one of the three independent-user
first-map reports required for v1.0.

## Frozen inputs

| Input | Recorded value |
| --- | --- |
| Base image | `docker.io/library/ros@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16` |
| Source branch | `product/build-provenance` |
| Source revision | `41cdfaadfa89b92b764079d1997fdf40e6fe78d7` |
| Checkout | fresh recursive HTTPS clone |
| Build | `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` |
| Profile | `rko_lio_graph_mid360_preset` |
| Bag metadata SHA-256 | `65d66875f49248e38ff14d80e6e749fb50606f6f80bd4be337160e3752691e9a` |
| Bag DB3 SHA-256 | `3bbd390a97e57af47ad6699baa36eb4c5f39f61b35275505ecaf221c126354f5` |
| Input source | Koide, *Driving SLAM Test with Livox MID360*, Zenodo DOI `10.5281/zenodo.14841855`, CC-BY 4.0 |

The trial did not mount an existing source, build, or install tree. `rosdep`
resolved dependencies inside the container before the full build. The public
bag was mounted read-only.

## Execution

The installed command surface was exercised in this order:

```text
lidarslam-map --version
lidarslam-map doctor /input/mid360
lidarslam-map run /input/mid360 \
  --profile rko_lio_graph_mid360_preset \
  --output-dir /evidence/mid360_humble_source_map
```

The doctor detected `/livox/lidar`, `/livox/imu`, and the per-point
`timestamp` field, then recommended the maintained RKO-LIO plus graph SLAM
path. The full build completed in 489 seconds. Existing PCL CMake policy and
pcap-disabled warnings were non-fatal.

## Acceptance results

| Check | Result |
| --- | --- |
| Full source build | PASS — 6 packages |
| Installed CLI exit | `0` |
| Manifest schema/status | schema `2`, `succeeded` |
| Lifecycle stage/runner exit | `complete` / `0` |
| Installed source identity | PASS — exact revision, `dirty=false` |
| Diagnosis | `success` |
| Independent verifier rerun | PASS — 8 pass, 0 warn, 0 fail |
| Corrected/raw poses | 577 / 2,681 |
| Point-cloud tiles | 364 |
| Lanelet2 lanelets | 42 |
| Output size | 127 MiB |
| `.partial` sibling absent | PASS |
| Host ownership after handoff | PASS — invoking user UID/GID |

Selected artifact identities:

| Artifact | SHA-256 |
| --- | --- |
| `run_manifest.json` | `6e32ec0941c1f1ce141d938aa5150404f42aafed2e926b8a5539d09626b3f8e7` |
| `autoware_map_diagnosis.json` | `217b567f5bb45ab1b84740a971963f8bbb9c5b3837bad656ca66829acebc2150` |
| `verify_autoware_map.log` | `bd11be44632b001da38bbfe57dba2896b33d71a5319bcbd243ea5cf3c06804e9` |
| `pointcloud_map_metadata.yaml` | `673406b767d672f4d01154185e5e2d682a8502227ced0cd898054e3d385f3a78` |
| `traj_corrected.tum` | `47e9a64d6f6c32682329f9f72289252b8680ff11a53472f1c0eb3badf827a032` |

## Scope and remaining findings

- This is one maintainer-operated amd64 Humble run. Jazzy has separate
  installed-prefix provenance coverage, but its full clean-source map run is
  recorded on the earlier source-usability branch and must be integrated in
  branch order.
- `lidarslam-map --version` and the manifest report `0.6.0`. Repository,
  release, image, and package version alignment remains a Phase 0 release
  blocker; this trial does not claim v0.9 packaging is complete.
- The initial dependency installation is large and visually noisy. A binary
  distribution path and clearer progress reporting remain onboarding work.
- Existing PCL policy warnings and disabled pcap support should be cleaned up,
  but did not affect this MID-360 PointCloud2 path.
- Pull-request CI remains authoritative for the supported build matrix.
