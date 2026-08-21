# SMALL_GICP / SMALL_VGICP external DSO and ODR gate

Date: 2026-08-21
Status: pluginlib discovery and replay compatibility **Go for offline
characterization**; the scoped Small-only symbol-isolated DSO gate is **Go**
for the pinned build and replays; live/default promotion remains **No-Go**.

## Decision

The historical dependency-enabled install discovered and instantiated the
real external pluginlib classes `lidarslam_default_plugins/SmallGicpPcl` and
`lidarslam_default_plugins/SmallVGicpPcl`.  Their receipts contain
`backend_kind: pluginlib`, the installed combined DSO path, and the
`registration_plugins_with_small.xml` manifest path; no
`lidarslam_builtin/*` host alias was selected.  HILTI exp04 and MID-360
first-100 and full replays were byte-identical to the legacy
same-translation-unit path for both selectors.

The first set of runs used the ordinary combined DSO and was not an
independent implementation proof: that DSO exports default-visible weak
`small_gicp::RegistrationPCL` template symbols, and the process binding trace
shows those symbols—and the concrete adapter's `align()`—resolving to
`libscanmatcher_component.so`, which already contains the same-translation-unit
instantiations.  The ordinary run therefore closes class discovery,
provenance, and replay-wiring only.  The scoped symbol-isolation experiment
below repeats the gate with a dedicated Small DSO and function-local binding;
it is still not a production default or absolute-accuracy claim.

## Isolated install and provenance

The normal system installation was not modified.  The runs sourced the
read-only vendor extraction and isolated workspace in this order:

```text
/tmp/small-gicp-vendor.7YYK8k/extracted/opt/ros/jazzy/opt/small_gicp_vendor
/tmp/small-gicp-build.2auPmA/install
```

The installed artifacts used by every external run were:

| artifact | value |
| --- | --- |
| DSO | `/tmp/small-gicp-build.2auPmA/install/lidarslam_default_plugins/lib/liblidarslam_default_plugins.so` |
| DSO SHA-256 | `a614240d6a7f96172babc581b263e5a7a03a1a944c765bb00d8aa36171b962ab` |
| manifest | `/tmp/small-gicp-build.2auPmA/install/lidarslam_default_plugins/share/lidarslam_default_plugins/registration_plugins_with_small.xml` |
| manifest SHA-256 | `10e00a3e02a60a1ff7320954b41d9d1f72851b6d5947efc908b97f36227775b9` |
| vendor | `ros-jazzy-small-gicp-vendor 2.1.0-1noble.20260309.122135`, extracted under `/tmp` |

The XML declares these Small external classes (alongside the package's NDT
and pclomp-GICP classes):

```xml
<class name="lidarslam_default_plugins/SmallGicpPcl" ... />
<class name="lidarslam_default_plugins/SmallVGicpPcl" ... />
```

The clean-install loader contract test passed for both classes, including
`backend_kind == pluginlib`, class identity, non-empty library path, and
non-empty manifest path:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /tmp/small-gicp-build.2auPmA/install/setup.bash
/tmp/small-gicp-build.2auPmA/build/lidarslam_registration_loader/\
  test_registration_plugin_loader \
  --gtest_filter=RegistrationPluginLoader.DiscoversAndConfiguresOptionalSmallClassesOnlyWhenAdvertised
```

Result: 1 test passed, 0 failed.  The receipts used by the replay runs are
preserved outside the repository under the output directories listed below.
Absolute receipt paths are provenance fields and must be normalized before
receipt-byte comparisons between install roots.

## Replay method

Each comparison used `ndt_num_threads:=1`, `async_map_update:=false`, the
same parameter file and topics on both sides, and `max_clouds:=100` for the
smoke or `max_clouds:=0` for the full run.  The legacy side left
`registration_plugin_enable` false and therefore constructed the built-in
same-TU path.  The external side set both:

```text
registration_plugin_enable:=true
registration_plugin_class:=<one of the two external IDs>
```

The class line is selected per run, not supplied twice.  Every run exported the
trajectory and submap CSV.  Full runs also exported the world-frame PCD;
equality was checked with `cmp`, MD5, and (for maps) SHA-256.

### First 100 scans

The external side was compared against the matching legacy artifact in each
directory.  Every listed pair was `cmp`-equal.

| input | selector | external trajectory MD5 | external submap MD5 | output |
| --- | --- | --- | --- | --- |
| HILTI exp04 | `SmallGicpPcl` | `829b23ed9e307f431af558546d27d000` | `21ee5902fc9208fab0b4502e32095b60` | `/tmp/small-gicp-dso-hilti-100.Kk1QYw/SMALL_GICP` |
| HILTI exp04 | `SmallVGicpPcl` | `ecc6e402c4d67e23154a67914a2bd3d3` | `0344cb056b47e03ec19a9a71566dd48d` | `/tmp/small-gicp-dso-hilti-100.Kk1QYw/SMALL_VGICP` |
| MID-360 | `SmallGicpPcl` | `84e99e44d6a3fafebfed77b227064dfc` | `d4d79270861753ab747f3d131a1dc0b9` | `/tmp/small-gicp-dso-mid360-100.pLBaDB/SMALL_GICP_dso` |
| MID-360 | `SmallVGicpPcl` | `89de1da6f7058050099fd27fd4d34dc7` | `43068c9a737307b4f78a9e4d27945aa6` | `/tmp/small-gicp-dso-mid360-100.pLBaDB/SMALL_VGICP_dso` |

HILTI used `/media/sasaki/aiueo1/datasets/hilti2022/exp04_ros2`,
`/hesai/pandar`, `/alphasense/imu`, and
`configs/hilti2022/lidarslam_competitive_v2.yaml`.  MID-360 used
`/media/sasaki/aiueo1/datasets/mid360_public/driving_slam_mid360/extracted/rosbag2_2024_04_16-14_17_01/rosbag2_2024_04_16-14_17_01`,
`/livox/lidar`, `/livox/imu`, and `lidarslam/param/lidarslam.yaml`.

### Full replay, one external run

The legacy column is the existing same-TU run 1 from the corresponding full
gate.  The external run was a new ordinary pluginlib process.  All four rows
were `cmp`-equal for trajectory, submaps, and map PCD; the map SHA-256 also
matched.

| input / selector | poses / submaps | trajectory MD5 | submap MD5 | map MD5 / bytes | map SHA-256 | external wall / peak RSS |
| --- | ---: | --- | --- | --- | --- | ---: |
| HILTI / `SMALL_GICP` | 1,258 / 26 | `c6b98f87d0411a1167a4b26e285e90fa` | `23bc8057ad3b22b1bcdb4c1868c886d0` | `3e6dd027db8937396eacc31c84c3fa77` / 6,229,590 | `051e1e48fe04970e5014592548cd5b0d363460c1aef052bcd709b4af2918d1c2` | 134.00 s / 214,832 KiB |
| HILTI / `SMALL_VGICP` | 1,258 / 38 | `f69ae783894db9e19a9c9af86c17d4a0` | `6d26d745e119e423df026875b5f351dd` | `718a4679bd8b3ad6b06949f2a9f8f4d4` / 8,300,217 | `bb64790cd8d46248c26bd6939428cb94e1f7b745d3c056eb23a8aa0f6b2d3925` | 94.80 s / 259,828 KiB |
| MID-360 / `SMALL_GICP` | 2,772 / 295 | `2e36e8d58cde280f2d12e7cad5cf850a` | `baaa92dcee874e7047a753e5bb7052de` | `947df4b495f92400137cb6a52c9a22ae` / 39,676,100 | `684d2983215ac76d91c108eec82916933e3d6854728e85104f4d1d5479644115` | 237.75 s / 754,884 KiB |
| MID-360 / `SMALL_VGICP` | 2,772 / 372 | `c6b1d03e2ced7acc694d2aa8880f070b` | `c975d42481af07077484109003069f27` | `f48f8e4c963675f7b57ca90ab7698647` / 49,469,494 | `ac7fd49dab3d0ea8f59c7c22c47cd235eac1c6d21c94aea6d0a677714e970b77` | 147.71 s / 896,584 KiB |

Full external artifacts are in
`/tmp/small-gicp-external-dso-hilti-full.20260821` and
`/tmp/small-gicp-external-dso-mid360-full.20260821`.  This gate did not rerun
the sparse HILTI GT or geometric evaluators: the existing HILTI APE and
map-quality values remain applicable because the external artifacts are
byte-identical, but no separate external-DSO accuracy claim is made here.
MID-360 has no paired GT; the prior full gate recorded 538 GICP and 332 VGICP
pose rejections.  Those counts are identical across the byte-identical paths,
but hash equality is not an absolute tracking-quality result.

## ODR / symbol-binding inspection

The ordinary DSO has no `SYMBOLIC` dynamic flag.  `readelf -Ws` and
`nm -D -C --defined-only` show default-visible weak (`W`) definitions for
`small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>` and for the
corresponding pclomp template specializations.  `readelf -d` shows no
`libsmall_gicp.so` `NEEDED` entry; the template code is present in the DSO.

An `LD_DEBUG=bindings` trace was captured while loading the real external
class in the scanmatcher process:

```text
/tmp/small-gicp-ld-bind.0NtAIA/ld.267048
```

Representative bindings include:

```text
liblidarslam_default_plugins.so -> libscanmatcher_component.so:
  small_gicp::RegistrationPCL<...>::computeTransformation(...)
liblidarslam_default_plugins.so -> libscanmatcher_component.so:
  lidarslam_default_plugins::SmallGicpRegistration::align(...)
```

The trace also records `setInputTarget`, RTTI, and destructor bindings for the
same `small_gicp::RegistrationPCL` specialization.  Thus the real external
class is discovered and constructed, but ordinary ELF symbol interposition
allows the host's same-TU template definitions to satisfy the DSO.  Exact
replay hashes must be treated as contaminated wiring evidence, not as proof
that an independently compiled DSO has the same numerical implementation.

## Historical ordinary-DSO gate outcome

| gate | result |
| --- | --- |
| external class IDs discovered from clean install | **Go** |
| pluginlib loader/session lifetime and provenance | **Go** |
| HILTI/MID-360 first-100 compatibility | **Go**, exact artifacts |
| HILTI/MID-360 full ordinary-DSO compatibility | **Go**, one external run each, exact artifacts |
| independent DSO/ODR isolation of the ordinary combined DSO | **No-Go**, binding trace proves host interposition |
| production default / absolute accuracy promotion | **No-Go** |

The production defaults and README claims remain unchanged.  The candidate
`-Wl,-Bsymbolic-functions` fix was then evaluated only for a dedicated Small
DSO; its evidence is recorded below.  The NDT/GICP DSO and any live default
route were not changed by that experiment.

## Scoped Small-only symbol-isolation experiment (2026-08-21)

The low-risk fix was applied only to the optional Small target.  The normal
`liblidarslam_default_plugins.so` now contains NDT/GICP only, while
`liblidarslam_small_gicp_plugins.so` contains both Small classes and is linked
with `-Wl,-Bsymbolic-functions`.  The plugin manifests are separate, valid XML
files, and pluginlib's ament resource lists both paths:

```text
share/lidarslam_default_plugins/registration_plugins.xml
share/lidarslam_default_plugins/registration_plugins_small.xml
```

The clean dependency-enabled build was isolated under
`/tmp/small-gicp-bsymbolic-build.5FLKtQ` using the read-only vendor extraction
`/tmp/small-gicp-vendor.7YYK8k`.  The loader test discovered and configured both
real Small classes; each receipt named
`liblidarslam_small_gicp_plugins.so` and `registration_plugins_small.xml`.
The normal DSO has no small_gicp NEEDED entry.  Artifact SHA-256 values are:

| artifact | SHA-256 |
| --- | --- |
| normal DSO | `944272cfe2266e9fb708c710cde88720851c67f005dfac437cbf876f9ca47ca8` |
| Small DSO | `013cc91a8c1bab9681781066701464ac1e9a8ae2319247ea4d77c16fc6426280` |
| normal manifest | `5d412a2ecadc449aaed12622a3a042259944088e7db73026aa2c2d82d92d0f97` |
| Small manifest | `fe3e55ba1d691e63c2142836071f86c4156ff3d0a3505b88f08aae40d1bd1fe6` |

`nm -D -C --defined-only` confirmed the Small DSO owns
`SmallGicpRegistration::align` and the
`small_gicp::RegistrationPCL<...>::computeTransformation` definition.  For
each of the four first-100 runs, `LD_DEBUG=bindings` showed zero bindings in
which the Small DSO requested either of those symbols from
`libscanmatcher_component.so`; the trace files were captured under the run
directories below.  RTTI/typeinfo bindings remain expected ABI sharing and
are not treated as algorithm implementation interposition.  No
`readelf -rW` relocation for either requested function remains in the
dedicated DSO.

The independent replay comparison used `ndt_num_threads=1`,
`async_map_update=false`, the same bag/topics/parameter file as the legacy
receipts, and `cmp` on trajectory, submaps, and map PCD.  First-100 artifacts
were exact for all four dataset/selector pairs:

| input / selector | trajectory MD5 | submap MD5 | output |
| --- | --- | --- | --- |
| HILTI / `SMALL_GICP` | `829b23ed9e307f431af558546d27d000` | `21ee5902fc9208fab0b4502e32095b60` | `/tmp/small-gicp-bsymbolic-hilti-gicp-100.Uz5LqX` |
| HILTI / `SMALL_VGICP` | `ecc6e402c4d67e23154a67914a2bd3d3` | `0344cb056b47e03ec19a9a71566dd48d` | `/tmp/small-gicp-bsymbolic-hilti-vgicp-100.oNLqg4` |
| MID-360 / `SMALL_GICP` | `84e99e44d6a3fafebfed77b227064dfc` | `d4d79270861753ab747f3d131a1dc0b9` | `/tmp/small-gicp-bsymbolic-mid-gicp-100.1Gx7IO` |
| MID-360 / `SMALL_VGICP` | `89de1da6f7058050099fd27fd4d34dc7` | `43068c9a737307b4f78a9e4d27945aa6` | `/tmp/small-gicp-bsymbolic-mid-vgicp-100.sZBS5N` |

Because the smoke paths were exact and had no forbidden implementation-symbol
binding, one full run per selector and dataset was completed.  The full
trajectory/submap/map PCD artifacts were also `cmp`-equal to the existing
legacy run-1 artifacts; map SHA-256 values matched as well:

| input / selector | trajectory MD5 | submap MD5 | map MD5 | map SHA-256 | wall / peak RSS |
| --- | --- | --- | --- | --- | --- |
| HILTI / `SMALL_GICP` | `c6b98f87d0411a1167a4b26e285e90fa` | `23bc8057ad3b22b1bcdb4c1868c886d0` | `3e6dd027db8937396eacc31c84c3fa77` | `051e1e48fe04970e5014592548cd5b0d363460c1aef052bcd709b4af2918d1c2` | 153.42 s / 211108 KiB |
| HILTI / `SMALL_VGICP` | `f69ae783894db9e19a9c9af86c17d4a0` | `6d26d745e119e423df026875b5f351dd` | `718a4679bd8b3ad6b06949f2a9f8f4d4` | `bb64790cd8d46248c26bd6939428cb94e1f7b745d3c056eb23a8aa0f6b2d3925` | 122.68 s / 263152 KiB |
| MID-360 / `SMALL_GICP` | `2e36e8d58cde280f2d12e7cad5cf850a` | `baaa92dcee874e7047a753e5bb7052de` | `947df4b495f92400137cb6a52c9a22ae` | `684d2983215ac76d91c108eec82916933e3d6854728e85104f4d1d5479644115` | 264.49 s / 753768 KiB |
| MID-360 / `SMALL_VGICP` | `c6b1d03e2ced7acc694d2aa8880f070b` | `c975d42481af07077484109003069f27` | `f48f8e4c963675f7b57ca90ab7698647` | `ac7fd49dab3d0ea8f59c7c22c47cd235eac1c6d21c94aea6d0a677714e970b77` | 169.05 s / 896808 KiB |

Full artifacts are under
`/tmp/small-gicp-bsymbolic-hilti-full-gicp-WciSGK`,
`/tmp/small-gicp-bsymbolic-hilti-full-vgicp-BfZ2qf`,
`/tmp/small-gicp-bsymbolic-mid-full-gicp-7Pvg4X`, and
`/tmp/small-gicp-bsymbolic-mid-full-vgicp-jcwEoc`.  These are compatibility
and ODR-isolation receipts, not an absolute accuracy result: MID-360 has no
paired GT, and the existing HILTI accuracy/profile evidence is retained as
the same byte artifacts rather than rerun under a new accuracy claim.

The scoped independent Small DSO/replay gate is therefore **Go** for this
pinned Jazzy/vendor/toolchain build.  Production default promotion remains
**No-Go** until live-node policy, broader compiler/ROS/vendor matrix, and
paired-ground-truth accuracy criteria are separately closed.  The default
NDT/GICP route, README claims, and production scanmatcher selection are
unchanged.
