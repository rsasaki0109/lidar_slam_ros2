# Distribution and installed CLI

This page describes the supported source and container installation paths and
the remaining boundary before `lidarslam_ros2` can publish ROS buildfarm
packages.

## Supported source install

Humble on Ubuntu 22.04 and Jazzy on Ubuntu 24.04 are the supported source-build
targets. Clone the submodules because the flagship RKO-LIO frontend is supplied
from the maintained fork in `Thirdparty/`.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The installed own-bag command is then available from any working directory:

```bash
lidarslam-map doctor /path/to/rosbag2
lidarslam-map run /path/to/rosbag2 --output-dir "$PWD/output/my_map"
lidarslam-map inspect "$PWD/output/my_map"
```

Use an absolute output path when it matters where artifacts are written. An
installed CLI defaults relative output to the current working directory; it
never writes into the read-only package share.

## Names and compatibility

The package has historically installed a C++ ROS node named `lidarslam`. That
name remains unchanged:

```bash
ros2 run lidarslam lidarslam
```

The product CLI deliberately uses two non-conflicting installed spellings:

```bash
lidarslam-map --help
ros2 run lidarslam lidarslam-cli --help
```

Both spellings dispatch the same `doctor`, `run`, and `inspect` contract. The
`ros2 run` form is a compatibility shim, not a fourth product workflow.
Inside a source checkout, `./scripts/lidarslam` exposes the same contract.

## What the installation contains

The `lidarslam` package installs a curated runtime set:

- launch files, parameter presets, and RViz configuration;
- the historical C++ node;
- `lidarslam-map` and the `lidarslam-cli` ROS shim;
- the product runner, preflight, diagnosis, verification, conversion, and
  viewer helpers required transitively by maintained profiles;
- the repository product version.

Research trainers, sweep tools, generated benchmark output, and repository
media are not copied into the product-script directory.

Every Humble/Jazzy default CI job creates a fresh, non-symlinked install prefix
and checks all curated resources from an unrelated working directory. The gate
also runs `--version`, `doctor`, an own-bag dry run, and `inspect`, and confirms
that `ros2 run lidarslam lidarslam` was not replaced.

The Docker image is likewise built without `--symlink-install` and verifies
`lidarslam-map --version` before its build tree is removed.

## Container install

GHCR is the supported prebuilt amd64 delivery path for both ROS distributions.
The moving convenience tags track `develop` and are useful for evaluation:

```bash
docker pull ghcr.io/rsasaki0109/lidar_slam_ros2:humble
docker run --rm ghcr.io/rsasaki0109/lidar_slam_ros2:humble \
  lidarslam-map --version

docker pull ghcr.io/rsasaki0109/lidar_slam_ros2:jazzy
docker run --rm ghcr.io/rsasaki0109/lidar_slam_ros2:jazzy \
  lidarslam-map --version
```

`latest` remains an alias of the Humble convenience image for compatibility.
It is not a release identifier.

Every tagged release publishes exact
`ghcr.io/rsasaki0109/lidar_slam_ros2:v<VERSION>-<distro>` images only after
the repository tag matches `VERSION`. For example:

```bash
IMAGE=ghcr.io/rsasaki0109/lidar_slam_ros2:v0.7.0-jazzy
docker pull "$IMAGE"
docker run --rm "$IMAGE" lidarslam-map --version
```

Use an exact digest for deployment or rollback. Each GitHub Release attaches
`release-image-humble.json` and `release-image-jazzy.json`; they record the
tested tag, digest, tag commit, platform, product version, and observed CLI
version:

```bash
DIGEST="$(jq -r .digest release-image-jazzy.json)"
docker run --rm \
  "ghcr.io/rsasaki0109/lidar_slam_ros2@${DIGEST}" \
  lidarslam-map --version
```

The release workflow builds from the tagged recursive checkout, publishes an
SBOM and maximum-mode BuildKit provenance with each image, creates a signed
GitHub artifact attestation, then pulls and smoke-tests the registry digest.
The GitHub Release is created only after both distro images pass. Verify the
GitHub provenance with:

```bash
gh attestation verify \
  oci://ghcr.io/rsasaki0109/lidar_slam_ros2:v0.7.0-jazzy \
  -R rsasaki0109/lidar_slam_ros2
```

The version examples illustrate the tag contract; use a tag listed on the
GitHub Releases page. Convenience tags are intentionally moving, so recording
their current digest is mandatory when they are used in evaluation evidence.

### Bind-mounted output ownership

The default demo starts as container root so it can use the prebuilt workspace
and its internal dataset cache. On Linux, pass the invoking UID and GID to
return the dedicated output mount to the host user when the demo exits:

```bash
mkdir -p "$PWD/lidarslam_output"
docker run --rm \
  -e LIDARSLAM_HOST_UID="$(id -u)" \
  -e LIDARSLAM_HOST_GID="$(id -g)" \
  -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:humble
```

Both variables are required together and must be numeric. The demo changes the
owner of the dedicated output mount, the requested run directory and any
failed `.partial` or post-processing lock sidecar; unrelated sibling contents
are not changed.

## Profile-specific extras

The flagship PointCloud2 + IMU profile is complete after the recursive source
install above. The `pointcloud_gnss_smoke` and `packet_applanix_smoke`
evaluation profiles additionally use the PyPI `rosbags` package to inspect raw
bag records. It has no Ubuntu 22.04/24.04 rosdep key, so keep it isolated in a
virtual environment:

```bash
python3 -m venv --system-site-packages ~/.venvs/lidarslam
source ~/.venvs/lidarslam/bin/activate
python3 -m pip install rosbags
source ~/ros2_ws/install/setup.bash
```

Activate that environment before invoking either GNSS profile. The workflow
fails before starting ROS processes and points back to this section when the
module is absent. This extra is not required by the default RKO-LIO path.

## Support matrix

| Delivery | Humble amd64 | Jazzy amd64 | arm64 / Jetson | Flagship RKO-LIO path |
| --- | --- | --- | --- | --- |
| Recursive source checkout + `colcon` | Tested in CI | Tested in CI | Evaluation; use the Jetson runbook | Included from the maintained submodule |
| GHCR image | Moving and versioned amd64 images | Moving and versioned amd64 images | Not yet published | Included |
| ROS buildfarm / apt | Not released | Not released | Not released | Packaging decision unresolved |

`amd64` is the tested product target. Jetson/MID-360 workflows have real-device
evidence, but arm64 installation and image publication are still an evaluation
tier rather than a release guarantee.

## Binary-release boundary

There is currently no supported
`sudo apt install ros-<distro>-lidarslam` golden path. Two packaging decisions
remain:

1. `ndt_omp_ros2`, which is a declared build dependency, must be released
   before the four core packages.
2. The flagship product profile depends at runtime on the maintained
   `rko_lio` fork. A binary product claim must either release that fork for
   Humble and Jazzy or deliberately switch the binary golden path to a
   frontend available from the same distribution.

The core packages can be prepared by bloom without declaring `rko_lio`, but
that fact only proves buildability; it does not make the installed flagship
workflow complete. See the [rosdistro release runbook](rosdistro-release.md)
for the maintainer procedure and remaining prerequisites.

Versioned Humble/Jazzy GHCR tags, release-image SBOM/provenance, digest smoke
tests, and attached installation evidence are automated for the next tagged
release. ROS buildfarm packages remain blocked by the dependency decisions
above. Upgrade testing across two released product versions and arm64 image
publication also remain Phase 2 work.
