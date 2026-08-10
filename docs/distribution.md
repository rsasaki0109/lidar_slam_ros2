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
lidarslam-map run /path/to/rosbag2 --guided
lidarslam-map doctor /path/to/rosbag2
lidarslam-map run /path/to/rosbag2 --output-dir "$PWD/output/my_map"
lidarslam-map inspect "$PWD/output/my_map"
lidarslam-map view "$PWD/output/my_map" --viewer foxglove  # optional
```

Enable command and option completion in Bash:

```bash
source "$(ros2 pkg prefix lidarslam)/share/lidarslam/product/completions/lidarslam-map.bash"
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

Both spellings dispatch the same `doctor`, `run` (including `--guided`),
`inspect`, and optional post-run `view` contract. The `ros2 run` form is a
compatibility shim, not a fourth product workflow. Inside a source checkout,
`./scripts/lidarslam` exposes the same contract.

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
also runs `--version`, `doctor`, an own-bag dry run, `inspect`, and the
non-launching `view` validation path, and confirms that
`ros2 run lidarslam lidarslam` was not replaced.

The Docker image is likewise built without `--symlink-install` and verifies
`lidarslam-map --version` before its build tree is removed.

## Source install upgrade contract

The separate install-upgrade gate starts with the immutable `v0.6.0`
`lidarslam` package, installs the candidate into that same non-symlinked merge
prefix, and compares it with a fresh candidate prefix:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 scripts/check_install_upgrade.py \
  --baseline-ref v0.6.0 \
  --evidence-dir output/install-upgrade/$ROS_DISTRO \
  --hardware-label my-amd64-$ROS_DISTRO-host
```

The gate rejects stale or missing package-owned paths, executable-bit drift,
and changed text resources after normalizing the install prefix. It then runs
the complete installed-CLI checker against upgraded and fresh prefixes,
including the historical ROS node. Binary content hashes are not compared
because independent build directories can change compiler build IDs.

The named Humble/Jazzy execution and machine-readable reports are in
[clean-prefix upgrade evidence](evidence/install-upgrade-2026-07-28.md).
This covers source-built prefix upgrades; Debian/ROS buildfarm package-manager
upgrades remain a separate release boundary.

## ROS apt install and upgrade gate

The manual `package-manager install and upgrade` workflow exercises that
separate boundary on clean Humble and Jazzy containers. It installs the exact
four product Debian versions, then verifies:

- all four packages have the requested upstream version;
- `ndt_omp_ros2 >= 0.1.0` and `rko_lio >= 0.3.2` resolved from ROS apt;
- every required command, runtime resource, schema, completion file, and
  compatibility shim passes the installed CLI contract under
  `/opt/ros/<distro>`;
- an in-place upgrade starts from a passing older main-channel installation,
  confirms its installed CLI identity, increases the product version, and
  leaves no path whose package ownership was removed;
- the package-manager-installed CLI produces and validates the pinned public
  MID-360 map under the same bounded real-data contract as the other
  distribution gates.

The read-only verifier is
`scripts/check_package_manager_install.py`; its report follows
[`package-manager-install-v1.schema.json`](schemas/package-manager-install-v1.schema.json).
It queries `dpkg`, package ownership, and the installed prefix, but never runs
`apt`. Package installation is isolated to the disposable workflow container.

The workflow is manual while no lidarslam buildfarm package exists, so normal
CI does not create a permanently failing network gate. Run clean-install
against ROS testing as soon as the candidate builds. While the older release
is still in main, run the main-to-testing upgrade before the next sync removes
that two-version window. Finally rerun clean-install against main after sync.
Only the last two named-distro artifacts together prove the normal apt path;
the implemented workflow alone is not evidence that unpublished packages
exist. Each dispatch exposes its exact source ref, product version, apt
channel, and mode in the immutable workflow run name. After the main-channel
run completes, verify its public identity and both matrix jobs without
installing packages locally:

```bash
python3 scripts/check_package_manager_release_readiness.py \
  --version "$(tr -d '\n' < VERSION)" \
  --require-ready
```

The resulting JSON contract is
[`package-manager-release-readiness-v1.schema.json`](schemas/package-manager-release-readiness-v1.schema.json).
GitHub API errors report `BLOCKED`, while absence of the exact successful run
reports `NOT_RUN`; neither state can satisfy live v1 readiness.

Before dispatching the expensive real-data workflow, inspect the two binary
dependencies in disposable Humble and Jazzy containers:

```bash
python3 scripts/check_ros_apt_dependency_readiness.py --require testing
```

The checker reads both public `main` and `testing` channels, requires
`ndt_omp_ros2 >= 0.1.0` and `rko_lio >= 0.3.2` in both named distributions,
and writes a
[`ros-apt-dependency-readiness-v1`](schemas/ros-apt-dependency-readiness-v1.schema.json)
report. `IN_PROGRESS` means at least one testing binary is not ready;
`TESTING_READY` authorizes the testing-channel package-manager E2E;
`MAIN_READY` means the dependency half of the normal-channel gate has synced.
Docker, network, apt, or schema failures are `BLOCKED`, never
`not-published`. This preflight does not replace the installed product E2E:
it prevents a workflow dispatch that is guaranteed to fail before installation.
The
[2026-07-31 public-channel snapshot](evidence/ros-apt-dependency-readiness-2026-07-31.json)
records RKO-LIO 0.3.2 as ready in both testing channels while both
`ndt_omp_ros2` testing binaries remain unpublished.

## Installed source identity

Every official install includes
`share/lidarslam/product/product-build-info.json`. The map runner uses this
file to populate `software.git_commit` and `software.git_dirty` in
`run_manifest.json`, even when the installed command runs outside its source
checkout.

A normal Git clone records its current 40-character commit and tracked dirty
state at build time. Untracked files are excluded, matching the runtime
manifest policy. The file contains no build timestamp or host path, so two
builds with the same source identity produce identical metadata.

Docker excludes `.git` from its build context. The Humble/Jazzy image
workflows therefore pass the checked-out revision and `dirty=false` explicitly
and reject an image whose installed revision differs from the checkout.
Packagers building from a Git-free source archive must provide the same
identity:

```bash
colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DLIDARSLAM_SOURCE_REVISION:STRING=<40-character-commit> \
  -DLIDARSLAM_SOURCE_DIRTY:STRING=false
```

Without Git metadata or an explicit revision, the generated file records
`revision: null` and `dirty: null` rather than inventing provenance. Official
clean-prefix checks reject that incomplete identity.

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
IMAGE=ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-jazzy
docker pull "$IMAGE"
docker run --rm "$IMAGE" lidarslam-map --version
```

Use an exact digest for deployment or rollback. Each GitHub Release attaches
`release-image-humble.json` and `release-image-jazzy.json`; they record the
tested tag, digest, tag commit, platform, product version, and observed CLI
version. The release workflow also rejects a runtime whose installed source
revision differs from that tag commit. It attaches the schema-validated
rollback plan generated from each record:

```bash
lidarslam-map rollback-plan release-image-jazzy.json
```

Run the printed pull, `gh attestation verify`, and CLI smoke commands, then
substitute the printed `ghcr.io/...@sha256:...` reference in the normal Docker
invocation. Keep the previous release-image JSON with deployment evidence so
the same last-known-good digest remains recoverable. Never implement rollback
by retagging an old image or moving a convenience/versioned tag; map outputs
are likewise immutable, so a post-rollback mapping run uses a new output
directory.

The release workflow builds from the tagged recursive checkout and initially
pushes each candidate by digest without a version tag. It requires the
installed CLI version and source-revision label to match, verifies the OCI
SBOM, maximum-mode BuildKit provenance, and signed GitHub attestation, and
then preflights the Humble/Jazzy pair together. Only tested digests are
promoted to version tags. A matching tag is safely reused during recovery; a
tag that already resolves to a different digest aborts the release instead of
moving it. The GitHub Release is created only after both distro images pass
and immutable promotion succeeds.

The deterministic release bundle embeds
`release-bundle-manifest-v1.json`, which records the tag commit and SHA-256 of
every included product document, evidence record, schema, contract, and media
file. `release-promotion.json` records which version tags were created or
reused and always declares `moving_tag_mutated: false`. Verify the GitHub
provenance with:

```bash
gh attestation verify \
  oci://ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:<digest> \
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
| ROS buildfarm / apt | Not released | Not released | Not released | Official RKO-LIO 0.3.2 passed the Humble/Jazzy product gate |

`amd64` is the tested product target. Jetson/MID-360 workflows have real-device
evidence, but arm64 installation and image publication are still an evaluation
tier rather than a release guarantee.

## Binary-release boundary

There is currently no supported
`sudo apt install ros-<distro>-lidarslam` golden path. Two packaging gates
remain:

1. `ndt_omp_ros2`, which is a declared build dependency, is release-ready at
   pinned commit `8b77fa5` but must still be tagged, Bloom-released, and
   accepted into rosdistro before the four core packages.
2. Official PRBonn `rko_lio 0.3.2-1` passed the clean installed golden-path
   E2E on Humble and Jazzy. `lidarslam` now declares
   `rko_lio >= 0.3.2`, but that version must sync from ROS testing to main
   before normal apt dependency resolution can succeed.

See the [official binary evidence](evidence/official-rko-binary-compatibility-2026-07-29.md)
and [rosdistro release runbook](rosdistro-release.md) for the verified
boundary and remaining maintainer prerequisites.

Versioned Humble/Jazzy GHCR tags, release-image SBOM/provenance, digest smoke
tests, and attached installation evidence are automated for the next tagged
release. ROS buildfarm packages remain blocked by the NDT release and RKO-LIO
main-sync gates above. The binary package-manager clean-install/upgrade gate
is implemented but cannot produce release evidence until those packages
exist; arm64 image publication also remains Phase 2 work.
