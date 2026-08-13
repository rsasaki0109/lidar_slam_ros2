# Getting Started

Use this page when you are new to `lidarslam_ros2` and want the shortest path
to a working map.

## Choose A Path

| You have | Run |
| --- | --- |
| An installation, but you are unsure where to begin | `lidarslam-map` |
| An installation you want to verify before choosing a bag | `lidarslam-map doctor` |
| Docker only, no ROS 2 workspace | Follow [Docker First Map](#docker-first-map-no-ros-2-workspace) below |
| Docker and your own compatible bag | `bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2` |
| A cloned source repository | `bash scripts/source_quickstart.sh` |
| A compatible sensor bag | `lidarslam-map start /path/to/rosbag2` |
| An RKO-LIO map you may clean up later | `lidarslam-map start /path/to/rosbag2 --editable` |
| Two or more overlapping completed maps | `lidarslam-map merge output/day1 output/day2 --output-dir output/site_project` |
| A new mounting with measured transforms | `lidarslam-map start /path/to/rosbag2 --lidar-to-base ... --imu-to-base ...` |
| A rosbag2 directory and a built workspace | `lidarslam-map run /path/to/rosbag2 --output-dir "$PWD/output/my_map"` |
| A bag, but you are not sure which topics it has | `lidarslam-map doctor /path/to/rosbag2` |
| You want the fixed public first-map demo | `lidarslam-map demo` |

With no arguments on an interactive terminal, `lidarslam-map` asks whether you
want to check the installation, run the fixed demo, map your own rosbag2, or
return to previous sessions. It displays the exact command before delegating to
an existing workflow. The installation check needs no confirmation because it
uses no network and writes no files. The demo cannot download or write until
you answer `yes`; own-bag mapping still requires the normal sensor and
calibration review. In a script, pipe, or other non-interactive context, no
arguments retain the stable usage error and exit code `2`.

### Check The Installation Before Choosing A Bag

```bash
lidarslam-map doctor
```

Without a bag, `doctor` checks the curated product helpers, matching source or
installed prefix, Humble/Jazzy environment, `ros2`, `rosbag2_py`, and free space
for the fixed demo. It returns stable finding codes and one copy-ready next
action for every missing requirement. It does not contact the network or write
a cache, report, or output file. Use `--json` for the versioned
`system-doctor-v1` report, `--demo-dir <dir>` to check another filesystem, or
`--min-free-space-gib <GiB>` to raise the default 8 GiB floor.

Provide a bag to retain the existing input inspection:

```bash
lidarslam-map doctor /path/to/rosbag2
```

That mode checks topics, point fields, timestamp order, and maintained profile
compatibility. Storage options are system-only and are rejected when a bag is
provided rather than being silently ignored.

## Docker First Map (No ROS 2 Workspace)

```bash
mkdir -p "$PWD/lidarslam_output"
docker run --rm \
  -e LIDARSLAM_HOST_UID="$(id -u)" \
  -e LIDARSLAM_HOST_GID="$(id -g)" \
  -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:humble
```

The first run downloads the tracked 517 MB MID-360 bag and prints periodic
byte, percentage and transfer-rate updates. The map is written to
`lidarslam_output/mid360_demo`. On Linux, the two ownership variables make the
container return the output directory to your user even if the run fails.
Omit them on platforms where Docker already maps bind-mount ownership.

The Docker image invokes the same `scripts/run_first_map_demo.sh` implementation
used by a sourced source workspace. Both paths use the fixed MID-360 dataset,
the `rko_lio_graph_mid360_preset`, and the same manifest, verifier, diagnosis,
and first-map receipt artifacts.
On Ubuntu 24.04, replace `:humble` with `:jazzy`; the entrypoint and first-map
contract are unchanged.

### Docker Own-Bag Map

To inspect and map your own bag without building a ROS 2 workspace, run one
host launcher from the cloned repository:

```bash
bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2
```

Replace only `/absolute/path/to/rosbag2`; it must be the directory containing
`metadata.yaml`. The launcher validates paths before Docker, mounts the bag
read-only, disables external container networking, runs as your UID/GID, and
sends setup, map, ROS logs, and the offline session page to a new
`lidarslam_output/<bag>-map/` directory. It delegates to the same
`lidarslam-map start` sensor-review and verified-map path as an installation;
the old lower-level `run --guided` Docker detour is not used.
Before creating that directory, it asks the selected image for `start --help`
and resolves the passing image to its local immutable image ID. An older image
is rejected as `[image-contract-missing]` instead of failing after writes begin.

Preview the exact `docker run` array without contacting Docker or creating the
output directory:

```bash
bash scripts/docker_map_bag.sh --dry-run /absolute/path/to/rosbag2
```

Use `--ros-distro jazzy` for the Jazzy image or `--image <tag-or-digest>` for
a reviewed immutable image. Product options go after `--`, for example:

```bash
bash scripts/docker_map_bag.sh /absolute/path/to/rosbag2 -- --editable
```

Interactive mode asks for sensor and calibration confirmation. For a reviewed
non-interactive run, append `-- --yes` plus either
`--accept-profile-extrinsics` or both measured transform options when the
selected RKO-LIO profile requires them. The launcher owns the container output
and viewer options so forwarded arguments cannot escape the writable mount;
the offline mapping run cannot contact external network services.
Existing or overlapping outputs are refused rather than overwritten. A
rejection prints a stable code such as `[metadata-missing]`,
`[imu-input-missing]`, or `[confirmation-required]` and a concrete `Next:` action before mapping.
The host launcher currently supports Linux x86_64, matching the published
image tier.

### Clone-free Docker launcher release gate

The next stable release after v0.9.0 is prepared to attach the same host
launcher directly as `lidarslam-map-docker`. That asset will need no repository
checkout: it contains no repository-relative imports and pins its default
Humble/Jazzy image to the matching immutable `v<VERSION>-<distro>` tag.
Do not use a guessed download URL before the asset appears on a finalized
release and passes the published-release audit. Until then, use the reviewed
repository command above.

Once a named release contains the asset, the verified route is:

```bash
RELEASE=v0.9.1  # replace only with a finalized release that has this asset
curl --fail --location --proto '=https' --tlsv1.2 \
  "https://github.com/rsasaki0109/lidar_slam_ros2/releases/download/${RELEASE}/lidarslam-map-docker" \
  --output lidarslam-map-docker
gh attestation verify ./lidarslam-map-docker \
  --repo rsasaki0109/lidar_slam_ros2
chmod 0755 lidarslam-map-docker
./lidarslam-map-docker --version
./lidarslam-map-docker /absolute/path/to/rosbag2
```

The printed launcher version and revision must match that release. The
launcher resolves the versioned image to its immutable local image ID before
creating output, then keeps the bag read-only and container networking off.

## 1. Install And Build From Source

Install ROS 2 Humble on Ubuntu 22.04 or ROS 2 Jazzy on Ubuntu 24.04 first. The
quickstart detects and sources the matching `/opt/ros` setup inside its own
process; you do not need to source it before this command.

Clone, then run one helper from the repository:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git
cd lidar_slam_ros2
bash scripts/source_quickstart.sh
```

The helper initializes missing pinned submodules, installs `rosdep` or `colcon`
when absent, resolves only this repository's dependencies, and builds only its
6 ROS packages with `BUILD_TESTING=OFF`. It may ask for your sudo password. It
does not install ROS itself, pipe a remote script into a shell, or discover and
build unrelated packages elsewhere in the workspace. Its final stage runs the
fixed verified first-map demo below.

Before dependency installation or compilation, it asks `colcon` for the source
inventory and requires exactly `graph_based_slam`, `lidarslam`,
`lidarslam_msgs`, `ndt_omp_ros2`, `rko_lio`, and `scanmatcher`. The build passes
that same explicit list to `--packages-select`. An omitted package or an
unexpected experimental package fails as `[source-package-inventory-mismatch]`
instead of silently expanding or weakening the beginner build.

Inspect the resolved ROS distribution, paths, missing tools, and exact commands
without network, APT, submodule checkout, build, or filesystem writes:

```bash
bash scripts/source_quickstart.sh --dry-run
```

Use `--viewer none` on a headless machine or `--build-only` to install without
downloading and mapping the public demo. Every stage is idempotent; a failed
build prints the same copy-ready quickstart command for retry. On success, the
helper prints the exact absolute installed `lidarslam-map` path. Run that path
from any directory in a fresh terminal: its launcher activates only the
matching workspace for its child process, without changing your shell or
requiring a remembered `source` step. Sourcing `~/ros2_ws/install/setup.bash`
remains available when you want the short command name and other ROS tools.
The lower-level dependency and full test commands remain in
[Operator Workflows](workflows.md).

## 2. Run the Fixed First-Map Demo

The default source quickstart runs this demo automatically. To repeat it later,
activate the completed workspace and use its root as the data/output directory:

```bash
source ~/ros2_ws/install/setup.bash
lidarslam-map demo ~/ros2_ws
```

This downloads the same fixed 517 MB MID-360 bag used by Docker and writes
`~/ros2_ws/output/mid360_demo/`. The command calls `lidarslam-map run` with
`rko_lio_graph_mid360_preset` and prints the paths of the versioned manifest,
verifier log, diagnosis, and privacy-bounded first-map receipt. It then opens
the offline browser review; use `--viewer none` on a headless machine. The
source and Docker first-map paths therefore have one fixed dataset and one
output contract.

Every live run re-hashes the registered ZIP and the two extracted rosbag2
files before mapping. A dry-run reports an existing cache as
`prepared_unverified`; verification is deliberately deferred until execution
so planning remains fast, network-free, and read-only.

The terminal reports three named demo stages and never invents a percentage or
ETA. If mapping finished but verification, finalization, diagnosis, or receipt
generation was interrupted, the next normal invocation prints a copy-ready
recovery command:

```bash
lidarslam-map demo ~/ros2_ws --resume --viewer none
```

`--resume` never starts mapping again. It is accepted only for a schema-valid
terminal `run_manifest.json`; a running, missing, ambiguous, or malformed
state is rejected and the retained evidence is left untouched.

Inspect the exact paths, cache state, 8 GiB free-space gate, dataset identity,
license, and steps without creating a directory or making a network request:

```bash
lidarslam-map demo ~/ros2_ws --viewer none --dry-run --json
```

The JSON follows the
[`first-map-demo-plan-v1` schema](schemas/first-map-demo-plan-v1.schema.json).
If `output/mid360_demo` already has receipt-bound PASS evidence, a repeat
command reuses it instead of running SLAM again. Existing partial, symlinked,
overlapping, or unverified output fails with a concrete next action. The
installed command delegates map construction to the same compatibility
implementation available as `bash scripts/run_first_map_demo.sh`; it does not
introduce another estimator path.
Allow at least 8 GiB free and budget roughly 30 minutes for a first source build
and map on a modern desktop with a fast connection. The download is
network-dependent, so slower links may take 30 minutes or more.

## 3. Run Your Own Bag

For a normal first map, use one command:

```bash
lidarslam-map start /path/to/rosbag2
```

`start` selects one of the four maintained input workflows and reports every
topic it will consume. It supports PointCloud2+Imu, PointCloud2+NavSatFix, and
VelodyneScan+Applanix GSOF49 bags through the same command. For RKO-LIO it also
reports both sensor-to-base transforms and writes nothing until you confirm
that the tracked extrinsics match the physical robot. After confirmation it
saves the setup, runs the normal atomic map lifecycle, requires verification,
and opens the self-contained browser view. Use `--viewer none` on a headless
host.

The session directory contains `sensor_setup.json`, pinned parameter snapshots,
and the completed map under `map/`. To inspect the full decision without writing:

```bash
lidarslam-map start /path/to/rosbag2 --yes --dry-run --json
```

If no maintained workflow is safe, `start` exits with code `2`, writes no
session, and prints `Sensor session: NOT READY`. Each finding has a stable code
such as `[range-input-missing]` and a copy-ready `Next:` action using the actual
bag path. The JSON form uses the versioned
[`sensor-setup-rejection-v1` schema](schemas/sensor-setup-rejection-v1.schema.json),
so tools should key on `reason.code` and `findings[].code` rather than parsing
English messages.

If mapping starts but later fails, `start` keeps the setup and every available
run artifact, then prints `Map session: ACTION REQUIRED`. The first line carries
a stable code such as `[storage-exhausted]`, `[workflow-interrupted]`, or
`[map-verification-failed]`; every finding includes a command that can be pasted
directly. The same handoff is saved as `map_session_recovery.json` in the setup
directory under the versioned
[`map-session-recovery-v1` schema](schemas/map-session-recovery-v1.schema.json).
If only terminal post-processing remains, `next_command` is the exact safe
`--resume` command. Otherwise, the receipt keeps an exact diagnosis command and
a retry of the same pinned setup into a fresh output directory. It never tells
you to overwrite the failed run.

Every started mapping attempt also writes `session.json` and the same
self-contained `session.html` landing page in the setup directory, whether its
state is `running`, `verified`, `unverified`, or `action_required`. In browser
mode, the running page opens before the map runner and refreshes every two
seconds. It advances only from atomically written run-manifest stages: preparing,
mapping, verification, finalization, and evidence generation. The contract uses
the versioned
[`map-session-index-v1` schema](schemas/map-session-index-v1.schema.json).
For each terminal state, `actions[0]` is the recommended copy-ready next action.
A verified session links its offline 3D preview and evidence; an unverified
diagnostic run is clearly labelled and offers a fresh verification-enabled
output; a failed session presents the detailed recovery receipt without trying
to display a bad map. `--viewer none` retains the live and final artifacts
without opening them. The page loads no network resources. Generation,
monitoring, or opening failure never hides authoritative JSON or changes the
map-run result.

The same page includes a four-part quality summary for workflow completion,
map output, Autoware verification, and evidence integrity. It derives these
cards from the seven checks in a schema-valid
`first_map_validation_receipt.json` and shows the source check IDs. It does not
invent a numeric quality score. Verification-off is shown as `NOT VERIFIED`,
not as a failed map; missing, malformed, or incomplete receipts are shown as
`UNAVAILABLE` instead of being guessed from nearby files.

After a verified run, the session page offers **Share this verified first
map**. Its copy-ready command revalidates the retained receipt and source
evidence, then prints the exact summary, JSON attachment, and public issue form
without writing or contacting GitHub:

```bash
lidarslam-map support /path/to/session_bundle --first-map
```

Review the named JSON before attaching it. The handoff never uploads anything
and explicitly excludes maps, bags, manifests, logs, trajectories, parameters,
and private-place screenshots.

Return to recent work without searching through timestamped directories:

```bash
lidarslam-map sessions
```

This scans only direct child bundles under `./output`, validates each
`session.json`, opens a local `sessions.html` catalog, and links each available
session page. Use `--status action_required` to focus on interrupted work,
`--viewer none` on a headless host, or `--json` for a read-only machine catalog.
An explicit alternate root is accepted as the optional positional argument.
The scanner does not recurse or follow symlinks, skips malformed or oversized
session records, and caps display at 200 entries.

Select any two cards in `sessions.html` and copy the generated command, or run:

```bash
lidarslam-map compare output/session-a output/session-b
```

The self-contained comparison page puts readiness, evidence-backed quality,
verification, sensor setup, and retained artifact names side by side. It reports
each row as `same`, `different`, or `unavailable`; it does not invent a score or
choose a winner. Missing, stale, malformed, or identity-mismatched setup
evidence stays unavailable. Use `--viewer none` on a headless host or `--json`
for a read-only machine report.

If a session needs maintainer help, choose **Get support** on its history card
and copy the command, or run:

```bash
lidarslam-map support output/session-a
```

This creates a new ZIP beside the session with exactly `README.txt`,
`issue-body.md`, and schema-valid `support-report.json`. It includes bounded
status, setup, diagnosis and evidence-hash fields, but excludes maps, bags, raw
logs, parameter contents, exact local paths and command credentials. Nothing is
uploaded. Review all three files before attaching the ZIP to a public issue.
Use `--json` to inspect the sanitized report without writing a ZIP.

For an RKO-LIO profile, add `--editable` when you want to retain deterministic
backend input for later loop-constraint repair.

`--yes` is explicit non-terminal setup confirmation. For a new physical
RKO-LIO mounting, provide both measured transforms in
`qx,qy,qz,qw,x,y,z` order:

```bash
lidarslam-map start /path/to/rosbag2 \
  --lidar-to-base 0,0,0,1,0.10,0,0.20 \
  --imu-to-base 0,0,0,1,0,0,0
```

Use `setup` alone when you only want a reusable configuration bundle, without
starting a map:

```bash
lidarslam-map setup /path/to/rosbag2
```

The GNSS and packet workflows pin their GNSS or GSOF inputs in
`sensor_setup.json`. RKO-only transform/frame options and `--editable` are
rejected on those profiles instead of being silently ignored. These two paths
remain evaluation-tier hardware workflows; a successful topic match is not a
universal sensor-calibration guarantee.

The lower-level `run` and compatibility `run --guided` paths remain available
for scripts and existing launchers.

```bash
mkdir -p "$PWD/output"
lidarslam-map run /path/to/rosbag2 \
  --output-dir "$PWD/output/my_map" \
  --dry-run
lidarslam-map run /path/to/rosbag2 \
  --output-dir "$PWD/output/my_map" \
  --editable
```

The dry run prints the selected public workflow before anything starts. For an
RKO-LIO graph profile, `--editable` also retains `backend_input/` and the exact
`graph_params.ros.yaml` needed to revisit accepted loop constraints. This is
opt-in because the replay bag can use substantial disk space.

## 4. Check The Result

Successful runs should leave these files:

- `pointcloud_map/`
- `pointcloud_map/pointcloud_map_metadata.yaml`
- `map_projector_info.yaml`
- `verify_autoware_map.log`
- `run_manifest.json`
- `autoware_map_diagnosis.json`
- `autoware_map_diagnosis.md`
- `first_map_validation_receipt.json`
- `first_map_validation_receipt.md`

The first-map receipt contains a copy-ready verification summary without map
geometry or private paths. At the end of a run, the CLI prints the reviewable
JSON receipt path and a direct link to the Independent First-map Validation
issue form. Both passing and failing reports improve the onboarding path; see
[Independent First-map Validation](external-first-map-validation.md) for the
privacy and acceptance rules.

Map generation and viewing have separate exit codes. After a successful run,
open the lightweight, self-contained 3D browser preview:

```bash
lidarslam-map view "$PWD/output/my_map"
```

The command creates `preview/mid360_robot_3d_map_preview.html` and opens it
when a desktop session is available. On a headless machine it prints the exact
HTML path and still succeeds; add `--no-open` to request that behavior
explicitly. The heavier live viewers remain available with `--viewer
foxglove` or `--viewer autoware`.

The browser review also supports non-destructive map cleanup:

1. Choose **Select 2 corners**, select the unwanted XY region, adjust the XYZ
   bounds, and add the removal.
2. Optionally uncheck an accepted loop constraint.
3. Download the edit plan and apply the command printed in the review:

```bash
lidarslam-map edit "$PWD/output/my_map" \
  --plan "$HOME/Downloads/map-edit-plan-<timestamp>.json" \
  --output-dir "$PWD/output/my_map_edited"
lidarslam-map view "$PWD/output/my_map_edited"
```

The source identity is pinned by SHA-256, the destination must not already
exist, PCD fields such as intensity are preserved for region removal, and the
candidate must pass both the map-bundle and Autoware pointcloud-map verifiers.
For a map created with `run --editable`, disabling an accepted loop uses the
retained backend input and graph parameters automatically from the same output;
the one-line command printed by the browser is sufficient after sourcing the
ROS environment. `edit --help-all` exposes path overrides for older outputs.
Without valid replay input the edit stops instead of presenting a stale pose
graph as a corrected map. Replay summaries and logs are retained in the new
candidate.

To combine repeat visits, put the most trusted map first as the anchor:

```bash
lidarslam-map merge \
  "$PWD/output/day1" \
  "$PWD/output/day2" \
  --output-dir "$PWD/output/site_project"
lidarslam-map view "$PWD/output/site_project"
```

`merge` validates every source and requires matching PCD fields, tile
resolution, frame, and projector origin. Low overlap fails before output
creation. Successful projects retain each transformed trajectory separately,
deduplicate overlapping points, preserve intensity and other PCD fields, and
publish SHA-256-pinned `map_project.json` and `map_merge_receipt.json`.

Or inspect an existing output directory:

```bash
lidarslam-map inspect output/<run_dir> --write
```

## Common First-Run Problems

| Symptom | Next check |
| --- | --- |
| `metadata.yaml not found` | Pass the rosbag2 directory, not a `.db3` file. |
| No compatible path is recommended | Follow the stable `[finding-code]` and its `Next:` action, then rerun `lidarslam-map doctor /path/to/rosbag2`. |
| Map verification fails | Open `verify_autoware_map.log` and `autoware_map_diagnosis.md` in the output directory. |
| Browser does not open on a headless machine | Open the printed self-contained HTML file on the host, or rerun with `--no-open` and `--preview-dir <dir>`. |
| Live viewer starts but no map appears | Confirm the run produced `pointcloud_map/`; verify the browser preview first, then try Foxglove before the full Autoware viewer. |

### Empty map or viewer: three-check recovery

If this is your first run, use the fixed public demo as the control experiment
before changing an own-bag topic or frame:

```bash
lidarslam-map demo ~/ros2_ws --viewer none
```

For a live run, replace every angle-bracket placeholder before running the
command. Choose `<POINTCLOUD_TOPIC>` from a topic listed as
`sensor_msgs/msg/PointCloud2` by `ros2 topic list -t`.

1. **Live PointCloud2 input**

   ```bash
   ros2 topic hz --window 5 <POINTCLOUD_TOPIC>
   ```

   Expected: `average rate:` continues to report a positive rate. If it stays
   at zero or reports no publisher, fix the input publisher or launch remap,
   then repeat this check. For an own rosbag2 directory, run
   `lidarslam-map doctor /path/to/rosbag2` before retrying.

2. **Non-empty sampled `frame_id`**

   ```bash
   timeout 5s ros2 topic echo --once --field header.frame_id <POINTCLOUD_TOPIC>
   ```

   Expected: one non-empty frame name, such as `livox_frame`. If the output is
   empty or times out, fix the publisher's `header.frame_id` and repeat the
   input check; do not guess a frame name in the viewer.

3. **Connected TF path**

   ```bash
   ros2 run tf2_ros tf2_echo <TF_TARGET_FRAME> <POINTCLOUD_FRAME>
   ```

   Replace `<TF_TARGET_FRAME>` with the runtime/viewer's target frame and
   `<POINTCLOUD_FRAME>` with the sampled frame from check 2. Expected: a
   repeated `At time ...` transform. If `tf2_echo` reports that the transform
   is unavailable, publish or correct that TF/static extrinsic, then rerun the
   check with the exact source and target frames.

These checks separate an input/TF failure from a viewer-only failure. If the
checks pass but no map message is produced, inspect the run diagnosis rather
than changing viewer settings:

```bash
timeout 5s ros2 topic echo --once /map/pointcloud_map
lidarslam-map inspect /path/to/output --write
```

No message means the map workflow still needs attention; open the generated
`autoware_map_diagnosis.md` and follow its first actionable finding. A message
means the map exists and the blank screen is a viewer configuration issue: set
the viewer fixed frame to `map` and select `/map/pointcloud_map`, then verify
the self-contained browser preview. The doctor, diagnosis, and preview stay
local; no map, bag, or raw log upload is required.

For the full operator reference, continue with
[Distribution and installed CLI](distribution.md),
[Autoware Quickstart](autoware-quickstart.md) and
[Operator Workflows](workflows.md).
