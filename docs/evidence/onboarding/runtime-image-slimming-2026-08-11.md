# Runtime image slimming pilot — 2026-08-11

## Result

Local multi-stage Humble and Jazzy images retained the installed first-map
contract while reducing Docker's local image-size measurement by more than
half. Both candidates were built from clean commit
`08d0040c2849412a88147f825841af2b3a516262`; neither image was pushed or used
to change a public tag.

| ROS distribution | Published local baseline | Local candidate | Reduction | 25% gate ceiling | Local result |
| --- | ---: | ---: | ---: | ---: | --- |
| Humble | 1,227,397,566 B | 569,009,937 B | 658,387,629 B (53.6409%) | 920,548,174 B | `PASS` |
| Jazzy | 1,357,460,149 B | 547,466,572 B | 809,993,577 B (59.6698%) | 1,018,095,111 B | `PASS` |

These values are the `.Size` returned by `docker image inspect` for the
locally present images. The public Jazzy registry layers summed to
1,357,449,522 bytes when the baseline was captured, but the candidates have no
registry artifact. The table therefore passes the local size proxy, not the
precommitted compressed-OCI promotion measurement. A reviewed registry export
or publication candidate must measure that value separately.

The local image identities were:

| ROS distribution | Published baseline image ID | Candidate image ID |
| --- | --- | --- |
| Humble | `sha256:f1a894d81b5cb7b4e2e55a7b3fc17e538722b59c07b0bec066f2ad499a5e8447` | `sha256:6ad0e172952995ce1593e0269f7dce6b267c773bd45f3fd15082a0026ceec99f` |
| Jazzy | `sha256:7b27bdc109c25a7881a884128a91708c2a3e431e776c02b066ec7e33d04b0f1c` | `sha256:9a89b816f4bfcc3c0811a1f961f2c0f4d4d401354e5a978dc856bbdf06d9731d` |

Image IDs are local BuildKit/Docker identities, not public OCI digests.

## Source-free runtime closure

The builder installs the supported dependencies and compiles the six-package
product closure. A deterministic collector then scans installed ELF files,
resolves their shared libraries, maps external files and maintained runtime
commands to Debian package owners, and rejects unresolved or unsafe paths.
The runtime stage starts again from `ros:<distro>-ros-core`, installs that
closure, and copies only `install/` plus the fail-closed entrypoint.

| Check | Humble | Jazzy |
| --- | ---: | ---: |
| Installed ELF files scanned | 28 | 27 |
| Linked libraries resolved | 200 | 225 |
| Direct runtime packages collected | 138 | 150 |
| Direct development packages | none | none |
| Installed ELF files with missing `ldd` links | 0 | 0 |

The final images contain no checkout, `.git`, `build/`, `log/`, or
`Thirdparty/` tree. `git`, `colcon`, and `rosdep` are absent in both images.
The installed provenance record in each image reports the exact 40-character
commit, `dirty: false`, and `source: override`.

Humble retains `gcc`, `g++`, `cc`, `c++`, and `cmake`; `make`, `ninja`, and
`build-essential` are absent. These tools are automatically installed Jammy
package dependencies rather than direct collector output. For example,
`python3-scipy` depends on `python3-pythran`, which in turn depends on `g++`,
development headers, and Boost. Deleting package-owned files after installation
was rejected because it would make the package database and supported security
updates inconsistent. Jazzy has none of `gcc`, `g++`, `cc`, `c++`, `make`, or
`ninja`; its `cmake` comes from the official `ros:jazzy-ros-core` base.

## Installed-product smoke matrix

Every row was run with `--network none` against the final local candidate.

| Contract | Humble | Jazzy |
| --- | --- | --- |
| `lidarslam-map --version` and `--help` | `PASS` | `PASS` |
| `ros2 pkg prefix lidarslam` | `PASS` | `PASS` |
| `ros2 launch --help` / `ros2 bag --help` | `PASS` / `PASS` | `PASS` / `PASS` |
| `rclpy`, `rosbag2_py`, `scipy`, `jsonschema`, `rosidl_runtime_py.utilities` | 5 / 5 | 5 / 5 |
| Installed first-map/demo helper files | 4 / 4 | 4 / 4 |
| Provenance revision and clean state | `PASS` | `PASS` |
| Source and build trees absent | `PASS` | `PASS` |
| Installed ELF `ldd` closure | 28 / 28 | 27 / 27 |

The four newly required installed helpers are `run_first_map_demo.sh`,
`run_docker_demo.sh`, `download_mid360_robot_public_dataset.py`, and
`mid360_robot_public_datasets.py`. The downloader no longer imports a
source-only helper. Its optional source recording checker still fails closed
with exit code 127 and a direct recovery message when requested from the
curated runtime.

## Network-isolated 50-second map proof

The unchanged default image command was exercised with the local 50-second
[MID-360 fixture candidate](mid360-onboarding-fixture-pilot-2026-08-10.md)
mounted read-only under the public bag name. The container had no network,
the output root was initially empty, and the product ownership handoff was set
to UID/GID `1000:1000`.

Fixture identity:

- ZIP size: 98,873,952 bytes;
- ZIP SHA-256:
  `20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3`;
- source archive size: 517,088,133 bytes;
- source archive SHA-256:
  `f8f89eebf2aaf9cc1d465bfa5451bbb599cd92d079b59949104bb4e5cb619bdd`.

| Observation | Humble | Jazzy |
| --- | ---: | ---: |
| Container exit | 0 | 0 |
| Observed wall time | 17.83 s | 16.98 s |
| Manifest / diagnosis / receipt schemas | `PASS` | `PASS` |
| Manifest status / lifecycle | `succeeded` / `complete` | `succeeded` / `complete` |
| Diagnosis / Autoware verification | `success` / 8 PASS, 0 WARN, 0 FAIL | `success` / 8 PASS, 0 WARN, 0 FAIL |
| First-map receipt | 7 / 7 `PASS` | 7 / 7 `PASS` |
| Manifest-recorded artifacts re-hashed | 116 / 116 | 114 / 114 |
| Raw / corrected poses | 372 / 88 | 351 / 87 |
| Pointcloud tiles | 89 | 87 |
| Pointcloud tile bytes | 4,038,829 | 3,991,647 |
| Combined `map.pcd` bytes | 4,017,969 | 3,971,519 |
| Regular-file / allocated bytes | 8,397,019 / 8,704,000 | 8,297,206 / 8,617,984 |
| Final transaction directory | absent | absent |
| Output ownership | `1000:1000` | `1000:1000` |

The receipts bind these non-published local artifacts:

| Artifact SHA-256 | Humble | Jazzy |
| --- | --- | --- |
| Run manifest | `6f6beea91d3b720188cc8e2458a490a8884c64919a2c0db1bb277a17057fd359` | `f2a6294e6741564bd3ff76b7dae7c732f046b01404e6aedc6ea8d0178ae72a6c` |
| Diagnosis | `18fe1bea5ea985a0ac09bd039b45664d4aaed841251e1bfededc7a672d06c621` | `18fe1bea5ea985a0ac09bd039b45664d4aaed841251e1bfededc7a672d06c621` |
| Verifier log | `f8d9a43f88615dbf2421bf1c42d303457f65e338f4836a267f6958149fbbdee5` | `f8d9a43f88615dbf2421bf1c42d303457f65e338f4836a267f6958149fbbdee5` |
| First-map receipt | `e018669d1e123481d2c19dc7e34179c0b77570189cf7fe51400df7206e406375` | `4da927cd9e87aac5a1dc3fa619063e9b1c3a993471f808d72f095f035ebdfddf` |

Do not interpret the two wall times or output-count differences as a ROS
distribution performance comparison. There is one warm local run per row,
launch scheduling was not controlled, and byte-identical map output was not a
gate. The shared contract is the schema-valid, hash-bound successful map and
verifier outcome.

## Findings and promotion decision

The local size proxy, source-free runtime, installed CLI, default command,
Autoware verifier, and receipt gates pass on both supported distributions.
The pilot proves that the previous image size was not required by the product
runtime path.

It is not ready for public image replacement. Promotion remains blocked on:

1. an explicitly reviewed compressed-OCI measurement;
2. clean dedicated-VM Humble and Jazzy Docker/source rows with cold RX, wall
   time, active time, command count, and isolated peak disk;
3. a full 277-second public demo rerun from the exact candidate revision;
4. publication review for the still-local 50-second fixture;
5. repair or deliberate removal of Jazzy's non-contract `.ros_log/latest`
   symlink, which retains the absolute `.partial` path after atomic output
   finalization; and
6. a Docker cache-boundary improvement: copying the full source before
   dependency installation currently makes small source changes repeat the
   expensive builder dependency layer.

No image, fixture, map, bag, private log, or geometry-bearing result was added
to Git or pushed by this pilot. The next release decision must use the
dedicated-VM evidence rather than promote these local tags directly.

## Reproduce locally

Build from an exact clean checkout and inject the source identity explicitly:

```bash
docker buildx build --load --progress=plain \
  --build-arg ROS_DISTRO=humble \
  --build-arg LIDARSLAM_SOURCE_REVISION=<40-character-commit> \
  --build-arg LIDARSLAM_SOURCE_DIRTY=false \
  -t lidarslam-runtime-pilot:humble .

docker buildx build --load --progress=plain \
  --build-arg ROS_DISTRO=jazzy \
  --build-arg LIDARSLAM_SOURCE_REVISION=<40-character-commit> \
  --build-arg LIDARSLAM_SOURCE_DIRTY=false \
  -t lidarslam-runtime-pilot:jazzy .
```

Use the published full public bag for release proof. If the unpublished
fixture is available for a bounded local check, mount its extracted bag
read-only, disable networking, use a new output directory, and invoke the image
without replacing its default command. Keep the geometry outside the checkout
and review privacy-bounded receipts before sharing them.
