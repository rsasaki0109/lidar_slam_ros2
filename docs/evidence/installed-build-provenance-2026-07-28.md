# Installed build provenance validation — 2026-07-28

## Decision

Installed source and container products now retain the exact source revision
needed by `run_manifest.json`, even after the source and build trees are
unavailable.

The validation passed both supported ROS distributions and both supported
identity paths:

- Git discovery for a normal source checkout;
- explicit, validated build identity for a Docker context without `.git`.

## Frozen inputs

| Input | Value |
| --- | --- |
| Source revision | `dae31fdc956a9a709534a3692639ea356e4e9702` |
| Humble environment | `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:9db1a467c99d69bd3a6d8d7a71e6555874f2a0e1e6f7d062ab2297dd7828c061` |
| Jazzy environment | `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:7b27bdc109c25a7881a884128a91708c2a3e431e776c02b066ec7e33d04b0f1c` |
| Build shape | `colcon build --merge-install`, without `--symlink-install` |
| Package rebuilt | `lidarslam` against the installed product-image dependencies |

The Humble run exposed the clean Git worktree and used no provenance
override. The Jazzy run mounted the source without usable Git metadata and
passed:

```text
-DLIDARSLAM_SOURCE_REVISION:STRING=dae31fdc956a9a709534a3692639ea356e4e9702
-DLIDARSLAM_SOURCE_DIRTY:STRING=false
```

## Results

| Distribution | Identity source | Package build | Installed validation | Installed metadata |
| --- | --- | ---: | --- | --- |
| Humble | Git checkout | 18.2 s | Pass | revision matched, `dirty=false`, `source=git` |
| Jazzy | Explicit Docker-style override | 19.3 s | Pass | revision matched, `dirty=false`, `source=override` |

The installed validator loaded the installed
`run_autoware_map_from_bag.py` and confirmed that `_software_identity()`
returned the same revision and dirty state as
`share/lidarslam/product/product-build-info.json`.

## Negative checks

Automated tests also proved:

- tracked edits set `dirty=true`;
- untracked files do not set dirty state, matching the existing manifest
  policy;
- a Git-free archive without an override records an explicit unknown state;
- official clean-prefix validation rejects that unknown identity;
- invalid or non-40-character overrides are rejected;
- a dirty override without a revision is rejected;
- malformed or semantically invalid installed metadata is not trusted by the
  runtime;
- generating the same identity twice does not rewrite the deterministic JSON
  file.

The initial Git-free container build without overrides failed exactly at the
new incomplete-provenance gate. The same build passed after supplying the
validated revision and dirty state.

## CI enforcement

Humble/Jazzy Docker and release workflows now:

1. resolve the checked-out commit;
2. pass that commit and `dirty=false` into the Docker build;
3. read the installed metadata from the built or published image;
4. fail if its revision differs from the checkout;
5. record the observed installed revision in release-image evidence.

The default source CI passes its checkout revision to the clean-prefix
validator, which also checks that the installed runner consumes the metadata.

## Limitations

- This validation rebuilt the changed package against published-image
  dependencies; pull-request CI remains authoritative for the full repository
  matrix and full Docker rebuild.
- It did not rerun the full real-data map because installed runtime identity
  consumption is checked directly and the manifest serialization contract is
  already covered by runner tests.
- Jazzy emitted existing CMake/PCL developer warnings; they did not change the
  build or validation result.
- This maintainer-run validation is not an independent-user first-map report.
