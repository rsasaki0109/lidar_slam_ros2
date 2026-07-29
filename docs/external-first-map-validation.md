# Independent First-map Validation

`lidarslam_ros2` does not call the v1.0 onboarding path ready until at least
three independent users generate and verify a first map from the public
documentation. Maintainer-operated demos, CI jobs, and users guided live
through each step do not count.

The current accepted count is recorded in the
[machine-readable validation ledger](evidence/external-first-map-validations.json).
An empty ledger is an honest `0 / 3`, not missing evidence.

## Participate

1. Choose one official path without private maintainer instructions:

   - [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace)
   - [source quickstart](getting-started.md#1-build-the-workspace)
   - [own-bag golden path](golden-path-cli.md)

2. Record the release tag, commit, or immutable image digest and the exact
   command you ran.
3. Keep `run_manifest.json`, `autoware_map_diagnosis.json`, and
   `verify_autoware_map.log`. Compute the manifest identity:

   ```bash
   sha256sum /path/to/output/run_manifest.json
   ```

4. Open the
   [Independent First-map Validation issue form](https://github.com/rsasaki0109/lidar_slam_ros2/issues/new?template=first-map-validation.yml).

Both passing and failing reports are useful. A failed attempt is an onboarding
finding, not an accepted validation, and must be resolved or explicitly
documented before v1.0.

!!! warning "Do not publish map geometry"

    Remove credentials, private paths, precise locations, rosbag payloads,
    point-cloud tiles, trajectories, and screenshots that reveal private
    places. The issue form asks only for command, environment, status,
    verifier summary, and a manifest checksum.

## Acceptance contract

A report counts only when all of these are true:

- the reporter self-attests that they are not a maintainer and did not receive
  live step-by-step help;
- the run started from one of the three public documentation paths;
- the report identifies the exact release, revision, or image digest;
- `run_manifest.json` says `succeeded`;
- the diagnosis status is `success`;
- Autoware map verification is `PASS`;
- a maintainer reviews the public issue and records its review link;
- every finding is either resolved or explicitly documented with a public
  resolution link;
- the reporter, issue URL, validation ID, and manifest SHA-256 are unique
  across the ledger.

The tracked
[`external-first-map-validations-v1` schema](schemas/external-first-map-validations-v1.schema.json)
requires successful, accepted evidence. Failed reports remain in GitHub
issues and are linked from their eventual resolution; they are never rewritten
as passing evidence.

## Reviewer workflow

After resolving any onboarding findings, add one accepted entry to
`docs/evidence/external-first-map-validations.json`. Then validate it:

```bash
python3 scripts/check_external_first_map_readiness.py --json
```

The command exits `0` when the ledger is structurally valid, even before three
reports exist, and reports `NOT_READY` with the exact remaining count. The
v1.0 release gate is stricter:

```bash
python3 scripts/check_external_first_map_readiness.py --require-complete
```

Exit codes are:

| Code | Meaning |
| ---: | --- |
| `0` | Ledger is valid; with `--require-complete`, the 3-user gate is ready |
| `1` | Ledger is valid but fewer than three accepted reports exist |
| `2` | Ledger or schema is invalid, including duplicate evidence |

The validator reports counts by documentation path for coverage visibility,
but the v1.0 contract does not invent a per-path quota: the published
requirement is three distinct independent users completing a first map from
public documentation.
