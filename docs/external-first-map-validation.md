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
3. A current `lidarslam-map run` writes
   `first_map_validation_receipt.json` and
   `first_map_validation_receipt.md` after finalizing the manifest,
   diagnosis, and Autoware verification log. Open the Markdown receipt and
   confirm it says `Receipt status: PASS`.

   For an existing output produced before automatic receipts were added,
   regenerate the same report:

   ```bash
   python3 scripts/create_first_map_validation_receipt.py \
     /path/to/output \
     --write
   ```

4. Open the
   [Independent First-map Validation issue form](https://github.com/rsasaki0109/lidar_slam_ros2/issues/new?template=first-map-validation.yml).
   Paste the receipt's `Verification summary` block into the form. Keep the
   JSON receipt until review; it contains evidence hashes but no map geometry,
   private paths, or exact command.

Both passing and failing reports are useful. A failed attempt is an onboarding
finding, not an accepted validation, and must be resolved or explicitly
documented before v1.0.

!!! warning "Do not publish map geometry"

    Remove credentials, private paths, precise locations, rosbag payloads,
    point-cloud tiles, trajectories, and screenshots that reveal private
    places. The issue form asks only for command, environment, status,
    verifier summary, and a manifest checksum. The generated receipt is
    privacy-bounded, but you must still review it and redact private paths
    from the separately pasted command.

## What the receipt proves

The
[`first-map-validation-receipt-v1` schema](schemas/first-map-validation-receipt-v1.schema.json)
contains only the product version/revision, profile, run ID, status values,
file hashes, and seven boolean checks. A passing receipt proves:

- the final manifest says `succeeded`, lifecycle `complete`, and runner exit
  code `0`;
- the diagnosis says `success`;
- the Autoware verification result is `PASS`;
- the diagnosis and verification-log hashes match the identities frozen into
  the manifest.

The receipt is derived after the final manifest write, so receipt files are
intentionally excluded from the manifest's artifact-checksum list. This avoids
a circular manifest → receipt → manifest hash while keeping the three
authoritative inputs cryptographically bound.

## Acceptance contract

A report counts only when all of these are true:

- the reporter self-attests that they are not a maintainer and did not receive
  live step-by-step help;
- the run started from one of the three public documentation paths;
- the report identifies the exact release, revision, or image digest;
- `run_manifest.json` says `succeeded`;
- the diagnosis status is `success`;
- Autoware map verification is `PASS`;
- the generated receipt says `PASS` and its manifest SHA-256 matches the
  submitted verification summary;
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

Do not hand-edit the tracked ledger first. After resolving any onboarding
findings:

1. Download the reporter's JSON receipt privately and confirm the public issue
   contains the matching verification summary.
2. Create `/tmp/validation-entry.json` with one `validation` object matching
   the
   [`external-first-map-validations-v1` schema](schemas/external-first-map-validations-v1.schema.json).
   Record the public issue-comment or pull-request review URL in
   `acceptance.review_url`.
3. Prepare a new ledger file:

   ```bash
   python3 scripts/prepare_external_first_map_acceptance.py \
     --entry /tmp/validation-entry.json \
     --receipt /path/to/first_map_validation_receipt.json \
     --output-ledger /tmp/external-first-map-validations.proposed.json
   ```

4. Review the proposed file and its diff, then copy that reviewed proposal to
   `docs/evidence/external-first-map-validations.json` in a normal pull
   request. Do not commit the reporter's receipt.

The intake command is fail-closed. It requires a schema-valid PASS receipt,
all seven receipt checks, exact verification-field and manifest-hash binding,
a release reference matching the receipt version/revision or an immutable
image digest, chronological submission and review timestamps, a public
repository review URL on the submitted issue (or its review PR), a known
maintainer reviewer, and an independent reporter. The current maintainer is
explicitly excluded as an independent reporter.
It appends the entry in memory and revalidates the complete ledger, so
duplicate reporters, issues, validation IDs, and manifest hashes are rejected.

The command never modifies the input ledger, refuses to write to its path, and
refuses to overwrite any existing proposal. Without `--output-ledger`, it
performs a dry run and prints a schema-valid
[`external-first-map-acceptance-v1` report](schemas/external-first-map-acceptance-v1.schema.json).

After reviewing and placing the proposal in the tracked ledger, validate it:

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

The intake command uses `0` for `READY_TO_PROPOSE` and `2` for invalid,
unbound, duplicate, unreviewed, or unsafe-to-write evidence.

The validator reports counts by documentation path for coverage visibility,
but the v1.0 contract does not invent a per-path quota: the published
requirement is three distinct independent users completing a first map from
public documentation.
