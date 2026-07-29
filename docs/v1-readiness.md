# v1.0 Readiness

`lidarslam_ros2` uses a fail-closed, machine-readable audit for the complete
v1.0 product goal. It covers the nine readiness dimensions in the
[v0.9 roadmap](roadmap/v0.9.md#1-measurable-v10-readiness-targets) plus the
stable v0.9 release-candidate publication gate.

Run the audit from a full git checkout:

```bash
python3 scripts/check_v1_readiness.py
```

For automation:

```bash
python3 scripts/check_v1_readiness.py --json
python3 scripts/check_v1_readiness.py --require-complete
```

The normal command reports an honest snapshot and exits zero when the
contract is valid, even while product work remains. `--require-complete`
exits 1 unless every gate is complete. An invalid contract, schema, evidence
path, adoption ledger, version, or git-tag query exits 2.

## Authoritative inputs

- [`contracts/v1-readiness.json`](contracts/v1-readiness.json) records the
  reviewed state, evidence paths, and explicit blockers for every gate.
- [`schemas/v1-readiness-contract-v1.schema.json`](schemas/v1-readiness-contract-v1.schema.json)
  prevents a gate, blocker, or evidence field from silently disappearing.
- [`schemas/v1-readiness-report-v1.schema.json`](schemas/v1-readiness-report-v1.schema.json)
  defines the generated JSON report.
- [`evidence/external-first-map-validations.json`](evidence/external-first-map-validations.json)
  remains the source of truth for the independent three-user gate.
- Root `VERSION` and local immutable git tags are checked for the v0.9
  publication gate.
- The distribution gate includes the read-only
  `scripts/check_ndt_omp_release_readiness.py` preflight and its
  [`ndt-omp-release-readiness-v1.schema.json`](schemas/ndt-omp-release-readiness-v1.schema.json)
  contract. It distinguishes a locally reviewed candidate from
  `READY_TO_TAG`, partial publication, and a completed rosdistro release.
  The reviewed live result is preserved in
  [`ndt-omp-release-preflight-2026-07-29.md`](evidence/ndt-omp-release-preflight-2026-07-29.md).
- The package-manager distribution evidence has a versioned
  [`package-manager-install-v1.schema.json`](schemas/package-manager-install-v1.schema.json)
  contract and Humble/Jazzy clean-install/upgrade workflow. It remains an
  open gate until actual ROS testing/main packages produce passing artifacts.
- Release publication and first rollback evidence use the read-only
  `scripts/check_published_release.py` audit and
  [`published-release-v1.schema.json`](schemas/published-release-v1.schema.json).
  The audit cross-checks the stable release, tag commit, six attached assets,
  and every file hash in the release bundle.

The checker verifies that all ten expected gate IDs are present exactly once,
that evidence paths remain inside the repository and exist, that semantic
versions and tags align, and that the independent-user ledger passes its own
schema and uniqueness rules. A tracked `complete` state is therefore an
audited attestation backed by named evidence—not a substitute for rerunning
the wider CI, real-data, or release workflows cited by that evidence.

## Current snapshot

The tracked state is **NOT_READY: 6/10 gates complete**.

| Open gate | Remaining proof |
| --- | --- |
| Distribution | `ndt_omp_ros2` preflight is currently `READY_TO_TAG`; publish it to both rosdistros, wait for official RKO-LIO 0.3.2 to reach the normal apt repository, then run package-manager install/upgrade E2E |
| Reliability | Publish the first tagged release and pass the recovery-asset publication audit |
| External adoption | Accept three distinct independent first-map validations; current ledger is 0/3 |
| Release publication | v0.9.0 metadata and notes are aligned; create its immutable tag and pass the stable publication audit |

This table is explanatory. The generated report and contract are
authoritative. Update a gate to `complete` only in the same reviewed change
that adds its final public evidence. Never remove an unmet gate or mark an
external action complete based on intent.
