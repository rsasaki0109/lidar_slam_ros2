# Release rollback and last-known-good images

This runbook defines how to select, verify and deploy a previously accepted
container image when a newer release has an operator-visible regression.
Rollback changes the deployment reference to an immutable digest. It never
moves or overwrites a release tag.

## Current readiness

The tracked ledger is `configs/release/last-known-good.json`.
It is intentionally `unassigned`: existing public releases predate the
Humble/Jazzy `release-image-*.json` acceptance assets, so none satisfies the
current promotion contract.

Confirm that state with:

```bash
python3 scripts/manage_last_known_good.py verify \
  configs/release/last-known-good.json
```

An unassigned ledger is valid repository state but is not deployable.
`verify --require-assigned` and `plan` fail closed until an accepted release
is promoted.

## What qualifies as last known good

One release may be promoted only after all of the following:

1. The GitHub Release contains both `release-image-humble.json` and
   `release-image-jazzy.json`.
2. Both records are `PASS`, refer to `linux/amd64`, use the expected versioned
   tags, and contain registry digests.
3. Product version and full git commit are identical across the two records.
4. GitHub artifact attestation verification succeeds for both digests.
5. Installed image smoke checks pass on both distributions.
6. The pinned real-data first-map release gate passes, and no unresolved
   release-blocking onboarding or security defect remains.
7. Promotion is reviewed through a pull request; a CI run alone does not edit
   the tracked ledger.

Moving `:humble`, `:jazzy`, and `:latest` tags never qualify as rollback
identity.

## Promote an accepted release

Download the image evidence from the accepted GitHub Release:

```bash
RELEASE_TAG=v0.9.0
mkdir -p "/tmp/lidarslam-lkg/${RELEASE_TAG}"
gh release download "$RELEASE_TAG" \
  -R rsasaki0109/lidar_slam_ros2 \
  --pattern 'release-image-*.json' \
  --dir "/tmp/lidarslam-lkg/${RELEASE_TAG}"
```

Validate each record before combining it:

```bash
python3 scripts/manage_last_known_good.py validate-evidence \
  "/tmp/lidarslam-lkg/${RELEASE_TAG}/release-image-humble.json" \
  --ros-distro humble
python3 scripts/manage_last_known_good.py validate-evidence \
  "/tmp/lidarslam-lkg/${RELEASE_TAG}/release-image-jazzy.json" \
  --ros-distro jazzy
```

Create a candidate ledger. Promotion refuses mismatched versions/commits and
refuses to overwrite an already assigned output:

```bash
python3 scripts/manage_last_known_good.py promote \
  --humble "/tmp/lidarslam-lkg/${RELEASE_TAG}/release-image-humble.json" \
  --jazzy "/tmp/lidarslam-lkg/${RELEASE_TAG}/release-image-jazzy.json" \
  --output /tmp/last-known-good.candidate.json \
  --reason 'Both distro image, attestation and pinned real-data gates passed.'
python3 scripts/manage_last_known_good.py verify \
  /tmp/last-known-good.candidate.json \
  --require-assigned
```

Review the candidate, replace the tracked unassigned ledger in a dedicated
pull request, and record links to the image and real-data acceptance runs in
the PR. Replacing an existing assigned ledger requires an explicit reviewed
file change; the tool will not do it in place.

## Execute a rollback

First preserve the currently deployed digest and relevant runtime
configuration in the incident record. Do not begin from a convenience tag.

Generate the verified plan for the affected ROS distribution:

```bash
python3 scripts/manage_last_known_good.py plan \
  configs/release/last-known-good.json \
  --ros-distro jazzy
```

The command prints, but does not execute:

- an immutable `repository@sha256:...` reference;
- `docker pull` for that digest;
- `gh attestation verify` against that digest;
- an installed `lidarslam-map --version` smoke check.

Run those commands exactly, then execute the fixed public demo or the
deployment's pinned acceptance bag with a new output directory. Require a
successful manifest, diagnosis and map verification before routing production
work to the old image.

Update the deployment configuration to the printed digest. Never retag the
last-known-good image as `:humble`, `:jazzy`, or `:latest`; doing so destroys
incident traceability and can race other deployments.

## Abort and recovery criteria

Abort the rollback when attestation verification fails, the observed CLI
version differs from the ledger, the digest cannot be pulled, or the
acceptance map fails. Preserve all output and escalate as a release-blocking
incident.

A rollback is containment, not resolution. Fix forward with a new versioned
release, repeat the full Humble/Jazzy and real-data gates, and promote that
new release through review. Keep the incident's before/after digests and
commands with the release evidence.

## Scope

This contract currently covers published `linux/amd64` GHCR images. Source
workspaces, ROS buildfarm packages, arm64 images, parameters, bags and map
schema migrations require their own rollback or compatibility procedure.
