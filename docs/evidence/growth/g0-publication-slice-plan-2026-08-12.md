# G0 follow-up publication slice plan — 2026-08-12

> Decision: **EXACT_LOCAL_PLAN / NO_AUTHORITY_DERIVED_FROM_PLAN**
>
> Public Draft PR: `#427`
>
> PR comparison base:
> `3f4dd70cdc58ad421192559213cdee0bdc41eba8`
>
> Frozen public review baseline (machine key `public_head_sha`):
> `8a931a3627be503fb10c255ef846c6d3c54a237c`
>
> Planned follow-up inventory: 233 paths; SHA-256
> `15b94422746d15290b1f45d1a3b95fef892cba4ef1ab7be78e57ec2948ac1a09`
>
> Last observed remote mutation: exact non-force push from `33f1340` to
> `065bae3`; mutations performed by the checker or review-card command:
> **none**

## Outcome

The local GLIM-convenience follow-up is now split into seven dependency-ordered
review focuses. Every tracked or untracked path relative to the exact PR base
has exactly one primary review owner. A fail-closed checker rejects missing,
stale, duplicated, unsafe, or digest-drifted paths and rejects a plan that
claims GitHub write authority.

These are review focuses, not seven independently cherry-pickable repositories.
Several product integration files register tests and installed helpers from
multiple lower slices. Those shared integration files intentionally belong to
S6, after all of their runtime dependencies. PR updates should preserve the
review order through bounded commits or clearly labeled commit hunks, then run
the complete candidate gate at the exact tip.

## Read-only public orientation

The GitHub repository and PR were inspected without mutation:

- Draft PR `#427`, `Prepare crash-safe guided mapping for G0 review`, remains
  open, draft, and mergeable into `develop`;
- at the baseline observation, the public PR head resolved to `8a931a3` and
  the later review fixes had not yet been published;
- at that observation, the PR exposed 253 changed files and 56 commits;
- all nine reported GitHub Actions checks pass on the public head, including
  Humble/Jazzy builds, default workflows, upgrade checks, documentation, and
  release-readiness guards; and
- no PR conversation, inline review, or submitted review was present at the
  observation time.

At the baseline observation, passing checks on `8a931a3` validated only the
then-published portion of the seven-slice candidate, not the then-local review
fixes. Every candidate tip newer than that frozen baseline requires its own
public CI result.

## Review order

| Slice | Paths | Reviewer outcome | Gate |
| --- | ---: | --- | --- |
| S1 runtime safety | 15 | unsafe point-cloud and VoxelGrid layouts, plus readable samples with empty frame IDs, fail closed without losing valid fields | public Humble/Jazzy CI |
| S2 first-map foundation | 32 | one bounded demo/own-bag route reaches a verified local 3D result and retains recovery state | public Humble/Jazzy CI |
| S3 map lifecycle | 25 | setup, history, compare, edit, merge, and support preserve provenance and receipts | focused local review |
| S4 source onboarding | 26 | a fresh terminal uses the exact six-package Humble/Jazzy source route, bounded Docker/source measurement, public preflight, and an auditable SHA-bound supplement path for retained observations | public Humble/Jazzy CI and clean-machine timing |
| S5 distribution readiness | 46 | NDT convergence, its copy-ready upstream PR packet, v0.9.1 metadata, exact-head/tag-aware bundle rehearsal, immutable upstream patch formatting, clone-free launcher identity, and authenticated fail-closed package-manager blockers remain explicit; no release/version reuse is implied | maintainer distribution decision |
| S6 product-shell integration | 83 | the installed home, bag-optional doctor, Japanese quickstart, neutral GLIM usability scorecard and fail-closed worksheet generators, truthful onboarding/growth snapshots, bounded starter queue and machine-evaluated validator cohort, CLI contract, docs, tests, support surface, schema-valid first-map handoff JSON, exact-identity observer packet, and one-command G0 readiness dashboard agree | complete product gate and public CI |
| S7 publication control | 6 | all 233 paths are owned once and external authority remains separate; the current action packet keeps E2/E3/E4 separate | exact-tip maintainer decision |

The machine-readable source of truth is
[`g0-publication-slice-plan-2026-08-12.json`](g0-publication-slice-plan-2026-08-12.json),
validated against
[`publication-slice-plan-v1.schema.json`](../../schemas/publication-slice-plan-v1.schema.json)
by `python3 scripts/check_publication_slice_plan.py --json`.

Reviewers can request one bounded, read-only card without manually extracting
paths or commands from the complete plan:

```bash
python3 scripts/check_publication_slice_plan.py --slice S1-runtime-safety
python3 scripts/check_publication_slice_plan.py \
  --slice S1-runtime-safety --json
```

The card revalidates the complete inventory and lineage first, then binds the
selected review outcome, dependency list, exact paths, verification commands,
publication gate, frozen public baseline, local HEAD, follow-up commit count,
and current worktree cleanliness. The baseline is an immutable review anchor,
not a live claim about the remote branch. A dirty worktree is shown with its
uncommitted path count rather than being mislabeled as an exact-tip candidate.
The card
does not execute the displayed commands and cannot authorize or report a
GitHub mutation. An unknown slice ID fails closed and lists the seven valid
IDs.

## Fail-closed invariants

The checker derives the candidate directly from Git rather than trusting the
human table. It combines tracked changes from exact PR base `3f4dd70` with
untracked, non-ignored paths, verifies that public baseline `8a931a3` descends
from that base and the local tip descends from the public head, then requires:

1. seven consecutive, dependency-safe slice orders;
2. sorted and canonical repository-relative paths;
3. one and only one owner for every candidate path;
4. exact 233-path coverage with the fixed inventory digest;
5. a local-only authority state with no claimed GitHub write; and
6. a report that always states whether a remote mutation occurred.

Adding, removing, renaming, or reassigning a path invalidates the plan until a
reviewer updates both the exact inventory and its digest. A green schema alone
cannot bypass live Git coverage.

## Verification

| Check | Result |
| --- | --- |
| exact Git-derived plan check | `PLAN_VALID_LOCAL_ONLY`; 233 paths, 7 slices, no remote mutation |
| checker regressions | 14 passed, including omission, stale path, duplicate owner, dependency inversion, digest drift, lineage drift, authority rejection, bounded human/JSON review cards, unknown-slice rejection, and self-contained read-only source dry-run execution |
| focused graph product/docs regressions | 25 passed in a Jazzy-sourced isolated package process |
| first-map submission UX regressions | 117 passed across support handoff, CLI contract, receipt, acceptance, readiness, and runner suites |
| validator cohort contract and operating state | 31 passed; path-specific immutable runtime identity, anonymized attempt lifecycle, accepted-ledger evidence binding, four operational stop signals, 48-hour freshness, WIP, batch/target transitions, attempt-10 thresholds, and a one-action human status card are enforced through the CLI; recruitment render remains blocked; no write authority |
| G0 readiness dashboard | 4 passed; one-command local HOLD card, optional read-only publication audit, JSON schema, one next action, and checker-error fail-closed behavior are covered |
| weekly growth snapshot | 14 passed; new snapshots re-derive cohort count/rate/state consistency while the immutable historical baseline remains schema-valid and identity-free |
| focused plan/source/NDT environment regressions | 32 passed after the clean worktree submodules were initialized |
| complete maintained Python gate | graph: 1,433 passed / 13 skipped / 11 existing ImageIO warnings; lidar_slam: 751 passed; 2,184 total |
| scanmatcher clean build and CTest | Jazzy RAM-backed clean build of `lidarslam_msgs`, `ndt_omp_ros2`, and `scanmatcher`; 109 tests passed |
| review follow-up regressions | malformed PointCloud2 recovery with padded organized XYZ-only continuation, metadata tile containment, source-bundle symlink rejection, non-interpolated immutable release-tag checkout, least-privilege release jobs, and self-contained source dry-run are covered |
| new Python style | `ament_flake8`: 4 files checked, no problems |
| documentation | `mkdocs build --strict`: PASS with pre-existing Material and navigation notices |
| machine formats and shells | all 37 planned JSON files parse; all 9 planned shell files pass `bash -n`; `git diff --check` PASS; immutable upstream patch carriers alone opt out of whitespace interpretation |

The two package test directories were intentionally run in separate pytest
processes through the repository's canonical contributor entrypoint because
they retain one known duplicate test-module basename. ROS Jazzy was sourced for
the focused ROS-bag tests and the entrypoint sourced it automatically for the
complete gate.

## Publication boundary

This plan was authored as a local review artifact and does not authorize a
push, PR update, comment, review, merge, tag, release, package, image, issue,
label, or external dependency change. Maintainer direction is evaluated
outside this artifact; the plan cannot manufacture authority from a green
local check.

Before any further publication, validate the clean exact-tip candidate from
these slices and inspect the resulting diff and object inventory. Any
non-force update to Draft PR `#427` must remain within current maintainer
direction; force pushes, merge, release, tag, deletion, and third-party
communication remain separate decisions.

## Remaining GLIM-convenience gate

This plan makes the work reviewable; it does not prove that the workflow feels
as easy as GLIM on a new machine. After a public candidate exists, the next
product evidence is a timed clean-machine Docker/source first-map trial on
Humble and Jazzy. The measured command count, active operator time, download,
peak disk, diagnosis quality, and verified result must update the onboarding
matrix without private paths or local-only source identities.
