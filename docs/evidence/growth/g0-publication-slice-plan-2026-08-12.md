# G0 follow-up publication slice plan — 2026-08-12

> Decision: **EXACT_LOCAL_PLAN / PUBLICATION_NOT_AUTHORIZED**
>
> Public Draft PR: `#427`
>
> Public and local follow-up base:
> `3f4dd70cdc58ad421192559213cdee0bdc41eba8`
>
> Planned follow-up inventory: 188 paths; SHA-256
> `e40852ea812a6f4c2a89ac7ce09261e267a0aa64558c8c1928753ba077428396`
>
> Remote mutations performed by this review pass: **none**

## Outcome

The local GLIM-convenience follow-up is now split into seven dependency-ordered
review focuses. Every tracked or untracked path relative to the exact follow-up base
has exactly one primary review owner. A fail-closed checker rejects missing,
stale, duplicated, unsafe, or digest-drifted paths and rejects a plan that
claims GitHub write authority.

These are review focuses, not seven independently cherry-pickable repositories.
Several product integration files register tests and installed helpers from
multiple lower slices. Those shared integration files intentionally belong to
S6, after all of their runtime dependencies. An eventual PR update should
preserve the review order through bounded commits or clearly labeled commit
hunks, then run the complete candidate gate at the exact tip.

## Read-only public orientation

The GitHub repository and PR were inspected without mutation:

- Draft PR `#427`, `Prepare crash-safe guided mapping for G0 review`, remains
  open, draft, and mergeable into `develop`;
- the public PR head resolves to `b12fc60`; the review fixes remain local-only;
- the PR currently exposes 253 changed files and 55 commits;
- all nine reported GitHub Actions checks pass on the public head, including
  Humble/Jazzy builds, default workflows, upgrade checks, documentation, and
  release-readiness guards; and
- no PR conversation, inline review, or submitted review was present at the
  observation time.

Passing checks on `b12fc60` validate the published seven-slice candidate, but
do not validate the current local review fixes. Public CI must run again
on the exact future candidate tip.

## Review order

| Slice | Paths | Reviewer outcome | Gate |
| --- | ---: | --- | --- |
| S1 runtime safety | 13 | unsafe point-cloud and VoxelGrid layouts fail closed without losing valid fields | public Humble/Jazzy CI |
| S2 first-map foundation | 31 | one bounded demo/own-bag route reaches a verified local 3D result and retains recovery state | public Humble/Jazzy CI |
| S3 map lifecycle | 25 | setup, history, compare, edit, merge, and support preserve provenance and receipts | focused local review |
| S4 source onboarding | 16 | a fresh terminal uses the exact six-package Humble/Jazzy source route and public preflight | public Humble/Jazzy CI and clean-machine timing |
| S5 distribution readiness | 42 | NDT convergence, v0.9.1 metadata, exact-head/tag-aware bundle rehearsal, immutable upstream patch formatting, clone-free launcher identity, and package-manager blockers remain explicit; no release/version reuse is implied | maintainer distribution decision |
| S6 product-shell integration | 56 | the installed home, bag-optional doctor, Japanese quickstart, neutral GLIM usability scorecard, truthful onboarding matrix, bounded starter queue, CLI contract, docs, tests, and support surface agree | complete product gate and public CI |
| S7 publication control | 5 | all 188 paths are owned once and external authority remains separate | exact-tip maintainer decision |

The machine-readable source of truth is
[`g0-publication-slice-plan-2026-08-12.json`](g0-publication-slice-plan-2026-08-12.json),
validated against
[`publication-slice-plan-v1.schema.json`](../../schemas/publication-slice-plan-v1.schema.json)
by `python3 scripts/check_publication_slice_plan.py --json`.

## Fail-closed invariants

The checker derives the candidate directly from Git rather than trusting the
human table. It combines tracked changes from exact public head `3f4dd70` with
untracked, non-ignored paths, then requires:

1. seven consecutive, dependency-safe slice orders;
2. sorted and canonical repository-relative paths;
3. one and only one owner for every candidate path;
4. exact 188-path coverage with the fixed inventory digest;
5. a local-only authority state with no claimed GitHub write; and
6. a report that always states whether a remote mutation occurred.

Adding, removing, renaming, or reassigning a path invalidates the plan until a
reviewer updates both the exact inventory and its digest. A green schema alone
cannot bypass live Git coverage.

## Verification

| Check | Result |
| --- | --- |
| exact Git-derived plan check | `PLAN_VALID_LOCAL_ONLY`; 188 paths, 7 slices, no remote mutation |
| checker regressions | 10 passed, including omission, stale path, duplicate owner, dependency inversion, digest drift, authority rejection, and self-contained read-only source dry-run execution |
| focused graph product/docs regressions | 25 passed in a Jazzy-sourced isolated package process |
| focused lidar_slam CLI/queue/plan/scorecard regressions | 91 passed in an isolated package process |
| complete maintained Python gate | graph: 1,433 passed / 13 skipped / 11 existing ImageIO warnings; lidar_slam: 708 passed; 2,141 total |
| scanmatcher clean build and CTest | Jazzy RAM-backed clean build of `lidarslam_msgs`, `ndt_omp_ros2`, and `scanmatcher`; 109 tests passed |
| review follow-up regressions | malformed PointCloud2 recovery with padded organized XYZ-only continuation, metadata tile containment, source-bundle symlink rejection, non-interpolated immutable release-tag checkout, least-privilege release jobs, and self-contained source dry-run are covered |
| new Python style | `ament_flake8`: 4 files checked, no problems |
| documentation | `mkdocs build --strict`: PASS with pre-existing Material and navigation notices |
| machine formats and shells | all 32 planned JSON files parse; all 9 planned shell files pass `bash -n`; `git diff --check` PASS; immutable upstream patch carriers alone opt out of whitespace interpretation |

The two package test directories were intentionally run in separate pytest
processes through the repository's canonical contributor entrypoint because
they retain one known duplicate test-module basename. ROS Jazzy was sourced for
the focused ROS-bag tests and the entrypoint sourced it automatically for the
complete gate.

## Publication boundary

The review follow-up remains local and this plan does not authorize a push, PR
update, comment, review, merge, tag, release, package, image, issue, label, or
external dependency change. Earlier push approvals do not automatically
extend to the new exact tip.

Before any publication, validate the clean exact-tip candidate from these
slices, inspect the resulting diff and object inventory, and request a
maintainer decision naming that exact 40-character tip for a non-force update
to Draft PR `#427` only.

## Remaining GLIM-convenience gate

This plan makes the work reviewable; it does not prove that the workflow feels
as easy as GLIM on a new machine. After a public candidate exists, the next
product evidence is a timed clean-machine Docker/source first-map trial on
Humble and Jazzy. The measured command count, active operator time, download,
peak disk, diagnosis quality, and verified result must update the onboarding
matrix without private paths or local-only source identities.
