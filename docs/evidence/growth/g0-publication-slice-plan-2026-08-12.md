# G0 follow-up publication slice plan — 2026-08-12

> Decision: **EXACT_LOCAL_PLAN / NO_AUTHORITY_DERIVED_FROM_PLAN**
>
> Public Draft PR: `#427`
>
> PR comparison base:
> `3f4dd70cdc58ad421192559213cdee0bdc41eba8`
>
> Frozen public review baseline (machine key `public_head_sha`):
> `3ed632e6f6aa1e3ca7f32d893773de1079086ffb`
>
> Planned follow-up inventory: 285 paths; SHA-256
> `03d0d48868bdee9c7ecbf7734d5cfec0f42780425d8308b9477c757795ce84d7`
>
> Exact current public Draft head:
> `e786c18ef05fc6b6e26606f35d06145475359e98`
>
> Last observed remote mutation: exact non-force push from `7fde9cc` to
> `e786c18`; mutations performed by the checker or review-card command:
> **none**

## Outcome

The local GLIM-convenience and release-evidence UX follow-up is now split into
seven dependency-ordered review focuses. Every tracked or untracked path
relative to the exact PR base has exactly one primary review owner. A
fail-closed checker rejects missing, stale, duplicated, unsafe, or
digest-drifted paths and rejects a plan that claims GitHub write authority.

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
- the current public PR head resolves to `e786c18` and exposes 335 changed
  files;
- ten GitHub Actions checks pass on that exact head and four publication jobs
  skip intentionally, including green Humble/Jazzy builds, default workflows,
  upgrade checks, documentation, and release-readiness guards; and
- no PR conversation, inline review, or submitted review was present at the
  observation time.

Passing checks on `e786c18` validate the complete public Draft candidate,
including the published-onboarding identity, atomic paired recorder, evidence
sync, and ament import-order follow-up. The actionable empty-release-evidence
report is the next local code-bearing increment and requires its own public CI
after publication.

## Review order

| Slice | Paths | Reviewer outcome | Gate |
| --- | ---: | --- | --- |
| S1 runtime safety | 15 | unsafe point-cloud and VoxelGrid layouts, plus readable samples with empty frame IDs, fail closed without losing valid fields | public Humble/Jazzy CI |
| S2 first-map foundation | 32 | one bounded demo/own-bag route reaches a verified local 3D result and retains recovery state | public Humble/Jazzy CI |
| S3 map lifecycle | 25 | setup, history, compare, edit, merge, and support preserve provenance and receipts | focused local review |
| S4 source onboarding | 35 | a fresh terminal uses the exact six-package Humble/Jazzy source route, bounded release-or-candidate Docker/source measurement, a read-only guided host-readiness card, content-bound Docker observer bootstrap, one-command run-to-session execution, public preflight, and an auditable SHA-bound supplement path for retained observations | public Humble/Jazzy CI and clean-machine timing |
| S5 distribution readiness | 65 | NDT convergence, its copy-ready upstream PR packet, v0.9.1 metadata, exact-head/tag-aware bundle rehearsal, actionable missing-benchmark reports, immutable upstream patch formatting, clone-free launcher identity, authenticated package-manager blockers, and a default-branch, protected-environment, digest-only candidate gate with a shared read-only environment preflight and four-artifact byte audit remain explicit; no E2 dispatch, tag, release, or version reuse is implied | maintainer distribution decision |
| S6 product-shell integration | 107 | the installed home, bag-optional doctor, Japanese quickstart, byte-bound public-docs deployment provenance, parse-safe and content-verified GLIM comparison, neutral GLIM usability scorecard, fail-closed worksheet generators, and atomic paired observation recorder, truthful onboarding/growth snapshots, bounded starter queue and machine-evaluated validator cohort, CLI contract, docs, tests, support surface, schema-valid first-map handoff JSON, atomic one-command candidate handoff, report-derived release or four-file-candidate exact-identity observer packet, live identity recheck, and one-command G0 readiness dashboard agree | complete product gate and public CI |
| S7 publication control | 6 | all 285 paths are owned once and external authority remains separate; the current action packet keeps E2/E3/E4 separate | exact-tip maintainer decision |

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
untracked, non-ignored paths, verifies that public baseline `0c67387` descends
from that base and the local tip descends from the public head, then requires:

1. seven consecutive, dependency-safe slice orders;
2. sorted and canonical repository-relative paths;
3. one and only one owner for every candidate path;
4. exact 285-path coverage with the fixed inventory digest;
5. a local-only authority state with no claimed GitHub write; and
6. a report that always states whether a remote mutation occurred.

Adding, removing, renaming, or reassigning a path invalidates the plan until a
reviewer updates both the exact inventory and its digest. A green schema alone
cannot bypass live Git coverage.

## Verification

| Check | Result |
| --- | --- |
| exact Git-derived plan check | `PLAN_VALID_LOCAL_ONLY`; 285 paths, 7 slices, no remote mutation |
| checker regressions | 14 passed, including omission, stale path, duplicate owner, dependency inversion, digest drift, lineage drift, authority rejection, bounded human/JSON review cards, unknown-slice rejection, and self-contained read-only source dry-run execution |
| focused graph product/docs regressions | 25 passed in a Jazzy-sourced isolated package process |
| first-map submission UX regressions | 117 passed across support handoff, CLI contract, receipt, acceptance, readiness, and runner suites |
| validator cohort contract and operating state | 33 passed; path-specific immutable runtime identity, byte-bound public documentation, anonymized attempt lifecycle, accepted-ledger evidence binding, four operational stop signals, 48-hour freshness, WIP, batch/target transitions, attempt-10 thresholds, and a one-action human status card are enforced through the CLI; recruitment render remains blocked; no write authority |
| protected candidate environment | 29 passed across the shared environment/candidate gate; complete authenticated inventory, GET-only transport, exact reviewer/self-review/branch policy, unknown-rule refusal, workflow path trigger, and no-write/E2 authority are enforced; actionlint v1.7.12 and CTest 2 / 2 pass |
| G0 readiness dashboard | 5 passed; one-command local HOLD card, optional read-only publication/environment audits, JSON schema, one next action, child-authority refusal, and checker-error fail-closed behavior are covered |
| published onboarding identity and packet | 18 passed; release packet identity is report-derived, manual overrides fail closed, exact tag commit and both live image digests are rechecked, release/candidate modes stay separate, and no trial or publication authority is added |
| affected registered CTest | 4 / 4 pass after a clean Jazzy reconfigure; publication-plan, G0-readiness, packet, and published-identity registrations execute through ament |
| weekly growth snapshot | 14 passed; new snapshots re-derive cohort count/rate/state consistency while the immutable historical baseline remains schema-valid and identity-free |
| focused plan/source/NDT environment regressions | 32 passed after the clean worktree submodules were initialized |
| candidate handoff/session/probe regressions | 67 passed; retained child receipts are byte-bound, Docker observer bootstrap is recipe-labelled, and preparation/execution failure states remain atomic |
| public documentation deployment provenance | 8 focused regressions pass; strict MkDocs produces a deterministic manifest binding exact source revision, product version, route fragments, byte count, and SHA-256; the read-only live audit correctly remains `BLOCKED` on the current Pages 404 until the reviewed workflow is deployed |
| clean candidate release bundle | exact-head reproducibility rehearsal passes in public CI at `e786c18`; the latest retained local exact bundle pair at `7fde9cc` contains 252 files with SHA-256 `344a6c4d79902e04ac56b5e1987a7200f7fc4d493ab833f7e0425759aca28c43`; all three scorecard preparation/recording tools are manifest-bound |
| complete maintained Python gate | graph: 1,442 passed / 13 skipped / 11 existing ImageIO warnings; lidar_slam: 982 passed; 2,424 total |
| paired scorecard recorder | 7 direct regressions, 20 recorder/checker regressions, and registered CTest 6 / 6 pass; incomplete observations remain non-comparable and atomic output/privacy boundaries fail closed |
| empty release-evidence UX | 29 release-profile regressions cover direct and wrapper-level empty-root behavior; the exact-head gate retains Markdown/CSV/log evidence, reports five blocking rows plus their remediations, and exits 2 without weakening release authority |
| GLIM reference-cache integrity | 10 focused regressions passed; registered CTest 1 / 1; missing lookup remains read-only; exact content, manifest, malformed-TUM, tamper, collision, shell, and bundle boundaries pass |
| scanmatcher clean build and CTest | Jazzy RAM-backed clean build of `lidarslam_msgs`, `ndt_omp_ros2`, and `scanmatcher`; 109 tests passed |
| review follow-up regressions | malformed PointCloud2 recovery with padded organized XYZ-only continuation, metadata tile containment, source-bundle symlink rejection, non-interpolated immutable release-tag checkout, least-privilege release jobs, and self-contained source dry-run are covered |
| candidate session Python style | direct flake8 for the three implementation and three focused regression files plus pydocstyle for all three implementation files: PASS |
| documentation | `mkdocs build --strict`: PASS with pre-existing Material and navigation notices |
| machine formats and shells | all 126 repository JSON files parse; all 81 Draft 7 schemas validate; all 82 tracked shell files pass `bash -n`; `git diff --check` PASS; immutable upstream patch carriers alone opt out of whitespace interpretation |

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
as easy as GLIM on a new machine. The digest-only candidate gate and read-only
four-file retained-artifact audit and atomic one-command observer handoff are
now in the review inventory, but the required
`candidate-images` environment and an E2 dispatch remain separate external
decisions. After one authorized pair is published, require the observer
packet's `REMOTE_AUDIT_PASS` preflight; its tag-free trial commands bind the
exact bundle and set SHA-256 values, run, source commit, and image references before timed
clean-machine Docker/source first-map trials on Humble and Jazzy. The measured
command count, active operator time,
download, peak disk, diagnosis quality, and verified result must update the
onboarding matrix without private paths or a release claim.
