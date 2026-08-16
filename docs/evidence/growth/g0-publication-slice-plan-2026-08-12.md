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
> Planned follow-up inventory: 304 paths; SHA-256
> `c8cef196cc20780c0551e1797490442d6ebe55aea7f8fc6a928920ace73482b7`
>
> Capture-time exact public Draft head:
> `4cb2e680eb90f156249ef47181fab01b100d2049`
>
> Exact actionable release-evidence implementation tip:
> `45cfdcb1c10756d1c33068fcd9594f612bf6ccca`
>
> Exact live contributor next-action implementation tip:
> `3543a71bde958278388aa8481330166d125944b9`

> Exact dependency-gated contributor next-action implementation tip:
> `76be89576ae544d77e661bc5d098b5087b497c5c`
>
> Exact copy-ready low-storage recovery implementation tip:
> `d01652080485bc68354f354043e4b2e732439223`
>
> Exact bounded map-quality symptom triage implementation tip:
> `ee453532a70d2d4b82a6c50c65f19b22d76c239f`
>
> Exact complete local validation carrier:
> `9f8a2058a3c702f69d159079568ced8433ee3377`
>
> Exact fail-closed docs artifact implementation tip:
> `5b8c8c477cceb4955184a64afa874712b9dea5aa`

> Exact fail-closed NDT reviewer-response implementation tip:
> `3e11f307eb2ccea1d33bbe9a2d1b37ae7ed699db`
>
> Exact claim-bounded social-media generator tip:
> `d0c84bb9bb7bef37d7e318000e3071a7f536d631`
>
> Last observed remote mutation: an exact non-force push advanced the Draft to
> `4cb2e68`; mutations performed by the checker or review-card command:
> **none**

## Outcome

The local GLIM-convenience, release-evidence, dependency-gated contributor
next-action, copy-ready low-storage, and bounded visual-symptom UX follow-up is
split into seven dependency-ordered review focuses. Every tracked or untracked
path relative to the exact PR base has exactly one primary review owner. A
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
- the captured public PR head resolves to `4cb2e68`;
- ten GitHub Actions checks pass on that exact head and four publication jobs
  skip intentionally, including green Humble/Jazzy builds, default workflows,
  upgrade checks, documentation, and release-readiness guards; and
- no PR conversation, inline review, or submitted review was present at the
  observation time.

Passing checks on `4cb2e68` validate the public Draft candidate through the
other-PointCloud2 self-service increment,
including the published-onboarding identity, atomic paired recorder, evidence
sync, ament import-order follow-up, actionable empty-release-evidence report,
the GET-only contributor next-action card, dependency-gated #422 handling, and
copy-ready low-storage recovery, pre-upload docs artifact gate, and fail-closed
NDT reviewer-response packet. That packet remains read-only and emits no reply
body until the exact canonical upstream Draft exists.

## Review order

| Slice | Paths | Reviewer outcome | Gate |
| --- | ---: | --- | --- |
| S1 runtime safety | 15 | unsafe point-cloud and VoxelGrid layouts, plus readable samples with empty frame IDs, fail closed without losing valid fields | public Humble/Jazzy CI |
| S2 first-map foundation | 32 | one bounded demo/own-bag route reaches a verified local 3D result, retains recovery state, and returns the exact shortage plus preserved retry command on low storage | public Humble/Jazzy CI |
| S3 map lifecycle | 25 | setup, history, compare, edit, merge, and support preserve provenance and receipts | focused local review |
| S4 source onboarding | 36 | a fresh terminal uses the exact six-package Humble/Jazzy source route, bounded release-or-candidate Docker/source measurement, a read-only guided host-readiness card, content-bound Docker observer bootstrap, one-command run-to-session execution, public preflight, and byte-bound validation-receipt plus SHA-bound supplement paths for retained observations | public Humble/Jazzy CI and clean-machine timing |
| S5 distribution readiness | 68 | NDT convergence, its copy-ready upstream PR packet, v0.9.1 metadata, exact-head/tag-aware bundle rehearsal, actionable missing-benchmark reports, write-free NTU acquisition planning with official archive identity and capacity fail-fast, immutable upstream patch formatting, clone-free launcher identity, authenticated package-manager blockers, and a default-branch, protected-environment, digest-only candidate gate with a shared read-only environment preflight and four-artifact byte audit remain explicit; no E2 dispatch, tag, release, or version reuse is implied | maintainer distribution decision |
| S6 product-shell integration | 122 | the installed home, path-private bag-optional doctor with copy-ready low-storage recovery, issue-driven other-PointCloud2 `doctor` → `start` handoff without tracked launch/YAML edits, bounded user-reported visual-map triage through the retained-run inspector, Japanese quickstart, byte-bound public-docs deployment provenance, parse-safe and content-verified GLIM comparison, neutral GLIM usability scorecard, claim-bounded short-demo card/video/captions/copy, fail-closed worksheet generators, atomic paired observation recorder, truthful onboarding/growth snapshots, bounded starter queue with a dependency-gated and schema-valid GET-only live next-action card, machine-evaluated validator cohort, CLI contract, docs, tests, support surface, schema-valid first-map handoff JSON, atomic one-command candidate handoff, report-derived release or four-file-candidate exact-identity observer packet, live identity recheck, and one-command G0 readiness dashboard agree | complete product gate and public CI |
| S7 publication control | 6 | all 304 paths are owned once and external authority remains separate; the current action packet keeps E2/E3/E4 separate | exact-tip maintainer decision |

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
untracked, non-ignored paths, verifies that public baseline `3ed632e` descends
from that base and the local tip descends from the public head, then requires:

1. seven consecutive, dependency-safe slice orders;
2. sorted and canonical repository-relative paths;
3. one and only one owner for every candidate path;
4. exact 304-path coverage with the fixed inventory digest;
5. a local-only authority state with no claimed GitHub write; and
6. a report that always states whether a remote mutation occurred.

Adding, removing, renaming, or reassigning a path invalidates the plan until a
reviewer updates both the exact inventory and its digest. A green schema alone
cannot bypass live Git coverage.

## Verification

| Check | Result |
| --- | --- |
| exact Git-derived plan check | `PLAN_VALID_LOCAL_ONLY`; 304 paths, 7 slices, no remote mutation |
| claim-bounded social media | 11 focused regressions plus 25 public docs/release entrypoint regressions pass; the generated 10.666-second H.264 candidate, four-cue WebVTT, exact-revision Japanese/English copy, and byte manifest retain no external publication authority |
| checker regressions | 14 passed, including omission, stale path, duplicate owner, dependency inversion, digest drift, lineage drift, authority rejection, bounded human/JSON review cards, unknown-slice rejection, and self-contained read-only source dry-run execution |
| focused graph product/docs regressions | 35 passed in a Jazzy-sourced isolated package process; an additional 15 diagnosis regressions cover all five user-reported map symptoms and missing-bag/root-cause boundaries |
| first-map submission UX regressions | 117 passed across support handoff, CLI contract, receipt, acceptance, readiness, and runner suites |
| validator cohort contract and operating state | 33 passed; path-specific immutable runtime identity, byte-bound public documentation, anonymized attempt lifecycle, accepted-ledger evidence binding, four operational stop signals, 48-hour freshness, WIP, batch/target transitions, attempt-10 thresholds, and a one-action human status card are enforced through the CLI; recruitment render remains blocked; no write authority |
| published starter dependency gate | 61 queue regressions pass; the schema-valid live card keeps #422 visible but ineligible under `WAITING_FOR_PUBLIC_GATES`, preserves unrelated starter eligibility, follows the stable issue number across a title edit, rejects arbitrary gate commands or write authority, and performs only GET-only GitHub reads |
| protected candidate environment | 29 passed across the shared environment/candidate gate; complete authenticated inventory, GET-only transport, exact reviewer/self-review/branch policy, unknown-rule refusal, workflow path trigger, and no-write/E2 authority are enforced; actionlint v1.7.12 and CTest 2 / 2 pass |
| G0 readiness dashboard | 5 passed; one-command local HOLD card, optional read-only publication/environment audits, JSON schema, one next action, child-authority refusal, and checker-error fail-closed behavior are covered |
| published onboarding identity and packet | 18 passed; release packet identity is report-derived, manual overrides fail closed, exact tag commit and both live image digests are rechecked, release/candidate modes stay separate, and no trial or publication authority is added |
| affected registered CTest | 4 / 4 pass after a clean Jazzy reconfigure; publication-plan, G0-readiness, packet, and published-identity registrations execute through ament |
| weekly growth snapshot | 14 passed; new snapshots re-derive cohort count/rate/state consistency while the immutable historical baseline remains schema-valid and identity-free |
| focused plan/source/NDT environment regressions | 32 passed after the clean worktree submodules were initialized |
| candidate handoff/session/probe regressions | 67 passed; retained child receipts are byte-bound, Docker observer bootstrap is recipe-labelled, and preparation/execution failure states remain atomic |
| public documentation deployment provenance | 9 focused regressions pass; strict MkDocs produces a deterministic, pre-write schema-valid manifest binding exact source revision, product version, route fragments, byte count, and SHA-256; the read-only live audit correctly remains `BLOCKED` on the current Pages 404 until the reviewed workflow is deployed |
| clean candidate release bundle | exact-head reproducibility rehearsal passes at local validation carrier `9f8a205`; two byte-identical bundles each contain 261 files, total 11,927,637 bytes, and have SHA-256 `51c025064de769d1f0c362f51718c52a0beed8492f0881c0e02403b33498e997` |
| complete maintained Python gate | local `9f8a205`: graph 1,452 passed / 13 skipped / 11 existing ImageIO warnings; lidarslam 1,022 passed; 2,474 total; strict MkDocs and changed-file Jazzy `ament_flake8` pass; latest public `4cb2e68` Actions remain 10 successful / 4 intentional skips / 0 failures; inherited registered CTest remains 93 / 93 plus graph CTest 232 / 232 |
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
