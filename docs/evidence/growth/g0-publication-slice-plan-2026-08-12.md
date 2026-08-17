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
> Planned follow-up inventory: 347 paths; SHA-256
> `18d1ccecba972cdf6acb43a17ef560f48f7161e8afa401db852dda9f9b1053fe`
>
> Composed whole-PR inventory: 394 paths from `86fa9b6`; SHA-256
> `125ce0a496285a48891b0deb14dd9e5551843b1c9dbf6a2af996ac8322db7580`
>
> Capture-time exact public Draft head:
> `c1fc2847fb06637cbcd2aac61f4fde318364dfd2`
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
> Exact single-action system-doctor recovery implementation tip:
> `a83bbfeaea8196a19513c7a26772d500fe8419b8`
>
> Exact independent starter-review implementation tip:
> `e4ce0aa6eb7d53423423a02af65f923472708f44`
>
> Exact starter-publication-handoff implementation tip:
> `0a34e724875d53b8ef74acd8a51fd500ce014ff5`
>
> Exact public-base-gated starter-publication implementation tip:
> `5dc04198549341b33becdeb2bc058117db9fe78f`
>
> Exact unified public-product-transition implementation tip:
> `4404f877263157d09ae6c451dae55f5ddbbd03af`
>
> Exact goal-based README chooser implementation tip:
> `8a8876a2d26c09cc92ad330b99d1fa217db1bd8d`
>
> Exact canonical three-goal onboarding chooser implementation tip:
> `1e56431161d3dea417d5d800bb1eedc3cdb51907`
>
> Exact honest FAIL-without-receipt intake implementation tip:
> `da99c7ef82136449727ef97a58c1a2db4ffd6955`
>
> Exact pre-session bug-intake implementation tip:
> `4b1707cdbc2dc41f3d7b52aa8c598841fc925767`
>
> Exact location-safe Autoware-intake implementation tip:
> `51496ca576b668d9e7dc0e7fda39ebdc21b7e1c8`
>
> Exact redaction-first first-map reporting implementation tip:
> `a0aaadc80b952d92074f45499c29a5103a2ad479`
>
> Exact bounded map-quality symptom triage implementation tip:
> `ee453532a70d2d4b82a6c50c65f19b22d76c239f`
>
> Exact privacy-safe symptom support-handoff implementation tip:
> `0d102e016717d2def3db3a99525755837461f759`
>
> Exact copy-ready G0 slice verification tip:
> `297115d14ea0a979ee0043e24d55a2a80746e382`
>
> Exact S1 rejected-map-update threshold recovery tip:
> `99cce93a07a7cc136eb925c446dd705bdcd7b37c`
>
> Exact S6 ordinary-shell review recovery tip:
> `0633c2a604489538e0f087c02385e7c6467540c3`
>
> Exact S6 review evidence carrier:
> `72a8c9e77eba33c1578a3cd9c8afe8fbe6933e33`
>
> Exact complete local validation carrier:
> `72a8c9e77eba33c1578a3cd9c8afe8fbe6933e33`
>
> Exact fail-closed docs artifact implementation tip:
> `5b8c8c477cceb4955184a64afa874712b9dea5aa`

> Exact fail-closed NDT reviewer-response implementation tip:
> `3e11f307eb2ccea1d33bbe9a2d1b37ae7ed699db`

> Exact NTU attached-storage implementation tip:
> `8a856f521de825976c80c6a3c410224c4fb4e433`
>
> Exact claim-bounded social-media generator tip:
> `d0c84bb9bb7bef37d7e318000e3071a7f536d631`
>
> Last observed remote mutation: an exact non-force push advanced the Draft to
> `7b3cb99`; mutations performed by the checker or review-card command:
> **none**

## Outcome

The local GLIM-convenience, release-evidence, dependency-gated contributor
next-action, copy-ready low-storage, and bounded visual-symptom UX follow-up is
split into seven dependency-ordered review focuses. Every tracked or untracked
path relative to the exact PR base has exactly one primary review owner. A
fail-closed checker rejects missing, stale, duplicated, unsafe, or
digest-drifted paths and rejects a plan that claims GitHub write authority.
The same checker composes the original 116-path audit, the exact two-commit /
11-path CI bridge, and this 347-path follow-up. Their union must equal the
current 394-path whole-PR diff with no uncovered or extraneous path. The bridge
is reviewed explicitly in
[`g0-pr-review-coverage-2026-08-17.md`](g0-pr-review-coverage-2026-08-17.md).

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
- the captured public PR head resolves to `c1fc284`;
- ten GitHub Actions checks pass on that exact head and four publication jobs
  skip intentionally, including green Humble/Jazzy builds, default workflows,
  upgrade checks, documentation, and release-readiness guards; and
- no PR conversation, inline review, or submitted review was present at the
  observation time.

Passing checks on `c1fc284` validate the public Draft candidate through the
other-PointCloud2 self-service and retained visual-symptom triage increments,
including the published-onboarding identity, atomic paired recorder, evidence
sync, ament import-order follow-up, actionable empty-release-evidence report,
the GET-only contributor next-action card, dependency-gated #422 handling,
independent local starter-review progress while that issue is blocked, and
copy-ready low-storage recovery, pre-upload docs artifact gate, and fail-closed
NDT reviewer-response packet. That packet remains read-only and emits no reply
body until the exact canonical upstream Draft exists.

## Review order

| Slice | Paths | Reviewer outcome | Gate |
| --- | ---: | --- | --- |
| S1 runtime safety | 16 | unsafe point-cloud and VoxelGrid layouts, plus readable samples with empty frame IDs, fail closed without losing valid fields; a rejected map update preserves the movement threshold for an immediate safe retry, asynchronous exceptions stay inside the component boundary, and low-capacity test environments do not weaken the production reserve | public Humble/Jazzy CI |
| S2 first-map foundation | 34 | one bounded demo/own-bag route reaches a verified local 3D result, retains recovery state, diagnoses recorded Odometry-to-TF connectivity and replay-order future gaps, and returns the exact shortage plus preserved retry command on low storage | public Humble/Jazzy CI |
| S3 map lifecycle | 25 | setup, history, compare, edit, merge, and support preserve provenance and receipts | focused local review |
| S4 source onboarding | 38 | a fresh terminal uses the exact six-package Humble/Jazzy source route, bounded release-or-candidate Docker/source measurement, a read-only guided host-readiness card, content-bound Docker observer bootstrap, one-command run-to-session execution, authenticated exact-host GET-only public preflight and fixture audit, and byte-bound validation-receipt plus SHA-bound supplement paths for retained observations | public Humble/Jazzy CI and clean-machine timing |
| S5 distribution readiness | 74 | NDT convergence, its copy-ready upstream PR packet, v0.9.1 metadata, exact-head/tag-aware bundle rehearsal, actionable missing-benchmark reports, write-free NTU and RTK-SLAM acquisition planning with official immutable identities, exact byte shortages, attached-storage discovery, and capacity fail-fast, immutable upstream patch formatting, clone-free launcher identity, authenticated package-manager blockers, and a default-branch, protected-environment, digest-only candidate gate with a shared read-only environment preflight and four-artifact byte audit remain explicit; no mount, E2 dispatch, tag, release, or version reuse is implied | maintainer distribution decision |
| S6 product-shell integration | 153 | the coherent three-goal README and canonical Getting Started choosers, installed home, path-private bag-optional doctor with one dependency-ordered **Do this now** action, copy-ready low-storage and Odometry/TF timing recovery, issue-driven other-PointCloud2 `doctor` → `start` handoff without tracked launch/YAML edits, bounded user-reported visual-map triage through the retained-run inspector, Japanese quickstart, byte-bound public-docs deployment provenance, parse-safe and content-verified GLIM comparison with atomically visible collision-preserving cache entries, neutral GLIM usability scorecard with a schema-bound GET-only public-pair preflight, SHA-bound preparation receipt, retained untouched source archive, and receipt-required final validation, claim-bounded short-demo card/video/captions/copy, fail-closed worksheet generators, atomic paired observation recorder, truthful onboarding/growth snapshots, bounded starter queue with a dependency-gated and schema-valid GET-only live card that keeps independent maintainer review moving while #422 is blocked, binds one title/labels/body handoff without granting issue authority, and requires PR #427 merged plus an exact public-`develop` queue match before publication review, source- and evidence-hashed issue-triage review packets whose #69 Draft/CI and stable-release claims are source-bound and live-checked, a shared helper that obtains existing gh credentials without exposing them and attaches them only to exact GitHub HTTPS GETs, machine-evaluated validator cohort, CLI contract, docs, tests, support surface, schema-valid first-map handoff JSON, honest FAIL-without-receipt validation intake, pre-session bug intake without fake ZIP claims or weakened diagnostics/privacy requirements, and location-safe Autoware intake that forbids precise projector coordinates and map geometry, atomic one-command candidate handoff, report-derived release or four-file-candidate exact-identity observer packet, live identity recheck, one-option Draft/environment/release transition audit with a schema-bound fresh-packet gate, compressed-bag playback evidence, one-command G0 readiness dashboard, role-based Draft review routing, and append-only anonymous lane evidence agree | complete product gate and public CI |
| S7 publication control | 7 | all 347 follow-up paths are owned once, all 394 whole-PR paths are covered by three sequential review phases, and external authority remains separate; the current action packet keeps E2/E3/E4 separate | exact-tip maintainer decision |

The machine-readable source of truth is
[`g0-publication-slice-plan-2026-08-12.json`](g0-publication-slice-plan-2026-08-12.json),
validated against
[`publication-slice-plan-v1.schema.json`](../../schemas/publication-slice-plan-v1.schema.json)
by `python3 scripts/check_publication_slice_plan.py --json`.

The companion [review routing](../../review-routing.md) groups S1–S7 into four
capability lanes without naming people: R1 runtime safety, R2 operator UX, R3
distribution, and R4 integration/publication. Its advisory target of two
reviewers is explicitly not a merge gate. The checker stores no username,
email, or organization and grants no reviewer-request, review, mark-ready, or
merge authority:

```bash
python3 scripts/check_product_draft_review_routing.py
```

Reviewers can request one bounded, read-only card without manually extracting
paths or commands from the complete plan:

```bash
python3 scripts/check_publication_slice_plan.py --overview
python3 scripts/check_publication_slice_plan.py --overview --json
python3 scripts/check_publication_slice_plan.py --slice S1-runtime-safety
python3 scripts/check_publication_slice_plan.py \
  --slice S1-runtime-safety --json
```

The overview revalidates the complete inventory and lineage first, then renders
one compact PR card with exact local tip, whole-PR commit/path totals, the three
sequential review ranges, all seven slice counts/dependencies/gates, overlap,
missing/extraneous-path results, merge count, current worktree cleanliness, and
an exact Git-derived review budget. The budget reports textual additions and
deletions, binary paths, and the three largest textual deltas for every phase
and slice. Binary paths are named in the exact slice card so media review is
not hidden behind a count.
It intentionally omits hundreds of individual paths; each slice card remains
the exact drill-down for its owned paths, outcome, and verification commands.
The baseline is an immutable review anchor, not a live claim about the remote
branch. A dirty worktree is shown with its uncommitted path count rather than
being mislabeled as an exact-tip candidate. Neither card executes commands or
can authorize or report a GitHub mutation. Overview and slice modes are
mutually exclusive, and an unknown slice ID fails closed while listing the
seven valid IDs.

The displayed verification commands are also part of the fail-closed
contract. Every pytest command disables its cache, one process may target only
one package test root, and commands that need ROS bag or colcon imports source
`/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash` themselves. This honors a caller's
Humble/Jazzy selection and gives an unsourced dual-distro review host a Jazzy
default. The checker rejects multiline or oversized commands and obvious
`git push`, PR/issue mutation, workflow-dispatch, or review submission
commands. The card still does not execute anything automatically.

Network-read-only checks use the shared GitHub API helper. It reuses an
explicit token or the existing `gh auth` login without printing the credential,
adds authorization only for exact `https://api.github.com` GET requests, and
rejects other hosts, schemes, ports, userinfo, and non-GET methods. This avoids
anonymous-rate-limit false blockers without creating GitHub write authority.

## Fail-closed invariants

The checker derives the candidate directly from Git rather than trusting the
human table. It combines tracked changes from exact PR base `3f4dd70` with
untracked, non-ignored paths, verifies that public baseline `3ed632e` descends
from that base and the local tip descends from the public head, then requires:

1. seven consecutive, dependency-safe slice orders;
2. sorted and canonical repository-relative paths;
3. one and only one owner for every candidate path;
4. exact 347-path follow-up coverage with the fixed inventory digest;
5. a local-only authority state with no claimed GitHub write; and
6. a report that always states whether a remote mutation occurred;
7. no mixed `lidarslam/test` and `graph_based_slam/test` pytest process;
8. an explicit ROS source prelude for ROS-dependent checks;
9. cache-disabled pytest commands; and
10. no recognized direct remote-write CLI form in any review slice;
11. exact 42-commit / 116-path initial-review identity;
12. exact two-commit / 11-path bridge identity and allowlist;
13. contiguous, linear initial → bridge → follow-up ancestry;
14. exact commit-count composition across all three ranges; and
15. exact 394-path whole-PR coverage by the three-phase union, with no missing
    or extraneous path; and
16. exact `git diff --numstat` path identity for the whole PR and all three
    phases before any review budget or hotspot is rendered.

Adding, removing, renaming, or reassigning a path invalidates the plan until a
reviewer updates both the exact inventory and its digest. A green schema alone
cannot bypass live Git coverage.

## Verification

| Check | Result |
| --- | --- |
| exact Git-derived plan check | `PLAN_VALID_LOCAL_ONLY`; 347 follow-up paths / 7 slices and 394 whole-PR paths / 3 phases; 11 bridge paths, 74 overlapping paths, 0 uncovered, 0 extraneous, 0 merge commits, no remote mutation |
| privacy-safe review routing | 4 role-based lanes cover all 7 slices, 347 paths, and 34 verification groups exactly once; advisory target 2, no stored identities, no reviewer request/review/ready/merge authority |
| anonymous review ledger | 9 focused regressions pass; exact-head/routing digest, append-only event sequence, lane dependencies, in-slice paths, bounded privacy-safe findings, canonical atomic external output, historical/current blocker separation, bundle inclusion, and all no-GitHub-authority fields are enforced |
| claim-bounded social media | 11 focused regressions plus 25 public docs/release entrypoint regressions pass; the generated 10.666-second H.264 candidate, four-cue WebVTT, exact-revision Japanese/English copy, and byte manifest retain no external publication authority |
| checker regressions | 34 passed, including omission, stale path, duplicate owner, dependency inversion, follow-up/bridge/whole-PR digest drift, bridge allowlist drift, phase discontinuity, uncovered whole-PR paths, unsafe or out-of-phase review records, lineage drift, commit-range composition, malformed/stale numstat rejection, exact line-budget composition across the seven slices, named binary review paths, authority rejection, bounded human/JSON overview and slice cards, mutually exclusive output modes, unknown-slice rejection, self-contained source, lifecycle ROS-bag, and product-shell ROS-bag verification, clean-checkout build-before-test enforcement, package-test process isolation, cache suppression, and recognized direct remote-write CLI refusal |
| canonical NDT Draft handoff | 25 focused regressions pass; only a 30/30 `READY_FOR_DRAFT_PR` report may carry the exact create-only branch, base/head SHAs, Draft copy, and four-step handoff, while blocked or partially checked states require `null` and all GitHub write, force, ready, and merge authority remains false |
| v1 live child-state propagation | 18 focused regressions pass; the parent accepts exactly the six package-manager report states, exposes the observed child status in schema-valid JSON and the human tuple, keeps unknown states invalid, and requires `READY` before distribution can close |
| exact displayed S1 command | from a clean checkout, an ordinary unsourced shell sourced `${ROS_DISTRO:-jazzy}`, built every tested package and its dependencies, tested `graph_based_slam` and `scanmatcher`, and reported 3,076 test cases with 0 errors, 0 failures, and 126 skips |
| S1 rejected-map-update threshold retry | exact implementation `99cce93`; one pure commit-state regression passes; the real asynchronous component rejects an unsafe translated cloud above a positive 0.02 m threshold, publishes the same-geometry safe retry without further travel, and passes 10 / 10 independent Jazzy processes; worker exceptions stay inside the component boundary; exact public `7b3cb99` runs the component target successfully on both Humble and Jazzy, while issue response and named release remain separate gates |
| exact displayed S6 product-shell command | command-contract implementation `0633c2a`, three-goal entrypoint tips `8a8876a`/`1e56431`, honest FAIL intake `da99c7e`, pre-session bug intake `4b1707c`, location-safe Autoware intake `51496ca`, and redaction-first first-map reporting `a0aaadc`; an ordinary unsourced shell restores `${ROS_DISTRO:-jazzy}` before importing `rosbag2_py` and passes all 41 graph docs/product-CLI tests; deleting the prelude now fails plan validation before a reviewer sees the card |
| focused graph product/docs regressions | 41 passed in a Jazzy-sourced isolated package process, including matching first-goal boundaries, parsed first-map and bug-form semantics, required `REDACTED` command-shape semantics, and parsed Autoware command/projector/artifact/privacy requirements that forbid precise locations and geometry; an additional 15 diagnosis regressions cover all five user-reported map symptoms and missing-bag/root-cause boundaries |
| redaction-first first-map reporting | exact implementation `a0aaadc`; the public form requires a redacted command shape with literal `REDACTED` placeholders while preserving executable/options/non-private values, and the read-only handoff prints four field-by-field completion lines from safe environment hints; 34 focused docs and 25 support/installed-contract regressions pass without changing `first-map-handoff-v1` | independent reports remain reproducible without soliciting credentials, private paths, host/user names, precise locations, or map geometry; no upload, issue, cohort acceptance, or GitHub write is performed |
| first-map submission UX regressions | 117 passed across support handoff, CLI contract, receipt, acceptance, readiness, and runner suites |
| validator cohort contract and operating state | 33 passed; path-specific immutable runtime identity, byte-bound public documentation, anonymized attempt lifecycle, accepted-ledger evidence binding, four operational stop signals, 48-hour freshness, WIP, batch/target transitions, attempt-10 thresholds, and a one-action human status card are enforced through the CLI; recruitment render remains blocked; no write authority |
| published starter dependency gate | 71 queue regressions pass; the schema-valid live card keeps #422 visible but ineligible under `WAITING_FOR_PUBLIC_GATES`, preserves unrelated starter eligibility, prioritizes duplicate review then one independent local maintainer preview, binds its exact title/labels/heading-free body and three digests, and requires PR #427 merged plus an exact canonical queue match on public `develop` before publication review; current GET-only state is `WAITING_FOR_PRODUCT_MERGE` / queue `ABSENT`, while closed-unmerged, missing, drift, wrong-base, wrapped-base64, 404, body/task tampering, arbitrary gate commands, and write authority all fail closed |
| issue-triage application packet | 34 application and 19 proposal regressions pass; the complete authenticated live audit remains `PASS` for all 29 open issues and the label catalog, source-binds #69 to exact public Draft head `4b2ab514` with 10 successful checks and 4 intentional skips, verifies latest stable `v0.9.0`/`0df0c4a` is 52 commits behind fix `a2368c4` while `v0.9.1` tag/release remain absent, produces 23 closure drafts, 4 reproduction requests, 9 dependency-review rows, and one monitor-only row, and keeps every write authority false |
| protected candidate environment | 29 passed across the shared environment/candidate gate; complete authenticated inventory, GET-only transport, exact reviewer/self-review/branch policy, unknown-rule refusal, workflow path trigger, and no-write/E2 authority are enforced; actionlint v1.7.12 and CTest 2 / 2 pass |
| G0 readiness dashboard | 25 focused regressions pass; one-command local HOLD card, mandatory 394-path / three-phase whole-PR review coverage, bounded exact-head Draft/CI audit, clean-worktree refusal, authenticated exact-host GET-only transport, a schema-bound overview → P0/P1/P2 → S1–S7 → R1–R4 review sequence, optional anonymous ledger summary, Draft → separate merge → environment dependency order, one-option complete public transition audit, release-state-bound fresh-packet eligibility, unsafe or mismatched version refusal, one next action, child-authority refusal, and checker-error fail-closed behavior are covered |
| published onboarding identity and packet | 18 passed; release packet identity is report-derived, manual overrides fail closed, exact tag commit and both live image digests are rechecked, release/candidate modes stay separate, and no trial or publication authority is added |
| affected registered CTest | 4 / 4 pass after a clean Jazzy reconfigure; publication-plan, G0-readiness, packet, and published-identity registrations execute through ament |
| weekly growth snapshot | 14 passed; new snapshots re-derive cohort count/rate/state consistency while the immutable historical baseline remains schema-valid and identity-free |
| focused plan/source/NDT environment regressions | 32 passed after the clean worktree submodules were initialized |
| candidate handoff/session/probe regressions | 67 passed; retained child receipts are byte-bound, Docker observer bootstrap is recipe-labelled, and preparation/execution failure states remain atomic |
| public documentation deployment provenance | 9 focused regressions pass; strict MkDocs produces a deterministic, pre-write schema-valid manifest binding exact source revision, product version, route fragments, byte count, and SHA-256; the read-only live audit correctly remains `BLOCKED` on the current Pages 404 until the reviewed workflow is deployed |
| attached RTK storage recovery | exact implementation `0c3f588`; 14 direct downloader regressions, 42 combined RTK/docs checks, 27 release/bundle checks, and changed-file Jazzy style pass; the real plan discovers the attached unmounted 2 TB `/dev/sda1`, leaves free capacity unverified, selects one mount action, and provides copy-ready `--dest-device` preflight/live commands without mounting, probing contents, writing, or starting network work |
| attached NTU storage recovery | exact implementation `8a856f5`, registration `657746f`, and exact-byte follow-up `d6e8bad`; 34 shared/NTU/RTK acquisition checks and 9 release-bundle checks pass; the real 49,209,878,965-byte plan reports the exact root shortfall, discovers the same unmounted 2 TB `/dev/sda1`, and provides option-preserving `--dest-device` commands; the curated bundle now contains the documented NTU helper and resolver |
| clean candidate release bundle | exact-head reproducibility rehearsal passes at Odometry-TF timing carrier `4bdd7ec`; two byte-identical and reverified bundles each contain 271 manifest files, total 11,965,449 archive bytes, and have SHA-256 `735a3683be43cfb2e2638e466b02b140339beb9da573bc5642872219090fc6a9`; this is local carrier evidence only |
| complete maintained Python gate | Odometry-TF timing carrier: source-explicit graph 1,489 passed / 13 skipped / 11 existing ImageIO warnings; source-explicit lidarslam 1,040 passed; 2,529 total; 74 focused tests, strict MkDocs, and changed-file Jazzy `ament_flake8` pass; exact public CI remains to be rerun on the later Draft head |
| paired scorecard recorder | 7 direct regressions, 20 recorder/checker regressions, and registered CTest 6 / 6 pass; incomplete observations remain non-comparable and atomic output/privacy boundaries fail closed |
| paired scorecard public identity | 16 preparer, 10 recorder, and 17 checker regressions plus graph 1,489 / 13 skipped and lidar_slam 1,094 pass; offline output stays non-public, exact canonical commit/tag/image/docs GETs are schema-bound, manual publicity flags fail closed, all three preparation outputs roll back together, the recorded session retains the exact source archive, and receipt-less, archive-tampered, or identity-drifted records cannot become CLI/index `READY`; the real GET preflight resolves public Draft `4b2ab514…` and GLIM `v1.2.2` → `faa264a1…` with both docs at HTTP 200; no observation or write authority is inferred |
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
