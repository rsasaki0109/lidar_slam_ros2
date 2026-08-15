# G0 current action packet — refreshed 2026-08-16

> Status: **CURRENT_PUBLIC_DRAFT_HANDOFF / NO_E2_E3_E4_ACTION_TAKEN**
>
> Repository: `rsasaki0109/lidar_slam_ros2`
>
> Draft PR: [#427](https://github.com/rsasaki0109/lidar_slam_ros2/pull/427)
>
> Capture-time public Draft baseline: `2a92cc5704fa55c16ddee343a601edc928a542c2`
>
> Exact reviewed product-candidate tip: `3d64ed556aca8a680f09e0f7e8c12a3c8d3e6a6d`
>
> Latest product/community follow-up tip: `bb1c2e7431c0634f9e5ba6613864b6d2a4c99eb0`
>
> Latest distribution-audit follow-up tip: `ca7c5b5b991e5624ca16e46ffd1a057e3a9f6ee9`
>
> Latest canonical-NDT publication-preflight tip: `856e59987018578963a7afdf13402200eab62bf8`
>
> Latest Docker publication-authority tip: `3225d9db357caa1150081ac61281ae4b0d281a2a`
>
> Latest immutable-candidate gate tip: `c70c18d32e0dd860969dbd050fa3a92632f1106e`
>
> Latest candidate-gate CI follow-up tip: `87f8f8d1bcdccb80235b8ae1832f9bc716c31d36`
>
> Latest tag-free candidate-observer tip: `363a971bfeb2a5ddf0b99fb9b20f5a201134a520`
>
> Latest one-command candidate-trial preparation tip: `f5ed80e11e3735b7a72ba458f740d0117711be6c`
>
> Latest one-command candidate-row execution tip: `feed0ba7a12135099b9eef299e914605ab2947e1`
>
> Latest one-command candidate-session tip: `8bc5ea4036277de1a1819008183f989a2c4bcb76`
>
> Latest guided candidate-readiness tip: `a286c6551f976c48d508ee7e9ecd7d9d4a30e734`
>
> Latest protected-environment preflight tip: `adecca6e19c38e6a682253d66516379bbface46d`
>
> Latest operator-handoff and GLIM parse-safety tip: `e2d916a66a57146b0efe4c74e57218d56342ed37`
>
> Latest publication-inventory tip: `e2d916a66a57146b0efe4c74e57218d56342ed37`

This reviewed tip is the code-bearing product-candidate revision; later
docs-only handoff synchronization and product UX follow-up commits must remain
identified separately.

This is the current, read-only handoff for the G0 release-hygiene decision.
It was first captured on 2026-08-14 and refreshed on 2026-08-16 after the
dashboard UX, CI-registration, version-priority, final PR-head CI,
packet-command-contract, fail-closed usability-worksheet, paired scorecard,
safe observer-packet-output, safe first-map-dry-run-plan-output, Docker JSON
own-bag-plan, source JSON quickstart-plan, custom PointCloud2 onboarding,
supported g2o recovery, canonical C2/C3 drift detection, contributor C1–C4
local-retirement, bounded contributor C5–C9 replenishment, and
publication-inventory follow-ups. The latest observer UX follow-up also turns
four artifact downloads, independent remote byte audit, and JSON/Markdown
packet generation into one fail-closed command with atomic local output. The
latest distribution slices also scope
optional GitHub authentication to read-only API requests, make explicit or
unknown NDT PR mergeability fail closed, restore exact-tip source-route
preflight under shared-IP quota exhaustion, and bind package-manager workflow
evidence to the exact immutable source tag commit. The package audit now keeps
missing refs, absent attempts, running attempts, failed attempts, and API or
identity failures separate. It does not print a dispatch command while the
required tag is absent.
The NDT review audit now also binds all check runs to each rosdistro PR's exact
head, blocks failed, pending, absent, inconsistent, or truncated check
evidence, and keeps unanswered-review actions visible beside a CI blocker.
The separate canonical-upstream publication preflight binds one clean local
candidate commit to the checked-in binary patch, verifies its exact parent and
subject, reads the current upstream branch and expected fork identity, and
fails closed if the proposed branch already exists, GitHub inspection fails,
or any open upstream PR matches the branch or semantic duplicate terms. Its
30 / 30 PASS result is technical evidence only: GitHub write authority remains
false and no upstream branch or PR was created.
The Docker workflow now separates verification from publication at the job
and token boundary. Pull requests and manual dispatches have contents-read
permission, build with `push: false`, load only into the disposable runner,
and cannot log in, attest, publish a package, or move a tag. Only the separate
job gated to a `develop` push can update the moving convenience tags. This
closes an accidental-publication path. The separate immutable-candidate gate
is now implemented at `c70c18d…`: it has no `workflow_dispatch`, runs its
write-capable path only from a default-branch `repository_dispatch`, validates
the exact same-repository PR head, VERSION, nine successful checks,
maintain/admin role, literal E2 approval, and a required-reviewer environment
restricted to `develop`, then publishes Humble/Jazzy by digest without tags.
Its request, per-image, and pair records preserve exact identity and state that
registry retention still requires a remote audit. The workflow is not yet on
`develop`, the live `candidate-images` environment is absent, no dispatch was
sent, and no candidate digest was published. The capture-time public baseline
also contains the CTest registration and cross-distro import-order repair for
the gate regression; both default workflows pass at that exact revision.
The tag-free candidate-observer follow-up at `363a971…` now requires one
canonical four-file evidence directory, re-derives both image records from the
authorized request and the set from those records, and hashes every retained
file. Remote mode re-downloads the exact four workflow artifacts into a
temporary directory, byte-compares them, removes the copies, and then checks
both manifests and attestations. Observer packet v3 and each candidate trial
record retain both bundle and set SHA-256 values plus source/run/image
identity, without inventing a release tag. This is local contract readiness
only: no candidate evidence bundle exists yet, no remote audit was run, and no
trial was executed.
The one-command preparation at `f5ed80e…` accepts only one exact repository
Actions run URL and one new output directory. It downloads all four canonical
artifacts, invokes the remote audit (which independently downloads all four
again), requires `REMOTE_AUDIT_PASS`, builds packet v3, and publishes the
artifacts, audit, JSON/Markdown packet, and schema-backed preparation receipt as
one directory. Any acquisition, identity, byte, registry, or attestation
failure removes staging and leaves the requested output absent. The contract
records network reads and local writes, but `trial_executed`, GitHub/registry
write authority, and remote mutations remain false.
The one-command row runner at `feed0ba…` consumes that complete handoff with one
row ID and one new output directory. It rebuilds the packet from artifact
bytes, runs only the selected row's live preflight, derives probe arguments
from structured identity, prompts for human observations only on a TTY, and
atomically distinguishes blocked preflight, valid PASS/FAIL evidence, and a
harness error. It neither creates candidate evidence nor expands E2/E3/E4
authority.
The one-command session at `8bc5ea4…` removes the remaining transfer/copy step
when a disposable row host can read the exact Actions run directly. It prepares
the authenticated handoff, runs one selected structured row, verifies the
retained child receipt bytes, and atomically publishes `handoff/`, `execution/`,
and a schema-backed `session.json`. Docker rows derive a local observer tag from
the exact Dockerfile SHA-256, build it only when absent and before timing, then
require contract, Ubuntu, and recipe labels plus its immutable local image ID.
It does not replace an existing tag or expand remote-write authority.
The guided readiness follow-up at `a286c65…` keeps that single command surface
and adds a read-only `--check-readiness` mode. It validates the exact request,
Ubuntu/ROS row, x86_64 host, measured filesystem and free-space floor, local
Docker or source runtime, source RX counter, neutral-observer measurement mode,
and explicit isolation confirmation before any network read, image build,
source mutation, evidence write, or trial. Its schema distinguishes `BLOCKED`,
`CONFIRMATION_REQUIRED`, runnable-but-`READY_NONCOMPARABLE`, and `READY`, then
prints one shell-safe next command. It neither proves human isolation nor turns
local host readiness into comparable evidence.
The protected-environment preflight at `adecca6…` first reads the complete
repository environment inventory, then reads the exact environment and
deployment-policy documents only when `candidate-images` is present. It shares
its reviewer, **Prevent self-review**, known-rule, and exact `develop` policy
validator with the publication authorization job. The authenticated live
inventory contains only `github-pages`, so the stable result is `ABSENT`, not
an inference from a 404. Its schema and the G0 dashboard keep environment
writes, artifact publication, remote mutation, and E2 dispatch authority
false even when the result eventually becomes `READY_FOR_SEPARATE_E2_REVIEW`.
The status-specific operator handoff at `e2d916a…` now prints the trusted settings URL and
exact creation/repair checklist only when complete evidence justifies it;
`BLOCKED` requests read-access recovery instead. It always preserves a
copy-ready GET-only verification command and `writes_performed: false`, so the
administrator still performs and independently reviews any settings change
outside this packet.
The code-bearing packet tip is required to be an ancestor of the current
checkout revision; later synchronization and product UX follow-up commits
must remain described in this handoff. It replaces
neither the historical 2026-08-11 decision packet nor any maintainer approval.
Its purpose is to prevent an old commit, old version, or one external action
gate from being mistaken for the current state.

## Current evidence

| Check | Current result | Meaning |
| --- | --- | --- |
| Draft PR #427 | open and draft at capture-time public baseline `2a92cc5…`; guided readiness is public, while protected-environment preflight tip `adecca6…` and operator-handoff/GLIM parse-safety tip `e2d916a…` with its 263-path inventory are the reviewed local follow-ups | the new source tip still requires its own public CI; the preflight and handoff change no E2 authority |
| Capture-time PR-head CI | completed public exact-tip result is **PASS** for `2a92cc5…`: 10 successful checks plus 4 intentionally skipped non-publication jobs, 0 failures | the skipped jobs are candidate authorization/publication/pair verification and Docker publication; green CI is not release/E2 approval |
| English support cards | docs entrypoint tests 25 passed | C1 g2o recovery is implemented; existing C2 empty-map and C3 Odometry/TF cards remain copy-ready and safety-bounded; Docker convenience and candidate-digest authority boundaries are both regression-bound; every tracked shell entry point now also has a parse regression |
| Custom PointCloud2 onboarding | implemented in the reviewed product UX tip | bounded topic/frame/time/TF/range/launch readiness guidance; it does not claim hardware support or accuracy |
| Contributor starter queue | C5–C9 `READY_LOCAL_ONLY`; 50 queue regressions and all five focused strict-MkDocs profiles passed | C1–C4 remain completed and retired; the fresh duplicate audit found no matching implementation PR, and no issue or label mutation occurred |
| Distribution preflights | source route `READY` at exact public `2a92cc5…`; rosdistro NDT remains `BLOCKED`: Humble #52949 and Jazzy #52950 each have 5 / 6 exact-head checks passing, one failing, and an unanswered review; package-manager E2E is `SOURCE_REF_MISSING` because `v0.9.1` does not resolve, with zero matching runs | the rosdistro failures are stale-base rosdep failures rather than the YAML delta, but neither external PR is green; collision-free convergence and current-base green replacement still precede clean-install E2E |
| Canonical NDT upstream Draft preflight | `READY_FOR_DRAFT_PR`; 30 / 30 PASS at local implementation `856e599…`; exact upstream `5495fd9…`, expected fork verified, proposed branch absent, 4 open PRs inspected, 0 duplicates, 0 API errors, and write authority false | this proves a technically coherent read-only publication state; it neither creates nor authorizes an upstream branch or PR |
| Docker publication boundary | convenience PR/manual runs remain verification-only; the candidate gate at `c70c18d…` uses trusted default-branch tooling, exact-head CI/identity checks, a protected `candidate-images` environment, digest-only output, disabled container networking during smoke tests, SBOM/provenance/attestation checks, and 30-day schema-backed evidence | the gate can create no tag or Release; complete authenticated inventory at `adecca6…` proves only `github-pages` exists, so authorization must stop until a separate environment/E2 decision |
| Candidate environment and gate regressions | 29 focused tests, actionlint v1.7.12, Python style, CTest 2 / 2, GET-only transport, shared authorization semantics, release-bundle inclusion, and exact-tip Humble/Jazzy default workflows pass | the live result is `ABSENT`; workflow-facing CLIs persist one request, two distinct image records, and one pair report exactly once; no workflow dispatch, environment mutation, or GHCR mutation occurred |
| Candidate observer contract | atomic preparation through `f5ed80e…`, exact row runner `feed0ba…`, one-command session `8bc5ea4…`, and guided local readiness `a286c65…`; four-file semantic derivation, exact remote artifact-byte comparison, content-bound Docker observer bootstrap, retained child-receipt binding, structured row execution, four-state host guidance, release-bundle inclusion, and docs pass 40 direct preparation/runner/session regressions plus 85 focused session/row/probe regressions | remote status is still **NOT_CHECKED** because no authorized bundle exists; local readiness and runner tests are not `REMOTE_AUDIT_PASS`, a trial, E2, or E4 authority |
| Complete local product gate | graph 1,442 passed / 13 skipped / 11 existing ImageIO warnings; lidarslam 948 passed; strict MkDocs, focused integration tests, CTest, Python style, 122 tracked JSON documents / 82 tracked shell scripts, all 77 first-party Draft 7 schemas, and diff checks pass at implementation tip `e2d916a…` | 2,390 local tests validate the exact feature candidate; the tracked-shell parse gate now prevents the repaired GLIM comparison branch from regressing, and public CI is still required for the later branch tip |
| Publication slice plan | `PLAN_VALID_LOCAL_ONLY`; 263 paths / 7 slices at implementation tip `e2d916a…`; inventory SHA-256 `f37f6a327beb5792f05f1cf791014893d9fa7228445ec54b5bd62010eb2be1fb` | environment and candidate audit paths belong to S5; row/session/readiness and parse-safe GLIM comparison belong to S4/S6; packet and atomic preparation belong to S6; synchronization changes no GitHub authority |
| v0.9.1 release audit | **NOT_PUBLISHED** | no `v0.9.1` tag or GitHub Release was found |
| v0.9.1 GHCR images | **ABSENT** for `v0.9.1-humble` and `v0.9.1-jazzy` | no immutable candidate image identity exists |
| Onboarding matrix | 4 / 4 product PASS; 0 / 4 comparable; **BLOCKED** | Docker is v0.9.0, source is v0.9.1, and human measurements are missing |
| v1 readiness | **8 / 10** | distribution and independent adoption remain incomplete |
| Accepted independent maps | **0 / 3** | cohort remains closed |

The exact public checks are intentionally re-runnable:

```bash
gh pr checks 427 --repo rsasaki0109/lidar_slam_ros2
python3 scripts/check_publication_slice_plan.py --json
python3 scripts/check_ndt_omp_release_readiness.py --json
GITHUB_TOKEN="$(gh auth token)" \
python3 scripts/check_canonical_ndt_convergence.py \
  --upstream-checkout "${NDT_UPSTREAM_CHECKOUT:?}" \
  --candidate-checkout "${NDT_CANDIDATE_CHECKOUT:?}" \
  --online --require-ready-for-draft-pr --json
python3 scripts/check_package_manager_release_readiness.py \
  --version 0.9.1 --json
python3 scripts/run_source_onboarding_probe.py \
  --public-preflight \
  --source-commit 2a92cc5704fa55c16ddee343a601edc928a542c2 \
  --product-version 0.9.1
GITHUB_TOKEN="$(gh auth token)" \
python3 scripts/check_candidate_environment.py --json --require-ready
python3 scripts/check_published_release.py --version 0.9.1 --json
python3 scripts/check_onboarding_trial_matrix.py --json
python3 scripts/check_v1_readiness.py --json
python3 scripts/first_map_validator_cohort.py --json
python3 scripts/check_g0_readiness.py \
  --include-candidate-environment \
  --include-published-release \
  --published-release-version 0.9.1
```

These commands perform read-only inspection. The last command must remain
`WAITING_FOR_PUBLIC_GATES` while the matrix is not comparable; `--render` must
not be used as a reason to recruit.

## Separated action gates

| Gate | Current state | Authorized scope |
| --- | --- | --- |
| L0 local preparation | complete for this packet | code, tests, docs, offline audits, and read-only inspection |
| E1 source review | public Draft baseline / CI PASS; latest preflight local | publish and review the exact follow-up tip; no merge is implied |
| E2 artifact hosting | **GATE_IMPLEMENTED_LOCAL / ENVIRONMENT_ABSENT / NOT_AUTHORIZED / NOT_PUBLISHED** | after the workflow is reviewed on `develop`, separately configure and review the protected environment; only a later exact E2 event may request digest-only candidate evidence |
| E3 community mutation | **NOT_AUTHORIZED** | no issue labels, comments, closures, starter issues, Discussions, or recruitment |
| E4 stable release | **HOLD / NOT_AUTHORIZED** | no tag, GitHub Release, package, image promotion, or announcement |

Approval of one row never approves another. In particular, green CI does not
authorize E2, E3, or E4, and a published identity would not by itself create a
comparable human trial.

## Safe transition order

1. Review the code-bearing candidate tip and this refreshed packet; confirm
   the current PR-head CI remains green, then stop if the branch, PR, or local
   inventory drifts. Do not measure mixed-version rows.
2. Review the dedicated candidate workflow at `c70c18d…` and shared
   environment preflight at `adecca6…`. Do not merge it or
   configure `candidate-images` as an implication of E1; the environment and
   its required reviewer/develop-only policy are a separate repository-admin
   decision. Never use the convenience-image manual dispatch as a substitute.
3. If E2 is separately chosen after the gate is on `develop` and the protected
   environment passes the read-only audit, send one exact
   `e2-publish-candidate-image` event. Publish only the two untagged candidate
   digests. On each prepared disposable row host, prefer
   `start_candidate_trial.py --workflow-run-url ... --row ... --output-dir ...
   --acknowledge-dedicated-trial-host --check-readiness` and require `READY`
   before running the exact command it prints. Require the session's retained
   `READY_FOR_OBSERVER` handoff, `REMOTE_AUDIT_PASS`, and terminal
   `session.json`; a missing output on pre-contract failure is intentional.
4. If E4 is separately chosen later, follow `RELEASING.md`; after publication,
   require `check_published_release.py --require-published` and record both
   image digests before using them in a trial.
5. If the handoff must be reviewed or transferred separately, use
   `prepare_candidate_trial.py` followed by `run_candidate_trial.py`. Both the
   combined and split routes revalidate structured identity, bootstrap the
   content-bound Docker observer when selected, and preserve blocked, FAIL, and
   harness outcomes without inventing human measurements. Keep the generated
   packet, receipt, audit, exact `artifacts/` bytes, and execution receipt
   together.
   For a separately published release, use its exact public source commit and
   both published image digests. Do not mix the modes or reuse the current
   v0.9.0/v0.9.1 matrix.
6. Run fresh dedicated Humble/Jazzy Docker and source trials with a human
   observer. Record active operator time, command count, workflow download,
   peak disk, wall time, and output size; blank measurements remain
   non-comparable.
7. Only after at least one comparable Docker PASS and one comparable source
   PASS may the E3 cohort decision be reconsidered. Three accepted independent
   reports are still required for the v1 gate.

## Current decision

```text
E2 artifact host: DEFER — no host or upload authorized
E2 candidate images: DEFER — gate at c70c18d, atomic preparation at f5ed80e, row execution at feed0ba, one-command session at 8bc5ea4, guided readiness at a286c65, and GET-only environment preflight at adecca6; workflow not on develop, candidate-images ABSENT from the complete inventory, no dispatch, digest publication, remote audit, or trial
E3 community mutation: DEFER — cohort and issue operations remain closed
E4 v0.9.1 release/images: DEFER — G0 matrix and distribution gates remain open
```

This packet is evidence and handoff text only. It performs no release, image,
fixture, issue, branch, review, or community mutation.
