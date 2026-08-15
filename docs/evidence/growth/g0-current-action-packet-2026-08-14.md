# G0 current action packet — refreshed 2026-08-15

> Status: **CURRENT_PUBLIC_DRAFT_HANDOFF / NO_E2_E3_E4_ACTION_TAKEN**
>
> Repository: `rsasaki0109/lidar_slam_ros2`
>
> Draft PR: [#427](https://github.com/rsasaki0109/lidar_slam_ros2/pull/427)
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
> Latest publication-inventory tip: `996679ccebe5aa3ad66ebfc657db20d24ab567ea`

This reviewed tip is the code-bearing product-candidate revision; later
docs-only handoff synchronization and product UX follow-up commits must remain
identified separately.

This is the current, read-only handoff for the G0 release-hygiene decision.
It was first captured on 2026-08-14 and refreshed on 2026-08-15 after the
dashboard UX, CI-registration, version-priority, final PR-head CI,
packet-command-contract, fail-closed usability-worksheet, paired scorecard,
safe observer-packet-output, safe first-map-dry-run-plan-output, Docker JSON
own-bag-plan, source JSON quickstart-plan, custom PointCloud2 onboarding,
supported g2o recovery, canonical C2/C3 drift detection, contributor C1–C4
local-retirement, bounded contributor C5–C9 replenishment, and
publication-inventory follow-ups. The latest distribution slices also scope
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
closes an accidental-publication path; it does not create the still-missing
immutable v0.9.1 matrix-candidate identity.
The code-bearing packet tip is required to be an ancestor of the current
checkout revision; later synchronization and product UX follow-up commits
must remain described in this handoff. It replaces
neither the historical 2026-08-11 decision packet nor any maintainer approval.
Its purpose is to prevent an old commit, old version, or one external action
gate from being mistaken for the current state.

## Current evidence

| Check | Current result | Meaning |
| --- | --- | --- |
| Draft PR #427 | open, draft, mergeable at public baseline `db1f9b4…`; the local Docker authority implementation and this packet refresh remain follow-ups until their non-force push | source candidate is publicly reviewable; do not claim that a local-only follow-up is public |
| PR-head CI | latest completed public exact-tip result before this Docker authority follow-up is **PASS** for `db1f9b4…` (9 / 9 checks); the local implementation and packet tip must receive their own CI after push | do not transfer the public baseline result to a newer tip; CI is not a release approval |
| English support cards | docs entrypoint tests 23 passed | C1 g2o recovery is implemented; existing C2 empty-map and C3 Odometry/TF cards remain copy-ready and safety-bounded; the added regression binds Docker verification/publication authority |
| Custom PointCloud2 onboarding | implemented in the reviewed product UX tip | bounded topic/frame/time/TF/range/launch readiness guidance; it does not claim hardware support or accuracy |
| Contributor starter queue | C5–C9 `READY_LOCAL_ONLY`; 50 queue regressions and all five focused strict-MkDocs profiles passed | C1–C4 remain completed and retired; the fresh duplicate audit found no matching implementation PR, and no issue or label mutation occurred |
| Distribution preflights | source route `READY` at exact public `db1f9b4…`; rosdistro NDT remains `BLOCKED`: Humble #52949 and Jazzy #52950 each have 5 / 6 exact-head checks passing, one failing, and an unanswered review; package-manager E2E is `SOURCE_REF_MISSING` because `v0.9.1` does not resolve, with zero matching runs | the rosdistro failures are stale-base rosdep failures rather than the YAML delta, but neither external PR is green; collision-free convergence and current-base green replacement still precede clean-install E2E |
| Canonical NDT upstream Draft preflight | `READY_FOR_DRAFT_PR`; 30 / 30 PASS at local implementation `856e599…`; exact upstream `5495fd9…`, expected fork verified, proposed branch absent, 4 open PRs inspected, 0 duplicates, 0 API errors, and write authority false | this proves a technically coherent read-only publication state; it neither creates nor authorizes an upstream branch or PR |
| Docker publication boundary | PR and manual runs are verification-only with `contents: read`, `push: false`, no registry login, and no attestation; only a `develop` push receives package/attestation write permission | a manual dispatch can no longer move `humble`, `jazzy`, or `latest`; a dedicated immutable candidate-digest workflow is still required before any matrix-enabling E2 decision |
| Publication slice plan | `PLAN_VALID_LOCAL_ONLY`; 233 paths / 7 slices, clean at Docker authority implementation `3225d9d…`; inventory SHA-256 `15b94422746d15290b1f45d1a3b95fef892cba4ef1ab7be78e57ec2948ac1a09` | all five changed paths already belong to S2/S5/S6; packet synchronization changes no path membership and cannot authorize a GitHub write |
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
  --source-commit db1f9b40088851dfd488e98a1aeb19b2c14909a4 \
  --product-version 0.9.1
python3 scripts/check_published_release.py --version 0.9.1 --json
python3 scripts/check_onboarding_trial_matrix.py --json
python3 scripts/check_v1_readiness.py --json
python3 scripts/first_map_validator_cohort.py --json
python3 scripts/check_g0_readiness.py \
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
| E1 source review | public Draft PR / CI PASS | review this exact tip; no merge is implied |
| E2 artifact hosting | **NOT_AUTHORIZED / NOT_PUBLISHED** | fixture host or immutable candidate-image evidence only after a separate host, license/provenance, checksum/digest, retention, and upload decision |
| E3 community mutation | **NOT_AUTHORIZED** | no issue labels, comments, closures, starter issues, Discussions, or recruitment |
| E4 stable release | **HOLD / NOT_AUTHORIZED** | no tag, GitHub Release, package, image promotion, or announcement |

Approval of one row never approves another. In particular, green CI does not
authorize E2, E3, or E4, and a published identity would not by itself create a
comparable human trial.

## Safe transition order

1. Review the code-bearing candidate tip and this refreshed packet; confirm
   the current PR-head CI remains green, then stop if the branch, PR, or local
   inventory drifts. Do not measure mixed-version rows.
2. Do not use the convenience-image manual dispatch as a candidate publication
   route. First implement and review a dedicated immutable candidate-digest
   workflow that cannot move `humble`, `jazzy`, `latest`, or release tags.
3. If E2 is separately chosen after that gate exists, publish only the exact
   candidate digests or explicitly selected fixture host, then perform a
   remote identity, provenance, retention, and checksum/digest audit.
4. If E4 is separately chosen later, follow `RELEASING.md`; after publication,
   require `check_published_release.py --require-published` and record both
   image digests before using them in a trial.
5. Generate a new observer packet from the exact public source commit and both
   published image digests. Use
   `scripts/prepare_onboarding_matrix_packet.py`; do not reuse the current
   v0.9.0/v0.9.1 mixed matrix.
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
E2 candidate images: DEFER — dedicated immutable-digest gate not implemented
E3 community mutation: DEFER — cohort and issue operations remain closed
E4 v0.9.1 release/images: DEFER — G0 matrix and distribution gates remain open
```

This packet is evidence and handoff text only. It performs no release, image,
fixture, issue, branch, review, or community mutation.
