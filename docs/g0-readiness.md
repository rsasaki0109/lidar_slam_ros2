# G0 readiness dashboard

Use the read-only dashboard when reviewing the current 1,000-Star roadmap
gates. It composes the existing checkers; it does not replace them, run a
trial, publish an artifact, or write GitHub/community state.

```bash
python3 scripts/check_g0_readiness.py
```

The default command is local-only. It reports the exact publication inventory,
the four-row Docker/source onboarding matrix, the independent first-map cohort,
v1 readiness, and the current action packet. It prints exactly one next action
with a safe read-only boundary. A `HOLD` is an honest gate state, not a checker
failure.

To bind the current checkout to public Draft PR #427 and its exact-head CI,
opt in to the GitHub GET-only product audit:

```bash
GITHUB_TOKEN="$(gh auth token)" \
python3 scripts/check_g0_readiness.py \
  --include-product-draft
```

The audit requires the canonical repository, PR number and URL, `develop`
base, `agent/product-g0-guided-ux` head branch, and full local/public commit to
agree. An open PR must be mergeable. The latest run for each required check
must contain ten successes and the four expected non-publication skips; a
missing, pending, failed, truncated, malformed, or mismatched result is
`BLOCKED`. Additional checks are accepted only when terminal and non-failing.
The report distinguishes `DRAFT_REVIEW_REQUIRED`,
`READY_FOR_SEPARATE_MERGE_REVIEW`, and `MERGED`, while keeping
`merge_authorized: false` in every state.

When the full local and public heads differ, the dashboard does not tell the
operator to repeat the same audit. It first checks the two commits in the
local object database. A verified fast-forward produces one structured
`NON_FORCE_PR_BRANCH_UPDATE` handoff containing the exact public head, exact
local tip, canonical PR branch, separate authority requirement, and post-update
GET-only verification command. The handoff deliberately contains no push
command and keeps `push_authorized`, `force_push_authorized`, and
`writes_performed` false. Divergent history instead selects a read-only
merge-base inspection; unavailable local history selects a bounded fetch plus
ancestry check from the canonical GitHub repository URL instead of trusting a
checkout-specific `origin`. Neither path authorizes a push, PR state change,
or merge.

Dependency order is explicit. A green Draft points to the seven-slice local
review plan. A non-Draft open PR still requires a separate maintainer merge
decision. Repository-environment work cannot become the next action until the
exact PR is observed as merged. If an environment audit is requested without
the product audit, the dashboard asks for the missing product audit first
instead of suggesting a settings change from incomplete evidence.

The selected seven-slice review card is copy-ready from an ordinary terminal.
Before drilling into one slice, the local-only overview makes the large Draft
budget visible without pasting every path:

```bash
python3 scripts/check_publication_slice_plan.py --overview
```

It binds whole-PR and three-phase path identity to Git numstat, then reports
text additions/deletions, binary counts, and the largest textual path for each
phase and slice. Each exact slice card expands to three hotspots and names any
binary paths requiring manifest/content review. The budget is a navigation aid,
not evidence that review occurred, and it grants no GitHub write authority.

ROS-dependent commands source the caller's `ROS_DISTRO` installation and
default to Jazzy when it is unset. Pytest commands disable cache writes, and
the two package test roots run in separate processes to avoid their known
duplicate module basename. Plan validation rejects a missing ROS prelude,
mixed-package pytest command, cache-producing pytest command, or remote-write
CLI form recognized by the checker before displaying the card.

To include the stable-release audit, which performs network reads but no
remote writes, opt in explicitly:

```bash
python3 scripts/check_g0_readiness.py \
  --include-published-release \
  --published-release-version 0.9.1
```

To inspect the protected candidate environment through GET requests only,
pass `--include-candidate-environment`. Authenticated inspection avoids
mistaking an inaccessible endpoint for an absent environment:

```bash
GITHUB_TOKEN="$(gh auth token)" \
python3 scripts/check_g0_readiness.py \
  --include-product-draft \
  --include-candidate-environment
```

The environment check first requires one complete repository-environment
inventory, then reads the exact `candidate-images` environment and its complete
deployment-policy list. It reports `ABSENT`, `MISCONFIGURED`, or `BLOCKED`
separately. `READY` requires one to six reviewers, **Prevent self-review**, no
unknown protection rule, and exactly one custom `develop` branch policy. Even
`READY` means only `READY_FOR_SEPARATE_E2_REVIEW`: environment writes and the
digest-publication dispatch remain unauthorized.

To take one complete read-only snapshot of the current external G0 gates, use:

```bash
GITHUB_TOKEN="$(gh auth token)" \
python3 scripts/check_g0_readiness.py \
  --include-product-draft \
  --include-candidate-environment \
  --include-published-release \
  --published-release-version 0.9.1
```

When this optional gate becomes the selected next action, the human card and
JSON contract carry the same bounded operator handoff. An absent or
misconfigured environment includes only the trusted repository-settings URL,
the exact reviewer/self-review/develop-only checklist, required external
authority, and the read-only verification command. An inaccessible audit does
not expose a settings URL or suggest mutation; it asks the operator to restore
read access and retry. The dashboard labels the handoff **not executed** and
rechecks that `writes_performed` remains false before displaying it.

For automation, use `--json`. The output follows the
[`g0-readiness-report-v1`](schemas/g0-readiness-report-v1.schema.json)
contract. `--require-ready` exits with status 1 while any summarized gate is
not ready and status 2 if a source checker or the dashboard contract is
invalid.

The dashboard deliberately does not turn a product `PASS` into a comparable
onboarding row. When Docker and source rows use different product versions,
its next action shows two structured, no-write choices: continue the current
candidate (which needs the protected `candidate-images` environment, a
separately authorized digest-only E2 dispatch, and a remote candidate-set
audit; the gate itself does not authorize publication),
or intentionally rebuild all four rows against one already-published version.
The second choice requires a fresh source preflight and fresh records; old
mixed-version measurements must never be reused. After one public identity is
selected and all rows are rebuilt or re-recorded against it, the next action
moves to the measurement gate. Human active time, submitted command count,
isolated disk measurements, and the external first-map acceptance gates remain
evidence requirements. Recruitment, release, image, issue, label, review, and
package actions remain separate decisions.

For the published-release choice, the dashboard now prints a copy-ready
pipeline from `check_published_release.py` into
`prepare_onboarding_matrix_packet.py --published-release-report -`. The packet
derives the tag commit and both image digests from the same schema-valid report
bytes and retains their SHA-256. Before a clean-host row starts,
`check_published_onboarding_identity.py` repeats the bounded network audit and
requires the live commit and both digests to match exactly. This removes four
manual identity fields without treating packet preparation as a release or a
trial.

When v1 is incomplete, the card and JSON report also expose each incomplete
gate's recorded detail and blocker list. This keeps distribution blockers such
as unresolved `ndt_omp` lineage, missing apt synchronization, or a missing
package-manager run visible without performing any external write. The
blockers are evidence for the next decision, not proof that an external action
has been taken.

When the independent first-map cohort is waiting for public gates, the card
also lists each pending launch prerequisite, such as comparable Docker/source
rows and the canonical documentation/runtime identity. The documentation gate
is byte-bound rather than URL-only. After a reviewed Pages deployment, audit
the selected route with:

```bash
python3 scripts/check_public_docs_deployment.py \
  --expected-revision <exact-40-character-public-commit> \
  --expected-product-version 0.9.1 \
  --route source-quickstart \
  --json
```

`VERIFIED` requires the deployment manifest revision, rendered page size and
SHA-256, product version, and selected route fragment to agree. `NOT_READY` or
`BLOCKED` keeps the cohort closed. The audit performs bounded network reads and
no writes. This makes the closed
cohort state actionable without rendering recruitment text or authorizing a
community write. Machine-readable JSON keeps the stable gate IDs; the human
card adds the concrete evidence required for each one, including the seven
measurements and immutable runtime identity. Unknown future IDs remain visible
and fail safe with a pointer to the cohort contract.

The contributor queue applies the same boundary to the already-published
tracking issue. `python3 scripts/contributor_starter_queue.py --next` does not
treat an open `good first issue` label as sufficient evidence. It evaluates
the declared cohort dependency locally and recommends #422 only when the
derived operating state is exactly `READY_FOR_NEXT_ATTEMPT`; otherwise it
reports the issue as blocked and points the maintainer back to the cohort
checker. Its JSON is validated against
[`contributor-next-action-v1`](schemas/contributor-next-action-v1.schema.json),
performs only bounded GitHub GETs, and cannot authorize recruitment or mutate
an issue.

The current packet is
[`g0-current-action-packet-2026-08-14.md`](evidence/growth/g0-current-action-packet-2026-08-14.md).
It supersedes the historical action snapshot for present handoff decisions
without authorizing remote mutation.
