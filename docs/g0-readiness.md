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
  --include-candidate-environment
```

The environment check first requires one complete repository-environment
inventory, then reads the exact `candidate-images` environment and its complete
deployment-policy list. It reports `ABSENT`, `MISCONFIGURED`, or `BLOCKED`
separately. `READY` requires one to six reviewers, **Prevent self-review**, no
unknown protection rule, and exactly one custom `develop` branch policy. Even
`READY` means only `READY_FOR_SEPARATE_E2_REVIEW`: environment writes and the
digest-publication dispatch remain unauthorized.

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

When v1 is incomplete, the card and JSON report also expose each incomplete
gate's recorded detail and blocker list. This keeps distribution blockers such
as unresolved `ndt_omp` lineage, missing apt synchronization, or a missing
package-manager run visible without performing any external write. The
blockers are evidence for the next decision, not proof that an external action
has been taken.

When the independent first-map cohort is waiting for public gates, the card
also lists each pending launch prerequisite, such as comparable Docker/source
rows and the canonical documentation/runtime identity. This makes the closed
cohort state actionable without rendering recruitment text or authorizing a
community write. Machine-readable JSON keeps the stable gate IDs; the human
card adds the concrete evidence required for each one, including the seven
measurements and immutable runtime identity. Unknown future IDs remain visible
and fail safe with a pointer to the cohort contract.

The current packet is
[`g0-current-action-packet-2026-08-14.md`](evidence/growth/g0-current-action-packet-2026-08-14.md).
It supersedes the historical action snapshot for present handoff decisions
without authorizing remote mutation.
