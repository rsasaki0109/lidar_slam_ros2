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

For automation, use `--json`. The output follows the
[`g0-readiness-report-v1`](schemas/g0-readiness-report-v1.schema.json)
contract. `--require-ready` exits with status 1 while any summarized gate is
not ready and status 2 if a source checker or the dashboard contract is
invalid.

The dashboard deliberately does not turn a product `PASS` into a comparable
onboarding row. Human active time, submitted command count, isolated disk
measurements, one aligned public product identity, and the external first-map
acceptance gates remain evidence requirements. Recruitment, release, image,
issue, label, review, and package actions remain separate decisions.

The current packet is
[`g0-current-action-packet-2026-08-14.md`](evidence/growth/g0-current-action-packet-2026-08-14.md).
It supersedes the historical action snapshot for present handoff decisions
without authorizing remote mutation.
