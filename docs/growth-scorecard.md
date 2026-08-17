# Weekly growth scorecard

The 1,000-Star roadmap treats Stars as a lagging signal. The weekly scorecard
records the smaller set of aggregate signals needed to tell whether people can
discover the project, produce a verified first map, and join the contributor
loop.

## Capture one snapshot

Authenticate the GitHub CLI with an account that can read repository traffic,
then run:

```bash
gh auth status
python3 scripts/collect_growth_snapshot.py \
  --captured-at 2026-08-10T00:00:00Z \
  --annotation "v0.9 onboarding baseline" \
  --output docs/evidence/growth/2026-08-10.json
```

Omit `--captured-at` for the current UTC time. The output path is created with
exclusive-create semantics: an existing weekly snapshot is never overwritten.
Without `--output`, the validated JSON is printed to stdout.

The first tracked aggregate is the
[2026-08-10 G0 baseline](evidence/growth/2026-08-10.json).
Its first product decision is the
[G0 activation decision](evidence/growth/g0-activation-decision-2026-08-10.md):
measure and repair the clean Docker/source first-map matrix before expanding
promotion.

The next privacy-bounded weekly snapshot is
[2026-08-13](evidence/growth/2026-08-13.json). It records 837 Stars, 321 unique
clones, 281 unique views, and 13 downloads of the primary v0.9.0 release
bundle. The product gates remain unchanged at 0/3 accepted independent maps
and 8/10 v1 readiness; the snapshot is a measurement update, not an adoption
claim.

The previous read-only snapshot is
[2026-08-15](evidence/growth/2026-08-15.json), captured at
`2026-08-14T19:00:23Z`. It records 837 Stars, 353 unique clones, 267 unique
views, 5 unique Autoware/TIER IV referrals, and 18 downloads of the primary
v0.9.0 release bundle. It still records 0/3 accepted independent maps and
8/10 v1 readiness. The snapshot contains aggregate metrics only; it does not
write stargazer identities, raw GitHub records, or product telemetry.

The current read-only snapshot is
[2026-08-17](evidence/growth/2026-08-17.json), captured at
`2026-08-16T20:52:07Z`. It records 839 Stars, 450 unique clones, 238 unique
views, 3 unique Autoware/TIER IV referrals, and 21 downloads of the primary
v0.9.0 release bundle. Relative to the previous rolling snapshot, Stars rose
by 2, unique clones by 97, and bundle downloads by 3, while unique views fell
by 29 and qualified referrals by 2. First-map adoption remains 0/3, external
merged contributors remain 0, untriaged issues remain 16, and v1 readiness
remains 8/10. This mixed funnel does not justify a promotion or roadmap-phase
transition: activation, qualified referral, distribution, and community gates
remain open even though discovery and artifact interest increased.

The parallel community decision is the
[2026-08-11 contributor backlog](evidence/growth/community-contributor-backlog-2026-08-11.md).
A read-only audit grouped all 29 open issues and prepared five tasks with exact
files, non-goals, acceptance criteria, and focused checks. The candidates have
not been published as GitHub issues and no labels were changed.

The external-adoption operating loop is prepared in the
[first-map validator cohort packet](evidence/growth/first-map-validator-cohort-launch-packet-2026-08-12.md).
Its machine contract fixes a five-attempt first batch, two-attempt review WIP,
the ten-attempt 80%/ten-minute decision gate, privacy and independence rules,
service levels, and stop/repair conditions. Recruitment text remains
fail-closed until one comparable Docker and source row, a canonical public
path, an exact public revision, and the copy-ready handoff are all public.
No community post or GitHub write is authorized by that packet.
Its anonymous operating state now makes the stop/repair loop executable: one
command cross-checks accepted IDs against the adoption ledger and derives
capacity, repeated-blocker, initial/hard-cap, completion-rate, and median-time
decisions without storing participant handles. The tracked empty state remains
an honest zero-attempt record and cannot render recruitment text. Operational
signals expire after 48 hours, and accepted attempts must also match the
ledger's public report, route, and immutable product identity.

The follow-up
[complete triage proposal](evidence/growth/open-issue-triage-proposal-2026-08-11.md)
covers all 29 open issues with a label, priority, disposition, evidence, and
application gate. Its checker validates offline coverage and can fail closed on
live GitHub drift using GET requests only. The proposal remains unapplied and
unauthorized. A 2026-08-17 live read-only audit still reports `PASS`: 29 issues,
23 close proposals, and 6 keep-open or current-reproduction proposals. This
confirms proposal freshness without treating it as completed triage.
The companion application-packet generator turns either one row or all 29 rows
into deterministic, evidence-hashed maintainer review cards. It has no output
file or GitHub write mode, keeps #422 monitor-only, and leaves nine
starter-dependent rows blocked for content review and separate authorization.
The P1 #69 card is synchronized to the public Draft and supported CI as of
2026-08-17, explains both leaf parameters and the resolution tradeoff, and
keeps the unavailable historical bag and missing carrying release explicit.
Its live mode now source-binds that dated statement to PR #427 head
`4b2ab514`, open/Draft/mergeable state, 10 successful checks, 4 intentional
skips, and zero pending or failing checks. A second GET-only claim binds latest
stable `v0.9.0` to commit `0df0c4a`, 52 commits behind fix `a2368c4`, and
requires both the `v0.9.1` tag and release to remain absent. Any Draft, CI,
stable ancestry, or candidate-publication drift produces no review packet.

The current
[G0 clean-candidate audit](evidence/growth/g0-clean-candidate-audit-2026-08-11.md)
binds the reviewed local product revision to the unchanged public `develop`
base, diff inventory, current gates, and dual-distro #69 verification. Its
[decision packet](evidence/growth/g0-external-action-decision-packet-2026-08-11.md)
keeps source push, fixture upload, issue mutations, and release publication as
four separate approvals. The candidate is reviewable locally; G0 remains
`HOLD`.

The [current capture-bound action packet](evidence/growth/g0-current-action-packet-2026-08-14.md)
supersedes the historical snapshot for present handoff decisions. It binds
Draft PR #427, the reviewed product-candidate tip
`3d64ed556aca8a680f09e0f7e8c12a3c8d3e6a6d`; this is the reviewed code-bearing
tip, followed by later docs-only handoff synchronization and product UX
follow-ups, capture-time public Draft baseline
`3ed632e6f6aa1e3ca7f32d893773de1079086ffb` with 10 successful checks and 4
intentional non-publication skips, the current 346-path local plan plus
complete 393-path / three-phase whole-PR review coverage, the
bounded one-card PR review overview and seven exact drill-down slices with
Git-derived text budgets, top-three hotspots, and named binary review paths, the
four role-based capability lanes with a two-reviewer advisory target and no
stored identities or reviewer-request authority, the optional append-only
exact-tip lane ledger with identity-free PASS/BLOCKED events and no GitHub
review authority, the
one-command candidate-session tip `8bc5ea4`, and its guided read-only host
readiness follow-up `a286c65` while the v0.9.1/image state remains unpublished.
The optional authenticated environment preflight at `adecca6` now proves from
a complete live inventory that only `github-pages` exists and
`candidate-images` remains `ABSENT`; it performs no settings write and cannot
authorize E2. Its human and G0 cards now turn that stable blocker into one
bounded administrator handoff: trusted settings URL, exact protected-policy
checklist, independent review, and a copy-ready GET-only verification command.
The handoff is schema-bound to `writes_performed: false`; it is guidance, not
evidence that repository settings changed. The exact operator-handoff and
parse-safety tip is `e2d916a66a57146b0efe4c74e57218d56342ed37`; the later
content-verified GLIM comparison tip is
`3de7f84bb51acd2bd1c2b40724529be9c281d2fe`; the byte-bound public-docs
deployment-provenance implementation tip is
`602c7ad5a64332eb458d98b1a53783ba4f3cdecb`, followed by public evidence refresh
`3ed632e6f6aa1e3ca7f32d893773de1079086ffb`. The current local release packet
also derives its commit and both Docker digests from one published-release
audit at implementation tip
`289f7675a242b00f342528483cde3e5f602a11fc`, then repeats the exact live
identity check before a clean-host row. The paired usability recorder
follow-up at
`0575fb6d67dc0b2069d9e41029a767bf3608687c` removes manual scorecard JSON
editing while preserving missing observations as non-comparable. It still has
no external paired record and makes no GLIM parity claim. The current local
doctor recovery tip
`a83bbfeaea8196a19513c7a26772d500fe8419b8` also turns a reproduced
five-finding unconfigured-shell result into one dependency-ordered **Do this
now** action while retaining all stable finding codes and performing no
network access or write. This is local activation evidence, not an external
first-attempt or GLIM parity result. The current local
paired-preparation follow-up removes manual public-resolvability Booleans: one
GET-only preflight must resolve both fixed canonical commit/tag or registry
identities and approved documentation redirects before either worksheet is
published. A fixed-name receipt SHA-binds both exact worksheet files; all three
outputs roll back together. The recorder requires and archives that untouched
triplet, and the checker rejects explicit completed records without the
preparation chain. Offline preparation remains explicitly non-public. This
makes the handoff safer but still creates no human observation or comparative
claim. The public-docs
read-only audit keeps the
cohort closed while the deployed Pages manifest is absent or its exact page
bytes, route, version, and source revision do not match.
It continues to keep E2 artifact hosting, E3 community mutation, and E4
release publication
separate and unauthorized. The earlier 219-path value remains historical
capture-time evidence only.

The [G0 readiness dashboard](g0-readiness.md) is the single read-only entry
point for rechecking those local gates:

```bash
python3 scripts/check_g0_readiness.py
```

Use `--include-product-draft`, `--include-published-release`, or
`--include-candidate-environment` only when the corresponding network-read
audit is wanted. The exact-head product audit now puts Draft review and a
separate merge decision ahead of candidate-environment administration; it
never turns green CI into merge authority. A clean exact green Draft now
produces one schema-bound overview → three-phase → seven-slice review sequence;
a dirty checkout stops at read-only status inspection so local bytes are not
mistaken for the public review.
Branch drift now carries a second, separately authorized no-write handoff for
the PR description. It renders one canonical exact-head body, hashes the
observed and desired descriptions, replaces stale commit/path scope with the
current whole-PR and P2 review budgets, requires the branch update first, and
keeps Draft, review, mark-ready, merge, and all write authority false. An exact
green head with a mismatched body is stopped at that refresh before review.
The canonical body now removes one more reviewer lookup step: P0–P2 are exact
GitHub compare links and S1–S7 are a bounded focus/path/check/gate table. Link
lineage, commit composition, slice composition, safe titles, clean-tip
identity, and no-write source authority are machine checked before that body
can be emitted.
The dashboard reports one next action and never interprets missing human
measurements, public identity, release, or community evidence as complete.
Its seven slice cards are now executable from an ordinary terminal: ROS state
is sourced explicitly, duplicate package-test basenames stay in separate
pytest processes, caches are disabled, and recognized direct remote-write CLI
forms fail plan validation before a command is displayed.

Local descendant `bce5a9d` additionally passes the real-component #69
unsafe-then-safe recovery sequence ten consecutive times and the full 10-suite
scanmatcher CTest on Humble and Jazzy. This closes a local reliability-evidence
gap; it does not satisfy the public-source, public-CI, fixture, matrix, or
release gates.

The parallel
[contributor Python test entrypoint](evidence/growth/contributor-python-test-entrypoint-2026-08-11.md)
now gives prepared contributors one package-scoped command with dependency
preflight and focused-test forwarding. Final local revision `e2a4dfc` passes
both complete Python product suites on Humble and Jazzy. This improves a
contribution leading signal; it does not increment the external-contributor
metric, and the revision still awaits E1 publication and public CI.

The first execution addendum is the
[2026-08-10 Docker machine-probe summary](evidence/onboarding/docker-machine-probes-2026-08-10.md).
Humble and Jazzy both reached a canonical-route product `PASS`; both remain
measurement `INCOMPLETE` because active operator time, human command count,
and isolated peak disk were deliberately left `null`. This changes measured
matrix coverage from `0 / 4` to `2 / 4`, but comparable coverage remains
`0 / 4`. The weekly GitHub snapshot is not overwritten to insert later trial
observations.

The 2026-08-13 source follow-up adds valid Humble and Jazzy product PASS
records, so the current machine-checked matrix is `4 / 4` present and `0 / 4`
comparable. The source records intentionally omit human active-time
observation, and they are `0.9.1` evidence beside the frozen Docker `0.9.0`
rows; neither condition advances the activation gate.

The collector fails closed if authentication is unavailable, an API response
is malformed, pagination would be incomplete, the local first-map ledger or
anonymous cohort state is invalid, cohort counts/rates/status disagree, or the
v1 readiness audit cannot be trusted. Its output is validated against
[`growth-snapshot-v1.schema.json`](schemas/growth-snapshot-v1.schema.json).

## Metric definitions

| Metric | Exact definition |
| --- | --- |
| Stars, forks, subscribers | Current aggregate fields from the repository API. |
| Views and clones | GitHub's 14-day aggregate totals and unique counts. Unique clones are not interpreted as users. |
| Autoware/TIER IV referrals | Sum of the unique counts for top referrers whose names contain `autoware` or `tier4`. GitHub exposes only its top referrers, and the sum is not a cross-referrer deduplicated user count. |
| Primary-bundle downloads | `download_count` for the single `lidarslam_ros2_v*_release_bundle.tar.gz` asset on the latest stable release. |
| Untriaged open issues | Open issues, excluding pull requests, with no labels. Applying a meaningful label is the minimum triage action. |
| External PRs, 90 days | Pull requests created in the trailing 90 days by non-maintainer, non-bot accounts. |
| External merged contributors, 180 days | Distinct non-maintainer, non-bot authors whose pull request was merged in the trailing 180 days. Only the count is retained. |
| First public response | Time from an external, non-bot issue opening to the first maintainer comment in the trailing 90-day cohort. The scorecard includes eligible, responded, unanswered, and responded-only median values. |
| External first maps | Aggregate accepted/required/remaining counts from the reviewed first-map ledger, plus the anonymous cohort's attempted/terminal/active/review-WIP/successful/accepted counts, completion rate, median active minutes, operational state, and fixed stop conditions. Historical snapshots without the optional cohort extension remain valid and are never rewritten. |
| v1 readiness | Complete/incomplete gate counts from the fail-closed local v1 audit. |

Pass each co-maintainer with `--maintainer LOGIN`; otherwise the default is
`rsasaki0109`. Maintainer logins are used in memory to classify responses and
contributions, but no author list is written.

## Privacy boundary

The collector never writes stargazer identities, issue authors, pull-request
authors, comment authors, comment bodies, issue text, issue/comment/profile
URLs, or raw referrer records. It writes only aggregate counts, durations,
public release metadata, contract identifiers, and short operator-supplied
annotations. Do not put names, private support details, or unpublished
campaign data in an annotation.

The cohort extension copies only bounded aggregate counts, rates, durations,
fixed state names, and fixed stop-condition IDs. It does not copy attempt IDs,
report/blocker URLs, accepted validation IDs, or participant handles into the
weekly snapshot.

The JSON contract fixes both privacy flags to `false`:

```json
{
  "personal_identifiers_written": false,
  "raw_records_written": false
}
```

Raw GitHub responses are held only in process memory. They are not cached by
this script.

## Reproduce without live API access

Use `--fixture-dir DIR` to run the same aggregation and schema validation from
reviewed JSON fixtures. The directory contains:

- `repository.json`
- `traffic-views.json`, `traffic-clones.json`, and
  `traffic-referrers.json`
- `latest-release.json`
- `pulls.json` and `issues.json`
- `issue-comments-N.json` for each external issue `N` in the 90-day response
  cohort, including an empty array when it has no comments

Fixtures are a test and audit interface. Do not commit fixtures copied from the
live API because they contain identities and raw records.

## Weekly review

Capture on the same weekday and approximate UTC time. Compare four-week
rolling values and annotate only real interventions such as releases,
documentation launches, talks, or community posts.

Use the decision rules from the
[1,000-Star roadmap](roadmap/1000-stars.md):

- flat qualified traffic means repair distribution and positioning;
- rising traffic without first-map receipts means stop promotion and repair
  onboarding;
- rising first maps without durable discovery means improve the proof summary
  and post-success invitation;
- rising support load means close diagnosis and documentation gaps before
  opening another channel.

At quarter and phase boundaries, use the
[2026–2029 operating plan](roadmap/1000-stars-2026-2029.md) and its
[health-review template](roadmap/1000-stars-quarterly-review-template.md).
The quarterly review selects one largest funnel constraint, records external
authorization separately from local readiness, and limits the next portfolio
to one product slice, one community slice, and at most one research slice.

The scorecard is evidence for a product decision, not a target to game.
