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

The collector fails closed if authentication is unavailable, an API response
is malformed, pagination would be incomplete, the local first-map ledger is
invalid, or the v1 readiness audit cannot be trusted. Its output is validated
against
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
| External first maps | Aggregate accepted/required/remaining counts from the reviewed first-map ledger. |
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

The scorecard is evidence for a product decision, not a target to game.
