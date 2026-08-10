# 1,000 Stars roadmap — useful, trusted, sustainable

> Status: **active from 2026-08-10**
>
> Baseline: **837 GitHub Stars**
>
> Target: **1,000 Stars by 2027-06-30**
>
> Stretch target: **1,000 Stars by 2027-03-31**
>
> Product outcome: a third party reaches a verified Autoware-compatible map
> from a rosbag without reading the implementation or receiving private setup
> help.

This roadmap turns the existing product and research work into a sustainable
adoption loop. It complements the [v0.9 product roadmap](v0.9.md), the
[v1.0 readiness gate](../v1-readiness.md), and the evidence-driven SOTA
research plans. It does not weaken any accuracy, license, compatibility, or
release gate.

The 1,000-Star count is a discovery milestone, not the product's purpose. The
project succeeds only when people can install it, obtain a useful map, verify
the result, recover from failure, and contribute improvements.

## 1. Definition of success

The growth goal is complete only after all of the following are true:

- GitHub reports at least 1,000 current Stars in two weekly snapshots;
- the v1.0 readiness audit reports `10 / 10` complete;
- at least three independent users have accepted first-map validations;
- a current stable release is less than 90 days old;
- at least three non-maintainer contributors have had a contribution merged in
  the preceding 180 days;
- no supported-product P0 issue has waited more than seven days for a public
  disposition;
- every public performance or compatibility claim links to reproducible
  evidence and states its limits.

Crossing 1,000 before these quality conditions is a Star milestone, but not
completion of this roadmap.

### Guardrails

- Do not buy, exchange, automate, or otherwise manufacture Stars.
- Do not mass-message users, forks, issue authors, or unrelated communities.
- Do not make a SOTA, hardware-support, or Autoware-endorsement claim without
  the corresponding evidence.
- Do not add anonymous product telemetry. Use aggregate GitHub metrics and
  voluntary, privacy-bounded validation receipts.
- Do not let research artifacts, generated logs, or benchmark datasets enter
  the supported runtime package or release bundle.
- Do not ship features solely for attention. Every roadmap item must improve
  activation, trust, distribution, or maintainability.

## 2. Baseline on 2026-08-10

GitHub REST traffic data covers the preceding 14-day window. Stargazer dates
describe Stars that still exist today, so they are useful for trend estimates
but are not an exact historical net-growth ledger.

| Signal | Baseline | Meaning |
| --- | ---: | --- |
| Current Stars | 837 | 163 remain |
| Current stargazers added in last 30 days | 11 | recent organic pace |
| Current stargazers added in last 90 days | 35 | about 11.8 per 30.4 days |
| Forks / subscribers | 172 / 12 | strong latent usage, weak ongoing project followership |
| Repository views | 870 total / 299 unique | 14-day discovery baseline |
| Repository clones | 2,042 total / 259 unique | totals include likely automation; use uniques cautiously |
| Main v0.9.0 bundle downloads | 8 | GitHub asset count; GHCR pulls are not included |
| Open issues | 29 | 28 were opened before 2026 and need deliberate triage |
| Pull requests | 333 total / 324 merged | high maintainer throughput |
| Historical external PRs | 13 submitted / 10 merged | contribution path exists but is quiet |
| External PRs in the last 90 days | 1 | contributor growth is a primary constraint |
| API-visible contributors | 1 | the project still appears maintainer-dependent |
| Independent first-map validations | 0 / 3 | current v1.0 adoption blocker |
| v1.0 readiness | 8 / 10 | distribution and external adoption remain open |
| Repository size | 172,506 KiB | discovery and clone cost need an artifact/history audit |
| GitHub Discussions | disabled | usage questions remain mixed into the issue backlog |

The strongest 14-day referrers were Google (104 unique visitors) and GitHub
(74). `tier4.github.io` and `autowarefoundation.github.io` contributed only
six unique visitors combined. Search already finds the project; the larger
opportunity is to make the landing page easier to understand and to earn more
qualified Autoware referrals.

The active local development branch is also an operational risk: it is 229
commits ahead of `origin/develop`, changes 454 files with about 49,508 inserted
lines, and has additional dirty product, research, and generated-output work.
This work must be separated into reviewable product and research changes before
another public release.

At the current 90-day pace, another 163 Stars would take roughly 14 months,
placing the unassisted projection around October–November 2027. The base target
requires about 15.2 net new Stars per month, approximately 1.3 times the recent
pace. The March stretch target requires about 21 per month.

## 3. Positioning: win a narrower and more useful job

The project should not present itself as a generic replacement for every LiDAR
SLAM framework. FAST-LIO owns a widely cited efficient-LIO position; GLIM owns
a versatile, extensible mapping-framework position. Their current GitHub
counts are useful context, not goals to imitate.

| Project | Public position | Stars on 2026-08-10 |
| --- | --- | ---: |
| `lidar_slam_ros2` | rosbag2-to-verified Autoware map authoring | 837 |
| `koide3/glim` | versatile and extensible 3D mapping framework | 1,727 |
| `hku-mars/FAST_LIO` | efficient and robust LiDAR-inertial odometry | 5,043 |

The product wedge is:

> **The shortest trustworthy path from a rosbag2 recording to a verified
> Autoware-compatible map bundle.**

The three primary audiences are:

1. Autoware mapping teams that need a loadable point-cloud map and projector
   metadata, not only a trajectory;
2. ROS 2 field-robotics integrators who need a diagnosable offline workflow for
   their own sensor data;
3. researchers and students who need reproducible baselines and evidence
   contracts without turning research controls into product defaults.

Advanced coloured mapping, tunnel/fog resilience, and SOTA research remain
valuable proof and extension tracks. They belong below the first successful
map in the information hierarchy and must not obscure the primary job.

## 4. User-friendly means measured, including against GLIM

GLIM already offers maintained documentation, Docker images, broad sensor
support, visual examples, and interactive correction. We should not declare
victory from a longer feature list. Once per stable release, run a clean-machine
usability scorecard for both public workflows on equivalent supported hardware.

Measure these tasks independently:

| Task | Measurement |
| --- | --- |
| Discover the supported path | time until the user identifies the correct install/run command |
| Run a fixed demo | commands, download bytes, wall time, active operator time, failure count |
| Inspect an own bag | whether topics, frames, timestamps, and profile choice are explained before execution |
| Produce a downstream artifact | whether the result is a verified Autoware bundle without manual file assembly |
| Understand a failure | whether the public error links directly to one safe recovery action |
| Repeat or upgrade | whether the same command and output contract survive a supported release upgrade |

Only compare overlapping tasks, publish the exact versions and commands, and
record where the products intentionally solve different jobs. The desired
outcome is not “more user-friendly” as an unsupported slogan. It is:

- one obvious beginner path;
- no undocumented manual step before `map_verify: PASS`;
- a measured first-run completion rate of at least 80% by the tenth independent
  attempt;
- median active operator time below ten minutes for the fixed demo;
- every own-bag rejection includes a stable reason code and next action.

## 5. Growth model and leading indicators

Stars are a lagging signal. The working model is:

```text
qualified discovery
  -> first verified map
  -> confidence in evidence
  -> useful public report or contribution
  -> recommendation and Stars
```

The operational north-star metric is **independent verified first maps per
month**. It connects product utility to sustainable discovery more directly
than page views or repository activity.

### Scorecard

| Dimension | Baseline | 2026-11 target | 2027-06 target |
| --- | ---: | ---: | ---: |
| GitHub Stars | 837 | 900 | 1,000 |
| 14-day unique repository visitors | 299 | 375 | 500 |
| 14-day unique Autoware/TIER IV referrals | 6 | 15 | 30 |
| Primary release-bundle downloads per release | 8 | 20 | 50 |
| Accepted independent first maps | 0 | 3 | 10 cumulative |
| External merged contributors, trailing 180 days | 0 | 2 | 5 |
| External PRs, trailing 90 days | 1 | 3 | 6 |
| Untriaged open issues | 16 | 0 | 0 |
| Median first public response to new supported-product issues | not tracked | <= 5 days | <= 72 hours |
| v1.0 readiness | 8 / 10 | 10 / 10 | maintained at 10 / 10 |

The response-time values are internal operating targets, not a contractual
support SLA. If maintainer capacity cannot sustain them, reduce concurrent
work and document the actual support boundary instead of hiding the delay.

### Measurement rules

- Capture Stars, forks, subscribers, unique views/clones, referrers, release
  downloads, issue age, and contributor counts once each week.
- Keep compact weekly aggregates; do not commit user identities or a raw list
  of stargazers.
- Review four-week rolling values because release days and automated clones
  make daily totals noisy.
- Annotate releases, documentation launches, talks, and community posts so an
  increase can be associated with a real intervention.
- Never interpret clone totals as users; prefer unique clones, verified maps,
  and release artifacts.
- Re-estimate the target date monthly. Do not weaken quality gates to preserve
  a calendar forecast.

## 6. Workstreams

### A. First-map activation

- Reduce the README's first screen to one promise, one visual proof, one Docker
  command, and one own-bag command; move advanced evidence into clear links.
- Turn `lidarslam-map run BAG --guided` into the canonical own-bag route and
  package it in every supported distribution.
- Measure the current 517 MB demo, then provide a smaller onboarding fixture if
  download or run time dominates first success. Keep the full surveyed demo as
  proof rather than silently weakening validation.
- The first [Docker machine probes](../evidence/onboarding/docker-machine-probes-2026-08-10.md)
  measured 19–24 minute cold paths and 1.77–1.91 GB of workflow RX on Humble
  and Jazzy. Both product routes passed; the smaller-fixture experiment is now
  activated, while the full MID-360 route remains the proof path.
- The resulting [50-second MID-360 fixture pilot](../evidence/onboarding/mid360-onboarding-fixture-pilot-2026-08-10.md)
  produced a byte-reproducible 98,873,952-byte ZIP and a verified local map,
  reducing the dataset transfer by 80.879%. It remains unpublished and cannot
  replace the full gate. The next independent activation experiments are
  runtime-image slimming and clean comparable VM trials.
- The [runtime-image slimming pilot](../evidence/onboarding/runtime-image-slimming-2026-08-11.md)
  reduced Docker's local image-size measurement by 53.6409% on Humble and
  59.6698% on Jazzy while retaining the source-free installed CLI and
  schema-valid first-map result. This passes the local 25% proxy. Compressed
  OCI size, the full public demo, clean dedicated-VM trials, and the identified
  ROS-log symlink repair remain promotion gates; no candidate was pushed.
- Add tested sensor recipes for the most requested families, starting from
  issue evidence rather than an unbounded compatibility matrix.
- Make the final success output show the map path, verifier result, viewer path,
  run receipt, and exact next command in one screen.

### B. Trust and reproducible proof

- Finish v1.0 only through the existing `10 / 10` fail-closed gate.
- Publish a small human-readable benchmark scorecard whose rows link to exact
  datasets, revisions, commands, limitations, and machine-readable evidence.
- Treat the negative SOTA-v5/v6 outcomes as honest research records. Do not put
  an unresolved SOTA route on the product-release critical path.
- Convert major claims into short visual proof: input, command, output map,
  verifier result, and measured accuracy or runtime.
- Keep comparison language task-specific. Algorithm accuracy, onboarding,
  downstream map authoring, GPU requirements, and sensor breadth are different
  axes.

### C. Distribution and discovery

- Complete the current rosdistro dependency path. As of 2026-08-10,
  `ros/rosdistro` PRs
  [#52949](https://github.com/ros/rosdistro/pull/52949) and
  [#52950](https://github.com/ros/rosdistro/pull/52950) remain open.
- Exercise clean install and upgrade on Humble and Jazzy after the packages are
  available, then publish v1.0 from the same verified contract.
- Produce one sub-three-minute English demo with captions and one concise
  Japanese companion post. Both point to the same canonical quickstart.
- Create durable task pages for “ROS 2 LiDAR SLAM quickstart”, “rosbag2 to
  Autoware point-cloud map”, and supported sensor recipes. Avoid duplicate SEO
  pages with no additional operational value.
- Announce only meaningful evidence or releases through the relevant ROS and
  Autoware community channels. One canonical post plus substantive updates is
  preferable to repeated promotion.

### D. Community and contributor loop

- Review all 29 open issues. Label the supported surface, preserve reusable
  answers in docs, close resolved or obsolete reports with a reason, and move
  broad usage discussion only after a suitable public discussion route exists.
- Recruit the first three validators through the existing public validation
  issue and release documentation; do not provide private step-by-step help
  that would invalidate the evidence.
- Maintain five to ten bounded `good first issue` tasks with a fixture, expected
  output, relevant files, and a check command.
- Make the contributor path finish in under 30 minutes for documentation and
  small CLI changes. Provide focused tests instead of requiring every public
  dataset.
- Enable GitHub Discussions only after categories, moderation expectations,
  support boundaries, and a weekly triage slot are ready.
- Invite sustained contributors into review and roadmap discussion before
  expanding merge rights under `GOVERNANCE.md`.

### E. Releaseable architecture and repository hygiene

- Split the 229-commit local delta into product, research, and generated
  evidence inventories. Integrate reviewable product slices from a clean
  `origin/develop` base instead of publishing the whole development branch.
- Keep runtime code, bounded test fixtures, and durable research conclusions in
  Git. Keep bags, build logs, raw replay outputs, generated maps, and repeated
  benchmark artifacts outside the repository or in release evidence storage.
- Audit the 172,506 KiB repository history and release assets before deciding
  whether history migration is justified. Do not rewrite public history merely
  to improve a vanity size number.
- Maintain one product release candidate, one community task, and at most one
  exploratory research candidate in progress at once.
- Budget normal maintainer effort at approximately 60% product/reliability,
  20% distribution/community, and 20% research. Research can use spare capacity
  but cannot delay a failed product, support, or release gate.

### F. Sustainable communication

- Ship a compact monthly “what became easier” update, not a raw commit list.
- Show one before/after workflow, one evidence result, known limitations, and a
  contributor credit in each release note.
- Create case studies only with user permission and privacy-safe artifacts.
- Credit external reports, documentation, fixes, and benchmark submissions,
  not only code.

## 7. Phases and Star checkpoints

Star counts are forecast checkpoints, not phase-exit gates. Quality outcomes
control progression.

| Phase | Window | Quality exit | Star checkpoint |
| --- | --- | --- | ---: |
| G0 — release hygiene | 2026-08-10 to 2026-09-30 | clean product integration path, weekly metrics, all open issues triaged, public v0.9 first-map baseline measured | 860 |
| G1 — v1 activation | 2026-10-01 to 2026-11-30 | v1 audit 10/10, three accepted external first maps, canonical guided path in packaged products | 900 |
| G2 — proof and launch | 2026-12-01 to 2027-02-28 | stable v1 release, public UX/benchmark scorecard, demo video, at least two recent external contributors | 950 |
| G3 — ecosystem | 2027-03-01 to 2027-06-30 | ten cumulative first maps, five recent external contributors, maintained sensor recipes and support targets | 1,000 |
| G4 — sustain | 2027-07-01 to 2027-09-30 | 90-day post-milestone health review, current release, no regression in activation or contributor metrics | maintain >= 1,000 |

If the March stretch target is reached, do not skip G3 quality work. Move
directly into the sustainability review.

### Long-range dependency map

The phases are not independent feature buckets. The durable path to 1,000
Stars has one evidence-gated dependency chain:

```text
smaller runtime image + bounded onboarding fixture
  -> comparable Humble/Jazzy Docker and source trials
  -> public validator cohort + package-manager proof
  -> v1.0 readiness 10 / 10
  -> canonical proof launch
  -> maintained sensor and contributor ecosystem
  -> 1,000-Star milestone + 90-day sustainability audit
```

Issue triage, weekly aggregate measurement, release maintenance, and bounded
research run beside this chain rather than after it. An external rosdistro
delay must not stop onboarding repair or validator preparation. Conversely, a
research result, traffic spike, or early Star checkpoint cannot bypass a
failed activation, distribution, or adoption gate.

| Horizon | Product and trust | Adoption and community | Distribution and communication | Outcome checkpoint |
| --- | --- | --- | --- | --- |
| 2026 Aug–Sep — G0 | reduce and measure the cold first-map path; preserve full-gate parity | triage the backlog and prepare bounded validator tasks | establish comparable Humble/Jazzy Docker/source evidence | reviewable product line and 860-Star forecast checkpoint |
| 2026 Oct–Nov — G1 | close the 10/10 v1 audit from real package and first-map evidence | accept three independent first maps; keep five to ten bounded contribution tasks | finish package-manager proof and prepare the canonical v1 landing path | verified activation and 900-Star forecast checkpoint |
| 2026 Dec–2027 Feb — G2 | publish a current stable v1 release and reproducible UX/benchmark scorecard | retain at least two recent external contributors and publish reusable user findings | release one short English demo and Japanese companion through relevant channels | proof-led launch and 950-Star base checkpoint; 1,000 stretch only if quality gates already pass |
| 2027 Mar–Jun — G3 | maintain release freshness, claim evidence, and supported sensor recipes | reach ten cumulative first maps and five recent external contributors | grow durable Autoware/ROS task pages and qualified referrals | 1,000 Stars in two weekly snapshots, with all completion conditions still passing |
| 2027 Jul–Sep — G4 | prevent activation, reliability, and upgrade regressions | verify that support and review load remain maintainable | publish a 90-day health review instead of immediately expanding scope | sustain at least 1,000 Stars and a healthy project for 90 days |

Every six weeks, select the largest measured constraint in the discovery-to-
first-map-to-contribution funnel. Start no more than one product/release slice,
one community slice, and one bounded research slice at once. If the calendar
and a quality gate conflict, revise the forecast rather than the gate.

## 8. First 90 days

### Sprint 1 — separate what can safely ship (weeks 1–2)

1. Inventory the 229 unpublished commits and dirty files into product,
   research, evidence, and generated-output groups.
2. Start a clean product integration line from `origin/develop` and bring over
   only the guided CLI, product contract, packaging, and directly related tests
   in reviewable slices.
3. keep `log-*`, raw bags, maps, and replay evidence outside the source tree;
   add narrow ignore rules only after confirming every target is generated.
4. Run strict docs, supported package tests, release-bundle rehearsal, and the
   fixed Docker first-map path from the exact candidate revision.

Exit: a clean, reviewable candidate exists; no research result or generated
artifact is required to install or use it.

### Sprint 2 — measure and repair first success (weeks 3–4)

1. Run the public v0.9 Docker and source paths on clean Humble and Jazzy
   machines.
2. Record download size, wall time, active operator time, commands, peak disk,
   failure reason, verifier result, and receipt status with the
   [comparable onboarding trial contract](../onboarding-trials.md).
3. Test `--guided` with representative valid and invalid bag metadata.
4. Fix the largest observed activation blocker before changing the landing
   page or creating promotional material.

Exit: the project has a reproducible onboarding baseline and no undocumented
manual step on the fixed demo.

### Sprint 3 — close the two v1 blockers (weeks 5–8)

1. Track the two open `ndt_omp_ros2` rosdistro PRs and rerun the existing
   dependency-readiness check after external state changes.
2. Execute package-manager clean-install and upgrade evidence when the required
   repositories contain the packages.
3. Run a public first-map validation cohort. Treat every failed attempt as a
   product finding, resolve it publicly, and rerun without live guidance.
4. Require the complete v1 audit before tagging v1.0.

Exit: distribution is reproducible and three independent users have reached a
verified map, or the exact remaining external blocker is publicly recorded.

### Sprint 4 — publish proof, not promises (weeks 9–12)

1. Restructure the README around the measured first-map route.
2. Publish the short demo, UX scorecard, benchmark summary, limitations, and
   v1 release evidence from one canonical landing page.
3. Prepare five bounded contributor tasks from onboarding findings and the old
   issue backlog.
4. Announce the release through a small number of relevant channels and review
   qualified traffic, validation, support load, and Stars after two and four
   weeks.

Exit: a new visitor can understand the job, see proof, try it, report a result,
and find a contribution without maintainer guidance.

## 9. Decision rules

- If four-week Star growth is below 12 and qualified traffic is also flat,
  improve distribution and the clarity of the product wedge.
- If traffic rises but first-map receipts do not, stop promotion and repair
  onboarding.
- If first maps rise but Stars do not, improve the proof summary and the
  post-success invitation; do not add unrelated features.
- If support load exceeds the response target for two weeks, reduce launch
  activity and close documentation or diagnosis gaps before enabling another
  channel.
- If external contributors cannot run focused checks, invest in fixtures and
  development setup before creating more `good first issue` labels.
- If an exploratory SOTA route fails its preregistered gate, retain the honest
  result, run only the authorized bounded diagnostic, and remove the route from
  the product critical path.
- If a Star checkpoint is missed while activation and contribution improve,
  keep the quality plan and revise the forecast. If Stars rise while those
  signals decline, treat the growth as non-durable.

## 10. Immediate next development decision

The next project-wide sprint is **G0 release hygiene and product integration**,
not another broad algorithm expansion. The first deliverable is a clean,
reviewable path for the existing guided own-bag UX and v1 blockers. The current
v44 research route remains limited to its already-authorized bounded
failure-profile work and cannot block this sprint.

The
[2026-08-10 G0 activation decision](../evidence/growth/g0-activation-decision-2026-08-10.md)
selects the missing clean Docker/source onboarding baseline as the first
measured blocker. Broad promotion remains behind the four-row trial and repair
gate.

The immediate bounded G0 multi-stage runtime-image pilot has passed its local
Docker-size proxy and both distro smoke checks. Its
[evidence record](../evidence/onboarding/runtime-image-slimming-2026-08-11.md)
keeps the image and fixture reductions separate. Promotion is still paused
until compressed OCI size, the full public demo, the identified atomic ROS-log
symlink repair, and clean comparable-VM evidence pass from the reviewed
candidate. The next active G0 implementation is therefore the repair plus the
dedicated Humble/Jazzy Docker/source matrix, not a public image retag.

After G0, the next priority is the independent first-map cohort. It is both the
remaining v1 evidence and the fastest honest way to discover whether the
project is genuinely easier for a new user than its alternatives.

## 11. Baseline reproduction

The 2026-08-10 snapshot was taken from the authenticated GitHub REST API and
the local readiness audit. Maintainers can refresh the same aggregate sources
without collecting private product telemetry:

```bash
python3 scripts/collect_growth_snapshot.py \
  --output docs/evidence/growth/$(date -u +%F).json

gh api repos/rsasaki0109/lidar_slam_ros2
gh api repos/rsasaki0109/lidar_slam_ros2/traffic/views
gh api repos/rsasaki0109/lidar_slam_ros2/traffic/clones
gh api repos/rsasaki0109/lidar_slam_ros2/traffic/popular/referrers
gh api 'repos/rsasaki0109/lidar_slam_ros2/releases?per_page=20'
python3 scripts/check_v1_readiness.py --json
```

Repository and pull-request aggregates were cross-checked against
`origin/develop` and the GitHub pulls endpoint. The collector now encodes the
weekly aggregation, privacy boundary, and JSON schema; the individual endpoint
commands remain only as a maintainer audit aid.
