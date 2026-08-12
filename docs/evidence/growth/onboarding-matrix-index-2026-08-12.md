# Onboarding matrix evidence index — 2026-08-12

> Decision: **TRUTHFUL_2_OF_4 / ACTIVATION_GATE_CLOSED**
>
> Public source route inspected:
> `0a3d5f0c3263082360d87723af0055f74e324c80` (`0.9.1`)
>
> Preflight remote mutations performed: **none**

## Outcome

The shortest maintainer audit now reports the evidence already checked into
the repository instead of treating an omitted positional argument list as an
empty project history:

```bash
python3 scripts/check_onboarding_trial_matrix.py --json
```

The command loads
`docs/contracts/g0-onboarding-matrix-evidence-v1.json`, whose four explicit
rows point to the reviewed Humble and Jazzy Docker records and retain both
source rows as `null`. It does not glob a directory, choose a latest file, or
infer success from a missing record. Explicit paths remain available for
auditing provisional records before maintainers update the index.

## Current truthful matrix

| Row | Present | Outcome | Measurement | Comparable |
| --- | --- | --- | --- | --- |
| Docker Humble | yes | `PASS` | `INCOMPLETE` | no |
| Docker Jazzy | yes | `PASS` | `INCOMPLETE` | no |
| Source Humble | no | `MISSING` | `MISSING` | no |
| Source Jazzy | no | `MISSING` | `MISSING` | no |

The aggregate is two present rows, two product PASS outcomes, zero comparable
rows, and a closed activation gate. The Docker records still lack human active
time, human command count, and isolated peak disk. This change makes that state
easier to see; it does not upgrade either record.

## Public source diagnosis

The network-read-only source preflight against exact public Draft PR commit
`0a3d5f0` returns `READY` for product version `0.9.1`: it resolves all six
maintained source packages and all four route-contract files with no finding
codes or writes. This closes the public-identity prerequisite, but it does not
create an onboarding result. Humble and Jazzy source rows remain legitimately
absent until separate clean disposable hosts run the measured route.

The first exact-tip check also exposed a GitHub API boundary: the commits API
returns HTTP 422, rather than 404, for a syntactically valid but unpublished
40-character commit. The observer now treats 404 and 422 from that commit
lookup as `source-candidate-not-published`, while a 422 from a content lookup
remains an observer error. This keeps an unpublished candidate machine-
decidable without hiding authentication, validation, or malformed-content
failures.

## Fail-closed checks

The evidence-index schema fixes the matrix ID, four row IDs, repository, safe
repository-relative JSON paths, and no-remote-mutation authority. The checker
also requires the fixed row order, regular non-symlink files, and agreement
between an index row and the record's route, distribution, and environment.
Missing, escaped, shuffled, or mislabeled records fail validation.

Validation results:

- onboarding matrix regressions: **12 passed**;
- source public-preflight regressions: **20 passed**, including real 422
  unpublished-tip classification;
- no-argument report: **2/4 present, 2 PASS, 0 comparable**;
- runner and regression-test full flake8: **PASS**;
- complete maintained Python gate: graph **1,428 passed / 13 skipped**,
  lidar_slam **694 passed**, **2,122 total**;
- strict MkDocs build: **PASS** with pre-existing notices;
- source public preflight: **READY** at exact `0a3d5f0`, no writes;
- issue, PR, branch, release, image, and other mutation by the preflight:
  **none**.

## Next gate

Run disposable Humble and Jazzy hosts against exact source commit `0a3d5f0` and
product version `0.9.1`, then retain only schema-valid measured records.
Comparable Docker replacement rows additionally require a dedicated filesystem
and human active-time observation; another shared-host automation run would not
close that gap.
