# G0 clean candidate audit — 2026-08-11

> Decision: **REVIEWABLE_LOCAL_CANDIDATE / G0_HOLD**
>
> Audited product revision:
> `e5a5616802345935140e4a2712b5791cba036dfb`
>
> Exact base and live public `develop`:
> `86fa9b610c07ccf4d2b0f10939e17c129d34b40a`
>
> Remote mutations performed: **none**

## Outcome

The G0 product line is a clean, linear, locally reviewable candidate. It does
not contain the research worktree's generated outputs or large research media,
and its complete current test/documentation contracts pass. The candidate may
be proposed as a Draft pull request after explicit source-publication
authorization.

It is not release-ready and G0 cannot advance. The source revision is not
publicly resolvable, the onboarding matrix is incomplete, the fixture host and
upload remain undecided, GitHub issue mutations are unauthorized, v1 readiness
is `8 / 10`, and the existing `v0.9.0` tag correctly prevents a new bundle from
being rehearsed under the same version.

## Lineage and worktree

| Check | Observation | Result |
| --- | --- | --- |
| Local base | `origin/develop` = `86fa9b610c07ccf4d2b0f10939e17c129d34b40a` | PASS |
| Live remote base | read-only `git ls-remote origin refs/heads/develop` returned the same hash | PASS |
| Candidate | `e5a5616802345935140e4a2712b5791cba036dfb` | recorded |
| History | 36 commits ahead; 0 merge commits | PASS |
| Worktree | no modified, staged, or untracked path at audit time | PASS |
| Git object graph | `git fsck --no-dangling --no-progress` | PASS |
| Diff whitespace | `git diff --check origin/develop...e5a5616` | PASS |
| Remote branch/upstream | no upstream and no remote branch with the candidate name | LOCAL_ONLY |
| Public commit lookup | GitHub commit GET did not resolve `e5a5616` | UNRESOLVABLE |

The fixture generator `0f91452c`, fixture review `eae8547`, and VoxelGrid fix
`a2368c4` are all ancestors of the audited candidate. Publishing a reviewed
descendant would make those source revisions inspectable; it would not publish
the fixture ZIP or authorize any other remote change.

## Diff inventory

The candidate changes 110 paths: 60 added and 50 modified, with 21,271 lines
added and 298 removed.

| Top-level area | Changed paths |
| --- | ---: |
| `docs/` | 45 |
| `scripts/` | 22 |
| `lidarslam/` | 16 |
| `graph_based_slam/` | 8 |
| `scanmatcher/` | 6 |
| `docker/` | 3 |
| `.github/` | 3 |
| root product/configuration files | 7 |

The path and object audit found:

- no changed `build/`, `install/`, `log/`, `output/`, dataset, bag, map, PCD,
  PLY, LAS/LAZ, archive, or model file;
- no changed `docs/research/`, 3DGS asset, or `lidarslam/images/` path;
- no binary diff and no changed symlink;
- no `.data`, private, secret, credential, raw, or generated directory marker;
- largest added/modified blob: `scanmatcher_component.cpp`, 75,559 bytes.

The repository still contains older multi-megabyte research/visual assets in
the public base history. They were not added or changed by this candidate.
Repository-history reduction remains a separate evidence-backed decision; it
is not a reason to rewrite public history during G0.

## Candidate contents

The 36 commits form four review themes:

1. guided own-bag UX, shared Docker/source first-map path, and comparable trial
   contracts;
2. slim installed runtime images, resumable/safe public-data intake, and an
   immutable-host fixture audit;
3. privacy-bounded growth/community operations, a complete issue disposition
   proposal, and the 2026–2029 operating plan;
4. fail-closed classic scanmatcher VoxelGrid safety for issue #69.

No SOTA-v6 implementation, consumed research dataset, generated benchmark
output, or local research evidence is required to build or operate the product
candidate.

## Verification ledger

| Gate | Exact result | Candidate interpretation |
| --- | --- | --- |
| Complete Python product suite | `484 passed` | PASS |
| Issue proposal checker/tests | 29/29 coverage; live drift PASS; `18 passed`; 0 mutations | PASS, not authorized to apply |
| Humble scanmatcher build | immutable v0.9 Humble image, PCL 1.12, clean read-only source build | PASS |
| Humble scanmatcher CTest | 9 / 9 suites; VoxelGrid focused cases 11 / 11 | PASS |
| Jazzy scanmatcher build | PCL 1.14/GCC 13.3 clean temporary build | PASS |
| Jazzy scanmatcher CTest | 9 / 9 suites; VoxelGrid focused cases 11 / 11 | PASS |
| Documentation | `mkdocs build --strict` | PASS with pre-existing Material/nav notices |
| C++ new-file style | `ament_uncrustify`; `ament_cpplint` with package-consistent copyright filter | PASS |
| v1 live readiness | 8 / 10; stable v0.9 `PUBLISHED`; distribution and 0/3 adoption open | HOLD |
| Onboarding matrix | 2 / 4 present, 2 product PASS, 0 comparable; both source rows missing | HOLD |
| Fixture packet | 13 / 13 local checks; exact 98,873,952-byte ZIP; host unset; upload unauthorized | HOLD |
| Release-bundle rehearsal | refused because existing `v0.9.0` tag names `0df0c4a`, not candidate HEAD | EXPECTED_BLOCK |

The release-bundle refusal is a correct safety result. `VERSION` remains
`0.9.0`, and the immutable historical tag must not be moved or reused. Before
an actual release candidate, a separately reviewed release decision must choose
the next semantic version, update versioned package/release records, and rerun
the two-build bundle proof from that exact clean revision. This audit does not
authorize or select that version.

## G0 exit audit

| G0 exit | Current state | Required transition |
| --- | --- | --- |
| P1 #69 safe rejection | local implementation and dual-distro tests PASS | make reviewed revision public; add public unsafe-then-safe component evidence and carrying release |
| Reviewable product line | local clean candidate PASS | E1 authorization, push exact tip, open Draft PR, review CI/diff |
| Current issue triage | 29/29 proposal valid and live-current | E3 authorization before labels, comments, or closures |
| Public fixture identity | local packet PASS | E2 host and upload decision; fresh readiness and remote re-download audit |
| Four-row onboarding matrix | 2/4 outcomes, 0/4 comparable | public source/fixture identity, then fresh dedicated Humble/Jazzy Docker/source rows |
| v1 readiness | 8/10 | distribution completion and three independent first maps |

The phase decision is `HOLD`, not `ADVANCE`. A local quality PASS is useful
preparation but cannot be relabeled as public reproducibility or adoption.

## Next bounded action

Present the separate
[G0 external-action decision packet](g0-external-action-decision-packet-2026-08-11.md).
If E1 is approved, rerun this audit on the exact intended push tip, push only
that branch, open a Draft PR against `develop`, and wait for public CI/review.
E2 fixture publication and E3 community mutations remain independent choices.

If E1 is deferred, continue safe local reliability and release-readiness work;
do not create local-path source trials or claim matrix comparability.
