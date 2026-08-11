# G0 external-action decision packet — 2026-08-11

> Status: **DECISION_REQUIRED / NO_REMOTE_ACTION_TAKEN**
>
> Audited product revision: `e5a5616802345935140e4a2712b5791cba036dfb`
>
> Public base: `develop` at
> `86fa9b610c07ccf4d2b0f10939e17c129d34b40a`

This packet separates source publication, geometry-bearing fixture
publication, community mutations, and a stable release. Approval of one row
does not approve another.

## Recommended sequence

1. approve E1 source publication as a branch plus Draft PR only;
2. review public CI and the exact 36-commit product diff;
3. choose E2a Zenodo or E2b GitHub immutable Release for the fixture, or defer;
4. after a successful remote fixture audit, run the four fresh onboarding rows;
5. authorize any E3 issue operation only after its read-only drift check;
6. keep E4 stable release denied until G0 evidence and version planning pass.

This order makes the source and fixture identities independently inspectable
before asking external validators to use them.

## E1 — source branch and Draft PR

### Proposed operation

- rerun lineage, worktree, issue-drift, test, and docs checks on the exact tip;
- push local branch `agent/product-g0-guided-ux` to `origin` without force;
- open one **Draft** pull request into `develop`;
- describe the four review themes from the
  [clean-candidate audit](g0-clean-candidate-audit-2026-08-11.md);
- do not merge, tag, publish a release, move an image tag, or change an issue.

### Evidence

- exact public base equals local `origin/develop`;
- 36 linear commits, 0 merges, clean worktree, 110 product paths;
- no changed research/generated/bag/map/archive/binary path;
- Python 484 PASS, Humble/Jazzy scanmatcher 9/9 PASS, docs strict PASS;
- candidate and its fixture/#69 ancestors are currently public-unresolvable.

### Recommendation

**APPROVE E1** if the intended outcome is public review. A Draft PR is the
smallest reversible action that removes the source-identity blocker while
retaining merge and release control.

### Reversibility

The Draft PR can be closed and the remote branch deleted. Public review events
and commit objects may remain visible in GitHub history/caches, so publication
is not equivalent to a private preview. No force push is proposed.

## E2 — onboarding fixture host and upload

The exact derivative packet is:

| Item | Identity |
| --- | --- |
| ZIP | `mid360_onboarding_50s_v1.zip`; 98,873,952 bytes; SHA-256 `20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3` |
| Build manifest | SHA-256 `60c37f5c7efa7d61ca20f21803fa11b02add4bad047ae99d277e9e6811fbbb6e` |
| Geometry-free map receipt | SHA-256 `86d2b5d2aa493cbb6ecc6efd88095a591f247bcea4bc171c68093cf165cc0754` |
| Local gate | 13 / 13 PASS; two byte-identical clean rebuilds |
| License/provenance | CC BY 4.0 derivative attribution to the pinned public Zenodo MID-360 source |

### E2a — Zenodo derivative record (recommended)

Use a versioned Zenodo record because this is a dataset derivative with its
own attribution, checksum, and lifecycle. Reserve the DOI in a draft, review
metadata and exact files, then publish only after a fresh readiness PASS.

Trade-off: a published Zenodo version is intentionally durable and cannot be
treated like a disposable branch. Corrections require a new version and clear
linkage.

### E2b — future GitHub immutable Release asset

Use only a future release that reports `immutable: true`; the existing v0.9.0
release reports false and must not be altered for this purpose. This keeps the
file near source but couples dataset lifecycle to a source release and offers
no dataset DOI.

### E2c — defer

Keep the packet local. This is safe but leaves the smaller onboarding route and
publicly comparable fixture rows blocked. The full 517,088,133-byte public
source route remains available and unchanged.

### Recommendation

**APPROVE E2a only after E1 is publicly resolvable**, if publishing the
geometry-bearing derivative is intended. Approval must name Zenodo and the
exact SHA-256 above. Draft creation/upload and final publication should be
treated as two review points; no credential or local path enters evidence.

## E3 — GitHub issue/community mutations

The read-only proposal covers all 29 current open issues and currently passes
live drift. It proposes 23 reasoned closures, four current-reproduction
requests, and two keep-open issues. Five separate starter drafts are bounded
to approximately 30 minutes.

Choose each batch independently:

| Batch | Proposed mutation | Recommendation |
| --- | --- | --- |
| E3a — labels and reproduction requests | apply reviewed labels without removing observed labels; post four bounded current-reproduction requests | approve only with an immediate live drift check and an exact mutation log |
| E3b — 23 reasoned closures | post issue-specific answer/resolution/superseded/not-planned text and close | review wording as a separate batch; do not infer from E3a or generic cleanup approval |
| E3c — five starter issues | duplicate-check and create C1–C5 with files, non-goals, acceptance, and focused checks | useful after E1, so contributors can reference public candidate files |
| E3d — Discussions | enable/configure categories and moderation | defer until weekly moderation capacity and support boundary are staffed |

Issue comments and closures remain in public event history even if later
reopened. The checker has no write mode; any approved mutation needs a separate
execution log and post-write read-back.

## E4 — stable release

### Recommendation

**DO NOT APPROVE E4 now.**

`VERSION` remains `0.9.0`, and the immutable existing `v0.9.0` tag names its
historical release commit. The current bundle rehearsal correctly refuses to
reuse that version. The four-row onboarding matrix is also incomplete, source
and fixture identities are unpublished, and v1 readiness remains `8 / 10`.

A later release decision must select a new semantic version, update all
versioned package/release records, reproduce the two-build bundle and image
gates, and explicitly authorize tag/release/package/image publication.

## Exact decision form

A maintainer can respond with one line per gate, for example:

```text
E1: APPROVE branch push + Draft PR only
E2: APPROVE Zenodo draft/upload of SHA-256 20e515...bd3; final publish requires another review
E3a: APPROVE labels + four reproduction requests
E3b: DEFER 23 closures
E3c: APPROVE five starter issues after E1
E3d: DEFER Discussions
E4: DENY stable release for now
```

Omitted rows remain **not authorized**. “Continue,” “go ahead,” or approval of
the long-term goal does not silently expand into a push, upload, issue change,
or release operation.

## Decision consequences

| Choice | Immediate next evidence |
| --- | --- |
| E1 approved | exact-tip re-audit, public branch/PR identity, CI and review result |
| E2a/E2b approved after E1 | `PUBLICATION_READY`, host draft review, published immutable identity, remote re-download SHA-256 |
| E3 batch approved | pre-write live drift PASS, bounded mutation log, post-write read-back |
| E4 later approved | version plan, clean release-candidate revision, bundle/image/package publication audits |
| all deferred | continue local reliability and documentation; G0 remains HOLD |
