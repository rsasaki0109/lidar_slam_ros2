# GLIM usability scorecard contract — 2026-08-12

> Decision: **LOCAL_SCORECARD_CONTRACT_PASS / PAIRED_PUBLIC_TRIALS_MISSING**
>
> Candidate base: `c2be534bb2dbbc784d93238295614b9d08147472`
>
> Remote mutations performed: **none**

## Why this increment

The product candidate has an interactive home, a read-only doctor, fixed demo
routes, own-bag preflight, verified map artifacts, stable recovery actions, and
a Japanese beginner page. Those features can reduce operator work, but their
existence alone does not show that the public workflow is as convenient as
GLIM for a first-time user.

This increment converts that comparison into a fixed, neutral evidence
contract. It does not import private maintainer experience, local-only product
identities, or feature-count claims as usability results.

## Fixed paired protocol

The scorecard measures six overlapping jobs independently:

1. discover the supported path;
2. run one fixed public demo;
3. inspect the operator's own bag;
4. produce and verify a downstream artifact;
5. understand and recover from one public failure; and
6. repeat the documented contract after a supported upgrade.

The two records must bind exact public product versions and documentation,
clean matching hosts supported by each product's selected docs, one pair and
anonymous cohort, the same input for each task, exact command sequences,
task-specific metrics, transcript hashes, and no undocumented steps. One
product must be attempted first and the other second.
`READY` additionally requires an external first-time operator and all six tasks
to be comparable. A maintainer pair can reach only `PARTIAL`.

The schemas, empty evidence index, human protocol, and fail-closed checker are
versioned together. The checker rejects missing or reordered tasks, incomplete
task measurements, environment or input drift, non-public product identities,
dirty hosts, multiline commands, undocumented assistance, and authority to
make an automatic winner claim.

## Verification

| Check | Result |
| --- | --- |
| checked-in no-argument index | `NOT_READY`; 0 records; 0/6 comparable tasks |
| scorecard checker regressions | `13 passed` |
| documentation entrypoint and release-bundle integration | PASS |
| complete maintained Python gate | graph: `1,430 passed / 13 skipped / 11 existing warnings`; lidar_slam: `707 passed`; `2,137 total` |
| Python style for the checker and regression module | PASS |
| schema/index JSON parsing | PASS |
| remote writes | none |

The complete maintained Python gate and exact candidate inventory are recorded
in the publication-slice evidence after this increment is frozen.

## Honest boundary and next measurement

No paired GLIM and `lidarslam_ros2` trial records exist yet. The exact local
candidate is not public, so it cannot be used as a public product identity.
This evidence therefore proves only that a neutral comparison can be recorded
and validated; it does not prove parity, superiority, or a lower completion
time.

After the exact candidate is published, run both products from their public
landing pages on matching clean machines with one external first-time
operator. A stronger public claim requires a second external operator with the
product order reversed. Publish task-level values, commands, versions, and
limitations without collapsing them into an overall winner.
