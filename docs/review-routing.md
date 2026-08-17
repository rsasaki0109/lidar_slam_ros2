# Product Draft review routing

Draft PR #427 is intentionally reviewed as one integrated product candidate,
but no reviewer should need to understand all 337 follow-up paths before
helping. The local routing contract groups the seven validated review slices
into four capability lanes without storing a GitHub login, email address, or
other reviewer identity.

```bash
python3 scripts/check_product_draft_review_routing.py
```

The command is local and read-only. It reruns the exact publication overview,
requires a linear candidate, assigns every S1–S7 slice exactly once,
checks lane dependencies and path/check totals, and validates the result
against the report schema. A dirty checkout is labeled
`PREPARED_DIRTY_WORKTREE` and cannot be used as a public exact-tip handoff. It
does not execute a displayed slice command or
request, submit, approve, or dismiss a GitHub review.

## Capability lanes

| Lane | Scope | Capability |
| --- | --- | --- |
| `R1-runtime-safety` | S1 runtime safety and S2 first-map foundation | ROS 2 C++, PCL, and asynchronous failure containment |
| `R2-operator-ux` | S3 map lifecycle and S4 source onboarding | rosbag2, CLI, documentation, and onboarding |
| `R3-distribution` | S5 distribution readiness | ROS distribution, dependency ownership, and release evidence |
| `R4-integration-publication` | S6 product integration and S7 publication control | CI, product integration, and publication authority boundaries |

Render only one lane when deciding whether it matches your experience:

```bash
python3 scripts/check_product_draft_review_routing.py \
  --lane R1-runtime-safety
```

The lane card prints the exact local slice commands but does not run them.
Complete lanes in dependency order. At most two lanes should be active at once
so a large Draft does not create an unbounded review queue.

## Participation boundary

The target is two advisory reviewers across the four lanes. This is a capacity
and ownership-development target, not a merge gate and not a quota. A person
may volunteer for the lane matching their experience, or the maintainer may
later request a review through a separately authorized GitHub action.

The local contract deliberately contains roles rather than people:

- no username, email address, organization, or inferred identity;
- no claim that a reviewer consented or completed work;
- no review request body or GitHub mutation command;
- no authority to submit a review, mark the Draft ready, or merge; and
- final product decisions remain with the `lead-maintainer` role described in
  [Governance](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/GOVERNANCE.md).

Use `--json` for automation. The source contract and report follow
[`product-draft-review-routing-v1`](schemas/product-draft-review-routing-v1.schema.json)
and
[`product-draft-review-routing-report-v1`](schemas/product-draft-review-routing-report-v1.schema.json).
Changing a lane requires updating the machine contract and proving that every
validated slice still has exactly one lane, exact dependency closure, and an
unchanged no-write authority boundary.
