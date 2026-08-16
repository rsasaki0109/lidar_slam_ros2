# RTK-SLAM acquisition hardening — 2026-08-17

> Implementation: `25d712b7` plus capacity correction `2a997cb9`
>
> Outcome: **BLOCKED_INSUFFICIENT_SPACE / ACTIONABLE**
>
> Benchmark profiles satisfied by this work: **0**

The two RTK-SLAM release profiles remain `NO_DATA`, but obtaining their public
inputs no longer starts an unbounded multi-gigabyte download. The acquisition
tool now builds a source-pinned, resume-aware capacity plan before any network
or write side effect, and gives one copy-ready retry for a larger filesystem.

## Immutable source contract

The dataset is [RTK-SLAM Dataset](https://huggingface.co/datasets/Willyzw/rtk-slam-dataset),
CC-BY-4.0, fixed at revision
`87619d2da3f345109b9a2b0d3a192a8596b4d2d3`. The official tree API supplied
the exact DB3 byte counts and LFS SHA-256 identities. The small metadata files
were hashed through the same immutable revision URLs.

| Sequence | DB3 bytes | DB3 SHA-256 | Metadata bytes | Metadata SHA-256 |
| --- | ---: | --- | ---: | --- |
| Construction Seq2 | 10,656,112,640 | `9e808703a57d7be6afa6a37abb8f5d65c6566f71f4864cd4c24cb01f6ab82af5` | 1,930 | `2cc6cb1e4a53b2d1c371499e489582b88737b1dbce79f01df4b6c811e43db8ff` |
| Construction Seq1 | 13,180,936,192 | `adf7e5e8f8d73a0a3a0c09f80d846ca3a88446809ee391a818260d4bd3d03a7a` | 1,930 | `ad9b7f8b01862305740abd83e671dbc1c080c8d6e2b82a9e05b1152306382318` |
| Stadtgarten Seq2 | 16,793,665,536 | `d303eeaa773ae1606ddbefb38c509009d9ef9c81a7d4bc6b68869db4287d27e2` | 1,928 | `9538fe9ca6b3d496bd35eb13519e38136a4a86c2a857f1f0215a2ded6a62037c` |
| Stadtgarten Seq1 | 30,263,574,528 | `6f674fff7182e54d2aa12cac36b0be36d022f67e8624b3df4d1acf8018fa6e5b` | 1,933 | `b1fa518aead0436fd574db48b3425d3f1fdcb4a9e26a752f1be0733ed9af3aae` |

Surveyed checkpoints and example trajectories come from the official
[RTK-SLAM evaluation repository](https://github.com/Willyzw/rtk-slam-eval),
fixed at detached commit
`f2921a58caf5a87c1f4f73b48c6f2a5e35f92924`. The tool initializes an empty
repository and fetches that exact commit; it does not shallow-clone a moving
default branch. Reuse requires the exact clean commit and all four ground-truth
CSV files.

## Capacity result on the implementation carrier

This command was run from a clean `2a997cb9` checkout:

```bash
python3 scripts/download_rtk_slam_dataset.py \
  --sequence construction_seq2 \
  --eval-assets \
  --dest /tmp/rtk-slam-live-block-evidence
```

The normal, non-dry-run request exited `2` before creating the destination or
starting `wget`/Git network work:

| Field | Exact bytes |
| --- | ---: |
| remaining payload, including 150 MB eval allowance | 10,806,114,570 |
| 10% filesystem reserve | 1,080,611,457 |
| required additional working space | 11,886,726,027 |
| observed free space | 6,409,396,224 |
| additional space required | 5,477,329,803 |

The output status was `BLOCKED_INSUFFICIENT_SPACE` and its next action was:

```bash
python3 scripts/download_rtk_slam_dataset.py \
  --sequence construction_seq2 \
  --eval-assets \
  --dest /mnt/large/rtk_slam
```

The equivalent `--dry-run --json` returned a schema-stable plan with
`side_effects_started: false`; both tested destination paths remained absent.
No large DB3 was downloaded.

## Small-asset and handoff validation

The pinned evaluation assets were fetched once into a generated temporary
directory. The checkout was clean at the expected commit, contained the four
required CSVs, and occupied 92,874,519 bytes including `.git`. This observation
raised the explicit planning allowance from 50 MB to 150 MB; the separate
minimum 1 GB reserve remains in force. A second dry-run selected
`reuse-verified` with zero remaining payload.

The accuracy-suite preflight then accepted those checkpoint assets and rejected
only the absent Construction Seq2 rosbag metadata. This proves the downloader
and suite agree on their directory handoff without claiming a trajectory,
accuracy result, or completed release profile. The 90 MB checkout and 36 MB
MkDocs site used for validation were deleted afterward; they were generated
temporary data and are not recoverable.

## Enforced behavior

- `--dry-run` and `--list` perform no network access or filesystem writes;
- `--json` is accepted only with `--dry-run` and keeps stdout machine-readable;
- live capacity failure occurs before dependency checks, directory creation,
  downloads, or Git fetches;
- existing regular partial files reduce the exact remaining transfer;
- complete files are reused only after exact size and SHA-256 verification;
- oversized, same-size-but-wrong, non-regular, and symlinked inputs fail closed;
- completed downloads are checked by exact size and SHA-256, not a 1% warning;
- eval assets are accepted only at the pinned clean commit; and
- acquisition readiness never substitutes for exact-head benchmark evidence.

Focused downloader and suite regressions pass `16 / 16`, and changed-file
Jazzy `ament_flake8` passes. The complete product gate initially exposed only
the expected missing CTest registration and stale publication inventory; those
review-control failures are repaired in the evidence synchronization commit.
The subsequent complete rerun passes:

- `graph_based_slam`: 1,462 passed / 13 skipped / 11 known ImageIO warnings;
- `lidarslam`: 1,040 passed; and
- combined maintained Python gate: 2,502 passed / 13 skipped.

The synchronized publication plan is `PLAN_VALID_LOCAL_ONLY` with 314 paths,
seven slices, and inventory SHA-256
`78dc0990244e7b47e9f6b89d6ae71bebd9e5341d055c5a760d6416058b29119a`.

## Remaining release work

Run the copy-ready command on a filesystem with enough free space, then execute
`scripts/run_rtk_slam_accuracy_suite.py` for Construction Seq2 and Construction
Seq1 at the exact release candidate. Until both fresh outputs pass their
profile contracts, all four current hard-profile blockers remain unchanged:

- `newer_college_math_hard`;
- `ntu_viral_tnp_01`;
- `mid360_gt_rtkslam_construction_seq2`; and
- `mid360_gt_rtkslam_construction_seq1`.

This work grants no tag, Release, image, merge, environment, benchmark claim,
or dataset-publication authority.
