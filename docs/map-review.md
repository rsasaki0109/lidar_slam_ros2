# Review Map Updates

When a long-term map store takes a new session (`python -m lidarslam live ... --store STORE
--detect-changes`), the change detection removes old points it saw through and holds back new
points it saw only briefly. It is not always right: on the LUH development and holdout data it
left ghosts and missed new structure ([long-term mapping plan](roadmap/long-term-mapping-2026-11.md),
L5). The review step lets a person, or an agent, look at what changed and decide.

<iframe src="map-editor/index.html" title="Map review editor"
        style="width: 100%; height: 72vh; border: 1px solid rgba(127,127,127,0.3); border-radius: 8px;"
        loading="lazy"></iframe>

[Open the editor on its own page](map-editor/index.html). Press **Sample** to try it on two
sessions of the LUH long-term indoor dataset (S3 then S6, CC BY 4.0). The editor runs in the
browser; a pack you open is not uploaded.

## What is reviewed

`python -m lidarslam store candidates STORE` lists what the update changed, as clusters of
0.15 m store-frame voxels (the voxels the change evidence uses), at most about 3 m across:

| Kind | What it is |
| --- | --- |
| `removed_old` | old points the change evidence removed (a wall that was moved, or one it saw through by mistake) |
| `suppressed_new` | new points it held back (a person walking by, or a new desk seen only briefly) |
| `added` | new points where the earlier sessions had none (new structure) |

## Edit in the browser

```bash
python -m lidarslam store review-pack STORE --out map.lsreview
```

Open `map.lsreview` in the editor. Pick a candidate and press **Keep** (K) or **Remove** (R),
**Next** (N) for the next undecided one, or drag with Shift held to select points and keep or
remove the selection. **Save edit** downloads `edit-<store>.json` with your name and reason:

```bash
python -m lidarslam store apply-edit STORE --file edit-STORE.json
```

The edit records the store state its pack showed; it is refused when the store has taken another
session or edit since (review a new pack, or pass `--force`).

## Edit from the command line

```bash
python -m lidarslam store edit STORE --author human:alice --reason "the partition is still there" \
  --keep removed_old-db19fb7629
python -m lidarslam store edit STORE --author human:alice --reason "a person" \
  --remove-box 4.0 2.0 -0.5 5.0 3.0 2.0
```

`--keep` / `--remove` take candidate ids; `--keep-box` / `--remove-box` take a store-frame box
(min x y z, max x y z, metres). An author is `human:<name>` or `agent:<name>`.

## How edits apply

- Edits are append-only files `STORE/edits/<id>.json`, listed in `store.json` in the order they
  apply. Each keeps its author, reason and the store's sessions when it was made.
- The export (`STORE/export/`) applies them after the change evidence: for each voxel the last
  edit decides. **Keep** keeps the voxel's points even where the evidence removed them;
  **remove** drops them.
- `python -m lidarslam store rollback-edit STORE` takes the latest edit off the list and rebuilds
  the export; the edit's file stays. Without edits the export is the same as before.
- Edits address the store frame, so they stay in force when a later session sees the place
  again, until a later edit says otherwise.
