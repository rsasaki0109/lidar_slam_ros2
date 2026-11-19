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

## Let an agent review

`python -m lidarslam mcp STORE` is an MCP server (stdio) over the same review. Its tools list
the candidates, describe one with numbers (size, height above the floor, points per session,
how much of it touches structure that stays) and a picture (a top and a side view of it in its
surroundings), show any region, keep or remove candidates or boxes, commit the decisions as one
edit with a reason, and roll the latest edit back. The server tells the agent what each kind
of candidate means and when to keep or remove it; what it is unsure about it leaves undecided
for a person.

```bash
pip install "lidarslam[agent]"   # the MCP SDK
claude mcp add lidarslam-review -- python -m lidarslam mcp /path/to/store --propose-only
```

Then ask Claude Code to review the store. With `--propose-only` the agent cannot change the
store: each commit writes `STORE/proposals/<time>-<name>.json`, which you inspect (or open next
to a review pack in the editor) and apply with `store apply-edit`. Without it the agent's
edits go straight into the store as `agent:<name>` edits (`--author agent:NAME`), each undone
with `store rollback-edit`.
