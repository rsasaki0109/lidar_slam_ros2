"""An MCP server for reviewing a long-term map store (``python -m lidarslam mcp STORE``).

The tools are :class:`lidarslam.map_review.Review`'s methods: an agent (Claude Code, or any MCP
client) looks at what a map update changed, keeps or removes it, and commits one edit with its
reason, which ``python -m lidarslam store rollback-edit`` undoes. With ``--propose-only`` the
agent cannot change the store: a commit writes a proposal for a person to apply. Needs the MCP SDK
(``pip install lidarslam[agent]``).

Claude Code::

    claude mcp add lidarslam-review -- python -m lidarslam mcp /path/to/store --propose-only
"""

from __future__ import annotations

import json
from typing import Any

INSTRUCTIONS = """\
You review what a long-term LiDAR map update changed, in a map store. The store holds mapping
sessions of one place; the latest session's change detection removed old points it saw through
(removed_old), held back new points it saw only briefly (suppressed_new), and added new structure
(added). It is not always right. For each candidate decide:
- removed_old: remove (confirm) when the old structure is really gone (moved furniture, a taken-
  down partition); keep when it is permanent structure still there (walls, pillars, floor) that
  the update removed by mistake.
- suppressed_new: remove (confirm) for transient things (people, carts); keep for new permanent
  structure seen only briefly.
- added: keep for real new structure; remove for transient things and noise.
Start with store_info, page through list_candidates, call describe_candidate (numbers and a
top/side picture) before deciding, use view_region for context, then decide (or decide_box) and
commit with a reason. Leave a candidate undecided when unsure; a person reviews the rest.
Coordinates are metres in the store frame, z up."""


def build_server(store: str, author: str = 'agent:claude', propose_only: bool = False,
                 min_voxels: int = 5) -> Any:
    """The MCP server over one store (mcp.server.mcpserver.MCPServer)."""
    try:
        from mcp.server.mcpserver import MCPServer
        from mcp.server.mcpserver.utilities.types import Image
    except ImportError as error:  # pragma: no cover - depends on the environment
        raise SystemExit('the MCP server needs the MCP SDK 2.x: pip install "lidarslam[agent]"') \
            from error
    from lidarslam.map_review import Review

    review = Review(store, author=author, propose_only=propose_only, min_voxels=min_voxels)
    server = MCPServer('lidarslam-map-review', instructions=INSTRUCTIONS)

    @server.tool()
    def store_info() -> dict[str, Any]:
        """The store's sessions and edits, how many candidates of each kind, the pending
        decisions, the map's bounds, and whether this review only proposes edits."""
        return review.info()

    @server.tool()
    def list_candidates(kind: str | None = None, offset: int = 0, limit: int = 50,
                        undecided_only: bool = False) -> dict[str, Any]:
        """Candidates to review, largest first within each kind (removed_old, suppressed_new,
        added): id, kind, voxels, points, centroid, min and max (store frame, m), sessions and
        the pending decision."""
        return review.list_candidates(kind, offset, min(max(limit, 1), 200), undecided_only)

    @server.tool(structured_output=False)
    def describe_candidate(candidate_id: str) -> list[Any]:
        """A candidate's numbers (size, height above the floor, points per session and state,
        how much of it touches structure that stays) and a picture: top view left, side view
        right, 6 m around it, the candidate in yellow."""
        info, image = review.describe(candidate_id)
        return [json.dumps(info, indent=1), Image(data=image, format='png')]

    @server.tool(structured_output=False)
    def view_region(x: float, y: float, z: float, radius: float = 6.0) -> list[Any]:
        """A top and a side view of the map around a store-frame point (radius 0.5 to 100 m)."""
        return [f'map around ({x:.2f}, {y:.2f}, {z:.2f}), radius {radius} m',
                Image(data=review.view_region([x, y, z], radius), format='png')]

    @server.tool()
    def decide(candidate_id: str, op: str, note: str = '') -> dict[str, Any]:
        """Keep or remove a candidate (op: keep, remove, or clear to drop the decision). Pending
        until commit."""
        return review.decide(candidate_id, op, note)

    @server.tool()
    def decide_box(min_x: float, min_y: float, min_z: float, max_x: float, max_y: float,
                   max_z: float, op: str, note: str = '') -> dict[str, Any]:
        """Keep or remove every stored point inside a store-frame box. Pending until commit."""
        return review.decide_box([min_x, min_y, min_z], [max_x, max_y, max_z], op, note)

    @server.tool()
    def commit(reason: str) -> dict[str, Any]:
        """Write the pending decisions as one edit with this reason (or, in propose-only mode,
        a proposal file for a person to apply). The store's export is rebuilt."""
        return review.commit(reason)

    @server.tool()
    def rollback_edit() -> dict[str, Any]:
        """Take the store's latest edit off the map (its file stays). Not in propose-only mode."""
        return review.rollback_edit()

    return server


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(prog='lidarslam mcp', description=__doc__.splitlines()[0])
    parser.add_argument('store', help='map store directory')
    parser.add_argument('--author', default='agent:claude', help="'agent:<name>' (default)")
    parser.add_argument('--propose-only', action='store_true',
                        help='write proposals for a person to apply instead of editing the store')
    parser.add_argument('--min-voxels', type=int, default=5,
                        help='smallest candidate listed (0.15 m voxels)')
    args = parser.parse_args(argv)
    build_server(args.store, args.author, args.propose_only, args.min_voxels).run('stdio')
    return 0
