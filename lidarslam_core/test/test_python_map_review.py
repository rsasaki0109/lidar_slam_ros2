"""An agent's review of a map store (lidarslam.map_review, lidarslam.mcp_server; CI smoke test).

On the two-session store of test_python_map_edits: describing a candidate gives its numbers and a
PNG; decisions commit as one edit by the agent that rollback_edit undoes; a propose-only review
writes a proposal that apply-edit applies and never changes the store itself. The MCP tools are
checked when the MCP SDK is installed.
"""

from __future__ import annotations

import asyncio
import base64
import json
from pathlib import Path

import pytest

pytest.importorskip('lidarslam')
core = pytest.importorskip('lidarslam._core')
if not hasattr(core, 'store_points'):
    pytest.skip('lidarslam built without map store edits', allow_module_level=True)

from lidarslam.__main__ import main as cli  # noqa: E402
from lidarslam.map_review import Review  # noqa: E402
from lidarslam.store import Store  # noqa: E402
from test_python_map_edits import _bundle_files, store  # noqa: E402,F401


def test_agent_review_commits_and_rolls_back(store: Store) -> None:  # noqa: F811
    before = _bundle_files(store.path / 'export')
    review = Review(store.path, author='agent:test', min_voxels=1)
    info = review.info()
    assert info['candidates'].get('removed_old', 0) > 0 and info['pending_decisions'] == 0
    listed = review.list_candidates('removed_old', limit=2)
    first = listed['candidates'][0]
    details, image = review.describe(first['id'])
    assert image.startswith(b'\x89PNG\r\n\x1a\n') and details['size_m'] and \
        details['points_by_state']['removed_by_evidence'] > 0
    assert review.view_region(first['centroid'], 3.0).startswith(b'\x89PNG')
    with pytest.raises(KeyError):
        review.decide('removed_old-unknown', 'keep')
    with pytest.raises(ValueError):
        review.decide(first['id'], 'erase')
    assert review.decide(first['id'], 'keep', 'still there')['pending'] == 1
    assert review.decide(first['id'], 'keep')['pending'] == 1  # a new decision replaces it
    assert review.list_candidates('removed_old', undecided_only=True)['total'] == \
        listed['total'] - 1
    result = review.commit('the wall is still there')
    assert result['edit']['author'] == 'agent:test'
    assert Store.open(store.path).edits[0]['reason'] == 'the wall is still there'
    assert _bundle_files(store.path / 'export') != before
    with pytest.raises(ValueError):
        review.commit('nothing pending')
    review.rollback_edit()
    assert _bundle_files(store.path / 'export') == before


def test_propose_only_review_leaves_the_store(store: Store) -> None:  # noqa: F811
    before = _bundle_files(store.path / 'export')
    review = Review(store.path, author='agent:test', propose_only=True, min_voxels=1)
    lo = review.points['xyz'].min(axis=0)
    assert review.decide_box(lo.tolist(), (lo + 0.5).tolist(), 'remove')['voxels'] > 0
    result = review.commit('noise in a corner')
    proposal = Path(result['proposal'])
    assert proposal.is_file() and Store.open(store.path).edits == []
    assert _bundle_files(store.path / 'export') == before
    with pytest.raises(PermissionError):
        review.rollback_edit()
    assert cli(['store', 'apply-edit', str(store.path), '--file', str(proposal)]) == 0
    assert Store.open(store.path).edits[0]['author'] == 'agent:test'


def test_mcp_tools(store: Store) -> None:  # noqa: F811
    pytest.importorskip('mcp.server.mcpserver')
    from lidarslam.mcp_server import build_server

    server = build_server(str(store.path), author='agent:test', min_voxels=1)

    async def run() -> None:
        names = {tool.name for tool in await server.list_tools()}
        assert names == {'store_info', 'list_candidates', 'describe_candidate', 'view_region',
                         'decide', 'decide_box', 'commit', 'rollback_edit'}
        listed = await server.call_tool('list_candidates', {'limit': 1})
        candidate = json.loads(listed.content[0].text)['candidates'][0]['id']
        described = await server.call_tool('describe_candidate', {'candidate_id': candidate})
        kinds = [c.type for c in described.content]
        assert kinds == ['text', 'image']
        assert base64.b64decode(described.content[1].data).startswith(b'\x89PNG')
        await server.call_tool('decide', {'candidate_id': candidate, 'op': 'remove'})
        committed = await server.call_tool('commit', {'reason': 'confirmed'})
        assert json.loads(committed.content[0].text)['edit']['author'] == 'agent:test'

    asyncio.run(run())
    assert Store.open(store.path).edits[0]['reason'] == 'confirmed'
