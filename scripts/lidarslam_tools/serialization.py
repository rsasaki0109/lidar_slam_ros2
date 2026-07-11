"""Stable serialization helpers shared by command-line tools."""

from __future__ import annotations

import json
from typing import Any


def payload_to_json(payload: dict[str, Any]) -> str:
    """Serialize a CLI payload using the repository's canonical formatting."""
    return json.dumps(payload, indent=2, sort_keys=True)
