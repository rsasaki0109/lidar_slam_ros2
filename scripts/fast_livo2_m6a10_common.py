#!/usr/bin/env python3
"""Shared low-level helpers for the FAST-LIVO2 m6a10 evidence chain.

The v12..v23 candidates each keep their own authorization and runtime layers,
but the byte-level hashing primitive is identical across every layer.  Keeping
one canonical implementation avoids eleven copies drifting apart while the
layered evidence semantics stay untouched.
"""

from __future__ import annotations

import hashlib
from pathlib import Path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _sha256(path: Path) -> str:
    return sha256_file(path)


__all__ = ['sha256_file', '_sha256']
