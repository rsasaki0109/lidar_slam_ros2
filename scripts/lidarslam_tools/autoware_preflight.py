"""Adapter boundary for the existing Autoware bag preflight command."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any


class AutowarePreflightAdapter:
    """Load the legacy script lazily and expose only its payload API."""

    def __init__(self, repo_root: Path) -> None:
        self._script_path = repo_root / 'scripts' / 'preflight_autoware_map_bag.py'
        self._module: Any | None = None

    def build_payload(self, bag_path: Path) -> dict[str, Any]:
        return self._load_module().build_preflight_payload(bag_path)

    def _load_module(self) -> Any:
        if self._module is None:
            spec = importlib.util.spec_from_file_location(
                'preflight_autoware_map_bag', self._script_path)
            if spec is None or spec.loader is None:
                raise RuntimeError(f'failed to load {self._script_path}')
            self._module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self._module)
        return self._module
