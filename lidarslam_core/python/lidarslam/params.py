"""ROS 2 parameter files without ROS 2.

Reads the YAML parameter files the ROS 2 nodes take (``<node>: ros__parameters: ...``) into a flat
``{name: value}`` dict with dotted names, typed as rcl's parameter file parser types them:
``true``/``false`` are bools, numbers without a fraction or exponent are integers, other numbers
are floats (``1e-4`` included), quoted values are strings, and a list keeps the type of its
items. The backend reads each value with rclcpp's type rules, so a file gives the same values
here as in ROS 2.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Any, Iterable

import yaml


class _ParameterLoader(yaml.SafeLoader):
    """SafeLoader whose floats also include exponent forms without a dot or sign (``1e5``)."""


_ParameterLoader.add_implicit_resolver(
    'tag:yaml.org,2002:float',
    re.compile(r'''^(?:[-+]?(?:[0-9][0-9_]*)(?:\.[0-9_]*)?(?:[eE][-+]?[0-9]+)
                  |[-+]?(?:[0-9][0-9_]*)\.[0-9_]*
                  |[-+]?\.[0-9_]+(?:[eE][-+]?[0-9]+)?
                  |[-+]?\.(?:inf|Inf|INF)
                  |\.(?:nan|NaN|NAN))$''', re.X),
    list('-+0123456789.'))
# Integers stay integers: the int resolver is tried first for the characters both share.
for _first, _resolvers in _ParameterLoader.yaml_implicit_resolvers.items():
    _resolvers.sort(key=lambda r: r[0] != 'tag:yaml.org,2002:int')


def _flatten(prefix: str, value: Any, out: dict[str, Any]) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            _flatten(f'{prefix}.{key}' if prefix else str(key), item, out)
    else:
        out[prefix] = value


def _applies(key: str, node: str | None) -> bool:
    if node is None:
        return True
    key = key.strip()
    return key in ('/**', node, f'/{node}', f'/**/{node}')


def parse_yaml_value(text: str) -> Any:
    """A value as a parameter file or ``-p name:=value`` gives it (``'[1.0, 2.0]'``, ``'1e-4'``)."""
    return yaml.load(text, Loader=_ParameterLoader)


def load_parameters(paths: str | Path | Iterable[str | Path],
                    node: str | None = None) -> dict[str, Any]:
    """Parameters from ROS 2 parameter files, later files and sections overriding earlier ones.

    With ``node``, only the sections that apply to that node name (``/**``, ``node``,
    ``/node``, ``/**/node``) are read; without it, every ``ros__parameters`` section. A file
    without ``ros__parameters`` sections is read as a flat mapping.
    """
    if isinstance(paths, (str, Path)):
        paths = [paths]
    params: dict[str, Any] = {}
    for path in paths:
        data = yaml.load(Path(path).read_text(encoding='utf-8'), Loader=_ParameterLoader) or {}
        if not isinstance(data, dict):
            raise ValueError(f'{path}: expected a mapping')
        sections = [(k, v['ros__parameters']) for k, v in data.items()
                    if isinstance(v, dict) and isinstance(v.get('ros__parameters'), dict)]
        if not sections:
            _flatten('', data, params)
            continue
        for key, section in sections:
            if _applies(str(key), node):
                _flatten('', section, params)
    return params


def apply_overrides(params: dict[str, Any], overrides: Iterable[str]) -> dict[str, Any]:
    """``name:=value`` overrides (the ROS 2 ``-p`` syntax; ``name=value`` also works)."""
    for item in overrides:
        name, sep, value = item.partition(':=')
        if not sep:
            name, sep, value = item.partition('=')
        if not sep or not name:
            raise ValueError(f'expected name:=value, got {item!r}')
        params[name.strip()] = parse_yaml_value(value)
    return params
