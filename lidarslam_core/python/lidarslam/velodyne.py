"""Velodyne packets (``velodyne_msgs/msg/VelodyneScan``) as point clouds, without ROS 2.

The decoding is velodyne_pointcloud's (ros-drivers/velodyne 56fc178d, BSD-3-Clause, vendored in
``lidarslam._core``), as ``velodyne_transform_node`` runs it with ``organize_cloud: false`` and no
fixed or target frame: one unorganized cloud per scan with the fields x, y, z, intensity, ring and
time (seconds from the scan stamp), in the sensor frame of the scan.

Parameters (``velodyne.<name>`` in a parameter file of the odometry):

* ``model``: ``VLP16`` (default), ``32C``, ``32E``, ``VLS128``;
* ``calibration``: a velodyne_pointcloud calibration YAML (default: the model's file from
  velodyne_pointcloud, shipped with the package);
* ``min_range`` / ``max_range`` (default 0.9 / 200.0 m), ``view_direction`` (0.0 rad),
  ``view_width`` (2 pi rad).

The defaults of ``min_range`` and ``max_range`` are the ones the Leo Drive benchmark
(``run_open_data_applanix_velodyne_gnss_benchmark.sh``) passes to the node.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from lidarslam import _core

VELODYNE_SCAN = 'velodyne_msgs/msg/VelodyneScan'
PARAM_PREFIX = 'velodyne.'
CALIBRATION_DIR = Path(__file__).resolve().parent / 'velodyne'
# The calibration file velodyne_pointcloud's launch files use for each model.
MODEL_CALIBRATIONS = {
    'VLP16': 'VLP16db.yaml',
    '32C': 'VeloView-VLP-32C.yaml',
    '32E': '32db.yaml',
    'VLS128': 'VLS128.yaml',
}
DEFAULTS: dict[str, Any] = {
    'model': 'VLP16',
    'calibration': '',
    'min_range': 0.9,
    'max_range': 200.0,
    'view_direction': 0.0,
    'view_width': 2.0 * math.pi,
}

# velodyne_msgs (ros-drivers/velodyne), for logs that do not carry their message definitions.
_MSG_DEFINITIONS = {
    'velodyne_msgs/msg/VelodynePacket': 'builtin_interfaces/Time stamp\nuint8[1206] data\n',
    'velodyne_msgs/msg/VelodyneScan': 'std_msgs/Header header\nvelodyne_msgs/VelodynePacket[] packets\n',
}


def register_types(typestore: Any) -> None:
    """Adds the velodyne_msgs types to a rosbags typestore that lacks them."""
    from rosbags.typesys import get_types_from_msg

    types: dict[str, Any] = {}
    for name, text in _MSG_DEFINITIONS.items():
        if name not in typestore.types:
            types.update(get_types_from_msg(text, name))
    if types:
        typestore.register(types)


def load_calibration(path: str | Path) -> dict[str, Any]:
    """A velodyne_pointcloud calibration file with every scalar kept as written.

    The decoder converts the scalars as yaml-cpp does (``node.as<float>()``), so they are not
    parsed into Python numbers here.
    """
    with Path(path).open(encoding='utf-8') as handle:
        document = yaml.load(handle, Loader=yaml.BaseLoader)  # noqa: S506 (scalars stay strings)
    if not isinstance(document, dict) or not isinstance(document.get('lasers'), list):
        raise ValueError(f'{path}: not a velodyne_pointcloud calibration (no lasers list)')
    return document


def calibration_for_model(model: str) -> Path:
    try:
        return CALIBRATION_DIR / MODEL_CALIBRATIONS[model]
    except KeyError:
        raise ValueError(
            f'no default calibration for Velodyne model {model!r}; pass velodyne.calibration '
            f'(models with one: {", ".join(sorted(MODEL_CALIBRATIONS))})') from None


def split_parameters(params: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    """(the velodyne.* settings with defaults, the other parameters)."""
    settings = dict(DEFAULTS)
    rest = {}
    for name, value in params.items():
        if name.startswith(PARAM_PREFIX):
            key = name[len(PARAM_PREFIX):]
            if key not in DEFAULTS:
                raise KeyError(f'unknown parameter {name} (known: '
                               f'{", ".join(PARAM_PREFIX + k for k in DEFAULTS)})')
            settings[key] = value
        else:
            rest[name] = value
    return settings, rest


class Decoder:
    """VelodyneScan -> :class:`lidarslam._core.PackedCloud`, as velodyne_transform_node."""

    def __init__(self, model: str = DEFAULTS['model'], calibration: str | Path = '',
                 min_range: float = DEFAULTS['min_range'],
                 max_range: float = DEFAULTS['max_range'],
                 view_direction: float = DEFAULTS['view_direction'],
                 view_width: float = DEFAULTS['view_width']):
        path = Path(calibration) if calibration else calibration_for_model(str(model))
        self.calibration_path = path
        self.model = str(model)
        self._decoder = _core.VelodyneDecoder(
            load_calibration(path), self.model, float(min_range), float(max_range),
            float(view_direction), float(view_width))

    @classmethod
    def from_settings(cls, settings: dict[str, Any]) -> 'Decoder':
        return cls(**settings)

    @property
    def num_lasers(self) -> int:
        return self._decoder.num_lasers

    def decode(self, scan: Any) -> Any:
        """One VelodyneScan message (rosbags) -> its point cloud."""
        header = _core.Header()
        header.stamp.sec = int(scan.header.stamp.sec)
        header.stamp.nanosec = int(scan.header.stamp.nanosec)
        header.frame_id = str(scan.header.frame_id)
        packets = [(int(p.stamp.sec), int(p.stamp.nanosec), p.data.tobytes())
                   for p in scan.packets]
        return self._decoder.decode(header, packets)
