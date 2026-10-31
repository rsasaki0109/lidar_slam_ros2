#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.

"""
List how each RKO-LIO profile departs from the built-in defaults.

The defaults are read from the vendored RKO-LIO sources: the literal
initialisers of ``rko_lio::core::LIO::Config`` and the literal defaults of the
ROS node's ``declare_parameter`` calls. Each profile is flattened (a
``/**: ros__parameters:`` wrapper is removed) and every key whose value differs
from the default, or that has no known default, is reported.

Keys that only describe the sensor rig (topics, frames, extrinsics, units and
clock offsets) are reported separately from the algorithm keys, because a
single global configuration may still carry per-rig calibration.
"""

import argparse
import json
import math
import pathlib
import re
import sys

import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
RKO = REPO / 'Thirdparty' / 'rko_lio'

SENSOR_KEY_PATTERNS = (
    r'.*_topic$',
    r'.*_frame$',
    r'^extrinsic_.*',
    r'^imu_acceleration_unit$',
    r'^lidar_time_offset_sec$',
    r'^lidar_timestamps\..*',
    r'^invert_odom_tf$',
    r'^publish_.*',
    r'^publisher_queue_depth$',
    r'^output_subscriber_timeout_sec$',
    r'^odom_at_imu_rate$',
    r'^seq\..*',
    r'^max_num_threads$',
    r'^.*_calibration.*',
    r'^camera_.*',
    r'^radar_extrinsic.*',
    r'^photometric_model\..*',
    r'^photometric_image\.(masks|line_highpass|line_lowpass)$',
    r'^async\..*',
)

# Sub-configurations whose ROS keys carry a prefix: (header, struct, key prefix).
PREFIXED_STRUCTS = (
    ('photometric_features.hpp', 'PhotometricFeatureConfig', 'photometric_features.'),
    ('photometric_image.hpp', 'IntensityImageConfig', 'photometric_image.'),
)

# The representative profile of each dataset family: the one the current
# comparison or benchmark report quotes. Paths are relative to the repository.
FAMILY_PROFILES = {
    'mapshd_humanoid_mid360':
        'bench_report/params/rko_params_mapshd_gyro002_jerk1000_corr025_locn10.ros.yaml',
    'oxford_spires_hesai':
        'bench_report/params/rko_params_oxford_gyro002_jerk1000.ros.yaml',
    'hilti2022_pandar_exp04':
        'bench_report/params/rko_params_gyro002_jerk1000.ros.yaml',
    'hilti2022_competitive_v2':
        'configs/hilti2022/rko_lio_hilti2022_pandar_competitive_v2.yaml',
    'ntu_viral_competitive_v2': 'configs/ntu_viral/rko_lio_ntu_viral_competitive_v2.yaml',
    'enwide_os0_v10': 'configs/enwide/rko_lio_os0_open_ground_v10.yaml',
    'geode_alpha_handheld_v18': 'configs/geode/rko_lio_alpha_handheld_v18.yaml',
    'geode_alpha_v18': 'configs/geode/rko_lio_alpha_v18.yaml',
    'geode_beta_ugv_v19': 'configs/geode/rko_lio_beta_ugv_v19.yaml',
    'geode_gamma_ugv_v19': 'configs/geode/rko_lio_gamma_ugv_v19.yaml',
    'rtk_slam_mid360': 'configs/mid360_robot/rko_lio_rtk_slam_mid360.yaml',
    'koide_mid360_handheld_outdoor': 'lidarslam/param/rko_lio_mid360_handheld_outdoor.yaml',
    'mid360_product_default': 'lidarslam/param/rko_lio_mid360.yaml',
    'mid360_walking': 'lidarslam/param/rko_lio_mid360_walking.yaml',
    'kitti_odometry': 'lidarslam/param/rko_lio_kitti_odometry.yaml',
}

_CPP_LITERAL = re.compile(
    r'^\s*(?:bool|int|size_t|double|float|std::string)\s+(\w+)\s*=\s*([^;]+);')
_ROS_DECLARE = re.compile(
    r'declare_parameter<(?:bool|int|double|std::string)>\(\s*"([\w.]+)"\s*,\s*([^)]*)\)')


def _parse_literal(text):
    text = text.strip()
    if text in ('true', 'false'):
        return text == 'true'
    if text.startswith('"') and text.endswith('"'):
        return text[1:-1]
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return None


def _struct_literals(header, struct, prefix=''):
    """Return {prefix + member: literal initialiser} of one C++ struct."""
    found = {}
    inside = False
    for line in (RKO / 'rko_lio' / 'core' / header).read_text().splitlines():
        if re.match(rf'^\s*struct {struct}\s*\{{', line):
            inside = True
            continue
        if inside and re.match(r'^\s*\};', line):
            break
        if inside:
            match = _CPP_LITERAL.match(line)
            if match:
                value = _parse_literal(match.group(2))
                if value is not None:
                    found[prefix + match.group(1)] = value
    return found


def load_defaults():
    """Return {key: default} from the vendored RKO-LIO sources."""
    defaults = _struct_literals('lio.hpp', 'Config')
    for header, struct, prefix in PREFIXED_STRUCTS:
        defaults.update(_struct_literals(header, struct, prefix))
    node_src = (RKO / 'rko_lio' / 'ros' / 'base_node.cpp').read_text()
    for match in _ROS_DECLARE.finditer(node_src):
        key, raw = match.group(1), match.group(2)
        if key in defaults:
            continue
        value = _parse_literal(raw)
        if value is not None:
            defaults[key] = value
    # Node members initialised in the header supply the defaults that the
    # declare_parameter calls pass by name.
    node_hpp = (RKO / 'rko_lio' / 'ros' / 'base_node.hpp').read_text().splitlines()
    for line in node_hpp:
        match = _CPP_LITERAL.match(line)
        if match and match.group(1) not in defaults:
            value = _parse_literal(match.group(2))
            if value is not None:
                defaults[match.group(1)] = value
    return defaults


def load_profile(path):
    """Return the flat parameter dictionary of one profile."""
    data = yaml.safe_load(path.read_text()) or {}
    if isinstance(data, dict) and len(data) == 1:
        only_value = next(iter(data.values()))
        if isinstance(only_value, dict) and 'ros__parameters' in only_value:
            data = only_value['ros__parameters']
    if isinstance(data, dict) and 'ros__parameters' in data:
        data = data['ros__parameters']
    flat = {}

    def walk(prefix, node):
        for key, value in node.items():
            name = f'{prefix}{key}'
            if isinstance(value, dict):
                walk(f'{name}.', value)
            else:
                flat[name] = value

    walk('', data)
    return flat


def is_sensor_key(key):
    """Return whether a key describes the rig rather than the algorithm."""
    return any(re.match(pattern, key) for pattern in SENSOR_KEY_PATTERNS)


def same_value(left, right):
    """Compare a profile value with a default, numbers by value."""
    if isinstance(left, bool) or isinstance(right, bool):
        return left is right if isinstance(left, bool) and isinstance(right, bool) else False
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=1e-12, abs_tol=0.0)
    return left == right


def diff_profile(flat, defaults):
    """Return (algorithm_diffs, sensor_diffs) as {key: (default, value)}."""
    algorithm, sensor = {}, {}
    for key, value in sorted(flat.items()):
        default = defaults.get(key, '<no default>')
        if default != '<no default>' and same_value(default, value):
            continue
        (sensor if is_sensor_key(key) else algorithm)[key] = (default, value)
    return algorithm, sensor


def markdown(results):
    """Render the key-by-family matrix of algorithm overrides."""
    families = list(results)
    keys = sorted({key for algorithm, _ in results.values() for key in algorithm})
    defaults = {}
    for algorithm, _ in results.values():
        for key, (default, _) in algorithm.items():
            defaults[key] = default
    lines = ['| key | default | ' + ' | '.join(families) + ' | families |',
             '|---|---|' + '---|' * len(families) + '---:|']
    for key in keys:
        cells, count = [], 0
        for family in families:
            algorithm, _ = results[family]
            if key in algorithm:
                cells.append(f'`{algorithm[key][1]}`')
                count += 1
            else:
                cells.append('')
        lines.append(f'| `{key}` | `{defaults[key]}` | ' + ' | '.join(cells) + f' | {count} |')
    return '\n'.join(lines)


def main(argv=None):
    """Print the inventory of the given or the representative profiles."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--json', action='store_true', help='print JSON instead of Markdown')
    parser.add_argument('profiles', nargs='*', type=pathlib.Path,
                        help='profiles to compare (default: one per dataset family)')
    args = parser.parse_args(argv)

    defaults = load_defaults()
    if args.profiles:
        targets = {path.stem: path for path in args.profiles}
    else:
        targets = {name: REPO / rel for name, rel in FAMILY_PROFILES.items()}
    results = {}
    for name, path in targets.items():
        if not path.is_file():
            print(f'missing profile: {path}', file=sys.stderr)
            return 1
        results[name] = diff_profile(load_profile(path), defaults)

    if args.json:
        payload = {
            name: {
                'algorithm': {k: {'default': d, 'value': v} for k, (d, v) in alg.items()},
                'sensor': {k: {'default': d, 'value': v} for k, (d, v) in sen.items()},
            }
            for name, (alg, sen) in results.items()
        }
        print(json.dumps(payload, indent=2, sort_keys=True, default=str))
    else:
        print(markdown(results))
    return 0


if __name__ == '__main__':
    sys.exit(main())
