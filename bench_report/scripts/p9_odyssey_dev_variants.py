#!/usr/bin/env python3
"""Write the parameter and rig files of the Odyssey diagnosis variants (bench_report/data/odyssey_dev_2026-11-13/plan.txt).

usage: p9_odyssey_dev_variants.py G_PARAMS RIG OUT_DIR  ->  OUT_DIR/<variant>.yaml, OUT_DIR/rig_<variant>.yaml
"""
import copy
import sys

import yaml

TAC = ['max_tilt_correction_deg', 'gyro_bias_time_constant_sec', 'gyro_bias_start_sec']
KP = ['icp_keypoint_voxel_multiplier', 'min_icp_keypoints', 'keypoint_shortfall_voxel_multiplier']
CLOSE = ['close_scene_max_median_range', 'close_scene_max_correspondence_distance',
         'close_scene_localizability_boost', 'close_scene_localizability_normal_only',
         'close_scene_min_inlier_fraction']
UP = {'voxel_size': 1.0, 'min_range': 1.0, 'max_expected_jerk': 3.0, 'initialization_phase': False,
      'max_iterations': 100, 'convergence_criterion': 1.0e-5}
VARIANTS = {
    'g': ({}, []),
    'notac': ({}, TAC),
    'vox1': ({'voxel_size': 1.0}, []),
    'vox05': ({'voxel_size': 0.5}, []),
    'noclose': ({}, CLOSE),
    'jerk3': ({'max_expected_jerk': 3.0}, []),
    'noinit': ({'initialization_phase': False}, []),
    'nocompact': ({'local_map_compact_points': False}, []),
    'nokp': ({}, KP),
    'up': (UP, TAC + KP + CLOSE + ['local_map_compact_points', 'initialization_max_gyro_rad_s']),
    # addendum 2: the same file as 'up', run by p9_odyssey_dev.sh with the unmodified upstream 0.3.2 ROS node
    'ros032': (UP, TAC + KP + CLOSE + ['local_map_compact_points', 'initialization_max_gyro_rad_s']),
    # second round (plan.txt, addendum): combinations
    'vox1_notac': ({'voxel_size': 1.0}, TAC),
    'vox05_notac': ({'voxel_size': 0.5}, TAC),
}


def main():
    g, rig, out = sys.argv[1:]
    base, rig = yaml.safe_load(open(g)), yaml.safe_load(open(rig))
    for name, (setk, drop) in VARIANTS.items():
        c = copy.deepcopy(base)
        p = c['/**']['ros__parameters']
        p.update(setk)
        for k in drop:
            p.pop(k)
        yaml.safe_dump(c, open(f'{out}/{name}.yaml', 'w'), sort_keys=False)
        r = dict(rig, g_params=f'{out}/{name}.yaml')
        yaml.safe_dump(r, open(f'{out}/rig_{name}.yaml', 'w'), sort_keys=False)


if __name__ == '__main__':
    main()
