#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Generate a concise closeout report for exploratory place-recognition and classic-path work."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re


DEFAULT_PLACE_RUNS = {
    'distance_default': Path('output/bench_rko_lio_mid360_current_default_20260325'),
    'scan_context': Path('output/bench_rko_lio_mid360_sc055_yawguess_scagg_screg_20260326'),
    'bev_rerank': Path('output/bench_rko_lio_mid360_20260326_202840'),
    'solid_descriptor': Path('output/bench_rko_lio_mid360_20260326_194021'),
}

DEFAULT_CLASSIC_RUNS = {
    'driving30_gnss_only': Path('output/open_data_applanix_velodyne_gnss_benchmark_driving30_20260325b'),
    'driving30_velocity_always': Path('output/open_data_applanix_velodyne_gnss_benchmark_driving30_with_odom_velocity_planar_translation_20260327'),
    'driving30_velocity_recovery': Path('output/open_data_applanix_velodyne_gnss_benchmark_driving30_with_odom_velocity_planar_translation_recoveryonly_20260327'),
    'bag6_gnss_only': Path('output/open_data_applanix_velodyne_gnss_benchmark_bag6_front_gnss_sidecar_20260327'),
    'bag6_velocity_always': Path('output/open_data_applanix_velodyne_gnss_benchmark_bag6_front_gnss_sidecar_with_velocity_odom_20260327'),
    'bag6_velocity_recovery': Path('output/open_data_applanix_velodyne_gnss_benchmark_bag6_front_gnss_sidecar_with_velocity_odom_recoveryonly_20260327'),
}


def _ape_rmse(run_dir: Path) -> float:
    payload = json.loads((run_dir / 'metrics.json').read_text(encoding='utf-8'))
    return float(payload['evo']['ape']['rmse'])


def _classic_run(run_dir: Path) -> dict[str, object]:
    payload = json.loads((run_dir / 'metrics.json').read_text(encoding='utf-8'))
    cv = payload.get('cross_validation', {})
    lidarslam = payload.get('lidarslam', {})
    log_text = (run_dir / 'lidarslam.launch.log').read_text(encoding='utf-8', errors='replace')
    match = re.search(
        r'Added\s+(\d+)\s+GNSS position constraint edges\s+\((\d+)\s+RTK-like by covariance\)',
        log_text,
    )
    gnss_edges = int(match.group(1)) if match else 0
    rtk_edges = int(match.group(2)) if match else 0
    return {
        'run_dir': str(run_dir),
        'ape_rmse_m': float(payload['evo']['ape']['rmse']),
        'ape_pairs': int(payload['evo']['ape']['pairs']),
        'tum_lines': int(lidarslam.get('tum_lines', 0)),
        'estimated_path_length_m': float(cv.get('estimated_path_length_m', 0.0)),
        'reference_path_length_m': float(cv.get('reference_path_length_m', 0.0)),
        'gnss_edges': gnss_edges,
        'rtk_edges': rtk_edges,
    }


def _place_run(run_dir: Path) -> dict[str, object]:
    metrics = json.loads((run_dir / 'metrics.json').read_text(encoding='utf-8'))
    log_text = (run_dir / 'slam.launch.log').read_text(encoding='utf-8', errors='replace')
    return {
        'run_dir': str(run_dir),
        'ape_rmse_m': float(metrics['evo']['ape']['rmse']),
        'scan_context_candidates': len(re.findall(r'ScanContext loop candidate:', log_text)),
        'bev_rerank_hints': len(re.findall(r'Distance candidate reranked by BEV', log_text)),
        'solid_candidates': len(re.findall(r'SOLiD rerank candidate:', log_text)),
        'accepted_distance_loops': len(re.findall(r'loop_candidate_source:distance', log_text)),
        'accepted_scan_context_loops': len(re.findall(r'loop_candidate_source:scan_context', log_text)),
        'accepted_bev_loops': len(re.findall(r'loop_candidate_source:bev_descriptor', log_text)),
        'accepted_solid_loops': len(re.findall(r'loop_candidate_source:solid_descriptor', log_text)),
    }


def _fmt(value: float) -> str:
    return f'{value:.3f}'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a closeout report for place-recognition and classic-path exploration.',
    )
    for name, path in DEFAULT_PLACE_RUNS.items():
        parser.add_argument(f'--{name.replace("_", "-")}-dir', default=str(path))
    for name, path in DEFAULT_CLASSIC_RUNS.items():
        parser.add_argument(f'--{name.replace("_", "-")}-dir', default=str(path))
    parser.add_argument('--out', default='')
    parser.add_argument('--write-json', default='')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    place_runs = {
        name: _place_run(Path(getattr(args, f'{name}_dir')).expanduser().resolve())
        for name in DEFAULT_PLACE_RUNS
    }
    classic_runs = {
        name: _classic_run(Path(getattr(args, f'{name}_dir')).expanduser().resolve())
        for name in DEFAULT_CLASSIC_RUNS
    }

    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else (
            Path.cwd() / 'output' / f'exploration_closeout_report_{datetime.now().strftime("%Y%m%d")}.md'
        ).resolve()
    )
    json_path = Path(args.write_json).expanduser().resolve() if args.write_json else None

    payload = {
        'place_recognition': place_runs,
        'classic_path': classic_runs,
        'recommendations': {
            'place_recognition_default': 'distance_default',
            'classic_path_position': 'fallback_only',
        },
    }

    report = (
        '# Exploration Closeout Report\n\n'
        'This report closes the current exploratory work on place recognition and the classic fallback path.\n\n'
        '## Place Recognition\n\n'
        '| Strategy | APE RMSE (m) | Scan Context candidates | BEV rerank hints | SOLiD candidates | Accepted distance loops | Accepted SC loops | Accepted BEV loops | Accepted SOLiD loops |\n'
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n'
        f'| distance default | `{_fmt(place_runs["distance_default"]["ape_rmse_m"])}` | `0` | `0` | `0` | `{place_runs["distance_default"]["accepted_distance_loops"]}` | `0` | `0` | `0` |\n'
        f'| Scan Context | `{_fmt(place_runs["scan_context"]["ape_rmse_m"])}` | `{place_runs["scan_context"]["scan_context_candidates"]}` | `0` | `0` | `{place_runs["scan_context"]["accepted_distance_loops"]}` | `{place_runs["scan_context"]["accepted_scan_context_loops"]}` | `0` | `0` |\n'
        f'| BEV rerank | `{_fmt(place_runs["bev_rerank"]["ape_rmse_m"])}` | `0` | `{place_runs["bev_rerank"]["bev_rerank_hints"]}` | `0` | `{place_runs["bev_rerank"]["accepted_distance_loops"]}` | `0` | `{place_runs["bev_rerank"]["accepted_bev_loops"]}` | `0` |\n'
        f'| SOLiD | `{_fmt(place_runs["solid_descriptor"]["ape_rmse_m"])}` | `0` | `0` | `{place_runs["solid_descriptor"]["solid_candidates"]}` | `{place_runs["solid_descriptor"]["accepted_distance_loops"]}` | `0` | `0` | `{place_runs["solid_descriptor"]["accepted_solid_loops"]}` |\n\n'
        '### Place Recognition Decision\n\n'
        f'- Public default remains `distance` because it is the stable documented path (`{_fmt(place_runs["distance_default"]["ape_rmse_m"])} m`).\n'
        f'- `Scan Context` stays opt-in: it can reach `{_fmt(place_runs["scan_context"]["ape_rmse_m"])} m`, but accepted loops still come from the distance-side path in the tracked run.\n'
        f'- `BEV rerank` stays experimental: it improves the tracked run to `{_fmt(place_runs["bev_rerank"]["ape_rmse_m"])} m`, but it currently works as a distance-candidate reranker rather than an independent loop source.\n'
        f'- `SOLiD` stays experimental/off by default: the tracked strict run is `{_fmt(place_runs["solid_descriptor"]["ape_rmse_m"])} m` without a clear accepted-loop gain.\n\n'
        '## Classic Path\n\n'
        '| Dataset / mode | APE RMSE (m) | APE pairs | Corrected poses | Estimated path (m) | Reference path (m) | GNSS edges |\n'
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n'
        f'| driving_30_kmh GNSS only | `{_fmt(classic_runs["driving30_gnss_only"]["ape_rmse_m"])}` | `{classic_runs["driving30_gnss_only"]["ape_pairs"]}` | `{classic_runs["driving30_gnss_only"]["tum_lines"]}` | `{_fmt(classic_runs["driving30_gnss_only"]["estimated_path_length_m"])}` | `{_fmt(classic_runs["driving30_gnss_only"]["reference_path_length_m"])}` | `{classic_runs["driving30_gnss_only"]["gnss_edges"]}` |\n'
        f'| driving_30_kmh velocity prior always-on | `{_fmt(classic_runs["driving30_velocity_always"]["ape_rmse_m"])}` | `{classic_runs["driving30_velocity_always"]["ape_pairs"]}` | `{classic_runs["driving30_velocity_always"]["tum_lines"]}` | `{_fmt(classic_runs["driving30_velocity_always"]["estimated_path_length_m"])}` | `{_fmt(classic_runs["driving30_velocity_always"]["reference_path_length_m"])}` | `{classic_runs["driving30_velocity_always"]["gnss_edges"]}` |\n'
        f'| driving_30_kmh velocity prior recovery-only | `{_fmt(classic_runs["driving30_velocity_recovery"]["ape_rmse_m"])}` | `{classic_runs["driving30_velocity_recovery"]["ape_pairs"]}` | `{classic_runs["driving30_velocity_recovery"]["tum_lines"]}` | `{_fmt(classic_runs["driving30_velocity_recovery"]["estimated_path_length_m"])}` | `{_fmt(classic_runs["driving30_velocity_recovery"]["reference_path_length_m"])}` | `{classic_runs["driving30_velocity_recovery"]["gnss_edges"]}` |\n'
        f'| bag6_front GNSS sidecar only | `{_fmt(classic_runs["bag6_gnss_only"]["ape_rmse_m"])}` | `{classic_runs["bag6_gnss_only"]["ape_pairs"]}` | `{classic_runs["bag6_gnss_only"]["tum_lines"]}` | `{_fmt(classic_runs["bag6_gnss_only"]["estimated_path_length_m"])}` | `{_fmt(classic_runs["bag6_gnss_only"]["reference_path_length_m"])}` | `{classic_runs["bag6_gnss_only"]["gnss_edges"]}` |\n'
        f'| bag6_front velocity prior always-on | `{_fmt(classic_runs["bag6_velocity_always"]["ape_rmse_m"])}` | `{classic_runs["bag6_velocity_always"]["ape_pairs"]}` | `{classic_runs["bag6_velocity_always"]["tum_lines"]}` | `{_fmt(classic_runs["bag6_velocity_always"]["estimated_path_length_m"])}` | `{_fmt(classic_runs["bag6_velocity_always"]["reference_path_length_m"])}` | `{classic_runs["bag6_velocity_always"]["gnss_edges"]}` |\n'
        f'| bag6_front velocity prior recovery-only | `{_fmt(classic_runs["bag6_velocity_recovery"]["ape_rmse_m"])}` | `{classic_runs["bag6_velocity_recovery"]["ape_pairs"]}` | `{classic_runs["bag6_velocity_recovery"]["tum_lines"]}` | `{_fmt(classic_runs["bag6_velocity_recovery"]["estimated_path_length_m"])}` | `{_fmt(classic_runs["bag6_velocity_recovery"]["reference_path_length_m"])}` | `{classic_runs["bag6_velocity_recovery"]["gnss_edges"]}` |\n\n'
        '### Classic Path Decision\n\n'
        f'- `classic path` remains a fallback path, not the main public path.\n'
        f'- For `driving_30_kmh`, the best tracked classic variant is `velocity prior always-on` (`{_fmt(classic_runs["driving30_velocity_always"]["ape_rmse_m"])} m`), better than GNSS-only but still far from the main path.\n'
        f'- For `bag6_front`, `recovery-only` can be excellent (`{_fmt(classic_runs["bag6_velocity_recovery"]["ape_rmse_m"])} m`), but all tracked bag6 runs have `GNSS edges=0`, so that behavior is not a backend-GNSS win and should stay opt-in.\n'
        '- The repo should therefore keep classic-path improvements available for fallback experiments, but stop treating them as candidates for the default public workflow.\n'
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')

    print(out_path)
    if json_path is not None:
        print(json_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
