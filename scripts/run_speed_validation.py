#!/usr/bin/env python3
"""Run connected-map behavior at configured speeds."""

import argparse
import csv
import json
import math
from pathlib import Path

import yaml

from run_connected_validation import (ROOT, git_value, run_case, sha256,
                                      utc_now)


DEFAULT_CONFIG = ROOT / 'config/speed_validation.yaml'


def validate(config: dict, available_cases: dict) -> None:
    speeds = config.get('speeds_mps')
    if (not isinstance(speeds, list) or not speeds
            or any(isinstance(speed, bool)
                   or not isinstance(speed, (int, float))
                   or not math.isfinite(speed) or speed <= 0
                   for speed in speeds)
            or len(set(speeds)) != len(speeds)):
        raise ValueError('speeds_mps must contain distinct positive values')
    repeats = config.get('repeats')
    if (isinstance(repeats, bool) or not isinstance(repeats, int)
            or repeats < 1):
        raise ValueError('repeats must be a positive integer')
    cases = config.get('cases')
    if (not isinstance(cases, list) or not cases
            or len(set(cases)) != len(cases)
            or any(name not in available_cases for name in cases)):
        raise ValueError('cases must be distinct configured names')


def variant(template: dict, speed: float) -> dict:
    config = json.loads(json.dumps(template))
    parameters = config['waypoint_follower']['ros__parameters']
    parameters['target_speed'] = speed
    parameters['max_linear_speed'] = speed
    return config


def measurements(record: dict, pedestrian: bool) -> dict:
    results = record['results']
    lane = results.get('lane_result_file', {})
    lane_values = {
        'painted_lane_passed': lane.get('passed'),
        'centerline_overlap_samples': lane.get('centerline_overlap_samples'),
        'edge_line_overlap_samples': lane.get('edge_line_overlap_samples'),
        'max_centerline_overlap_m': lane.get('max_centerline_overlap_m'),
        'max_edge_line_overlap_m': lane.get('max_edge_line_overlap_m'),
    }
    if pedestrian:
        result = results.get('red_result_file', {})
        safety = results.get('pedestrian_safety_file', {})
        return {
            'achieved_max_speed_mps': result.get('max_speed_mps'),
            'minimum_front_gap_m': result.get('minimum_front_gap_m'),
            'stop_line_passed': result.get('passed'),
            'collision': safety.get('collision'),
            'scenario_completed': safety.get('scenario_completed'),
            **lane_values,
        }
    result = results.get('result_file', {})
    return {
        'achieved_max_speed_mps': result.get('max_speed_mps'),
        'goal_reached': result.get('goal_reached'),
        'max_corner_error_m': result.get('max_corner_error_m'),
        'outside_corridor_samples': result.get('outside_corridor_samples'),
        'corridor_passed': result.get('passed'),
        **lane_values,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--case', action='append',
                        help='Only run this named case; may be repeated')
    parser.add_argument('--speed', type=float, action='append',
                        help='Only run this speed in m/s; may be repeated')
    parser.add_argument('--timeout-wall', type=float, default=420.0)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    base_path = ROOT / config['base_validation_config']
    base_cases = yaml.safe_load(base_path.read_text(encoding='utf-8'))['cases']
    validate(config, base_cases)
    if args.timeout_wall <= 0:
        parser.error('--timeout-wall must be positive')
    names = args.case or config['cases']
    speeds = args.speed or config['speeds_mps']
    selected = {**config, 'cases': names, 'speeds_mps': speeds}
    validate(selected, base_cases)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    variants = output / 'route_variants'
    variants.mkdir()
    summary = {
        'schema_version': 1, 'kind': 'connected_speed_validation',
        'started_utc': utc_now(), 'config': selected,
        'config_sha256': sha256(args.config),
        'base_validation_sha256': sha256(base_path),
        'git_commit': git_value('rev-parse', 'HEAD'),
        'working_tree_dirty': bool(git_value('status', '--porcelain')),
        'criterion_note': (
            'Route results use a 1.75 m chassis-corner distance to the '
            'configured path. A separate monitor checks painted centerline '
            'and lane-edge overlap on marked straight arms, excluding '
            'unmarked junctions. Red results use the independent '
            'front-bumper-to-stop-line monitor.'),
        'runs': [],
    }
    rows = []
    for name in names:
        base_case = base_cases[name]
        route_source = ROOT / 'ros2_ws/src/safe_drive_control/config' / (
            base_case['route_config'])
        template = yaml.safe_load(route_source.read_text(encoding='utf-8'))
        for speed in speeds:
            for repeat in range(1, config['repeats'] + 1):
                tag = f'{name}_{speed:g}mps_rep{repeat:02d}'.replace('.', 'p')
                route_path = variants / f'{tag}.yaml'
                route_path.write_text(
                    yaml.safe_dump(variant(template, float(speed)),
                                   sort_keys=False), encoding='utf-8')
                case = {**base_case, 'route_config': str(route_path)}
                print(f'Running {tag}', flush=True)
                record = run_case(tag, case, output, args.timeout_wall)
                observed = measurements(record, bool(base_case['pedestrian']))
                record.update({
                    'requested_speed_mps': float(speed),
                    'route_variant_sha256': sha256(route_path),
                    'measurements': observed,
                })
                record_file = output / tag / 'record.json'
                record_file.write_text(json.dumps(record, indent=2) + '\n',
                                       encoding='utf-8')
                summary['runs'].append({
                    'case': name, 'speed_mps': speed, 'repeat': repeat,
                    'passed': record['passed'],
                    'record_file': str(record_file.relative_to(output)),
                    'measurements': observed,
                })
                rows.append({
                    'case': name, 'requested_speed_mps': speed,
                    'repeat': repeat, 'passed': record['passed'],
                    **observed,
                })
                print(f"{tag}: {'PASS' if record['passed'] else 'FAIL'}",
                      flush=True)
    summary['finished_utc'] = utc_now()
    summary['all_passed'] = all(run['passed'] for run in summary['runs'])
    (output / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (output / 'runs.csv').open('w', newline='',
                                    encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return 0 if summary['all_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
