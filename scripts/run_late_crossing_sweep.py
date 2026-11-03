#!/usr/bin/env python3
"""Repeat the connected late-crossing event across speed and trigger values."""

import argparse
import csv
import json
import math
from pathlib import Path

import yaml

from run_connected_validation import (ROOT, git_value, run_case, sha256,
                                      utc_now)
from run_speed_validation import variant as speed_variant


DEFAULT_CONFIG = ROOT / 'config/late_crossing_sweep.yaml'


def positive_values(values: object, name: str) -> list[float]:
    if (not isinstance(values, list) or not values
            or any(isinstance(value, bool)
                   or not isinstance(value, (int, float))
                   or not math.isfinite(value) or value <= 0
                   for value in values)
            or len(set(values)) != len(values)):
        raise ValueError(f'{name} needs distinct positive finite numbers')
    return [float(value) for value in values]


def crossing_variant(template: dict, distance: float) -> dict:
    result = json.loads(json.dumps(template))
    parameters = result['signal_crossing']['ros__parameters']
    parameters['late_trigger_distance'] = distance
    return result


def measured(record: dict) -> dict:
    results = record['results']
    stop = results.get('red_result_file', {})
    crossing = results.get('pedestrian_result_file', {})
    safety = results.get('pedestrian_safety_file', {})
    lane = results.get('lane_result_file', {})
    return {
        'event_valid': record['passed'],
        'risk_observed': record.get('risk_observed'),
        'collision': safety.get('collision'),
        'minimum_actor_center_distance_m': safety.get('min_distance'),
        'ego_speed_at_entry_mps': crossing.get('start_ego_speed_mps'),
        'ego_x_at_entry_m': crossing.get('start_ego_x_m'),
        'entry_sim_time_s': crossing.get('start_sim_time'),
        'vehicle_signal_at_entry': crossing.get('start_vehicle_signal'),
        'pedestrian_signal_at_entry': crossing.get('start_signal'),
        'minimum_red_stop_gap_m': stop.get('minimum_front_gap_m'),
        'centerline_overlap_samples': lane.get('centerline_overlap_samples'),
        'edge_line_overlap_samples': lane.get('edge_line_overlap_samples'),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--speed', type=float, action='append')
    parser.add_argument('--distance', type=float, action='append')
    parser.add_argument('--gui', action='store_true',
                        help='Show Gazebo for each selected condition')
    parser.add_argument('--timeout-wall', type=float, default=180.0)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    speeds = positive_values(args.speed or config['speeds_mps'], 'speeds_mps')
    distances = positive_values(
        args.distance or config['trigger_distances_m'], 'trigger_distances_m')
    repeats = config['repeats']
    if (isinstance(repeats, bool) or not isinstance(repeats, int)
            or repeats < 1):
        parser.error('repeats must be a positive integer')
    if args.timeout_wall <= 0:
        parser.error('--timeout-wall must be positive')

    base_path = ROOT / config['base_validation_config']
    base_case = yaml.safe_load(base_path.read_text(encoding='utf-8'))[
        'cases'][config['case']]
    if not base_case.get('allow_collision'):
        parser.error('selected case must measure risk outcomes')
    route_source = ROOT / 'ros2_ws/src/safe_drive_control/config' / (
        base_case['route_config'])
    crossing_source = ROOT / 'ros2_ws/src/safe_drive_scenario/config' / (
        base_case['crossing_config'])
    route_template = yaml.safe_load(route_source.read_text(encoding='utf-8'))
    crossing_template = yaml.safe_load(
        crossing_source.read_text(encoding='utf-8'))

    output = args.output_dir.resolve()
    if output.exists():
        parser.error(f'output directory exists: {output}; use a new path')
    output.mkdir(parents=True, exist_ok=False)
    variants = output / 'variants'
    variants.mkdir()
    summary = {
        'schema_version': 1,
        'kind': 'continuous_late_crossing_sweep',
        'started_utc': utc_now(),
        'config': {**config, 'speeds_mps': speeds,
                   'trigger_distances_m': distances},
        'config_sha256': sha256(args.config),
        'base_validation_sha256': sha256(base_path),
        'git_commit': git_value('rev-parse', 'HEAD'),
        'working_tree_dirty': bool(git_value('status', '--porcelain')),
        'gui': args.gui,
        'criterion_note': (
            'Valid means red stop, moving green/DONT_WALK entry, completed '
            'crossing, and finite distance were measured. Risk means contact '
            'or actor-center distance below the configured 1.5 m threshold. '
            'Route completion and painted-line compliance are separate.'),
        'runs': [],
    }
    rows = []
    interrupted = False
    for speed in speeds:
        if interrupted:
            break
        for distance in distances:
            if interrupted:
                break
            for repeat in range(1, repeats + 1):
                tag = (f'v{speed:g}_d{distance:g}_r{repeat:02d}'
                       .replace('.', 'p'))
                route_path = variants / f'{tag}_route.yaml'
                crossing_path = variants / f'{tag}_crossing.yaml'
                route_path.write_text(yaml.safe_dump(
                    speed_variant(route_template, speed), sort_keys=False),
                    encoding='utf-8')
                crossing_path.write_text(yaml.safe_dump(
                    crossing_variant(crossing_template, distance),
                    sort_keys=False), encoding='utf-8')
                case = {**base_case,
                        'route_config': str(route_path),
                        'crossing_config': str(crossing_path),
                        'late_trigger_distance_m': distance}
                print(f'Running {tag}', flush=True)
                record = run_case(tag, case, output, args.timeout_wall,
                                  gui=args.gui)
                values = measured(record)
                record['requested_speed_mps'] = speed
                record['trigger_distance_m'] = distance
                record['measurements'] = values
                (output / tag / 'record.json').write_text(
                    json.dumps(record, indent=2) + '\n', encoding='utf-8')
                row = {'case': config['case'], 'requested_speed_mps': speed,
                       'trigger_distance_m': distance, 'repeat': repeat,
                       **values}
                rows.append(row)
                summary['runs'].append({
                    **row, 'record_file': f'{tag}/record.json',
                })
                status = ('INTERRUPTED' if record.get('interrupted') else
                          'VALID' if record['passed'] else 'INVALID')
                detail = (f" ({record['error']})"
                          if record.get('error') else '')
                print(f'{tag}: {status}{detail}', flush=True)
                if record.get('interrupted'):
                    interrupted = True
                    break
    summary['finished_utc'] = utc_now()
    summary['interrupted'] = interrupted
    summary['all_valid'] = (not interrupted
                            and all(row['event_valid'] for row in rows))
    (output / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    with (output / 'runs.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]),
                                lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    return 130 if interrupted else (0 if summary['all_valid'] else 1)


if __name__ == '__main__':
    raise SystemExit(main())
