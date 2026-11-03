#!/usr/bin/env python3
"""Compare random and risk-guided grid selection with actual Gazebo runs."""

import argparse
import csv
import heapq
import json
import math
import os
from pathlib import Path
import random
import subprocess

import yaml

from run_pedestrian_sweep import ROOT, git_value, run_one, sha256, utc_now


DEFAULT_CONFIG = ROOT / 'config/risk_exploration.yaml'
DIMENSIONS = ('ego_speed_mps', 'trigger_distance_m', 'pedestrian_speed_mps')


def validate(config: dict) -> None:
    seeds = config.get('seeds')
    if (not isinstance(seeds, list) or not seeds
            or any(isinstance(seed, bool) or not isinstance(seed, int)
                   for seed in seeds)
            or len(set(seeds)) != len(seeds)):
        raise ValueError('seeds must be distinct integers')
    for key in ('budget_per_method', 'adaptive_max_depth'):
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f'{key} must be a positive integer')
    for key in DIMENSIONS:
        bounds = config['parameters'][key]
        if (not isinstance(bounds, list) or len(bounds) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in bounds)
                or not 0 < bounds[0] < bounds[1]):
            raise ValueError(f'{key} needs positive, increasing finite bounds')
    risk = config['risk']
    threshold = risk['minimum_actor_center_distance_below_m']
    if (not isinstance(risk['collision_is_risk'], bool)
            or not isinstance(threshold, (int, float))
            or not math.isfinite(threshold) or threshold <= 0):
        raise ValueError('invalid risk criterion')
    distance = config['fixed_pedestrian_spawn_distance_m']
    if (not isinstance(distance, (int, float)) or not math.isfinite(distance)
            or distance <= 0):
        raise ValueError('fixed crossing distance must be positive')


def center(bounds: dict) -> dict:
    return {key: (bounds[key][0] + bounds[key][1]) / 2 for key in DIMENSIONS}


def sample_random(bounds: dict, rng: random.Random) -> dict:
    return {key: rng.uniform(*bounds[key]) for key in DIMENSIONS}


def split(bounds: dict) -> list[dict]:
    """Return eight octants in stable order."""
    children = [{}]
    for key in DIMENSIONS:
        low, high = bounds[key]
        mid = (low + high) / 2
        children = [{**child, key: interval} for child in children
                    for interval in ((low, mid), (mid, high))]
    return children


def observed_priority(record: dict, risk: dict) -> float:
    if not record['measurement_valid']:
        return -1.0
    safety = record['safety']
    distance = safety['min_distance']
    threshold = risk['minimum_actor_center_distance_below_m']
    clearance_score = max(0.0, threshold - distance) / threshold
    return (2.0 if safety['collision'] else 0.0) + clearance_score


class AdaptiveGrid:
    """Refine evaluated cells; prioritize children of risky measurements."""

    def __init__(self, bounds: dict, max_depth: int):
        self.bounds = bounds
        self.max_depth = max_depth
        self.pending = []
        self.serial = 0
        self.last_cell = (bounds, 0)

    def next(self) -> dict:
        if not self.pending:
            return center(self.bounds)
        _, _, bounds, depth = heapq.heappop(self.pending)
        self.last_cell = (bounds, depth)
        return center(bounds)

    def observe(self, record: dict, risk: dict) -> None:
        bounds, depth = self.last_cell
        if depth >= self.max_depth:
            return
        priority = observed_priority(record, risk)
        for child in split(bounds):
            self.serial += 1
            heapq.heappush(self.pending,
                           (-priority, self.serial, child, depth + 1))


def summarize(records: list[dict]) -> dict:
    valid = [record for record in records if record['measurement_valid']]
    risky = [record for record in valid
             if record['risk_classification'] == 'risky']
    first = next((index for index, record in enumerate(records, 1)
                  if record['risk_classification'] == 'risky'), None)
    unique = {
        tuple(record['sampled_parameters'][key] for key in DIMENSIONS)
        for record in risky
    }
    return {'budget': len(records), 'valid_runs': len(valid),
            'risky_runs': len(risky), 'first_risky_run': first,
            'unique_risky_conditions': len(unique)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout-wall', type=float, default=120.0)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    validate(config)
    if args.timeout_wall <= 0:
        parser.error('--timeout-wall must be positive')
    template_path = ROOT / config['scenario_template']
    template = yaml.safe_load(template_path.read_text(encoding='utf-8'))
    gazebo_version_result = subprocess.run(
        ['dpkg-query', '-W', '-f=${Version}', 'gazebo'],
        capture_output=True, text=True, check=False)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    summary = {
        'schema_version': 1, 'kind': 'measured_risk_exploration',
        'started_utc': utc_now(), 'config': config,
        'config_sha256': sha256(args.config),
        'scenario_template_sha256': sha256(template_path),
        'world_sha256': sha256(
            ROOT / 'ros2_ws/src/safe_drive_sim/worlds/minimal_road.world'),
        'vehicle_xacro_sha256': sha256(
            ROOT / 'ros2_ws/src/safe_drive_sim/urdf/'
            'ego_vehicle.gazebo.urdf.xacro'),
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'gazebo_package_version': (
            gazebo_version_result.stdout.strip()
            if gazebo_version_result.returncode == 0 else 'unknown'),
        'git_commit': git_value('rev-parse', 'HEAD'),
        'working_tree_dirty': bool(git_value('status', '--porcelain')),
        'selection_note': (
            'Both methods evaluate the domain center first. Random then '
            'uses uniform continuous samples; adaptive splits measured cells '
            'into octants and prioritizes children of smaller clearance or '
            'collision.'),
        'seed_note': (
            'Seeds control selection only; the current pedestrian scenario '
            'has no random draw.'),
        'groups': [],
    }
    rows = []
    for seed in config['seeds']:
        for method in ('random', 'adaptive'):
            group_dir = output / f'seed_{seed}' / method
            group_dir.mkdir(parents=True)
            rng = random.Random(seed)
            grid = AdaptiveGrid(config['parameters'],
                                config['adaptive_max_depth'])
            records = []
            for index in range(1, config['budget_per_method'] + 1):
                if method == 'adaptive':
                    params = grid.next()
                else:
                    params = (center(config['parameters']) if index == 1
                              else sample_random(config['parameters'], rng))
                case = {'seed': seed, **params}
                name = f'{method}_{index:03d}'
                print(f'seed={seed} {name} {params}', flush=True)
                record = run_one(name, 1, case, config, template, group_dir,
                                 args.timeout_wall)
                record['method'] = method
                record['selection_seed'] = seed
                record['sampled_parameters'] = params
                record['selection_index'] = index
                record['adaptive_cell_depth'] = (
                    grid.last_cell[1] if method == 'adaptive' else None)
                record_path = group_dir / record['run_id'] / 'record.json'
                record_path.write_text(json.dumps(record, indent=2) + '\n',
                                       encoding='utf-8')
                if method == 'adaptive':
                    grid.observe(record, config['risk'])
                records.append(record)
                rows.append({'seed': seed, 'method': method, 'run': index,
                             **params,
                             'classification': record['risk_classification'],
                             'collision': record.get('safety', {}).get(
                                 'collision'),
                             'min_center_distance_m': record.get(
                                 'safety', {}).get('min_distance'),
                             'min_los_ttc_s': record.get('safety', {}).get(
                                 'min_ttc'),
                             'median_real_time_factor': record.get(
                                 'median_real_time_factor'),
                             'valid': record['measurement_valid'],
                             'record_file': str(
                                 record_path.relative_to(output))})
            summary['groups'].append({'seed': seed, 'method': method,
                                      **summarize(records),
                                      'records': [
                                          str((group_dir / record['run_id'] /
                                               'record.json').relative_to(
                                                   output))
                                          for record in records]})
    summary['finished_utc'] = utc_now()
    summary['all_measurements_valid'] = all(row['valid'] for row in rows)
    summary['all_performance_recorded'] = all(
        row['median_real_time_factor'] is not None for row in rows)
    (output / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    with (output / 'runs.csv').open('w', newline='',
                                    encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return 0 if (summary['all_measurements_valid']
                 and summary['all_performance_recorded']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
