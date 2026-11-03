#!/usr/bin/env python3
"""Run configured connected-map checks and retain actual Gazebo results."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'config/connected_validation.yaml'


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def git_value(*args: str) -> str:
    result = subprocess.run(['git', *args], cwd=ROOT, text=True,
                            capture_output=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else 'unknown'


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def input_path_label(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def observed_pass(case: dict, results: dict) -> bool:
    lane_passed = (case.get('allow_collision') is True
                   or 'lane_result_file' not in case['expected']
                   or results.get('lane_result_file', {}).get(
                       'passed') is True)
    if case['pedestrian']:
        red = results.get('red_result_file', {})
        crossing = results.get('pedestrian_result_file', {})
        safety = results.get('pedestrian_safety_file', {})
        route_passed = ('result_file' not in case['expected']
                        or results.get('result_file', {}).get(
                            'passed') is True)
        chronological = ('result_file' not in case['expected']
                         or red.get('end_sim_time', float('inf'))
                         < results.get('result_file', {}).get(
                             'end_sim_time', float('-inf')))
        return (lane_passed and route_passed and chronological
                and red.get('passed') is True
                and crossing.get('completed') is True
                and safety.get('scenario_completed') is True
                and (case.get('allow_collision') is True
                     or safety.get('collision') is False)
                and isinstance(safety.get('min_distance'), (int, float))
                and math.isfinite(safety['min_distance']))
    return (lane_passed
            and results.get('result_file', {}).get('passed') is True)


def run_case(name: str, case: dict, output_dir: Path,
             timeout_seconds: float) -> dict:
    case_dir = output_dir / name
    case_dir.mkdir(parents=True, exist_ok=False)
    expected = {argument: case_dir / filename
                for argument, filename in case['expected'].items()}
    completion_files = {argument: path for argument, path in expected.items()
                        if argument != 'lane_result_file'}
    command = [
        'ros2', 'launch', 'safe_drive_bringup',
        'signal_intersection.launch.py', 'gui:=false',
        'world_file:=connected_road.world',
        f"spawn_x:={case.get('spawn_x', -30)}",
        f"route_config:={case['route_config']}",
        *([f"fixed_state:={case['fixed_state']}"]
          if case['fixed_state'] else []),
        'monitor_red_stop:=' + str(case.get(
            'monitor_red_stop', False)).lower(),
        f"pedestrian:={str(case['pedestrian']).lower()}",
        'crossing_config:=' + case.get(
            'crossing_config', 'signal_crossing.yaml'),
        *[f'{argument}:={path}' for argument, path in expected.items()],
    ]
    start_wall = time.monotonic()
    record = {
        'case': name, 'configuration': case, 'seed': None,
        'started_utc': utc_now(), 'command': command,
        'expected_files': {
            key: str(path) for key, path in expected.items()
        },
        'input_sha256': {
            input_path_label(path): sha256(path)
            for path in (
                ROOT / 'ros2_ws/src/safe_drive_sim/worlds'
                / 'connected_road.world',
                ROOT / 'ros2_ws/src/safe_drive_control/config'
                / case['route_config'],
                ROOT / 'ros2_ws/src/safe_drive_control/config'
                / 'signal_controller.yaml',
                ROOT / 'ros2_ws/src/safe_drive_scenario/config'
                / 'traffic_signal.yaml',
                ROOT / 'ros2_ws/src/safe_drive_scenario/config'
                / 'signal_crossing.yaml',
            )
        },
    }
    log_path = case_dir / 'launch.log'
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, cwd=ROOT / 'ros2_ws',
                                   stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            while True:
                if all(path.is_file() and path.stat().st_size > 0
                       for path in completion_files.values()):
                    break
                if process.poll() is not None:
                    record['error'] = (
                        f'launch exited before results: {process.returncode}'
                    )
                    break
                if 'gzserver-1]: process has died' in log_path.read_text(
                        encoding='utf-8', errors='replace'):
                    record['error'] = 'Gazebo server exited before results'
                    break
                if time.monotonic() - start_wall > timeout_seconds:
                    record['error'] = (
                        f'wall timeout after {timeout_seconds:g} s'
                    )
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            record['error'] = 'interrupted by user'
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=8)
    results = {}
    for argument, path in expected.items():
        if path.is_file():
            try:
                results[argument] = json.loads(
                    path.read_text(encoding='utf-8'))
            except json.JSONDecodeError as error:
                record['error'] = f'invalid {path.name}: {error}'
        else:
            record['error'] = f'missing {path.name}'
    record['results'] = results
    if 'risk_distance_m' in case:
        safety = results.get('pedestrian_safety_file', {})
        distance = safety.get('min_distance')
        if isinstance(distance, (int, float)) and math.isfinite(distance):
            record['risk_observed'] = (
                safety.get('collision') is True
                or distance < float(case['risk_distance_m']))
        else:
            record['risk_observed'] = None
    record['passed'] = not record.get('error') and observed_pass(case, results)
    record['finished_utc'] = utc_now()
    record['elapsed_wall_seconds'] = round(time.monotonic() - start_wall, 3)
    (case_dir / 'record.json').write_text(json.dumps(record, indent=2) + '\n',
                                          encoding='utf-8')
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--case', action='append',
                        help='Run only this named case; repeatable')
    parser.add_argument('--timeout-wall', type=float, default=420.0)
    args = parser.parse_args()
    if args.timeout_wall <= 0:
        parser.error('--timeout-wall must be positive')
    cases = yaml.safe_load(args.config.read_text(encoding='utf-8'))['cases']
    names = args.case or list(cases)
    for name in names:
        if name not in cases:
            parser.error(f'unknown case: {name}')
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    summary = {
        'schema_version': 1, 'kind': 'connected_scene_validation',
        'started_utc': utc_now(), 'config_file': str(args.config.resolve()),
        'config_sha256': sha256(args.config),
        'git_commit': git_value('rev-parse', 'HEAD'),
        'working_tree_dirty': bool(git_value('status', '--porcelain')),
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'cases': [],
    }
    for name in names:
        print(f'Running {name}', flush=True)
        record = run_case(name, cases[name], output_dir, args.timeout_wall)
        summary['cases'].append({
            'case': name, 'passed': record['passed'],
            'record_file': str(output_dir / name / 'record.json'),
            'error': record.get('error'),
        })
        print(f"{name}: {'PASS' if record['passed'] else 'FAIL'}", flush=True)
    summary['finished_utc'] = utc_now()
    summary['passed'] = all(case['passed'] for case in summary['cases'])
    summary_path = output_dir / 'summary.json'
    summary_path.write_text(json.dumps(summary, indent=2) + '\n',
                            encoding='utf-8')
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
