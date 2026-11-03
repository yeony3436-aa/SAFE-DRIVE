#!/usr/bin/env python3
"""Run fixed sudden-pedestrian conditions and save measured outcomes."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'config/pedestrian_sweep.yaml'


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_value(*args: str) -> str:
    result = subprocess.run(['git', *args], cwd=ROOT, text=True,
                            capture_output=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else 'unknown'


def classify(safety: dict, risk: dict) -> tuple[bool, str]:
    """Classify only a completed run with an observed center distance."""
    distance = safety.get('min_distance')
    if (safety.get('scenario_completed') is not True
            or not isinstance(distance, (int, float))
            or not math.isfinite(distance)
            or not isinstance(safety.get('collision'), bool)):
        return False, 'unknown'
    risky = ((risk['collision_is_risk'] and safety['collision'])
             or distance < risk['minimum_actor_center_distance_below_m'])
    return True, 'risky' if risky else 'not_risky'


def scenario_for_case(template: dict, name: str, case: dict,
                      fixed_distance: float) -> dict:
    scenario = json.loads(json.dumps(template))
    definition = scenario['scenario']
    definition['id'] = f"{definition['id']}-{name}"
    definition['seed'] = int(case['seed'])
    parameters = definition['parameters']
    parameters['ego_speed'] = float(case['ego_speed_mps'])
    parameters['trigger_distance'] = float(case['trigger_distance_m'])
    parameters['pedestrian_speed'] = float(case['pedestrian_speed_mps'])
    parameters['pedestrian_spawn_distance'] = fixed_distance
    return scenario


def run_one(name: str, repeat: int, case: dict, sweep: dict,
            template: dict, output_dir: Path, timeout_wall: float) -> dict:
    import rclpy
    from gazebo_msgs.msg import PerformanceMetrics

    run_id = f'{name}_rep{repeat:02d}'
    run_dir = output_dir / run_id
    run_dir.mkdir()
    definition = scenario_for_case(
        template, name, case, float(sweep['fixed_pedestrian_spawn_distance_m']))
    scenario_file = run_dir / 'scenario.yaml'
    scenario_file.write_text(yaml.safe_dump(definition, sort_keys=False),
                             encoding='utf-8')
    safety_file = run_dir / 'safety.json'
    command = [
        'ros2', 'launch', 'safe_drive_bringup', 'scenario.launch.py',
        'gui:=false', f'scenario_config:={scenario_file}',
        f'safety_result_file:={safety_file}',
    ]
    record = {
        'run_id': run_id, 'case': name, 'repeat': repeat,
        'seed': int(case['seed']), 'parameters': definition['scenario']['parameters'],
        'risk_criterion': sweep['risk'], 'started_utc': utc_now(),
        'command': command, 'scenario_sha256': sha256(scenario_file),
    }
    start = time.monotonic()
    real_time_factors = []
    rclpy.init()
    performance_node = rclpy.create_node('safe_drive_performance_recorder')

    def on_performance(message: PerformanceMetrics) -> None:
        if math.isfinite(message.real_time_factor):
            real_time_factors.append(message.real_time_factor)

    performance_node.create_subscription(
        PerformanceMetrics, '/performance_metrics', on_performance, 10)
    environment = os.environ.copy()
    environment['ROS_LOG_DIR'] = str(run_dir / 'ros_logs')
    environment['GAZEBO_LOG_PATH'] = str(run_dir / 'gazebo_logs')
    with (run_dir / 'launch.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, cwd=ROOT / 'ros2_ws',
                                   stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=environment)
        try:
            while not safety_file.is_file() or safety_file.stat().st_size == 0:
                if process.poll() is not None:
                    record['error'] = f'launch exited before result: {process.returncode}'
                    break
                if time.monotonic() - start > timeout_wall:
                    record['error'] = f'wall timeout after {timeout_wall:g} s'
                    break
                rclpy.spin_once(performance_node, timeout_sec=0.5)
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
            performance_node.destroy_node()
            rclpy.shutdown()
    record['real_time_factor_samples'] = real_time_factors
    record['median_real_time_factor'] = (
        statistics.median(real_time_factors) if real_time_factors else None)
    if safety_file.is_file():
        try:
            record['safety'] = json.loads(safety_file.read_text(encoding='utf-8'))
        except json.JSONDecodeError as error:
            record['error'] = f'invalid safety JSON: {error}'
    valid, classification = classify(record.get('safety', {}), sweep['risk'])
    record['measurement_valid'] = valid and not record.get('error')
    record['risk_classification'] = classification if record['measurement_valid'] else 'unknown'
    record['elapsed_wall_seconds'] = round(time.monotonic() - start, 3)
    record['finished_utc'] = utc_now()
    record_path = run_dir / 'record.json'
    record_path.write_text(json.dumps(record, indent=2) + '\n',
                           encoding='utf-8')
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--case', action='append', help='Run only this case')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--timeout-wall', type=float, default=120.0)
    args = parser.parse_args()
    if args.repeats < 1 or args.timeout_wall <= 0:
        parser.error('--repeats and --timeout-wall must be positive')
    sweep = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    cases = sweep['cases']
    names = args.case or list(cases)
    if len(names) != len(set(names)) or any(name not in cases for name in names):
        parser.error('case names must be unique and present in the config')
    threshold = float(sweep['risk']['minimum_actor_center_distance_below_m'])
    if threshold <= 0 or not math.isfinite(threshold):
        parser.error('risk distance threshold must be positive and finite')
    template_path = ROOT / sweep['scenario_template']
    template = yaml.safe_load(template_path.read_text(encoding='utf-8'))
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    summary = {
        'schema_version': 1, 'kind': 'fixed_pedestrian_parameter_sweep',
        'started_utc': utc_now(), 'config_file': str(args.config.resolve()),
        'config_sha256': sha256(args.config),
        'scenario_template_sha256': sha256(template_path),
        'git_commit': git_value('rev-parse', 'HEAD'),
        'working_tree_dirty': bool(git_value('status', '--porcelain')),
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'seed_note': 'Seed is recorded; the current scenario has no random draw.',
        'risk_criterion': sweep['risk'], 'runs': [],
    }
    for name in names:
        for repeat in range(1, args.repeats + 1):
            print(f'Running {name} repeat {repeat}', flush=True)
            record = run_one(name, repeat, cases[name], sweep, template,
                             output_dir, args.timeout_wall)
            summary['runs'].append(record)
            print(f"{record['run_id']}: {record['risk_classification']}", flush=True)
    summary['finished_utc'] = utc_now()
    summary['all_measurements_valid'] = all(
        run['measurement_valid'] for run in summary['runs'])
    summary_path = output_dir / 'summary.json'
    summary_path.write_text(json.dumps(summary, indent=2) + '\n',
                            encoding='utf-8')
    fields = ['run_id', 'case', 'repeat', 'seed', 'ego_speed_mps',
              'trigger_distance_m', 'pedestrian_speed_mps', 'scenario_completed',
              'collision', 'min_distance_m', 'min_ttc_s',
              'measurement_valid', 'risk_classification', 'elapsed_wall_seconds']
    with (output_dir / 'runs.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for run in summary['runs']:
            safety = run.get('safety', {})
            writer.writerow({
                'run_id': run['run_id'], 'case': run['case'],
                'repeat': run['repeat'], 'seed': run['seed'],
                'ego_speed_mps': run['parameters']['ego_speed'],
                'trigger_distance_m': run['parameters']['trigger_distance'],
                'pedestrian_speed_mps': run['parameters']['pedestrian_speed'],
                'scenario_completed': safety.get('scenario_completed'),
                'collision': safety.get('collision'),
                'min_distance_m': safety.get('min_distance'),
                'min_ttc_s': safety.get('min_ttc'),
                'measurement_valid': run['measurement_valid'],
                'risk_classification': run['risk_classification'],
                'elapsed_wall_seconds': run['elapsed_wall_seconds'],
            })
    return 0 if summary['all_measurements_valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
