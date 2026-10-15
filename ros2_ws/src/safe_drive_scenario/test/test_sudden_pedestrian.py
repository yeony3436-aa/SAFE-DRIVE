from pathlib import Path

import pytest

from safe_drive_scenario.sudden_pedestrian import (
    load_sudden_pedestrian_config,
    ScenarioState,
    SuddenPedestrianConfig,
    SuddenPedestrianScenario,
)


def make_config(**overrides):
    values = {
        'scenario_id': 'SCENARIO-TEST',
        'seed': 42,
        'ego_speed': 2.0,
        'pedestrian_speed': 1.0,
        'pedestrian_spawn_distance': 10.0,
        'trigger_distance': 3.0,
        'timeout': 30.0,
        'pedestrian_start_y': -4.0,
        'pedestrian_end_y': 4.0,
        'update_rate_hz': 20.0,
    }
    values.update(overrides)
    return SuddenPedestrianConfig(**values)


def test_scenario_arms_relative_to_initial_ego_pose():
    scenario = SuddenPedestrianScenario(make_config())

    scenario.arm(initial_ego_x=-15.0)

    assert scenario.state == ScenarioState.ARMED
    assert scenario.crossing_x == pytest.approx(-5.0)
    assert scenario.pedestrian_y == pytest.approx(-4.0)


def test_pedestrian_stays_armed_before_trigger_distance():
    scenario = SuddenPedestrianScenario(make_config())
    scenario.arm(initial_ego_x=-15.0)

    scenario.step(ego_x=-9.0, elapsed_seconds=0.5)

    assert scenario.state == ScenarioState.ARMED
    assert scenario.pedestrian_y == pytest.approx(-4.0)


def test_pedestrian_moves_after_trigger():
    scenario = SuddenPedestrianScenario(make_config())
    scenario.arm(initial_ego_x=-15.0)

    scenario.step(ego_x=-8.0, elapsed_seconds=0.5)

    assert scenario.state == ScenarioState.ACTIVE
    assert scenario.pedestrian_y == pytest.approx(-3.5)


def test_scenario_completes_after_pedestrian_crosses_road():
    scenario = SuddenPedestrianScenario(
        make_config(pedestrian_start_y=-1.0, pedestrian_end_y=1.0)
    )
    scenario.arm(initial_ego_x=-15.0)

    scenario.step(ego_x=-8.0, elapsed_seconds=2.0)

    assert scenario.state == ScenarioState.COMPLETED
    assert scenario.pedestrian_y == pytest.approx(1.0)


def test_scenario_times_out_without_trigger():
    scenario = SuddenPedestrianScenario(make_config(timeout=2.0))
    scenario.arm(initial_ego_x=-15.0)

    scenario.step(ego_x=-15.0, elapsed_seconds=2.0)

    assert scenario.state == ScenarioState.TIMED_OUT


def test_repository_config_loads():
    config_path = Path(__file__).parents[1] / 'config' / 'sudden_pedestrian.yaml'

    config = load_sudden_pedestrian_config(config_path)

    assert config.scenario_id == 'SCENARIO-0001'
    assert config.ego_speed == pytest.approx(2.0)
    assert config.pedestrian_end_y > config.pedestrian_start_y
