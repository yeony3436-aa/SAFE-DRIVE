"""Simulator-independent sudden-pedestrian scenario state machine."""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import yaml


class ScenarioState(str, Enum):
    """Lifecycle states published by the scenario manager."""

    WAITING_FOR_EGO = 'WAITING_FOR_EGO'
    ARMED = 'ARMED'
    ACTIVE = 'ACTIVE'
    COMPLETED = 'COMPLETED'
    TIMED_OUT = 'TIMED_OUT'


@dataclass(frozen=True)
class SuddenPedestrianConfig:
    """Validated configuration for one sudden-pedestrian execution."""

    scenario_id: str
    seed: int
    ego_speed: float
    pedestrian_speed: float
    pedestrian_spawn_distance: float
    trigger_distance: float
    timeout: float
    pedestrian_start_y: float
    pedestrian_end_y: float
    update_rate_hz: float


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be a mapping')
    return value


def load_sudden_pedestrian_config(path: str | Path) -> SuddenPedestrianConfig:
    """Load and validate a structured sudden-pedestrian YAML definition."""
    with Path(path).open(encoding='utf-8') as config_file:
        root = _mapping(yaml.safe_load(config_file), 'root')

    scenario = _mapping(root.get('scenario'), 'scenario')
    parameters = _mapping(scenario.get('parameters'), 'scenario.parameters')
    termination = _mapping(scenario.get('termination'), 'scenario.termination')
    environment = _mapping(scenario.get('environment'), 'scenario.environment')

    if scenario.get('type') != 'sudden_pedestrian':
        raise ValueError('scenario.type must be sudden_pedestrian')

    config = SuddenPedestrianConfig(
        scenario_id=str(scenario['id']),
        seed=int(scenario['seed']),
        ego_speed=float(parameters['ego_speed']),
        pedestrian_speed=float(parameters['pedestrian_speed']),
        pedestrian_spawn_distance=float(parameters['pedestrian_spawn_distance']),
        trigger_distance=float(parameters['trigger_distance']),
        timeout=float(termination['timeout']),
        pedestrian_start_y=float(environment['pedestrian_start_y']),
        pedestrian_end_y=float(environment['pedestrian_end_y']),
        update_rate_hz=float(environment['update_rate_hz']),
    )
    _validate_config(config)
    return config


def _validate_config(config: SuddenPedestrianConfig) -> None:
    if not config.scenario_id:
        raise ValueError('scenario.id must not be empty')
    if config.ego_speed <= 0.0:
        raise ValueError('ego_speed must be positive')
    if config.pedestrian_speed <= 0.0:
        raise ValueError('pedestrian_speed must be positive')
    if config.pedestrian_spawn_distance <= 0.0:
        raise ValueError('pedestrian_spawn_distance must be positive')
    if config.trigger_distance <= 0.0:
        raise ValueError('trigger_distance must be positive')
    if config.timeout <= 0.0:
        raise ValueError('timeout must be positive')
    if config.pedestrian_end_y <= config.pedestrian_start_y:
        raise ValueError('pedestrian_end_y must be greater than pedestrian_start_y')
    if config.update_rate_hz <= 0.0:
        raise ValueError('update_rate_hz must be positive')


class SuddenPedestrianScenario:
    """Deterministic trigger and pedestrian-motion calculation."""

    def __init__(self, config: SuddenPedestrianConfig) -> None:
        self.config = config
        self.state = ScenarioState.WAITING_FOR_EGO
        self.crossing_x: float | None = None
        self.pedestrian_y = config.pedestrian_start_y
        self.elapsed = 0.0

    def arm(self, initial_ego_x: float) -> None:
        """Place the crossing point ahead of the first observed ego pose."""
        if self.state != ScenarioState.WAITING_FOR_EGO:
            raise RuntimeError('scenario can only be armed once')
        self.crossing_x = initial_ego_x + self.config.pedestrian_spawn_distance
        self.state = ScenarioState.ARMED

    def step(self, ego_x: float, elapsed_seconds: float) -> ScenarioState:
        """Advance the scenario using elapsed simulation time."""
        if self.state == ScenarioState.WAITING_FOR_EGO:
            raise RuntimeError('scenario must be armed before stepping')
        if self.state in (ScenarioState.COMPLETED, ScenarioState.TIMED_OUT):
            return self.state
        if elapsed_seconds < 0.0:
            raise ValueError('elapsed_seconds must not be negative')

        self.elapsed += elapsed_seconds
        if self.elapsed >= self.config.timeout:
            self.state = ScenarioState.TIMED_OUT
            return self.state

        if self.state == ScenarioState.ARMED:
            assert self.crossing_x is not None
            distance_to_crossing = self.crossing_x - ego_x
            if distance_to_crossing <= self.config.trigger_distance:
                self.state = ScenarioState.ACTIVE

        if self.state == ScenarioState.ACTIVE:
            self.pedestrian_y = min(
                self.config.pedestrian_end_y,
                self.pedestrian_y + self.config.pedestrian_speed * elapsed_seconds,
            )
            if self.pedestrian_y >= self.config.pedestrian_end_y:
                self.state = ScenarioState.COMPLETED

        return self.state
