"""The pedestrian must wait for WALK and clear after permission changes."""

from safe_drive_scenario.signal_crossing import SignalCrossing
from safe_drive_scenario.sudden_pedestrian import ScenarioState


def test_starts_only_on_walk_and_finishes_during_clearance():
    crossing = SignalCrossing(-4.25, 4.25, 1.5)
    assert crossing.step(None, 2.0) == ScenarioState.ARMED
    assert crossing.step('DONT_WALK', 2.0) == ScenarioState.ARMED
    assert crossing.y == -4.25
    assert crossing.step('WALK', 1.0) == ScenarioState.ACTIVE
    assert crossing.y == -2.75
    assert crossing.step('DONT_WALK', 5.0) == ScenarioState.COMPLETED
    assert crossing.y == 4.25
