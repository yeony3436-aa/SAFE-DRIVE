"""Tests for deterministic signal phases and the stop-line speed profile."""

from safe_drive_scenario.traffic_signal import SignalTiming


def test_signal_transitions_and_reset():
    timing = SignalTiming(red_seconds=12.0, green_seconds=16.0,
                          yellow_seconds=3.0, pedestrian_walk_seconds=7.0)
    assert [timing.state_at(t) for t in (0, 11.9, 12, 27.9, 28, 31)] == [
        'RED', 'RED', 'GREEN', 'GREEN', 'YELLOW', 'RED'
    ]
    assert [timing.pedestrian_state_at(t) for t in (0, 6.9, 7, 12, 28, 31)] == [
        'WALK', 'WALK', 'DONT_WALK', 'DONT_WALK', 'DONT_WALK', 'WALK'
    ]
