"""Fixed-red result must require an approach and detect line crossing."""

from safe_drive_safety.red_stop import RedStopObservation


def test_requires_approach_and_stable_stop():
    monitor = RedStopObservation(-9.25, 2.1, 0.6)
    assert monitor.observe(0.0, -20.0, 0.0, 'RED') is None
    assert monitor.observe(10.0, -20.0, 0.0, 'RED') is None
    assert monitor.observe(11.0, -12.0, 0.2, 'RED') is None
    assert monitor.observe(12.0, -11.95, 0.01, 'RED') is None
    assert monitor.observe(16.9, -11.95, 0.01, 'RED') is None
    result = monitor.observe(17.0, -11.95, 0.01, 'RED')
    assert result['passed']
    assert result['minimum_front_gap_m'] >= 0.6 - 1e-6


def test_crossing_on_red_fails_immediately():
    monitor = RedStopObservation(-9.25, 2.1, 0.6)
    assert monitor.observe(0.0, -12.0, 0.4, 'GREEN') is None
    assert monitor.observe(1.0, -12.0, 0.4, 'RED') is None
    result = monitor.observe(2.0, -11.0, 0.4, 'RED')
    assert result['reason'] == 'stop_line_crossed'
    assert not result['passed']
