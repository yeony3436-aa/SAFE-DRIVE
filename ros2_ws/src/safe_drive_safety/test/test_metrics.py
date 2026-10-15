"""Unit checks for ground-truth safety calculations."""

import pytest

from safe_drive_safety.metrics import ActorMotion, distance_and_ttc, SafetyMetrics


def test_closing_ttc_and_minima():
    metrics = SafetyMetrics()
    first = metrics.observe(
        ActorMotion(0.0, 0.0, 2.0, 0.0),
        ActorMotion(10.0, 0.0, 0.0, 0.0),
    )
    second = metrics.observe(
        ActorMotion(4.0, 0.0, 2.0, 0.0),
        ActorMotion(10.0, 0.0, 0.0, 0.0),
    )
    assert first == pytest.approx((10.0, 5.0))
    assert second == pytest.approx((6.0, 3.0))
    assert metrics.min_distance == pytest.approx(6.0)
    assert metrics.min_ttc == pytest.approx(3.0)


def test_non_closing_and_coincident_are_unavailable():
    stationary = ActorMotion(0.0, 0.0, 0.0, 0.0)
    assert distance_and_ttc(
        stationary, ActorMotion(3.0, 4.0, 0.0, 0.0)
    ) == (5.0, None)
    assert distance_and_ttc(stationary, stationary) == (0.0, None)


def test_lateral_closing_projection():
    distance, ttc = distance_and_ttc(
        ActorMotion(0.0, 0.0, 1.0, 0.0),
        ActorMotion(3.0, 4.0, 0.0, 0.0),
    )
    assert distance == pytest.approx(5.0)
    assert ttc == pytest.approx(25.0 / 3.0)


def test_invalid_motion_rejected():
    with pytest.raises(ValueError, match='finite'):
        distance_and_ttc(
            ActorMotion(float('nan'), 0.0, 0.0, 0.0),
            ActorMotion(1.0, 0.0, 0.0, 0.0),
        )
