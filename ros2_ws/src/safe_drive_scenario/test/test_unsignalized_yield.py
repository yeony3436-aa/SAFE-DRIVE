"""Behavioral checks for conservative pedestrian yield decisions."""

from safe_drive_scenario.unsignalized_yield import pedestrian_requires_yield


def test_active_pedestrian_blocks_entry():
    assert pedestrian_requires_yield('ACTIVE')


def test_finished_pedestrian_releases_entry():
    assert not pedestrian_requires_yield('COMPLETED')


def test_missing_or_timed_out_state_blocks_entry():
    assert pedestrian_requires_yield(None)
    assert pedestrian_requires_yield('TIMED_OUT')


def test_armed_actor_at_curb_does_not_block_entry():
    assert not pedestrian_requires_yield('ARMED')
