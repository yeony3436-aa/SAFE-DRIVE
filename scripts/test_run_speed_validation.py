"""Checks for configured speed variants and observed result extraction."""

import unittest

from run_speed_validation import measurements, validate, variant


BASE = {'speeds_mps': [1.2, 1.6], 'repeats': 1,
        'cases': ['green_right', 'red_crossing']}
CASES = {'green_right': {}, 'red_crossing': {}}


class SpeedValidationTests(unittest.TestCase):
    def test_speed_settings_are_independent(self):
        source = {'waypoint_follower': {'ros__parameters': {
            'target_speed': 1.2, 'max_linear_speed': 1.2,
            'waypoint_x': [0.0, 1.0]}}}
        changed = variant(source, 1.6)
        self.assertEqual(source['waypoint_follower']['ros__parameters'][
            'target_speed'], 1.2)
        self.assertEqual(changed['waypoint_follower']['ros__parameters'][
            'target_speed'], 1.6)
        self.assertEqual(changed['waypoint_follower']['ros__parameters'][
            'max_linear_speed'], 1.6)

    def test_rejects_duplicate_or_invalid_speed(self):
        validate(BASE, CASES)
        for speeds in ([1.2, 1.2], [0.0], [float('inf')]):
            with self.assertRaises(ValueError):
                validate({**BASE, 'speeds_mps': speeds}, CASES)

    def test_read_actual_speed_and_stop_gap(self):
        red = {'results': {
            'red_result_file': {'max_speed_mps': 1.1,
                                'minimum_front_gap_m': 0.9,
                                'passed': True},
            'pedestrian_safety_file': {'collision': False,
                                       'scenario_completed': True}}}
        self.assertEqual(measurements(red, True)['minimum_front_gap_m'], 0.9)
        self.assertEqual(
            measurements(red, True)['achieved_max_speed_mps'], 1.1)

    def test_continuous_case_uses_route_speed_and_reports_red_approach(self):
        record = {'results': {
            'red_result_file': {'max_speed_mps': 1.0,
                                'minimum_front_gap_m': 0.8,
                                'passed': True},
            'result_file': {'max_speed_mps': 3.3, 'goal_reached': True,
                            'max_corner_error_m': 1.5,
                            'outside_corridor_samples': 0,
                            'passed': True},
            'pedestrian_safety_file': {'collision': False,
                                       'scenario_completed': True}}}
        values = measurements(record, True)
        self.assertEqual(values['achieved_max_speed_mps'], 3.3)
        self.assertEqual(values['approach_max_speed_mps'], 1.0)
        self.assertTrue(values['goal_reached'])


if __name__ == '__main__':
    unittest.main()
