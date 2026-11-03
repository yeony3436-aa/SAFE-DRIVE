"""Boundary checks for the predefined measured-risk rule."""

import unittest

from run_pedestrian_sweep import classify


RISK = {'collision_is_risk': True,
        'minimum_actor_center_distance_below_m': 3.0}


class RiskClassificationTests(unittest.TestCase):
    def test_distance_threshold_is_strict(self):
        safety = {'scenario_completed': True, 'collision': False,
                  'min_distance': 3.0}
        self.assertEqual(classify(safety, RISK), (True, 'not_risky'))
        safety['min_distance'] = 2.99
        self.assertEqual(classify(safety, RISK), (True, 'risky'))

    def test_collision_counts_even_with_large_center_distance(self):
        safety = {'scenario_completed': True, 'collision': True,
                  'min_distance': 5.0}
        self.assertEqual(classify(safety, RISK), (True, 'risky'))

    def test_missing_measurement_is_unknown(self):
        safety = {'scenario_completed': True, 'collision': False,
                  'min_distance': None}
        self.assertEqual(classify(safety, RISK), (False, 'unknown'))


if __name__ == '__main__':
    unittest.main()
