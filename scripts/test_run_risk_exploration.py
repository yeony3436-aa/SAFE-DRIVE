"""Checks for equal-budget selectors and measured-result accounting."""

import random
import unittest

from run_risk_exploration import (AdaptiveGrid, center, observed_priority,
                                  sample_random, split, summarize, validate)


BOUNDS = {'ego_speed_mps': [1.5, 2.5],
          'trigger_distance_m': [3.0, 9.0],
          'pedestrian_speed_mps': [0.7, 1.3]}
RISK = {'collision_is_risk': True,
        'minimum_actor_center_distance_below_m': 3.0}


def record(distance, collision=False, valid=True):
    return {'measurement_valid': valid, 'risk_classification': (
        'risky' if valid and (collision or distance < 3) else
        'not_risky' if valid else 'unknown'),
        'safety': {'collision': collision, 'min_distance': distance},
        'sampled_parameters': center(BOUNDS)}


class ExplorationTests(unittest.TestCase):
    def test_config_rejects_invalid_budget_and_ranges(self):
        config = {'seeds': [42, 137], 'budget_per_method': 3,
                  'adaptive_max_depth': 3, 'parameters': BOUNDS,
                  'risk': RISK, 'fixed_pedestrian_spawn_distance_m': 10.0}
        validate(config)
        config['budget_per_method'] = 0
        with self.assertRaises(ValueError):
            validate(config)
        config['budget_per_method'] = 3
        config['parameters'] = {**BOUNDS, 'trigger_distance_m': [9.0, 3.0]}
        with self.assertRaises(ValueError):
            validate(config)

    def test_random_selection_reproducible_and_bounded(self):
        first = [sample_random(BOUNDS, random.Random(42))]
        second = [sample_random(BOUNDS, random.Random(42))]
        self.assertEqual(first, second)
        for key, value in first[0].items():
            self.assertLessEqual(BOUNDS[key][0], value)
            self.assertLessEqual(value, BOUNDS[key][1])

    def test_adaptive_refines_observed_risky_cell(self):
        grid = AdaptiveGrid(BOUNDS, 3)
        self.assertEqual(grid.next(), center(BOUNDS))
        grid.observe(record(4.0), RISK)
        first_child = grid.next()
        self.assertEqual(len(split(BOUNDS)), 8)
        grid.observe(record(1.0, collision=True), RISK)
        next_point = grid.next()
        self.assertNotEqual(next_point, first_child)
        self.assertEqual(grid.last_cell[1], 2)
        self.assertGreater(observed_priority(record(1.0, True), RISK),
                           observed_priority(record(4.0), RISK))

    def test_summary_counts_valid_risks_and_first_detection(self):
        records = [record(4.0), record(2.0), record(1.0, valid=False)]
        self.assertEqual(summarize(records),
                         {'budget': 3, 'valid_runs': 2, 'risky_runs': 1,
                          'first_risky_run': 2, 'unique_risky_conditions': 1})


if __name__ == '__main__':
    unittest.main()
