"""Checks that painted-lane results are independent pass criteria."""

import unittest

from run_connected_validation import observed_pass


class ConnectedResultTests(unittest.TestCase):
    def test_route_requires_both_corridor_and_painted_lines(self):
        case = {'pedestrian': False, 'expected': {
            'result_file': 'route.json', 'lane_result_file': 'lane.json'}}
        results = {'result_file': {'passed': True},
                   'lane_result_file': {'passed': True}}
        self.assertTrue(observed_pass(case, results))
        results['lane_result_file']['passed'] = False
        self.assertFalse(observed_pass(case, results))

    def test_red_requires_painted_lines(self):
        case = {'pedestrian': True, 'expected': {
            'lane_result_file': 'lane.json'}}
        results = {
            'red_result_file': {'passed': True},
            'pedestrian_result_file': {'completed': True},
            'pedestrian_safety_file': {
                'scenario_completed': True, 'collision': False,
                'min_distance': 5.0},
            'lane_result_file': {'passed': False},
        }
        self.assertFalse(observed_pass(case, results))
        results['lane_result_file']['passed'] = True
        self.assertTrue(observed_pass(case, results))

    def test_continuous_case_requires_stop_before_route_completion(self):
        case = {'pedestrian': True, 'expected': {
            'result_file': 'route.json', 'lane_result_file': 'lane.json'}}
        results = {
            'red_result_file': {'passed': True, 'end_sim_time': 9.0},
            'pedestrian_result_file': {'completed': True},
            'pedestrian_safety_file': {
                'scenario_completed': True, 'collision': False,
                'min_distance': 5.0},
            'result_file': {'passed': True, 'end_sim_time': 120.0},
            'lane_result_file': {'passed': True},
        }
        self.assertTrue(observed_pass(case, results))
        results['result_file']['end_sim_time'] = 8.0
        self.assertFalse(observed_pass(case, results))
        results['result_file']['end_sim_time'] = 120.0
        results['result_file']['passed'] = False
        self.assertFalse(observed_pass(case, results))

    def test_risk_case_accepts_measured_collision(self):
        case = {'pedestrian': True, 'allow_collision': True,
                'expected': {'lane_result_file': 'lane.json'}}
        results = {
            'red_result_file': {'passed': True, 'end_sim_time': 9.0},
            'pedestrian_result_file': {'completed': True},
            'pedestrian_safety_file': {
                'scenario_completed': True, 'collision': True,
                'min_distance': 0.8},
            'lane_result_file': {'passed': False},
        }
        self.assertTrue(observed_pass(case, results))
        case['allow_collision'] = False
        self.assertFalse(observed_pass(case, results))


if __name__ == '__main__':
    unittest.main()
