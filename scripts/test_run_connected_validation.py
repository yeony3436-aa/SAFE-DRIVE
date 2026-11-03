"""Checks that painted-lane results are independent pass criteria."""

import unittest

from run_connected_validation import (launch_command, launch_exit_error,
                                      observed_pass)


class ConnectedResultTests(unittest.TestCase):
    def test_port_conflict_is_reported_before_missing_results(self):
        message = launch_exit_error(
            1, 'Gazebo master at http://127.0.0.1:11345 is already running')
        self.assertIn('Gazebo is already running', message)

    def test_launch_command_can_show_gazebo_gui(self):
        case = {'route_config': 'connected_right_loop.yaml',
                'fixed_state': '', 'pedestrian': True}
        command = launch_command(case, {}, gui=True)
        self.assertIn('gui:=true', command)
        self.assertFalse(any(part.startswith('fixed_state:=')
                             for part in command))
        self.assertIn('gui:=false', launch_command(case, {}, gui=False))

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

    def test_late_entry_requires_moving_ego_and_unpermitted_signal(self):
        case = {'pedestrian': True, 'allow_collision': True,
                'late_trigger_distance_m': 3.0, 'expected': {}}
        results = {
            'red_result_file': {'passed': True},
            'pedestrian_result_file': {
                'completed': True, 'start_signal': 'DONT_WALK',
                'start_vehicle_signal': 'GREEN',
                'start_ego_x_m': -7.9, 'start_ego_speed_mps': 0.8,
                'crossing_x_m': -5.0},
            'pedestrian_safety_file': {
                'scenario_completed': True, 'collision': True,
                'min_distance': 1.9},
        }
        self.assertTrue(observed_pass(case, results))
        results['pedestrian_result_file']['start_vehicle_signal'] = 'RED'
        self.assertFalse(observed_pass(case, results))
        results['pedestrian_result_file']['start_vehicle_signal'] = 'GREEN'
        results['pedestrian_result_file']['start_ego_speed_mps'] = 0.0
        self.assertFalse(observed_pass(case, results))

    def test_demo_stop_requires_measured_gap_and_settled_speed(self):
        case = {'pedestrian': True, 'allow_collision': False,
                'minimum_pedestrian_front_gap_m': 0.3,
                'maximum_crossing_end_speed_mps': 0.15, 'expected': {}}
        results = {
            'red_result_file': {'passed': True},
            'pedestrian_result_file': {'completed': True},
            'pedestrian_safety_file': {
                'scenario_completed': True, 'collision': False,
                'min_distance': 3.0,
                'minimum_front_gap_during_crossing_m': 0.4,
                'ego_speed_at_crossing_end_mps': 0.02},
        }
        self.assertTrue(observed_pass(case, results))
        results['pedestrian_safety_file'][
            'minimum_front_gap_during_crossing_m'] = 0.1
        self.assertFalse(observed_pass(case, results))

    def test_demo_contact_must_end_on_measured_contact(self):
        case = {'pedestrian': True, 'allow_collision': True,
                'stop_on_contact': True, 'expected': {},
                'contact_lane_y_m': -1.75,
                'contact_lateral_tolerance_m': 0.5,
                'contact_front_gap_min_m': -0.3,
                'contact_front_gap_max_m': 0.6}
        results = {
            'red_result_file': {'passed': True},
            'pedestrian_result_file': {
                'completed': False, 'contact_terminated': True,
                'final_y_m': -1.55},
            'pedestrian_safety_file': {
                'scenario_completed': False, 'state': 'CONTACT',
                'collision': True, 'min_distance': 1.8,
                'minimum_front_gap_during_crossing_m': 0.28},
        }
        self.assertTrue(observed_pass(case, results))
        results['pedestrian_result_file']['final_y_m'] = -2.85
        self.assertFalse(observed_pass(case, results))
        results['pedestrian_result_file']['final_y_m'] = -1.55
        results['pedestrian_safety_file']['collision'] = False
        self.assertFalse(observed_pass(case, results))


if __name__ == '__main__':
    unittest.main()
