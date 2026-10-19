import math
import unittest

from navigation_validation import analyze


class NavigationValidationTests(unittest.TestCase):
    def test_consistent_trace_still_requires_physical_confirmation(self):
        transitions = [(0., False), (2., True), (3., False)]
        poses = []
        for i in range(20):
            t = i * .5
            moving = 2. <= t < 3.
            x = .08 if t >= 3. else (.04 if moving else 0.)
            poses.append((t, x, 0., math.radians(8 if t >= 3 else 0), moving))
        result = analyze(poses, transitions, .1)
        self.assertTrue(result['slam_consistency_pass'])
        self.assertTrue(result['physical_confirmation_required'])
        self.assertIn('awaiting physical confirmation', result['verdict'])

    def test_pose_jump_and_settled_drift_fail(self):
        transitions = [(0., False), (1., True), (2., False)]
        poses = [(i, 0., 0., 0., i == 1) for i in range(10)]
        poses[3] = (3., .30, 0., 0., False)
        poses[4] = (4., .45, 0., math.radians(8), False)
        result = analyze(poses, transitions)
        self.assertFalse(result['slam_consistency_pass'])
        self.assertTrue(any('pose step' in item for item in result['failures']))
        self.assertTrue(any('settled drift' in item for item in result['failures']))

    def test_no_motion_is_not_a_navigation_pass(self):
        poses = [(float(i), 0., 0., 0., False) for i in range(12)]
        result = analyze(poses, [(0., False)])
        self.assertFalse(result['slam_consistency_pass'])
        self.assertIn('no commanded motion burst observed', result['failures'])


if __name__ == '__main__':
    unittest.main()
