import math
import unittest

from map_explore import ExploreMap, FREE, OCCUPIED, UNKNOWN


class SurveyPlannerTests(unittest.TestCase):
    def test_default_clearance_adds_leg_margin_to_thirty_centimetre_footprint(self):
        self.assertAlmostEqual(ExploreMap().clearance, .43)

    def test_goal_is_known_free_and_outside_arrival_tolerance(self):
        grid = ExploreMap(size_m=4., cell=.1)
        grid.state[10:30, 10:30] = FREE
        grid.have_map = True
        path, goal = grid.plan_frontier((0., 0., 0.))
        self.assertIsNotNone(path)
        self.assertGreaterEqual(math.hypot(*goal), .5)
        for x, y in path:
            self.assertEqual(grid.state[grid.to_cell(x, y)], FREE)

    def test_information_gain_prefers_longer_useful_frontier_route(self):
        grid = ExploreMap(size_m=6., cell=.1, frontier_goal_distance=1.5)
        grid.state[:] = FREE
        # A small nearby unknown patch competes with a larger classroom-sized
        # unexplored patch farther down the connected free region.
        grid.state[29:32, 35:38] = UNKNOWN
        grid.state[20:41, 49:58] = UNKNOWN
        grid.have_map = True
        path, goal = grid.plan_frontier((.05, .05, 0.))
        self.assertIsNotNone(path)
        self.assertGreater(goal[0], 1.0)
        self.assertGreaterEqual(math.hypot(goal[0]-.05, goal[1]-.05), 1.5)

    def test_unknown_space_is_not_a_route_to_distant_free_island(self):
        grid = ExploreMap(size_m=4., cell=.1)
        grid.state[20, 20] = FREE
        grid.state[22:30, 22:30] = FREE
        grid.have_map = True
        self.assertEqual(grid.plan_frontier((.05, .05, 0.)), (None, None))

    def test_tiny_nearby_frontier_does_not_make_zero_progress_goal(self):
        grid = ExploreMap(size_m=4., cell=.1)
        grid.state[18:23, 18:23] = FREE
        grid.have_map = True
        self.assertEqual(grid.plan_frontier((0., 0., 0.)), (None, None))

    def test_enclosed_map_uses_connected_known_free_survey_viewpoint(self):
        grid = ExploreMap(size_m=4., cell=.1)
        grid.state[9:32, 9:32] = OCCUPIED
        grid.state[10:31, 10:31] = FREE
        grid.have_map = True
        path, goal = grid.plan_frontier((.05, .05, 0.))
        self.assertIsNotNone(path)
        self.assertGreaterEqual(math.hypot(goal[0]-.05, goal[1]-.05), .5)
        self.assertEqual(grid.state[grid.to_cell(*goal)], FREE)
        self.assertEqual(grid.last_plan_kind, 'survey viewpoint')

    def test_selected_goal_uses_nearest_reachable_safe_cell(self):
        grid = ExploreMap(size_m=4., cell=.1, robot_radius=.15, margin=.02)
        grid.state[8:32, 8:20] = FREE
        grid.state[8:32, 20] = OCCUPIED
        grid.state[8:32, 21:32] = FREE
        grid.have_map = True
        path, goal = grid.plan_to_point((-.55, .05, 0.), (.8, .05))
        self.assertIsNotNone(path)
        # The wall is never crossed; the route approaches it from the
        # connected side while maintaining inflated body clearance.
        self.assertLess(goal[0], 0.)
        for x, y in path:
            r, c = grid.to_cell(x, y)
            self.assertEqual(grid.state[r, c], FREE)
            self.assertGreater(grid.obstacle_distance()[r, c], grid.clearance)

    def test_object_goal_uses_standoff_ring(self):
        grid = ExploreMap(size_m=4., cell=.1, robot_radius=.15, margin=.02)
        grid.state[:] = FREE
        grid.have_map = True
        path, goal = grid.plan_to_point((-.95, .05, 0.), (.75, .05), standoff=.70)
        self.assertIsNotNone(path)
        self.assertAlmostEqual(math.hypot(goal[0]-.75, goal[1]-.05), .70, delta=.08)
        self.assertEqual(grid.last_plan_kind, 'object approach')


if __name__ == '__main__':
    unittest.main()
