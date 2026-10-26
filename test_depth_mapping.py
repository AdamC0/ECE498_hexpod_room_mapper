"""Tests for the temporary registered-depth planner layer."""

import unittest
from unittest.mock import patch

import numpy as np

from depth_mapping import (_clustered_cells, project_depth_cells,
                           project_obstacle_cells, quaternion_matrix)
from lidar_mapping import project_lidar_cells
from map_explore import ExploreMap, FREE, OCCUPIED


class DepthProjectionTests(unittest.TestCase):
    # Optical z points along map x, optical x points along -map y, and the
    # camera is 0.20 m above the floor. A center pixel therefore traces a
    # horizontal ray through the configured low-obstacle height band.
    optical_to_map = ((0., 0., .20), (-.5, .5, -.5, .5))
    single_pixel_k = (100., 0., 0., 0., 100., 0., 0., 0., 1.)

    def project_single(self, value):
        return project_depth_cells(
            np.array([[value]], np.float32), self.single_pixel_k,
            self.optical_to_map, cell=.1, decimation=1,
            range_min=.1, range_max=2., height_min=.025,
            height_max=.5, min_points=1, protected_range=.6)

    def test_identity_quaternion_matrix(self):
        np.testing.assert_allclose(quaternion_matrix((0., 0., 0., 1.)), np.eye(3))

    def test_packed_clustering_counts_distinct_rays_with_signed_cells(self):
        cells = _clustered_cells(
            [-2, -2, -2, -2, 1], [-3, -3, -3, -3, 4], 3,
            ray_ids=[0, 0, 1, 2, 3])
        self.assertEqual(cells, {(-2, -3)})

    def test_clustered_above_floor_points_become_one_cell(self):
        depth = np.full((8, 8), .20, np.float32)
        k = (100., 0., 3.5, 0., 100., 3.5, 0., 0., 1.)
        cells = project_obstacle_cells(
            depth, k, ((0., 0., 0.), (0., 0., 0., 1.)),
            cell=.1, decimation=1, range_min=.1, range_max=1.,
            height_min=.025, height_max=.5, min_points=3)
        self.assertEqual(cells, [(-1, -1), (-1, 0), (0, -1), (0, 0)])

    def test_ground_and_high_geometry_are_excluded(self):
        k = (100., 0., 3.5, 0., 100., 3.5, 0., 0., 1.)
        transform = ((0., 0., 0.), (0., 0., 0., 1.))
        for value in (.02, .8):
            cells = project_obstacle_cells(
                np.full((8, 8), value, np.float32), k, transform,
                cell=.1, decimation=1, range_min=.01, range_max=1.,
                height_min=.025, height_max=.5, min_points=3)
            self.assertEqual(cells, [])

    def test_farther_surface_marks_removed_obstacle_cell_observed_free(self):
        occupied, observed_free = self.project_single(1.0)
        self.assertIn((0, 10), occupied)
        self.assertNotIn((0, 10), observed_free)

        occupied, observed_free = self.project_single(1.5)
        self.assertIn((0, 15), occupied)
        self.assertIn((0, 10), observed_free)
        self.assertNotIn((0, 15), observed_free)

    def test_nearer_invalid_and_protected_rays_do_not_clear_old_cells(self):
        _, occluded_free = self.project_single(.8)
        self.assertNotIn((0, 10), occluded_free)
        for invalid in (0., np.nan, np.inf):
            occupied, observed_free = self.project_single(invalid)
            self.assertEqual(occupied, [])
            self.assertEqual(observed_free, [])

        near_occupied, _ = self.project_single(.5)
        _, farther_free = self.project_single(1.0)
        self.assertIn((0, 5), near_occupied)
        self.assertNotIn((0, 5), farther_free)


class LidarProjectionTests(unittest.TestCase):
    identity = ((0., 0., 0.), (0., 0., 0., 1.))

    def test_hit_marks_endpoint_and_ray_before_it_free(self):
        occupied, observed_free = project_lidar_cells(
            [1.0], 0., 1., self.identity, cell=.1,
            range_min=.1, range_max=2., decimation=1,
            min_rays=1, free_margin=.1)
        self.assertIn((0, 10), occupied)
        self.assertIn((0, 5), observed_free)
        self.assertNotIn((0, 10), observed_free)

    def test_no_return_supplies_free_evidence_without_fake_endpoint(self):
        occupied, observed_free = project_lidar_cells(
            [np.inf], 0., 1., self.identity, cell=.1,
            range_min=.1, range_max=2., decimation=1, min_rays=1)
        self.assertEqual(occupied, [])
        self.assertIn((0, 10), observed_free)
        occupied, observed_free = project_lidar_cells(
            [np.nan], 0., 1., self.identity, cell=.1,
            range_min=.1, range_max=2., decimation=1, min_rays=1)
        self.assertEqual((occupied, observed_free), ([], []))


class DepthOverlayTests(unittest.TestCase):
    def test_depth_cell_is_temporary_and_lidar_map_is_preserved(self):
        explorer = ExploreMap(size_m=2., cell=.1, robot_radius=.1, margin=0.)
        raw = np.zeros((40, 40), np.int16)
        explorer.update_from_slam(raw.ravel(), 40, 40, .05, -1., -1.)
        center = explorer.to_cell(.05, .05)
        self.assertEqual(explorer.state[center], FREE)
        explorer.update_depth_obstacles([(0, 0)], now=10.)
        self.assertEqual(explorer.state[center], OCCUPIED)
        self.assertEqual(explorer.depth_obstacle_count(now=39.9), 1)
        self.assertEqual(explorer.depth_obstacle_count(now=40.1), 0)
        self.assertEqual(explorer.state[center], FREE)

    def test_depth_mask_identifies_only_live_overlay_cells(self):
        explorer = ExploreMap(size_m=2., cell=.1)
        raw = np.zeros((40, 40), np.int16)
        explorer.update_from_slam(raw.ravel(), 40, 40, .05, -1., -1.)
        explorer.update_depth_obstacles([(0, 0)], now=10.)
        center = explorer.to_cell(.05, .05)
        self.assertTrue(explorer.depth_obstacle_mask(now=39.9)[center])
        self.assertFalse(explorer.depth_obstacle_mask(now=40.1).any())

    def test_new_slam_update_keeps_live_depth_overlay(self):
        explorer = ExploreMap(size_m=2., cell=.1)
        raw = np.zeros((40, 40), np.int16)
        explorer.update_from_slam(raw.ravel(), 40, 40, .05, -1., -1.)
        explorer.update_depth_obstacles([(0, 0)], now=100., ttl=10.)
        with patch('map_explore.time.time', return_value=101.):
            explorer.update_from_slam(raw.ravel(), 40, 40, .05, -1., -1.)
        self.assertEqual(explorer.state[explorer.to_cell(.05, .05)], OCCUPIED)

    def test_two_consecutive_clear_observations_remove_moved_obstacle(self):
        explorer = ExploreMap(size_m=4., cell=.1)
        raw = np.zeros((80, 80), np.int16)
        explorer.update_from_slam(raw.ravel(), 80, 80, .05, -2., -2.)
        cell = (0, 10)
        explorer.update_depth_obstacles([cell], now=10.)

        cleared = explorer.update_depth_obstacles(
            [], now=11., observed_free_cells=[cell], clear_confirmations=2)
        self.assertEqual(cleared, 0)
        self.assertEqual(explorer.depth_obstacle_count(now=11.), 1)
        cleared = explorer.update_depth_obstacles(
            [], now=12., observed_free_cells=[cell], clear_confirmations=2)
        self.assertEqual(cleared, 1)
        self.assertEqual(explorer.depth_obstacle_count(now=12.), 0)

    def test_clear_streak_requires_consecutive_valid_observations(self):
        explorer = ExploreMap(size_m=4., cell=.1)
        raw = np.zeros((80, 80), np.int16)
        explorer.update_from_slam(raw.ravel(), 80, 80, .05, -2., -2.)
        cell = (0, 10)
        explorer.update_depth_obstacles([cell], now=10.)
        explorer.update_depth_obstacles([], now=11., observed_free_cells=[cell])
        # An invalid, occluded, or out-of-view frame supplies no free evidence
        # and breaks the streak without refreshing the obstacle's TTL.
        explorer.update_depth_obstacles([], now=12., observed_free_cells=[])
        cleared = explorer.update_depth_obstacles(
            [], now=13., observed_free_cells=[cell])
        self.assertEqual(cleared, 0)
        self.assertEqual(explorer.depth_obstacle_count(now=13.), 1)

    def test_simultaneous_occupied_evidence_wins_and_resets_clear_count(self):
        explorer = ExploreMap(size_m=4., cell=.1)
        raw = np.zeros((80, 80), np.int16)
        explorer.update_from_slam(raw.ravel(), 80, 80, .05, -2., -2.)
        cell = (0, 10)
        explorer.update_depth_obstacles([cell], now=10.)
        explorer.update_depth_obstacles([], now=11., observed_free_cells=[cell])
        explorer.update_depth_obstacles(
            [cell], now=12., observed_free_cells=[cell])
        cleared = explorer.update_depth_obstacles(
            [], now=13., observed_free_cells=[cell])
        self.assertEqual(cleared, 0)
        self.assertEqual(explorer.depth_obstacle_count(now=13.), 1)

    def test_clearing_overlay_never_erases_lidar_occupancy(self):
        explorer = ExploreMap(size_m=4., cell=.1)
        raw = np.zeros((80, 80), np.int16)
        raw[40:42, 60:62] = 100
        explorer.update_from_slam(raw.ravel(), 80, 80, .05, -2., -2.)
        cell = (0, 10)
        planner_cell = explorer.to_cell(1.05, .05)
        self.assertEqual(explorer.slam_state[planner_cell], OCCUPIED)
        explorer.update_depth_obstacles([cell], now=10.)
        explorer.update_depth_obstacles([], now=11., observed_free_cells=[cell])
        explorer.update_depth_obstacles([], now=12., observed_free_cells=[cell])
        self.assertFalse(explorer.depth_obstacle_mask(now=12.)[planner_cell])
        self.assertEqual(explorer.state[planner_cell], OCCUPIED)

    def test_route_opens_only_after_depth_cell_is_confirmed_clear(self):
        explorer = ExploreMap(size_m=2., cell=.1, robot_radius=.05, margin=0.)
        raw = np.zeros((40, 40), np.int16)
        explorer.update_from_slam(raw.ravel(), 40, 40, .05, -1., -1.)
        wall = [(row, 0) for row in range(-10, 10)]
        with patch('map_explore.time.time', return_value=10.):
            explorer.update_depth_obstacles(wall, now=10.)
            _, blocked_goal = explorer.plan_to_point((-.75, .05, 0.), (.75, .05))
        self.assertLess(blocked_goal[0], 0.)

        explorer.update_depth_obstacles([], now=11., observed_free_cells=wall)
        with patch('map_explore.time.time', return_value=12.):
            explorer.update_depth_obstacles([], now=12., observed_free_cells=wall)
            _, open_goal = explorer.plan_to_point((-.75, .05, 0.), (.75, .05))
        self.assertGreater(open_goal[0], .5)

    def test_repeated_depth_with_live_lidar_promotes_session_obstacle(self):
        explorer = ExploreMap(size_m=4., cell=.1)
        raw = np.zeros((80, 80), np.int16)
        explorer.update_from_slam(raw.ravel(), 80, 80, .05, -2., -2.)
        cell = (0, 10)
        for now in (10., 11., 12.):
            result = explorer.update_fused_observations(
                [cell], [], [], [(0, 5)], now=now, ttl=5.,
                promotion_confirmations=3, joint_clear_confirmations=2)
        planner_cell = explorer.to_cell(1.05, .05)
        self.assertEqual(result['promoted'], 1)
        self.assertIn(planner_cell, explorer.persistent_depth_obstacles)
        # A later observation must not put the promoted cell back into the
        # magenta temporary mask.
        explorer.update_fused_observations(
            [cell], [], [], [(0, 5)], now=13., ttl=5.,
            promotion_confirmations=3, joint_clear_confirmations=2)
        self.assertFalse(explorer.depth_obstacle_mask(now=100.)[planner_cell])
        self.assertEqual(explorer.state[planner_cell], OCCUPIED)

    def test_joint_free_suppresses_stale_hector_cell_but_preserves_raw_map(self):
        explorer = ExploreMap(size_m=4., cell=.1)
        raw = np.zeros((80, 80), np.int16)
        explorer.update_from_slam(raw.ravel(), 80, 80, .05, -2., -2.)
        cell = (0, 10)
        planner_cell = explorer.to_cell(1.05, .05)
        explorer.slam_state[planner_cell] = OCCUPIED
        explorer.state[planner_cell] = OCCUPIED

        first = explorer.update_fused_observations(
            [], [cell], [], [cell], now=10.,
            promotion_confirmations=3, joint_clear_confirmations=2)
        self.assertEqual(first['suppressed_slam'], 0)
        self.assertEqual(explorer.state[planner_cell], OCCUPIED)
        second = explorer.update_fused_observations(
            [], [cell], [], [cell], now=11.,
            promotion_confirmations=3, joint_clear_confirmations=2)
        self.assertEqual(second['suppressed_slam'], 1)
        self.assertEqual(explorer.slam_state[planner_cell], OCCUPIED)
        self.assertEqual(explorer.state[planner_cell], FREE)

        # A renewed hit restores occupancy immediately without waiting for a
        # new Hector publication.
        explorer.update_fused_observations(
            [cell], [], [], [], now=12.,
            promotion_confirmations=3, joint_clear_confirmations=2)
        self.assertNotIn(planner_cell, explorer.suppressed_slam_obstacles)
        self.assertEqual(explorer.state[planner_cell], OCCUPIED)

    def test_one_sensor_or_nearby_occupied_endpoint_cannot_clear_black_cell(self):
        explorer = ExploreMap(size_m=4., cell=.1)
        raw = np.zeros((80, 80), np.int16)
        explorer.update_from_slam(raw.ravel(), 80, 80, .05, -2., -2.)
        cell = (0, 10)
        planner_cell = explorer.to_cell(1.05, .05)
        explorer.slam_state[planner_cell] = OCCUPIED
        explorer.state[planner_cell] = OCCUPIED
        for now in (10., 11., 12.):
            explorer.update_fused_observations(
                [], [cell], [], [], now=now,
                promotion_confirmations=3, joint_clear_confirmations=2)
        self.assertEqual(explorer.state[planner_cell], OCCUPIED)

        for now in (13., 14.):
            explorer.update_fused_observations(
                [], [cell], [(0, 11)], [cell], now=now,
                promotion_confirmations=3, joint_clear_confirmations=2)
        self.assertEqual(explorer.state[planner_cell], OCCUPIED)

    def test_joint_free_removes_promoted_depth_cell_and_reset_forgets_fusion(self):
        explorer = ExploreMap(size_m=4., cell=.1)
        raw = np.zeros((80, 80), np.int16)
        explorer.update_from_slam(raw.ravel(), 80, 80, .05, -2., -2.)
        cell = (0, 10)
        planner_cell = explorer.to_cell(1.05, .05)
        for now in (10., 11.):
            explorer.update_fused_observations(
                [cell], [], [], [], now=now,
                promotion_confirmations=2, joint_clear_confirmations=2)
        self.assertIn(planner_cell, explorer.persistent_depth_obstacles)
        explorer.update_fused_observations(
            [], [cell], [], [cell], now=12.,
            promotion_confirmations=2, joint_clear_confirmations=2)
        result = explorer.update_fused_observations(
            [], [cell], [], [cell], now=13.,
            promotion_confirmations=2, joint_clear_confirmations=2)
        self.assertEqual(result['cleared_persistent'], 1)
        self.assertEqual(explorer.state[planner_cell], FREE)
        explorer.reset()
        self.assertFalse(explorer.persistent_depth_obstacles)
        self.assertFalse(explorer.suppressed_slam_obstacles)


if __name__ == '__main__':
    unittest.main()
