"""Tests for the temporary registered-depth planner layer."""

import unittest
from unittest.mock import patch

import numpy as np

from depth_mapping import project_obstacle_cells, quaternion_matrix
from map_explore import ExploreMap, FREE, OCCUPIED


class DepthProjectionTests(unittest.TestCase):
    def test_identity_quaternion_matrix(self):
        np.testing.assert_allclose(quaternion_matrix((0., 0., 0., 1.)), np.eye(3))

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


if __name__ == '__main__':
    unittest.main()
