#!/usr/bin/env python3
"""Tests for particle_filter.py."""

import unittest
from unittest.mock import Mock

import particle_filter
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import math
from tf.transformations import euler_from_quaternion


class TestParticleFilter(unittest.TestCase):
    def setUp(self):
        self.particle_filter = particle_filter.ParticleFilter(test_mode=True)
        pass

    def test_get_map(self):
        grid_data = OccupancyGrid()
        grid_data.data = [-1, 100, 0, 100, 0, -1]
        self.particle_filter.get_map(grid_data)
        self.assertEqual(self.particle_filter.occupancy_field, [2, 4])

    def test_get_pose_from_index(self):
        width = 10
        height = 20
        resolution = 0.5
        origin = Pose()
        origin.position = Point(0, 0, 0)
        origin.orientation = Quaternion(0, 0, 0, 1)
        self.particle_filter.map = OccupancyGrid()
        self.particle_filter.map.info.origin = origin
        self.particle_filter.map.info.width = width
        self.particle_filter.map.info.height = height
        self.particle_filter.map.info.resolution = resolution

        pose = self.particle_filter._get_pose_from_index(0)
        self.assertEqual(pose.position, Point(0, 0, 0))

        pose = self.particle_filter._get_pose_from_index(15)
        self.assertEqual(pose.position, Point(2.5, 0.5, 0))
        yaw = abs(
            euler_from_quaternion(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )[2]
        )
        # check that the yaw is one of the expected angles.
        self.assertIn(
            True,
            map(
                lambda x: round(x - yaw, 7) == 0,
                self.particle_filter.angles,
            ),
        )

    def test_normalize_particles(self):
        self.particle_filter.particle_cloud = [
            particle_filter.Particle(Pose(), 1) for i in range(10)
        ]
        self.particle_filter.normalize_particles()

        for particle in self.particle_filter.particle_cloud:
            self.assertAlmostEqual(particle.w, 0.1)

        weights = [1, 2, 3, 4]
        normalized_weights = [0.1, 0.2, 0.3, 0.4]

        self.particle_filter.particle_cloud = [
            particle_filter.Particle(Pose(), w) for w in weights
        ]
        self.particle_filter.normalize_particles()

        for particle, w in zip(
            self.particle_filter.particle_cloud, normalized_weights
        ):
            self.assertAlmostEqual(particle.w, w)

    def test_update_particle_weights_with_measurement_model(self):
        self.particle_filter.particle_cloud = [
            particle_filter.Particle(Pose(), 1)
        ]
        data = particle_filter.LaserScan()
        data.ranges = [0 for i in range(360)]

        # create a mock of the likelihood field to set return value of
        # get_closest_object_distance.return value
        self.particle_filter.likelihoodfield = Mock()
        self.particle_filter.likelihoodfield.get_closest_obstacle_distance.return_value = (
            0.1
        )

        # make mock of gaussian
        particle_filter.compute_prob_zero_centered_gaussian = Mock()
        particle_filter.compute_prob_zero_centered_gaussian.return_value = 10

        self.particle_filter.update_particle_weights_with_measurement_model(
            data
        )

        w_expected = 10000

        part = self.particle_filter.particle_cloud[0]

        self.assertAlmostEqual(w_expected, part.w)


if __name__ == "__main__":
    unittest.main()
