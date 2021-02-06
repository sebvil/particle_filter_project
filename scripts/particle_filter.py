#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from likelihood_field import LikelihoodField

import numpy as np
from numpy.random import random_sample
import math
from copy import deepcopy

from random import randint, random, choice
import math
import time


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = euler_from_quaternion(
        [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    )[2]

    return yaw


def set_orientation_from_yaw(yaw, pose):
    orientation = quaternion_from_euler(0, 0, yaw)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]


def draw_random_sample(choices, probabilities, n):
    """Return a random sample of n elements from the set choices with the specified probabilities
    choices: the values to sample from represented as a list
    probabilities: the probability of selecting each element in choices represented as a list
    n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

class Particle:
    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w


class ParticleFilter:
    def __init__(self, test_mode=False):
        n_angles = 4
        self.angles = [i * math.tau / n_angles for i in range(n_angles)]
        # prevents initialization to test functions individually
        if test_mode:
            return
        # once everything is setup initialized will be set to true
        self.initialized = False

        # initialize this particle filter node
        rospy.init_node("turtlebot3_particle_filter")

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()
        self.occupancy_field = None

        # the number of particles used in the particle filter
        self.num_particles = 5000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = np.pi / 6

        self.odom_pose_last_motion_update = None

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher(
            "particle_cloud", PoseArray, queue_size=10
        )

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher(
            "estimated_robot_pose", PoseStamped, queue_size=10
        )

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # initialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True

        # initialize likelihood field
        self.likelihoodfield = LikelihoodField()

    def get_map(self, data):

        self.map = data
        # self.occupancy_field = OccupancyField(data)

        self.occupancy_field = [i for i, x in enumerate(data.data) if x == 0]

    def _get_pose_from_index(self, index):
        width = self.map.info.width
        resolution = self.map.info.resolution
        origin = self.map.info.origin
        x_offset = index % width * resolution
        y_offset = index // width * resolution

        pose = Pose()
        pose.position.x = origin.position.x + x_offset
        pose.position.y = origin.position.y + y_offset
        set_orientation_from_yaw(choice(self.angles), pose)

        return pose

    def initialize_particle_cloud(self):
        # makes sure we have data before proceeding
        while self.occupancy_field is None:
            pass
        probabilities = [1 / len(self.occupancy_field)] * len(
            self.occupancy_field
        )
        chosen_particles = draw_random_sample(
            self.occupancy_field, probabilities, self.num_particles
        )
        self.particle_cloud = [
            Particle(self._get_pose_from_index(i), 1) for i in chosen_particles
        ]

        self.normalize_particles()
        self.publish_particle_cloud()

    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        sum_of_weights = sum(particle.w for particle in self.particle_cloud)
        for particle in self.particle_cloud:
            particle.w /= sum_of_weights

    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(
            stamp=rospy.Time.now(), frame_id=self.map_topic
        )
        particle_cloud_pose_array.poses = []

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)
        print("test")

    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(
            stamp=rospy.Time.now(), frame_id=self.map_topic
        )
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)

    def resample_particles(self):
        """ resamples particle cloud based with probability based on weight"""
        # TODO test
        
        # get list of weights of particles (to be used as probability)
        weights = [float(particle.w) for particle in self.particle_cloud]

        # resample particles to create new particle cloud
        self.particle_cloud = draw_random_sample(
            self.particle_cloud, weights, self.num_particles
        )
        

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not (self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not (
            self.tf_listener.canTransform(
                self.base_frame, data.header.frame_id, data.header.stamp
            )
        ):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated)
        self.tf_listener.waitForTransform(
            self.base_frame,
            self.odom_frame,
            data.header.stamp,
            rospy.Duration(0.5),
        )
        if not (
            self.tf_listener.canTransform(
                self.base_frame, data.header.frame_id, data.header.stamp
            )
        ):
            return

        # calculate the pose of the laser distance sensor
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0), frame_id=data.header.frame_id)
        )

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp, frame_id=self.base_frame),
            pose=Pose(),
        )

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (
                np.abs(curr_x - old_x) > self.lin_mvmt_threshold
                or np.abs(curr_y - old_y) > self.lin_mvmt_threshold
                or np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold
            ):

                # This is where the main logic of the particle filter is carried out
                start_time = time.time()
                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                processing_time = time.time() - start_time)
                
                self.odom_pose_last_motion_update = self.odom_pose

    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        # get the mean of the turtlebot position

        # TODO test

        # initialize and sum values for x,y, and theta across all particles
        x_mean = 0
        y_mean = 0
        theta_mean = 0
        for particle in self.particle_cloud:
            x_mean += particle.pose.position.x
            y_mean += particle.pose.position.y
            theta_mean += get_yaw_from_pose(particle.pose)

        # set robot pose to mean values above
        self.robot_estimate.position.x = x_mean/ self.num_particles
        self.robot_estimate.position.y = y_mean/ self.num_particles
        set_orientation_from_yaw(theta_mean/self.num_particles, self.robot_estimate)

    def update_particle_weights_with_measurement_model(self, data):

        # TODO tests

        # decided to use likelihood field bc computational tractable means
        # to find robot position

        # intialize angle to look for particle & robot scan data
        direction_index_list = [0, 90, 180, 270]
        z = data.ranges

        # loop through every particle
        for particle in self.particle_cloud:
            q = 1
            
            for ang in direction_index_list:
                
                # if distance outside of range skip this value
                if z[ang] > 3.5:
                    continue
                
                # get position of to check for object from particle's curr
                # position & yaw
                x = particle.pose.position.x
                y = particle.pose.position.y
                theta =get_yaw_from_pose(particle.pose)

                # convert ang to radians
                ang_rad = ang * math.pi /180.0

                # calculate particle's location and orientation from laser scen
                x_k = x + z[ang] * math.cos(theta + ang_rad)
                y_k = y + z[ang] * math.sin(theta + ang_rad)

                # calculate distance between predicted particle laser scane & closest object
                dist = self.likelihoodfield.get_closest_obstacle_distance(x_k, y_k)

                # if the particle is out of map boundaries nan is returned
                # thus we skip over particles that return nan
                if math.isnan(dist):
                    continue

                # compute probability with zero-gaussian & sd = 0.1 
                # set new q
                sd = 0.1
                q *= compute_prob_zero_centered_gaussian(dist, sd)
            
            particle.w = q
            

        

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO tests
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        for particle in self.particle_cloud:
            particle.pose.position.x += curr_x - old_x
            particle.pose.position.y += curr_y - old_y
            new_yaw = get_yaw_from_pose(particle.pose) + (curr_yaw - old_yaw)
            set_orientation_from_yaw(
                new_yaw,
                particle.pose,
            )


if __name__ == "__main__":

    pf = ParticleFilter()

    rospy.spin()
