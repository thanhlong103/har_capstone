#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from leg_detector_msgs.msg import PersonArray, Person # subscriber message type
from fused_people_msgs.msg import FusedPersonArray, FusedPerson
import std_msgs.msg
import numpy as np
from scipy.optimize import linear_sum_assignment
import scipy.stats
import scipy.spatial
from geometry_msgs.msg import PointStamped, Point
import time

from pykalman import KalmanFilter 
toe2leg = 195.0
scan_frequency = 10
distance_sources_threshold = 0.5

class PersonFusion(Node):
    """ Single tracked person fusion between lidar and vision"""
    def __init__(self):
        super().__init__('kalman_filter')
        self.create_subscription(PersonArray, 'people_detected_vision', self.vision_callback, 10) # change this
        self.create_subscription(PersonArray, 'people_tracked', self.laser_callback, 10) # lidar_ws ngoài vision_leg_tracker
        self.people_fused_pub = self.create_publisher(FusedPersonArray, 'people_fused', 10)
        
        self.dist_travelled = 0.

        delta_t = 1./scan_frequency

        std_process_noise = 0.05 
        std_pos = std_process_noise
        std_vel = std_process_noise
        std_obs = 0.1  
        std_obs_vision = 0.0001
        var_pos = std_pos**2
        var_vel = std_vel**2
        var_obs_local = std_obs**2 
        self.var_obs = (std_obs + 0.4)**2

        self.filtered_state_means = np.array([0, 0, 0, 0]) # Initialize at origin
        self.pos_x = 0
        self.pos_y = 0
        self.vel_x = 0
        self.vel_y = 0

        self.pos_x_laser = 0
        self.pos_y_laser = 0

        self.pos_x_vision = 0
        self.pos_y_vision = 0

        self.filtered_state_covariances = 0.5*np.eye(4) 

        transition_matrix = np.array([[1, 0, delta_t,        0],
                                      [0, 1,       0,  delta_t],
                                      [0, 0,       1,        0],
                                      [0, 0,       0,        1]])

        observation_matrix = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0]])

        transition_covariance = np.array([[var_pos,       0,       0,       0],
                                          [      0, var_pos,       0,       0],
                                          [      0,       0, var_vel,       0],
                                          [      0,       0,       0, var_vel]])

        observation_covariance = var_obs_local*np.eye(2)

        self.kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            transition_covariance=transition_covariance,
            observation_covariance=observation_covariance,
        )


    def update(self, observations, observation_covariance):
        """Update tracked object with new observations"""

        if observation_covariance != []:  # Check for empty list
            self.filtered_state_means, self.filtered_state_covariances = self.kf.filter_update(
                filtered_state_mean = self.filtered_state_means,
                filtered_state_covariance = self.filtered_state_covariances,
                observation = observations,
                observation_covariance = observation_covariance
            )
        else:
            self.filtered_state_means, self.filtered_state_covariances = self.kf.filter_update(
                filtered_state_mean = self.filtered_state_means,
                filtered_state_covariance = self.filtered_state_covariances,
                observation = observations
            )

        self.pos_x = self.filtered_state_means[0]
        self.pos_y = self.filtered_state_means[1]
        self.vel_x = self.filtered_state_means[2]
        self.vel_y = self.filtered_state_means[3]

    def is_nearest(self, x_vision, y_vision, x_laser, y_laser, threshold): #so sánh distance pos hiện tại vs prev_pos or any pos
        """Check if detected persons from two data streams are aligned"""
        dis = ((x_vision - x_laser)**2 + (y_vision - y_laser)**2)**0.5
        return dis <= threshold

    def vision_update(self, pos_x, pos_y):
        """Update filter with vision data"""
        std_obs_vision = 0.0001
        var_obs_local = std_obs_vision**2 
        observation_covariance = var_obs_local*np.eye(2)
        self.pos_x_vision = pos_x
        self.pos_y_vision = pos_y
        observations = np.array([pos_x, pos_y])
        
        self.update(observations, observation_covariance)
        # self.get_logger().info(f"Update with vision information before: {pos_x}, {pos_y} after: {self.pos_x}, {self.pos_y}")


    def vision_callback(self, people):
        """Process vision data"""
        if people.people:
            person = people.people[0] # single person appear in the scence
            pos_x = person.pose.position.x
            pos_y = person.pose.position.y

            if self.is_nearest(pos_x, pos_y, self.pos_x_laser, self.pos_y_laser, distance_sources_threshold):
                self.vision_update(pos_x, pos_y)
                self.talker(people)


    def laser_update(self, pos_x, pos_y):
        """Update filter with lidar data"""
        self.pos_x_laser = pos_x
        self.pos_y_laser = pos_y
        observations = np.array([pos_x, pos_y])
        self.update(observations, [])  # Empty list for default covariance
        # self.get_logger().info(f"Update with laser information before: {pos_x}, {pos_y} after: {self.pos_x}, {self.pos_y}")


    def laser_callback(self, people):
        """Process lidar data""" 
        if people.people:
            for person in people.people:
                pos_x = person.pose.position.x
                pos_y = person.pose.position.y

                if self.pos_x_vision == 0 and self.pos_y_vision == 0:
                    if self.is_nearest(pos_x, pos_y, 0, 0, 2):
                        self.laser_update(pos_x, pos_y)
                        self.talker(people)
                else:
                    if self.is_nearest(pos_x, pos_y, self.pos_x_vision, self.pos_y_vision, distance_sources_threshold):
                        self.laser_update(pos_x, pos_y)
                        self.talker(people)
                        

    def talker(self, msg):
        """Publish fused person data"""
        pubPersonArray = FusedPersonArray()
        pubPersonArray.header = std_msgs.msg.Header()
        pubPersonArray.header.stamp = self.get_clock().now().to_msg()
        pubPersonArray.header.frame_id = msg.header.frame_id
        pubPersonArray.people = []

        pubPerson= FusedPerson()
        pubPerson.position.x = self.pos_x
        pubPerson.position.y = self.pos_y
        pubPerson.position.z = 0.0
        pubPerson.velocity.x = self.vel_x
        pubPerson.velocity.y = self.vel_y
        pubPerson.velocity.z = 0.0

        pubPersonArray.people.append(pubPerson)
        print("Publish 1 fused person location", pubPerson)

        self.people_fused_pub.publish(pubPersonArray)      

def main(args=None):
    rclpy.init(args=args)
    kf = PersonFusion()
    rclpy.spin(kf)
    kf.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
