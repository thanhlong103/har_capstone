#!/usr/bin/env python3
import rospy
from people_msgs.msg import People, Person
from leg_tracker.msg import PersonArray
import std_msgs
import numpy as np
from scipy.optimize import linear_sum_assignment
import scipy.stats
import scipy.spatial
from geometry_msgs.msg import PointStamped, Point
import time

# import random
# import math
# import tf
# import copy
# import timeit
# import message_filters
# import sys
# import csv

from pykalman import KalmanFilter 
tip2leg = 195.0
scan_frequency = 10

class PersonFusion:
    """ Single tracked person fusion between lidar and vision"""
    def __init__(self, x, y):
        self.people_vision_sub = rospy.Subscriber('people_detected_vision', People, self.vision_callback)
        self.people_laser_sub = rospy.Subscriber('people_tracked', PersonArray, self.laser_callback)
        self.people_fused_pub = rospy.Publisher('people_fused', People)
        
        self.dist_travelled = 0.

        # scan_frequency = rospy.get_param("scan_frequency", 7.5)
        delta_t = 1./scan_frequency

        std_process_noise = 0.05 # process noise get from lidar package
        std_pos = std_process_noise
        std_vel = std_process_noise
        std_obs = 0.1 # observation standard deviation 
        std_obs_vision = 0.0001
        var_pos = std_pos**2
        var_vel = std_vel**2
        var_obs_local = std_obs**2 
        self.var_obs = (std_obs + 0.4)**2

        self.filtered_state_means = np.array([x, y, 0, 0])
        self.pos_x = x
        self.pos_y = y
        self.vel_x = 0
        self.vel_y = 0

        self.pos_x_laser = 0
        self.pos_y_laser = 0

        self.pos_x_vision = 0
        self.pos_y_vision = 0

        self.filtered_state_covariances = 0.5*np.eye(4) 

        # Constant velocity motion model
        transition_matrix = np.array([[1, 0, delta_t,        0],
                                      [0, 1,       0,  delta_t],
                                      [0, 0,       1,        0],
                                      [0, 0,       0,        1]])

        # Oberservation model. Can observe pos_x and pos_y (unless person is occluded). 
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
        rospy.spin()
    
    def update(self, observations, observation_covariance):
        """
        Update our tracked object with new observations
        """
        if (observation_covariance != []):
            self.filtered_state_means, self.filtered_state_covariances = (
                self.kf.filter_update(
                    filtered_state_mean = self.filtered_state_means,
                    filtered_state_covariance = self.filtered_state_covariances,
                    observation = observations,
                    observation_covariance = observation_covariance
                )
            )
        else:
            self.filtered_state_means, self.filtered_state_covariances = (
                self.kf.filter_update(
                filtered_state_mean = self.filtered_state_means,
                filtered_state_covariance = self.filtered_state_covariances,
                observation = observations
                )
            )
        # delta_dist_travelled = ((self.pos_x - self.filtered_state_means[0])**2 + (self.pos_y - self.filtered_state_means[1])**2)**(1./2.) 
        # if delta_dist_travelled > 0.01: 
        #     self.dist_travelled += delta_dist_travelled

        self.pos_x = self.filtered_state_means[0]
        self.pos_y = self.filtered_state_means[1]
        self.vel_x = self.filtered_state_means[2]
        self.vel_y = self.filtered_state_means[3]

        # return [self.pos_x, self.pos_y, self.vel_x, self.vel_y]
    
    def is_nearest(self, x_vision, y_vision, x_laser, y_laser, threshold):
        """Check if the the detected person from two data streams are aligned"""
        dis = ((x_vision - x_laser)**2 + (y_vision - y_laser)**2)**(1./2.)
        # print(dis <= threshold)
        return dis <= threshold
    
    def vision_update(self, pos_x, pos_y):
        """Update filter with information from vision module"""
        std_obs_vision = 0.0001
        var_obs_local = std_obs_vision**2 
        observation_covariance = var_obs_local*np.eye(2)
        self.pos_x_vision = pos_x
        self.pos_y_vision = pos_y
        observations = np.array([pos_x, pos_y])
        
        self.update(observations, observation_covariance)
        print("Update with vision information", "before",pos_x, pos_y,"after", self.pos_x, self.pos_y)

    def vision_callback(self, people):
        """People information from vision package"""
        if len(people.people) !=0:
            # Require a good experiment setting with only one person appear so that the code can work well
            person = people.people[0] # single tracking
            pos_x = person.position.x
            pos_y = person.position.y

            # print(pos_x, pos_y, self.pos_x_laser, self.pos_y_laser)
            # Human information from vision module must be less than 0.3m from laser module
            if (self.is_nearest(pos_x, pos_y, self.pos_x_laser, self.pos_y_laser, 0.4)):
                self.vision_update(pos_x, pos_y)
                self.talker(people)

    def laser_update(self, pos_x, pos_y):
        """Update filter with information from lidar module"""
        self.pos_x_laser = pos_x
        self.pos_y_laser = pos_y
        observations = np.array([pos_x, pos_y])
        self.update(observations, []) 
        print("Update with laser information","before",pos_x, pos_y,"after", self.pos_x, self.pos_y)

    def laser_callback(self, people):
        """People information from lidar package"""
        if len(people.people) != 0:
            # Require a good experiment setting with only one person appear so that the code can work well

            # loop through detected person array
            for person in people.people:
                pos_x = person.pose.position.x
                pos_y = person.pose.position.y

                # initial step when no data from vision module
                if self.pos_x_vision == 0 and self.pos_y_vision == 0:
                    # only detected person within 2m radius is considered 
                    if self.is_nearest(pos_x, pos_y, 0, 0, 2):
                        self.laser_update(pos_x, pos_y)
                        self.talker(people) # publish the fused people estimation
                else:
                # or check if the data is near the latest detected human from vision module 
                    if self.is_nearest(pos_x, pos_y, self.pos_x_vision, self.pos_y_vision, 0.4):
                        self.laser_update(pos_x, pos_y)
                        self.talker(people) # publish the fused people estimation

    
    def talker(self, msg):
        """Publish the fused data of tracked person by lidar and vision"""
        pubPeople = People()
        pubPeople.header = std_msgs.msg.Header()
        pubPeople.header.stamp = rospy.Time.now()
        pubPeople.header.frame_id = msg.header.frame_id
        pubPeople.people = []

        pubPerson= Person()
        pubPerson.position.x = self.pos_x
        pubPerson.position.y = self.pos_y
        pubPerson.position.z = 0.0
        pubPerson.velocity.x = self.vel_x
        pubPerson.velocity.y = self.vel_y
        pubPerson.velocity.z = 0.0

        # x = self.pos_x
        # y = self.pos_y
        # dict = {'x': x, 'y': y}

        # # writing to csv file
            
        # # writing data rows
        # writer.writerow(dict)

        pubPerson.reliability = 0.8 #NEED MORE LIT-REVIEW TO UNDERSTAND THIS ATTRIBUTE FROM SOCIAL-LAYER PACKAGE
        pubPeople.people.append(pubPerson)

        rospy.sleep(0.01)
        self.people_fused_pub.publish(pubPeople)      

if __name__ == "__main__":
    rospy.init_node('kalman_filter', anonymous=True)
    kf = PersonFusion(0,0)

