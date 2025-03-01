#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import std_msgs.msg
import numpy as np
from scipy.optimize import linear_sum_assignment
import scipy.stats
import scipy.spatial
from geometry_msgs.msg import PointStamped, Point
import time

# Custom messages
from leg_detector_msgs.msg import Person, PersonArray, Leg, LegArray
from people_msgs.msg import People, MyPerson

from pykalman import KalmanFilter

toe2leg = 195.0
scan_frequency = 50
distance_sources_threshold = 1


class PersonFusion(Node):
    """Single tracked person fusion between lidar and vision"""

    def __init__(self):
        super().__init__("Kalman_Filter_Node")
        self.create_subscription(People, "/people_vision", self.vision_callback, 10)
        self.create_subscription(
            PersonArray, "/people_tracked", self.laser_callback, 10
        )
        self.people_fused_pub = self.create_publisher(
            People, "/people_fused", 10
        )

        self.dist_travelled = 0.0

        delta_t = 1.0 / scan_frequency

        std_process_noise = 0.05
        std_pos = std_process_noise
        std_vel = std_process_noise
        std_obs = 0.1

        var_pos = std_pos**2
        var_vel = std_vel**2
        var_obs_local = std_obs**2
        self.var_obs = (std_obs + 0.4) ** 2

        transition_matrix = np.array(
            [[1, 0, delta_t, 0], [0, 1, 0, delta_t], [0, 0, 1, 0], [0, 0, 0, 1]]
        )

        observation_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        transition_covariance = np.array(
            [
                [var_pos, 0, 0, 0],
                [0, var_pos, 0, 0],
                [0, 0, var_vel, 0],
                [0, 0, 0, var_vel],
            ]
        )

        observation_covariance = var_obs_local * np.eye(2)

        self.kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            transition_covariance=transition_covariance,
            observation_covariance=observation_covariance,
        )

        self.lidar_people = {}
        self.vision_people = {}
        self.people = {}
        self.vision_people_array = []
        self.lidar_people_array = []
        self.next_id = 0
        self.pose_threshold = 2.0

    def add_person(self, dict, person_id, pos_x, pos_y, orientation, activity):
        """Initialize a new person with a unique ID"""
        dict[person_id] = {
            "filtered_state_means": [pos_x, pos_y, 0.0, 0.0],
            "filtered_state_covariances": 0.5 * np.eye(4),
            "pos_x": pos_x,
            "pos_y": pos_y,
            "orientation_x": orientation[0],
            "orientation_y": orientation[1],
            "orientation_z": orientation[2],
            "orientation_w": orientation[3],
            "last_updated": time.time(),
            "activity": activity
        }

    def find_matching_person(self, dict, pos_x, pos_y):
        """Find an existing person within a certain distance threshold"""
        for person_id, data in dict.items():
            distance = np.linalg.norm(
                [data["pos_x"] - pos_x, data["pos_y"] - pos_y]
            )
            if distance < self.pose_threshold:
                return person_id  # Return existing person's ID if matched
        return None  # No match found

    def remove_stale_people(self, dict):
        """Remove people who haven't been updated in 3 seconds"""
        current_time = time.time()
        timeout = 3  # Time threshold (seconds)

        stale_people = [pid for pid, data in dict.items() 
                        if current_time - data["last_updated"] > timeout]

        for pid in stale_people:
            del dict[pid]  # Remove stale person
            print(f"Removed {pid} due to inactivity")

    def update(self, person_dict, person_id, observations, observation_covariance):
        """Update the Kalman filter for a tracked person."""
        if person_id not in person_dict:
            print(f"Person ID {person_id} not found in dictionary.")
            return

        person = person_dict[person_id]

        if "filtered_state_means" not in person:
            # Initialize Kalman state if not present
            person["filtered_state_means"] = np.array([observations[0], observations[1], 0.0, 0.0])
            person["filtered_state_covariances"] = self.filtered_state_covariances.copy()
        
        # Perform Kalman filter update
        if observation_covariance is not None:
            person["filtered_state_means"], person["filtered_state_covariances"] = (
                self.kf.filter_update(
                    filtered_state_mean=person["filtered_state_means"],
                    filtered_state_covariance=person["filtered_state_covariances"],
                    observation=observations,
                    observation_covariance=observation_covariance,
                )
            )
        else:
            person["filtered_state_means"], person["filtered_state_covariances"] = (
                self.kf.filter_update(
                    filtered_state_mean=person["filtered_state_means"],
                    filtered_state_covariance=person["filtered_state_covariances"],
                    observation=observations,
                )
            )
        
        # Update the person's estimated position and velocity
        person["pos_x"] = person["filtered_state_means"][0]
        person["pos_y"] = person["filtered_state_means"][1]
        person["vel_x"] = person["filtered_state_means"][2]
        person["vel_y"] = person["filtered_state_means"][3]
        
        # Update the last updated timestamp
        person["last_updated"] = time.time()


    def vision_callback(self, people):
        """Process vision data and track people"""
        self.vision_people_array = []  # Reset vision people array

        std_obs_vision = 0.0001
        var_obs_local = std_obs_vision**2
        observation_covariance = var_obs_local * np.eye(2)

        for pose in people.people:
            pos_x = pose.pose.position.x
            pos_y = pose.pose.position.y
            orientation = [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]

            print(pose.activity)
            # Check if this person already exists
            person_id = self.find_matching_person(self.vision_people, pos_x, pos_y)

            if person_id is None:
                # If not found, create a new person with a unique ID
                person_id = f"vision_{self.next_id}"
                self.next_id += 10
                self.add_person(self.vision_people, person_id, pos_x, pos_y, orientation, pose.activity)

            # Update the person's position and timestamp
            self.vision_people[person_id].update({
                "pos_x": pos_x,
                "pos_y": pos_y,
                "orientation_x": orientation[0],
                "orientation_y": orientation[1],
                "orientation_z": orientation[2],
                "orientation_w": orientation[3],
                "last_updated": time.time(),
                "activity": pose.activity
            })

            # Store for reference
            self.vision_people_array.append([pos_x, pos_y] + orientation)

            person = self.vision_people[person_id]

            self.update(self.vision_people, person_id, [person["pos_x"], person["pos_y"]], observation_covariance)
            
        self.remove_stale_people(self.vision_people)  # Remove old detections
        self.publish(self.vision_people)

    def laser_callback(self, people):
        """Process lidar data"""
        self.lidar_people_array = []
        if people.people:
            for person in people.people:
                pos_x = person.pose.position.x
                pos_y = person.pose.position.y

                # Check if this person already exists
                person_id = self.find_matching_person(self.lidar_people, pos_x, pos_y)

                if person_id is None:
                    # If not found, create a new person with a unique ID
                    person_id = f"lidar_{self.next_id}"
                    self.next_id += 10
                    
                    # Find closest matching vision person
                    matched_vision_id = self.find_matching_person(self.vision_people, pos_x, pos_y)

                    # Assign activity from vision person if a match is found, otherwise set to unknown (0)
                    activity = self.vision_people[matched_vision_id]["activity"] if matched_vision_id else 0

                    # Create new lidar person with activity from vision person
                    self.add_person(self.lidar_people, person_id, pos_x, pos_y, [0.0, 0.0, 0.0, 0.0], activity)


                # Update the person's position and timestamp
                self.lidar_people[person_id].update({
                    "pos_x": pos_x,
                    "pos_y": pos_y,
                    "last_updated": time.time(),
                })

                # Store for reference
                self.lidar_people_array.append([pos_x, pos_y])

                person = self.lidar_people[person_id]

                self.update(self.lidar_people, person_id, [person["pos_x"], person["pos_y"]], None)

        # print(self.lidar_people)

        self.remove_stale_people(self.lidar_people)  # Remove old detections
        self.publish(self.lidar_people)

    def publish(self, dict):
        """Publish fused people data"""
        fused_people_msg = People()
        fused_people_msg.header.stamp = self.get_clock().now().to_msg()
        fused_people_msg.header.frame_id = "base_link"  # Adjust based on your frame of reference

        id = 0

        for person_id, person in dict.items():
            fused_person = MyPerson()
            fused_person.pose.position.x = person["pos_x"]
            fused_person.pose.position.y = person["pos_y"]
            fused_person.pose.position.z = 0.0
            fused_person.pose.orientation.x = person["orientation_x"]
            fused_person.pose.orientation.y = person["orientation_y"]
            fused_person.pose.orientation.z = person["orientation_z"]
            fused_person.pose.orientation.w = person["orientation_w"]
            fused_person.velocity.x = person["vel_x"]
            fused_person.velocity.y = person["vel_y"]
            fused_person.velocity.z = 0.0
            fused_person.activity = person["activity"]
            fused_person.id = id

            id += 1

            fused_people_msg.people.append(fused_person)

        print(fused_people_msg)
        id = 0

        self.people_fused_pub.publish(fused_people_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
