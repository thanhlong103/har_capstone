#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import std_msgs.msg
import numpy as np
from scipy.optimize import linear_sum_assignment
import scipy.stats
import scipy.spatial
from geometry_msgs.msg import PointStamped, Point
import time
from pykalman import KalmanFilter

scan_frequency = 50


class PersonFusion(Node):
    """People position fusion between lidar and vision"""

    def __init__(self):
        super().__init__("kalman_filter")
        self.create_subscription(
            PoseArray, "people_vision_pos", self.vision_callback, 10
        ) 
        # self.create_subscription(PoseArray, "people_lidar_pos", self.laser_callback, 10)
        self.people_fused_pub = self.create_publisher(PoseArray, "people_fused", 10)
        
    def vision_callback(self, msg):
        # Print the entire message or the specific data you want to check
        self.get_logger().info(f"Received {len(msg.poses)} people from vision.")

        for i, pose in enumerate(msg.poses):
            self.get_logger().info(
                f"Person {i + 1}: Position - x: {pose.position.x}, y: {pose.position.y}, theta: {pose.orientation.z}"
            )

            """Publish fused person data"""
            pubPersonArray = PoseArray()
            pubPersonArray.header = std_msgs.msg.Header()
            pubPersonArray.header.stamp = self.get_clock().now().to_msg()
            pubPersonArray.header.frame_id = msg.header.frame_id
            # pubPersonArray.people = []

            pubPerson= Pose()
            pubPerson.position.x = pose.position.x
            pubPerson.position.y = pose.position.y
            pubPerson.position.z = pose.position.z
            pubPerson.orientation.x = pose.orientation.x
            pubPerson.orientation.y = pose.orientation.y
            pubPerson.orientation.z = pose.orientation.z
            pubPerson.orientation.w = pose.orientation.w

            pubPersonArray.poses.append(pubPerson)
            print("Publish fused people location", pubPerson)

        self.people_fused_pub.publish(pubPersonArray)    

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
