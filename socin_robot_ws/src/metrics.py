import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from people_msgs.msg import People
import math 
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

class SocialCostCalculator(Node):
    def __init__(self):
        super().__init__('social_cost_calculator')

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to robot position, velocity, and human detections
        self.create_subscription(Odometry, '/odom', self.robot_pose_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.robot_velocity_callback, 10)
        self.create_subscription(People, '/people', self.human_detections_callback, 10)

        # Publisher for social cost metrics
        self.social_cost_pub = self.create_publisher(Float32MultiArray, '/social_cost_metrics', 10)

        # Store robot and human states
        self.robot_position = (0.0, 0.0)  # Now in MAP frame
        self.robot_velocity = (0.0, 0.0)
        self.humans = []  # List of (x, y, activity_type)

        # Timer to compute metrics
        self.create_timer(0.1, self.compute_and_publish_metrics)  # 10 Hz

    def robot_pose_callback(self, msg):
        """Transforms the robot position from /odom to /map frame."""
        try:
            # Lookup the transform from /odom to /map
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())

            # Transform robot position
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, transform)
            self.robot_position = (transformed_pose.position.x, transformed_pose.position.y)

        except tf2_ros.LookupException:
            self.get_logger().warn("TF LookupException: Waiting for transform from /odom to /map")
        except tf2_ros.ConnectivityException:
            self.get_logger().warn("TF ConnectivityException")
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("TF ExtrapolationException")

    def robot_velocity_callback(self, msg):
        self.robot_velocity = (msg.linear.x, msg.linear.y)

    def human_detections_callback(self, msg):
        """Stores human positions (which are already in /map frame)."""
        self.humans = [(h.pose.position.x, h.pose.position.y, h.activity) for h in msg.people]

    def compute_CI(self):
        CI = 0
        for (x_h, y_h, _) in self.humans:
            x_r, y_r = self.robot_position
            sigma_px0, sigma_py0 = 0.45, 0.6  # Standard deviations for personal space
            distance_factor = ((x_r - x_h)**2 / (2 * sigma_px0**2)) + ((y_r - y_h)**2 / (2 * sigma_py0**2))
            CI = max(CI, np.exp(-distance_factor))
        return CI

    def compute_II(self):
        II = 0
        for (x_h, y_h, activity) in self.humans:
            if activity in [1, 2, 3]:  # Assuming activity codes for talking, working, group_interaction
                sigma_ox, sigma_oy = 0.5, 0.5
                distance_factor = ((self.robot_position[0] - x_h)**2 / (2 * sigma_ox**2)) + \
                                  ((self.robot_position[1] - y_h)**2 / (2 * sigma_oy**2))
                II = max(II, np.exp(-distance_factor))
        return II

    def compute_PI(self):
        PI = 0
        for (x_h, y_h, _) in self.humans:
            overlap = self.compute_overlap(x_h, y_h)
            PI = max(PI, overlap)  # Take the maximum overlap value
        return PI


    def compute_ADI(self):
        ADI = 0
        for (x_h, y_h, activity) in self.humans:
            W_a = self.get_activity_weight(activity)
            sigma_ax, sigma_ay = self.get_activity_spread(activity)
            distance_factor = ((self.robot_position[0] - x_h)**2 / (2 * sigma_ax**2)) + \
                              ((self.robot_position[1] - y_h)**2 / (2 * sigma_ay**2))
            ADI = max(ADI, W_a * np.exp(-distance_factor))
        return ADI

    def compute_robot_extended_space(self):
        sigma_rx, sigma_ry = 0.6, 0.8  # Robot's influence area parameters
        return np.pi * sigma_rx * sigma_ry

    def compute_human_extended_space(self, x_h, y_h):
        sigma_hx, sigma_hy = 0.45, 0.6  # Human's personal space parameters
        return np.pi * sigma_hx * sigma_hy

    def compute_overlap(self, x_h, y_h):
        # Robot and human space parameters (std deviations)
        sigma_rx, sigma_ry = 0.6, 0.8  # Robot space
        sigma_hx, sigma_hy = 0.45, 0.6  # Human space

        # Robot position
        x_r, y_r = self.robot_position

        # Compute Mahalanobis distance
        D_M2 = ((x_r - x_h)**2 / (sigma_rx**2 + sigma_hx**2)) + \
            ((y_r - y_h)**2 / (sigma_ry**2 + sigma_hy**2))

        # Compute Gaussian-based overlap
        return math.exp(-D_M2)

    def get_activity_weight(self, activity):
        weights = {0: 0.5, 1: 0.8, 2: 0.9, 3: 1.0}
        return weights.get(activity, 0.5)

    def get_activity_spread(self, activity):
        spreads = {0: (0.5, 0.5), 1: (1.0, 1.0), 2: (1.2, 1.2), 3: (1.5, 1.5)}
        return spreads.get(activity, (0.5, 0.5))

    def compute_and_publish_metrics(self):
        CI = float(self.compute_CI())
        II = float(self.compute_II())
        PI = float(self.compute_PI())
        ADI = float(self.compute_ADI())

        msg = Float32MultiArray()
        msg.data = [CI, II, PI, ADI]
        self.social_cost_pub.publish(msg)

        self.get_logger().info(f"Metrics: CI={CI:.2f}, II={II:.2f}, PI={PI:.2f}, ADI={ADI:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SocialCostCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()