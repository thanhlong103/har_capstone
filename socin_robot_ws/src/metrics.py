import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from people_msgs.msg import People, PeopleGroupArray
import math 
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import tf_transformations
from scipy.stats import multivariate_normal
import csv
import time
import os

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
        self.create_subscription(PeopleGroupArray, '/people_groups', self.group_callback, 10)

        # Publisher for social cost metrics
        self.CI_pub = self.create_publisher(Float32, '/CI', 10)
        self.II_pub = self.create_publisher(Float32, '/II', 10)
        self.PI_pub = self.create_publisher(Float32, '/PI', 10)
        self.ADI_pub = self.create_publisher(Float32, '/ADI', 10)

        # Store robot and human states
        self.robot_position = (0.0, 0.0, 0.0)  # Now in MAP frame
        self.robot_velocity = (0.0, 0.0)
        self.humans = []  # List of (x, y, activity_type)
        self.groups = []

        # CSV File Setup
        self.csv_file = "social_cost_metrics.csv"
        self.initialize_csv()

        # Timer to compute metrics
        self.create_timer(0.1, self.compute_and_publish_metrics)  # 10 Hz

    def initialize_csv(self):
        """Creates CSV file if it doesn't exist and writes the header."""
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp", "CI", "II", "PI", "ADI", "Robot_X", "Robot_Y", "Robot_Theta"])

    def log_metrics(self, CI, II, PI, ADI):
        """Appends computed metrics and robot position to the CSV file."""
        timestamp = time.time()
        x_r, y_r, theta_r = self.robot_position  # Get robot position

        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, CI, II, PI, ADI, x_r, y_r, theta_r])

    def robot_pose_callback(self, msg):
        """Transforms the robot position from /odom to /map frame."""
        try:
            # Lookup the transform from /odom to /map
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())

            # Transform robot position
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, transform)
            quaternion = (
                transformed_pose.orientation.x,
                transformed_pose.orientation.y,
                transformed_pose.orientation.z,
                transformed_pose.orientation.w
            )

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
            
            self.robot_position = (transformed_pose.position.x, transformed_pose.position.y, yaw)

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
        
    def group_callback(self, msg):
        self.groups = [(group.centroid.x, group.centroid.y, group.radius, group.activity) for group in msg.groups]
        # print(self.groups)

    def compute_CI(self):
        CI = 0
        for (x_h, y_h, _) in self.humans:
            x_r, y_r, _ = self.robot_position
            sigma_px0, sigma_py0 = 0.45, 0.45  # Standard deviations for personal space
            distance_factor = ((x_r - x_h)**2 / (2 * sigma_px0**2)) + ((y_r - y_h)**2 / (2 * sigma_py0**2))
            CI = max(CI, np.exp(-distance_factor))
        return CI

    def compute_II(self):
        II = 0
        for (x_g, y_g, radius, _) in self.groups:
            x_r, y_r, _ = self.robot_position
            sigma_px0, sigma_py0 = radius/5, radius/5  # Standard deviations for personal space
            distance_factor = (((x_r - x_g)/(math.sqrt(2)*sigma_px0))**2 ) + (((y_r - y_g)/(math.sqrt(2)*sigma_py0))**2 )
            II = max(II, np.exp(-distance_factor))
        return II
    
    def compute_extended_personal_space(self, x, y, sigma_x, sigma_y):
        """Creates Gaussian distribution parameters for EPS."""
        return {
            'mean': [x, y],
            'cov': [[sigma_x**2, 0], [0, sigma_y**2]],
            'x_min': x - 2 * sigma_x,
            'x_max': x + 2 * sigma_x,
            'y_min': y - 2 * sigma_y,
            'y_max': y + 2 * sigma_y
        }

    def compute_overlap(self, S_pi, S_r):
        """Compute the overlap between human and robot EPS areas."""
        resolution = 0.05  # Grid resolution
        x_min, x_max = min(S_pi['x_min'], S_r['x_min']), max(S_pi['x_max'], S_r['x_max'])
        y_min, y_max = min(S_pi['y_min'], S_r['y_min']), max(S_pi['y_max'], S_r['y_max'])

        x_range, y_range = np.arange(x_min, x_max, resolution), np.arange(y_min, y_max, resolution)
        X, Y = np.meshgrid(x_range, y_range)
        grid_points = np.column_stack([X.ravel(), Y.ravel()])

        # Compute probability density functions (PDFs)
        rv_pi = multivariate_normal(mean=S_pi['mean'], cov=S_pi['cov'])
        rv_r = multivariate_normal(mean=S_r['mean'], cov=S_r['cov'])

        prob_pi = rv_pi.pdf(grid_points).reshape(X.shape)
        prob_r = rv_r.pdf(grid_points).reshape(X.shape)

        # Compute overlapping area
        overlap_area = np.sum(np.minimum(prob_pi, prob_r)) * (resolution ** 2)
        return overlap_area

    def compute_PI(self):
        """Calculates the Psychological Index (PI) based on overlap areas."""
        if self.robot_position is None or self.robot_velocity is None:
            self.get_logger().warn("Waiting for robot position and velocity data...")
            return
        
        x_r, y_r, theta_r = self.robot_position
        v_r = np.linalg.norm(self.robot_velocity)  # Compute robot speed
        
        # Adjust robot's EPS sigma based on velocity (higher velocity = larger EPS)
        sigma_r = 0.45 * (1 + v_r)  

        # Compute robot's extended personal space (EPS)
        S_r = self.compute_extended_personal_space(x_r, y_r, sigma_r, sigma_r)

        total_overlap = 0.0
        S_max = np.pi * (2 * sigma_r) ** 2  # Maximum possible robot EPS area

        for (x_p, y_p, _) in self.humans:
            # Compute human's EPS
            S_pi = self.compute_extended_personal_space(x_p, y_p, 0.45, 0.45)
            overlap = self.compute_overlap(S_pi, S_r)
            total_overlap += overlap

        # Normalize PI value between 0 and 1
        PI = total_overlap / S_max if S_max > 0 else 0.0

        return PI

    def compute_ADI(self):
        ADI_person = 0
        ADI_group = 0
        for (x_h, y_h, activity) in self.humans:
            W_a = self.get_activity_weight(activity)
            sigma_ax, sigma_ay = W_a * 0.45, W_a * 0.45
            distance_factor = ((self.robot_position[0] - x_h)**2 / ((2 * sigma_ax)**2)) + \
                              ((self.robot_position[1] - y_h)**2 / ((2 * sigma_ay)**2))
            ADI_person = max(ADI_person, np.exp(-distance_factor))

        for (x_g, y_g, radius, activity) in self.groups:
            W_a = self.get_activity_weight(activity)
            sigma_ax, sigma_ay = W_a * radius / 6, W_a * radius / 6
            distance_factor = ((self.robot_position[0] - x_g)**2 / ((2 * sigma_ax)**2)) + \
                              ((self.robot_position[1] - y_g)**2 / ((2 * sigma_ay)**2))
            ADI_group = max(ADI_group, np.exp(-distance_factor))

        if ADI_person > ADI_group:
            return ADI_person
        else:
            return ADI_group
  
    def get_activity_weight(self, activity):
        weights = {0: 1.0, 1: 1.2, 2: 1.0, 3: 0.9, 4: 1.0, 5: 1.0, 6: 1.1, 7: 2.0}
        return weights.get(activity, 1.0)

    def compute_and_publish_metrics(self):
        CI = float(self.compute_CI())
        II = float(self.compute_II())
        PI = float(self.compute_PI())
        ADI = float(self.compute_ADI())

        self.log_metrics(CI, II, PI, ADI)

        CI_msg = Float32()
        II_msg = Float32()
        PI_msg = Float32()
        ADI_msg = Float32()

        CI_msg.data = CI
        II_msg.data = II
        PI_msg.data = PI
        ADI_msg.data = ADI
        self.CI_pub.publish(CI_msg)
        self.II_pub.publish(II_msg)
        self.PI_pub.publish(PI_msg)
        self.ADI_pub.publish(ADI_msg)

        self.get_logger().info(f"Metrics: CI={CI:.2f}, II={II:.2f}, PI={PI:.2f}, ADI={ADI:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SocialCostCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()