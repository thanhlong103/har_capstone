import rclpy
import csv
import os
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')

        # Subscribe to /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Create CSV file
        self.csv_filename = os.path.join(os.path.expanduser('~'), 'odom_log.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'theta'])

        self.start_time = None  # Variable to store the first timestamp

        self.get_logger().info(f"Logging odometry data to {self.csv_filename}")

    def odom_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract orientation as quaternion
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # Convert quaternion to Euler angles
        _, _, theta = euler_from_quaternion(quaternion)

        # Get current timestamp (in seconds)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Set initial timestamp as 0
        if self.start_time is None:
            self.start_time = current_time
        
        relative_time = current_time - self.start_time

        # Write to CSV
        self.csv_writer.writerow([relative_time, x, y, theta])

        # Print to console (optional)
        self.get_logger().info(f"Saved: {relative_time:.4f}, {x:.4f}, {y:.4f}, {theta:.4f}")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
