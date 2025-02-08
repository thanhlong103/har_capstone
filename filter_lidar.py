import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class FilteredScanNode(Node):
    def __init__(self):
        super().__init__('filtered_scan_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, '/filtered_scan', 10)

    def scan_callback(self, msg):
        filtered_msg = msg  # Copy the original message

        # Convert to NaN-safe NumPy array
        ranges_array = np.array(msg.ranges)

        angle_min = msg.angle_min  
        angle_increment = msg.angle_increment

        # Define 20-degree cut range around 180° (pi)
        cut_angle = np.pi  # 180 degrees in radians
        cut_range = np.deg2rad(10)  # 10 degrees in radians

        start_index = int((cut_angle - cut_range - angle_min) / angle_increment)
        end_index = int((cut_angle + cut_range - angle_min) / angle_increment)

        # Set the selected range to NaN
        ranges_array[start_index:end_index] = np.nan

        # Assign modified ranges back to the message
        filtered_msg.ranges = ranges_array.tolist()

        # Publish filtered scan
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FilteredScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
