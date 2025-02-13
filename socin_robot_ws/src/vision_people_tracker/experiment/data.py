import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float32
import math
import csv
import os

# Assuming the /people_groups topic publishes a message with centroid as a Point
from people_msgs.msg import PeopleGroupArray  # Change this to the actual message type

class CircleMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('circle_marker_subscriber')

        # Subscribe to /people_groups topic
        self.subscription = self.create_subscription(
            PeopleGroupArray,  # Replace with actual message type
            '/people_groups',
            self.people_groups_callback,
            10
        )

        self.tracker_runtime_subscription = self.create_subscription(
            Float32,  # Replace with actual message type
            '/tracker_runtime',
            self.tracker_runtime_callback,
            10
        )

        self.group_runtime_subscription = self.create_subscription(
            Float32,  # Replace with actual message type
            '/group_runtime',
            self.tracker_runtime_callback,
            10
        )
        
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        self.centroid = None  # Default: No centroid received yet
        self.radius = None
        self.tracker_runtime = 0.0
        self.group_runtime = 0.0

        # Open CSV file for logging
        self.csv_filename = "people_groups_data.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header to CSV
        self.csv_writer.writerow(["Timestamp", "Group Index", "Centroid X", "Centroid Y", "Group Radius", "Person Index", "Person X", "Person Y", "Runtime"])

    def tracker_runtime_callback(self, msg):
        """Update tracker runtime value."""
        self.tracker_runtime = msg.data

    def group_runtime_callback(self, msg):
        """Update group runtime value."""
        self.group_runtime = msg.data

    def people_groups_callback(self, msg):
        """Callback function for /people_groups subscription"""
        timestamp = self.get_clock().now().to_msg().sec  # Get timestamp

        runtime = self.tracker_runtime + self.group_runtime

        for group_idx, group in enumerate(msg.groups):
            self.centroid = group.centroid
            self.radius = group.radius + 0.25

            # Save centroid, group radius, and each person's position
            for person_idx, person in enumerate(group.people):
                self.csv_writer.writerow([
                    timestamp, group_idx, self.centroid.x, self.centroid.y,
                    group.radius, person_idx, person.pose.position.x, person.pose.position.y, runtime
                ])

            self.get_logger().info(f"Logged group {group_idx} with centroid at ({self.centroid.x}, {self.centroid.y}, {self.centroid.z})")

        self.publish_circle_marker()

    def publish_circle_marker(self):
        if self.centroid is None:
            self.get_logger().warn("No centroid received yet.")
            return

        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_laser"  # Adjust to your TF frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "circle"
        marker.id = 0   
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Circle parameters
        radius = self.radius
        num_points = 36  # Increase for smoother circle

        # Generate circle points centered at the centroid
        for i in range(num_points + 1):  # Extra point to close the loop
            angle = 2 * math.pi * i / num_points
            point = Point()
            point.x = self.centroid.x + radius * math.cos(angle)
            point.y = self.centroid.y + radius * math.sin(angle)
            point.z = self.centroid.z  # Assuming it's on the same Z level
            marker.points.append(point)

        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0
        marker.color.b = 1.0

        self.publisher.publish(marker)
        self.get_logger().info(f"Published circle marker at ({self.centroid.x}, {self.centroid.y}, {self.centroid.z})")

    def destroy_node(self):
        """Ensure the CSV file is closed when the node shuts down."""
        self.csv_file.close()
        self.get_logger().info(f"CSV file {self.csv_filename} saved successfully.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CircleMarkerSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
