import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import math

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
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        self.centroid = None  # Default: No centroid received yet
        self.radius = None

    def people_groups_callback(self, msg):
        """Callback function for /people_groups subscription"""
        self.centroid = msg.groups[0].centroid
        self.radius = msg.groups[0].radius + 0.25
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
        radius = self.radius  # Adjust as needed
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

def main(args=None):
    rclpy.init(args=args)
    node = CircleMarkerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
