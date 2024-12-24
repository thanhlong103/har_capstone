import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math


class SocialCostmapNode(Node):
    def __init__(self):
        super().__init__("social_costmap_node")

        # Subscriber to 'personArray' topic
        self.subscription = self.create_subscription(
            PoseArray, "/person_coordinates", self.person_callback, 10
        )

        # Publisher for the costmap (OccupancyGrid)
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid, "/social_costmap", 10
        )

        # Publisher for visualization in RViz
        self.marker_pub = self.create_publisher(Marker, "/human_marker", 10)

    def person_callback(self, msg: PoseArray):
        # Create a new OccupancyGrid message
        costmap = OccupancyGrid()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = "camera_frame"

        # Define costmap size and resolution (for simplicity, set to 10x10 meters and 1m resolution)
        width = 100
        height = 100
        resolution = 0.05
        costmap.info.resolution = resolution
        costmap.info.width = width
        costmap.info.height = height
        costmap.info.origin.position.x = 2.5  # Origin at (-5, -5) meters
        costmap.info.origin.position.y = 2.5
        costmap.info.origin.orientation.w = 0.0

        # Initialize costmap to be free space (value 0)
        costmap.data = [0] * (width * height)

        # Define the robot's costmap radius and the robot's center
        sigma = 5.0  # 2 meters for the social distance zone
        for person in msg.poses:
            # Get the person's position (x, y)
            x = person.position.x
            y = person.position.y

            # Convert world coordinates to grid coordinates
            grid_x = -int((x - costmap.info.origin.position.x) / resolution)
            grid_y = -int((y - costmap.info.origin.position.y) / resolution)

            # Check if the grid coordinates are within bounds
            if 0 <= grid_x < width and 0 <= grid_y < height:
                # Apply Gaussian function to calculate cost based on distance
                for dx in range(-width//2, width//2):
                    for dy in range(-height//2, height//2):
                        # Compute distance from the current grid cell to the human's position
                        dist = math.sqrt((dx - grid_x)**2 + (dy - grid_y)**2)
                        cost = int(100 * math.exp(-0.5 * (dist**2) / (sigma**2)))
                        
                        # Update the costmap data for the grid cell
                        if 0 <= grid_x + dx < width and 0 <= grid_y + dy < height:
                            costmap.data[(grid_y + dy) * width + (grid_x + dx)] = min(100, max(costmap.data[(grid_y + dy) * width + (grid_x + dx)], cost))
        
        # Publish the costmap
        self.costmap_publisher.publish(costmap)


        # Publish the costmap
        self.costmap_publisher.publish(costmap)

        # Optionally, publish a marker to visualize the person's position
        self.publish_marker(msg.poses)

    def publish_marker(self, poses):
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "person_markers"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        for pose in poses:
            marker.points.append(pose.position)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SocialCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
