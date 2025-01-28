import rclpy
from rclpy.node import Node
from people_msgs.msg import People, Person
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class PeoplePublisherNode(Node):
    def __init__(self):
        super().__init__('people_publisher_node')
        
        # Publisher for the /people topic
        self.people_publisher = self.create_publisher(People, '/people', 10)
        
        # Publisher for the visualization marker
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # Timer to publish the message at a fixed rate
        self.timer = self.create_timer(1.0, self.publish_people)

        # Tags to be sent with each Person
        self.tags = ['tag1', 'tag2', 'tag3']  # Modify as needed

        # Marker counter to ensure unique IDs for each marker
        self.marker_id = 0

    def publish_people(self):
        # Create a People message
        people_msg = People()

        # Set header frame_id to "map"
        people_msg.header = Header()
        people_msg.header.stamp = self.get_clock().now().to_msg()
        people_msg.header.frame_id = 'map'  # Set frame_id to "map"
        
        # Create a Person message
        person_msg = Person()
        person_msg.name = "John Doe"
        person_msg.position = Point(x=5.0, y=2.0, z=0.0)  # Example position
        person_msg.velocity = Point(x=0.15, y=0.15, z=0.0)
        person_msg.reliability = 0.95
        person_msg.tagnames = self.tags  # Add the tags
        
        # Add the person to the people array
        people_msg.people = [person_msg]
        
        # Publish the People message
        self.people_publisher.publish(people_msg)

        # Create a Marker for visualization
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.id = self.marker_id
        marker.type = Marker.SPHERE  # You can change this to CUBE, ARROW, etc.
        marker.action = Marker.ADD
        marker.pose.position = person_msg.position
        marker.pose.orientation.w = 1.0  # Default orientation (no rotation)
        marker.scale.x = 0.5  # Set scale of the sphere
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 0.0  # No blue
        marker.color.a = 1.0  # Full opacity

        # Add the marker to the MarkerArray
        marker_array = MarkerArray()
        marker_array.markers.append(marker)

        # Publish the MarkerArray to visualize the people in RViz
        self.marker_publisher.publish(marker_array)
        
        # Increment marker_id to avoid duplicates
        self.marker_id += 1
        
        self.get_logger().info(f'Publishing people message with tags: {self.tags}')


def main(args=None):
    rclpy.init(args=args)
    node = PeoplePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
