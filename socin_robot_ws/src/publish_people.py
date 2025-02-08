import rclpy
from rclpy.node import Node
from people_msgs.msg import People, Person
from geometry_msgs.msg import Point
from std_msgs.msg import Header

class PeoplePublisher(Node):
    def __init__(self):
        super().__init__('people_publisher')
        
        # Publisher for the People message
        self.publisher_ = self.create_publisher(People, '/people', 10)

        # Create a timer to publish data periodically
        self.timer = self.create_timer(1.0, self.publish_people)

        # Initialize the People message
        self.people_msg = People()
        
        # Create two people (as an example)
        self.create_people()

    def create_people(self):
        # Person 1
        person1 = Person()
        person1.name = "Person 1"
        person1.position = Point(x=1.0, y=2.0, z=0.0)
        person1.velocity = Point(x=0.1, y=0.1, z=0.0)
        person1.reliability = 0.95
        person1.tagnames = ["tag1", "tag2"]
        person1.tags = ["tag_value_1", "tag_value_2"]
        
        # Person 2
        person2 = Person()
        person2.name = "Person 2"
        person2.position = Point(x=3.0, y=4.0, z=0.0)
        person2.velocity = Point(x=-0.1, y=-0.1, z=0.0)
        person2.reliability = 0.90
        person2.tagnames = ["tag3", "tag4"]
        person2.tags = ["tag_value_3", "tag_value_4"]
        
        # Add people to the people message
        self.people_msg.people = [person1, person2]

    def publish_people(self):
        # Populate the header (set the timestamp to the current time)
        self.people_msg.header = Header()
        self.people_msg.header.frame_id = "map"
        self.people_msg.header.stamp = self.get_clock().now().to_msg()

        # Log publishing
        self.get_logger().info('Publishing people data')

        # Publish the people data
        self.publisher_.publish(self.people_msg)

def main(args=None):
    rclpy.init(args=args)

    people_publisher = PeoplePublisher()

    # Spin the node to keep it running and publishing
    rclpy.spin(people_publisher)

    # Shutdown ROS when done
    people_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
