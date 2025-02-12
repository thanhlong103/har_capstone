import rclpy
from rclpy.node import Node
from people_msgs.msg import People, MyPerson
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
from people_msgs.msg import PeopleGroupArray, PeopleGroup, MyPerson  # Assuming correct package

class PeoplePublisher(Node):
    def __init__(self):
        super().__init__('people_publisher')
        
        # Publisher for the People message
        self.people_publisher = self.create_publisher(People, '/people', 10)
        
        # Publisher for the PeopleGroupArray message
        self.group_publisher = self.create_publisher(PeopleGroupArray, '/people_groups', 10)

        # Create a timer to publish data periodically
        self.timer = self.create_timer(1.0, self.publish_people)

        # Initialize the People message
        self.people_msg = People()
        
        # Create people data
        self.create_people()

    def create_people(self):
        # Example people
        self.people_msg.people = [
            MyPerson(
                pose=Pose(position=Point(x=5.0, y=9.5, z=0.0)),  # Corrected
                velocity=Point(x=1.0, y=-0.8, z=0.0),
                activity = 1,
            ),
            MyPerson(
                pose=Pose(position=Point(x=6.0, y=10.5, z=0.0)),  # Corrected
                velocity=Point(x=1.0, y=-2.0, z=0.0),
                activity = 1,
            ),
            MyPerson(
                pose=Pose(position=Point(x=5.5, y=7.5, z=0.0)),  # Corrected
                velocity=Point(x=0.1, y=0.1, z=0.0),
                activity = 1,
            )
        ]

    def publish_people(self):
        # Populate the header
        self.people_msg.header = Header()
        self.people_msg.header.frame_id = "map"
        self.people_msg.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info('Publishing people data')
        self.people_publisher.publish(self.people_msg)

        # Generate people groups and publish
        self.publish_people_groups()

    def publish_people_groups(self):
        group_msg = PeopleGroupArray()
        group_msg.header = self.people_msg.header

        # Example grouping logic: based on proximity
        group1 = PeopleGroup(id=1, people=[
            MyPerson(pose=self.people_msg.people[0].pose, velocity=self.people_msg.people[0].velocity, id=101),
            MyPerson(pose=self.people_msg.people[1].pose, velocity=self.people_msg.people[1].velocity, id=102),
            MyPerson(pose=self.people_msg.people[2].pose, velocity=self.people_msg.people[2].velocity, id=103)
        ], centroid=Point(x=7.0, y=7.0), activity = 1)
        
        # group2 = PeopleGroup(id=2, people=[
        #     FusedPerson(position=Pose(position=self.people_msg.people[2].position), velocity=self.people_msg.people[2].velocity, id=103),
        #     FusedPerson(position=Pose(position=self.people_msg.people[3].position), velocity=self.people_msg.people[3].velocity, id=104)
        # ], centroid=Point(x=5.0, y= 8.5))

        group_msg.groups = [group1]

        self.get_logger().info('Publishing people group data')
        self.group_publisher.publish(group_msg)


def main(args=None):
    rclpy.init(args=args)
    people_publisher = PeoplePublisher()
    rclpy.spin(people_publisher)
    people_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
