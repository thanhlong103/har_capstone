import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
from people_msgs.msg import People, Person
from fused_people_msgs.msg import FusedPersonArray, FusedPerson  # Replace with actual package name

class PeopleFusedConverter(Node):
    def __init__(self):
        super().__init__('people_fused_converter')
        self.subscription = self.create_subscription(
            FusedPersonArray,
            '/people_fused',
            self.people_fused_callback,
            10)
        self.publisher = self.create_publisher(People, '/people', 10)
        
        print("START")
    def people_fused_callback(self, msg):
        people_msg = People()
        people_msg.header.stamp = self.get_clock().now().to_msg()
        people_msg.header.frame_id = 'map'
        print(msg)
        for fused_person in msg.people:
            person = Person()
            person.name = f'Person_{fused_person.id}'
            person.position = Point(
                x=fused_person.position.position.x + 5.0,
                y=fused_person.position.position.y + 2.0,
                z=fused_person.position.position.z)
            person.velocity = fused_person.velocity
            person.reliability = 1.0  # Assuming perfect reliability, adjust as needed
            person.tagnames = ["Tag1"]  # Placeholder, update if needed
            person.tags = []  # Placeholder, update if needed
            
            people_msg.people.append(person)
        
        self.publisher.publish(people_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PeopleFusedConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
