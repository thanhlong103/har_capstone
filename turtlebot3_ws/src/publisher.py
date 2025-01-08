#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from people_msgs.msg import People, Person


class PeoplePublisher(Node):
    def __init__(self):
        super().__init__("people_publisher")
        self.publisher = self.create_publisher(People, "/people", 10)
        self.timer = self.create_timer(1.0, self.publish_people)
        self.get_logger().info("People publisher node has started.")

    def publish_people(self):
        # Create a People message
        people_msg = People()
        people_msg.header.stamp = self.get_clock().now().to_msg()
        people_msg.header.frame_id = "base_link"  # Adjust frame as needed

        # Add people to the message
        person_1 = Person()
        person_1.name = "Person1"
        person_1.position.x = 1.0
        person_1.position.y = 0.5
        person_1.position.z = 0.0
        person_1.velocity.x = 0.1
        person_1.velocity.y = 0.0
        person_1.velocity.z = 0.0
        person_1.reliability = 0.9

        person_2 = Person()
        person_2.name = "Person2"
        person_2.position.x = -1.0
        person_2.position.y = -0.5
        person_2.position.z = 0.0
        person_2.velocity.x = 0.0
        person_2.velocity.y = -0.1
        person_2.velocity.z = 0.0
        person_2.reliability = 0.85

        # Add the Person messages to the People message
        people_msg.people.append(person_1)
        people_msg.people.append(person_2)

        # Publish the message
        self.publisher.publish(people_msg)
        self.get_logger().info(
            f"Published People message with {len(people_msg.people)} people."
        )


def main(args=None):
    rclpy.init(args=args)
    node = PeoplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
