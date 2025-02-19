import rclpy
from rclpy.node import Node
from people_msgs.msg import People, MyPerson
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
from people_msgs.msg import PeopleGroupArray, PeopleGroup, MyPerson  # Assuming correct package
import threading
import sys
import termios
import tty

class PeoplePublisher(Node):
    def __init__(self):
        super().__init__('people_publisher')
        
        # Publisher for the People message
        self.people_publisher = self.create_publisher(People, '/people', 10)

        # Create a timer to publish data periodically
        self.timer = self.create_timer(1.0, self.publish_people)

        self.static_people = [
            MyPerson(
                pose=Pose(position=Point(x=3.5, y=8.5, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            ),
            MyPerson(
                pose=Pose(position=Point(x=6.0, y=10.5, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            ),
            MyPerson(
                pose=Pose(position=Point(x=9.5, y=6.3, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            ),
            MyPerson(
                pose=Pose(position=Point(x=13.8, y=4.7, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            ),
            MyPerson(
                pose=Pose(position=Point(x=14.5, y=7.5, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            ),
            MyPerson(
                pose=Pose(position=Point(x=17.7, y=7.0, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            ),
            MyPerson(
                pose=Pose(position=Point(x=19.9, y=2.5, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            ),
            MyPerson(
                pose=Pose(position=Point(x=20.7, y=11.8, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            ),
            MyPerson(
                pose=Pose(position=Point(x=13.65, y=11.0, z=0.0)),  # Corrected
                velocity=Point(x=0.0, y=0.0, z=0.0),
                activity = 0,
            )
        ]

        # Define moving people with initial positions
        self.moving_people = {
            'A': MyPerson(pose=Pose(position=Point(x=1.0, y=9.0, z=0.0)), velocity=Point(x=0.0, y=-0.5, z=0.0), activity=0),
            'S': MyPerson(pose=Pose(position=Point(x=4.0, y=0.5, z=0.0)), velocity=Point(x=1.0, y=0.0, z=0.0), activity=0),
            'D': MyPerson(pose=Pose(position=Point(x=22.5, y=4.0, z=0.0)), velocity=Point(x=0.0, y=0.5, z=0.0), activity=0),
            'F': MyPerson(pose=Pose(position=Point(x=23.5, y=4.0, z=0.0)), velocity=Point(x=0.0, y=0.5, z=0.0), activity=0),
        }

        # Movement state for each moving person (True = moving, False = stationary)
        self.movement_state = {key: False for key in self.moving_people}

        # Start key listener in a separate thread
        self.listener_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.listener_thread.start()

    def key_listener(self):
        """ Listens for key presses to toggle movement of specific people. """
        while True:
            key = self.get_key()
            if key in self.movement_state:
                self.movement_state[key] = not self.movement_state[key]
                state = "moving" if self.movement_state[key] else "stopped"
                self.get_logger().info(f"Person {key} is now {state}")

    def get_key(self):
        """ Reads a single keypress from stdin without blocking. """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key.upper()  # Convert to uppercase for consistency

    def update_positions(self):
        """ Updates positions of moving people. """
        for key, person in self.moving_people.items():
            if self.movement_state[key]:  # Move only if the person is active
                person.pose.position.x += person.velocity.x
                person.pose.position.y += person.velocity.y

                # Optional: Reverse direction at boundary
                if person.pose.position.x > 10 or person.pose.position.x < 0:
                    person.velocity.x *= -1
                if person.pose.position.y > 10 or person.pose.position.y < 0:
                    person.velocity.y *= -1

    def publish_people(self):
        """ Publishes both static and moving people to the /people topic. """
        self.update_positions()

        # Populate the header
        people_msg = People()
        people_msg.header = Header()
        people_msg.header.frame_id = "map"
        people_msg.header.stamp = self.get_clock().now().to_msg()

        # Combine static and moving people
        people_msg.people = self.static_people + list(self.moving_people.values())

        print(people_msg)

        self.get_logger().info(f'Publishing {len(people_msg.people)} people')
        self.people_publisher.publish(people_msg)

def main(args=None):
    rclpy.init(args=args)
    people_publisher = PeoplePublisher()
    rclpy.spin(people_publisher)
    people_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()