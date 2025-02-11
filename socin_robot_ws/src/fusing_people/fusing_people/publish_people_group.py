import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, PoseArray
from std_msgs.msg import Header
from fused_people_msgs.msg import FusedPerson, FusedPersonArray  # Replace 'custom_msgs' with your actual package name
import math
import random

def calculate_facing_orientation(px, py, cx, cy):
    """
    Calculate orientation (yaw) to face approximately towards the center point (cx, cy) from (px, py)
    with a random offset of ±10 degrees.
    """
    angle = math.atan2(cy - py, cx - px)
    angle += math.radians(random.uniform(-10, 10))  # Add random deviation
    return angle

class GroupPublisher(Node):
    def __init__(self, num_people=3):
        super().__init__('group_facing_publisher')
        self.publisher_ = self.create_publisher(FusedPersonArray, 'people_fused', 10)
        self.pose_publisher_ = self.create_publisher(PoseArray, 'fused_people_poses', 10)
        self.timer = self.create_timer(1.0, self.publish_group)
        self.num_people = num_people

    def publish_group(self):
        center_x, center_y = 4.0, 5.0  # Center point
        radius = 1.0  # Distance from center
        
        people_positions = [
            (
                center_x + radius * math.cos(2 * math.pi * i / self.num_people),
                center_y + radius * math.sin(2 * math.pi * i / self.num_people)
            ) for i in range(self.num_people)
        ]
        
        people_array = FusedPersonArray()
        people_array.header = Header()
        people_array.header.stamp = self.get_clock().now().to_msg()
        people_array.header.frame_id = 'laser_frame'

        pose_array = PoseArray()
        pose_array.header = people_array.header
        
        people_array.people = []
        for i, (px, py) in enumerate(people_positions):
            person = FusedPerson()
            person.id = i + 1
            
            person.position.position.x = px
            person.position.position.y = py
            person.position.position.z = 0.0
            person.velocity.x = 1.0
            person.velocity.y = 1.0
            
            yaw = calculate_facing_orientation(px, py, center_x, center_y)
            person.position.orientation.z = math.sin(yaw / 2.0)
            person.position.orientation.w = math.cos(yaw / 2.0)
            
            person.velocity = Point(x=0.0, y=0.0, z=0.0)

            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = 0.0
            pose.orientation.z = person.position.orientation.z
            pose.orientation.w = person.position.orientation.w
            pose_array.poses.append(pose)
            
            people_array.people.append(person)
        
        self.publisher_.publish(people_array)
        self.pose_publisher_.publish(pose_array)
        self.get_logger().info(f"Published {self.num_people} people facing approximately towards center")


def main(args=None):
    rclpy.init(args=args)
    num_people = 3  # Change this number to adjust the number of people
    node = GroupPublisher(num_people=num_people)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
