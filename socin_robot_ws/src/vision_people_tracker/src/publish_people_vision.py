import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, PoseArray, TransformStamped
from std_msgs.msg import Header
import math
import random
import tf_transformations as tf
from tf2_ros import TransformBroadcaster
# from leg_detector_msgs.msg import Person, PersonArray
import time

class PeopleVisionPublisher(Node):
    def __init__(self, num_people=6):
        super().__init__('people_vision_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/people_vision', 10)
        # self.lidar_publisher_ = self.create_publisher(PersonArray, '/people_tracked', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.publish_pose_array)
        self.num_people = num_people

    def calculate_facing_orientation(self, px, py, cx, cy):
        """
        Calculate orientation (yaw) to face approximately towards the center point (cx, cy) from (px, py)
        with a random offset of ±20 degrees.
        """
        angle = math.atan2(cy - py, cx - px)
        angle += math.radians(random.uniform(-10, 10))  # Add random deviation
        return angle

    def publish_pose_array(self):
        center_x, center_y = 0.0, 0.0  # Center point
        radius = random.uniform(0.95,1.05)  # Distance from center
        # radius = 1.0

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'base_laser'

        # lidar_pose_array = PersonArray()
        # lidar_pose_array.header = Header()
        # lidar_pose_array.header.stamp = self.get_clock().now().to_msg()
        # lidar_pose_array.header.frame_id = 'map'

        for i in range(self.num_people):
            px = center_x + radius * math.cos(2 * math.pi * i / self.num_people)
            py = center_y + radius * math.sin(2 * math.pi * i / self.num_people)

            px += random.uniform(-0.1, 0.1)
            py += random.uniform(-0.1, 0.1)

            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = 0.0

            yaw = self.calculate_facing_orientation(px, py, center_x, center_y)
            quaternion = tf.quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]



            # lidar_person = Person()
            # lidar_person.pose.position.x = px
            # lidar_person.pose.position.y = py
            # lidar_person.pose.position.z = 0.0

            # lidar_person.pose.orientation.x = quaternion[0]
            # lidar_person.pose.orientation.y = quaternion[1]
            # lidar_person.pose.orientation.z = quaternion[2]
            # lidar_person.pose.orientation.w = quaternion[3]

            # lidar_person.id = i+10
            
            pose_array.poses.append(pose)
            # lidar_pose_array.people.append(lidar_person)

        self.publisher_.publish(pose_array)
        # time.sleep(0.1)
        # self.lidar_publisher_.publish(lidar_pose_array)
        self.get_logger().info(f'Published {self.num_people} people facing approximately towards center')

        # Publish transform from map to base_link
        self.publish_transform()

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        quaternion = tf.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published transform from map to base_link')


def main(args=None):
    rclpy.init(args=args)
    num_people = 4  # Change this number to adjust the number of people
    node = PeopleVisionPublisher(num_people=num_people)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
