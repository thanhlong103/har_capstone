import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import random
from tf_transformations import quaternion_from_euler


class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__("pose_array_publisher")
        self.publisher_ = self.create_publisher(PoseArray, "/people_vision", 10)
        self.timer = self.create_timer(
            0.5, self.publish_pose_array
        )  # Publish every 0.5 seconds
        self.get_logger().info("PoseArray Publisher Node has started.")

    def generate_random_pose(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "base_laser"  # Adjust frame_id as needed

        pose1 = Pose()
        q = quaternion_from_euler(0, 0, 0)
        pose1.position.x = 0.0
        pose1.position.y = 0.0
        pose1.orientation.x = q[0]
        pose1.orientation.y = q[1]
        pose1.orientation.z = q[2]
        pose1.orientation.w = q[3]

        pose_array.poses.append(pose1)

        pose2 = Pose()
        q = quaternion_from_euler(0, 0, -3.14/2)
        pose2.position.x = 1.0
        pose2.position.y = 1.0
        pose2.orientation.x = q[0]
        pose2.orientation.y = q[1]
        pose2.orientation.z = q[2]
        pose2.orientation.w = q[3]

        pose_array.poses.append(pose2)

        pose3 = Pose()
        q = quaternion_from_euler(0, 0, 3.14/2)
        pose3.position.x = 1.0
        pose3.position.y = -1.0
        pose3.orientation.x = q[0]
        pose3.orientation.y = q[1]
        pose3.orientation.z = q[2]
        pose3.orientation.w = q[3]

        pose_array.poses.append(pose3)

        pose4 = Pose()
        q = quaternion_from_euler(0, 0, 3.14)
        pose4.position.x = 2.0
        pose4.position.y = 0.0
        pose4.orientation.x = q[0]
        pose4.orientation.y = q[1]
        pose4.orientation.z = q[2]
        pose4.orientation.w = q[3]

        pose_array.poses.append(pose4)

        return pose_array

    def publish_pose_array(self):
        pose_array = self.generate_random_pose()

        self.publisher_.publish(pose_array)
        self.get_logger().info(
            f"Published PoseArray with {len(pose_array.poses)} poses."
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PoseArray Publisher Node is shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
