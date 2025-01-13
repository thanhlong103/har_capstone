import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import random


def generate_random_pose():
    pose = Pose()
    pose.position.x = -1.0
    pose.position.y = -0.05
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    return pose


class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__("pose_array_publisher")
        self.publisher_ = self.create_publisher(PoseArray, "/people_vision", 10)
        self.timer = self.create_timer(
            0.5, self.publish_pose_array
        )  # Publish every 0.5 seconds
        self.get_logger().info("PoseArray Publisher Node has started.")

    def publish_pose_array(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "base_laser"  # Adjust frame_id as needed

        # Add random poses to the PoseArray
        for _ in range(1):  # Publish 5 random poses
            pose_array.poses.append(generate_random_pose())

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
