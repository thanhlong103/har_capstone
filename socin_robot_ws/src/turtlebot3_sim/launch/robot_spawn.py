#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')

        # Create a client for the spawn_entity service
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

        # Wait until the service is available
        self.get_logger().info('Waiting for /spawn_entity service...')
        self.client.wait_for_service()
        self.get_logger().info('/spawn_entity service available.')

    def spawn_robot(self, urdf_path, robot_name, x=0.0, y=0.0, z=0.0):
        # Read the URDF file
        try:
            with open(urdf_path, 'r') as file:
                robot_description = file.read()
        except FileNotFoundError:
            self.get_logger().error(f"URDF file not found: {urdf_path}")
            return

        # Create a request for the spawn_entity service
        request = SpawnEntity.Request()
        request.name = robot_name
        request.xml = robot_description
        request.robot_namespace = ''
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        # Call the service
        self.get_logger().info(f'Spawning robot: {robot_name} at position ({x}, {y}, {z})')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Successfully spawned robot: {robot_name}")
        else:
            self.get_logger().error(f"Failed to spawn robot: {robot_name}")

def main():
    rclpy.init()

    # Define URDF file path and robot details
    urdf_path = '/home/irs/har_capstone/socin_robot_ws/src//turtlebot3_sim/descriptions/robot.urdf'  # Update this to your URDF file path
    robot_name = 'mybot'

    # Create the spawner node and spawn the robot
    spawner = RobotSpawner()
    spawner.spawn_robot(urdf_path, robot_name, x=-8.5, y=-4.0, z=0.0)

    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
