import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion
import time

class MoveCylinderNode(Node):

    def __init__(self):
        super().__init__('move_cylinder_node')
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        
        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_entity_state not available, waiting again...')
        
        # Set the initial position of the cylinder
        self.cylinder_position = -5.0
        self.velocity = 0.5 # m/s
        self.last_time = time.time()

        # Start moving the cylinder
        self.timer = self.create_timer(0.1, self.move_cylinder)  # Update every 0.1s (10Hz)

    def move_cylinder(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if self.cylinder_position >= 8.1:
            return
        
        # Update the position based on the velocity and elapsed time
        self.cylinder_position += self.velocity * dt
        
        # Create a request to set the entity state
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = 'moving_cylinder_2'
        
        # Update the position along the x-axis (y and z stay the same)
        request.state.pose = Pose(
            position=Point(x=self.cylinder_position, y=-3.5, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Identity quaternion for no rotation
        )
        
        # Call the service
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully set the position of moving_cylinder to x={self.cylinder_position:.2f}.')
            else:
                self.get_logger().error('Failed to set the position of moving_cylinder.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveCylinderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
