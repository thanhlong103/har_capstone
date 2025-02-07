import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion
import time

class MoveCylindersNode(Node):

    def __init__(self):
        super().__init__('move_cylinders_node')
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        
        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_entity_state not available, waiting again...')
        
        # Set the initial positions of the cylinders
        self.cylinder_positions = {
            'moving_cylinder_3': {'x': 13.8, 'y': 0.0, 'z': 0.0},
            'moving_cylinder_4': {'x': 15.0, 'y': 0.0, 'z': 0.0}
        }
        
        self.velocity = 0.1  # m/s for both cylinders
        self.last_time = time.time()

        # Start moving the cylinders
        self.timer = self.create_timer(0.1, self.move_cylinders)  # Update every 0.1s (10Hz)

    def move_cylinders(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Update the position of both cylinders based on the velocity and elapsed time
        for cylinder_name in self.cylinder_positions:
            if self.cylinder_positions[cylinder_name]['y'] >= 8.25:
                return
            else:
                self.cylinder_positions[cylinder_name]['y'] += self.velocity * dt
            
                # Create a request to set the entity state
                request = SetEntityState.Request()
                request.state = EntityState()
                request.state.name = cylinder_name
                
                # Update the position along the x-axis (y and z stay the same)
                request.state.pose = Pose(
                    position=Point(
                        x=self.cylinder_positions[cylinder_name]['x'],
                        y=self.cylinder_positions[cylinder_name]['y'],
                        z=self.cylinder_positions[cylinder_name]['z']
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Identity quaternion for no rotation
                )
                
                # Call the service
                future = self.client.call_async(request)
                future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully set the position of {future.result().state.name}.')
            else:
                self.get_logger().error(f'Failed to set the position of {future.result().state.name}.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveCylindersNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
