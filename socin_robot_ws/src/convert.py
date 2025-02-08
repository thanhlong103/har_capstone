import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
from people_msgs.msg import People, Person
from fused_people_msgs.msg import FusedPersonArray, FusedPerson  # Replace with actual package name
import tf2_ros
import tf_transformations
import numpy as np

class PeopleFusedConverter(Node):
    def __init__(self):
        super().__init__('people_fused_converter')

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            FusedPersonArray,
            '/people_fused',
            self.people_fused_callback,
            10)
        self.publisher = self.create_publisher(People, '/people', 10)

        self.target_frame = 'map'  # Convert to this frame
        print("START")

    def people_fused_callback(self, msg):
        people_msg = People()
        people_msg.header.stamp = self.get_clock().now().to_msg()
        people_msg.header.frame_id = self.target_frame  # Change to target frame

        # Try to get the transform from 'map' to 'base_link'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,  # Target frame
                'base_link',  # Source frame
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)  # Wait for transform
            )
        except Exception as e:
            self.get_logger().warn(f"Transform from 'map' to '{self.target_frame}' not available: {e}")
            return  # Skip processing if transform is not available

        # Extract translation
        t_x = transform.transform.translation.x
        t_y = transform.transform.translation.y
        t_z = transform.transform.translation.z

        # Extract rotation (quaternion)
        q_x = transform.transform.rotation.x
        q_y = transform.transform.rotation.y
        q_z = transform.transform.rotation.z
        q_w = transform.transform.rotation.w

        # Convert quaternion to a rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix([q_x, q_y, q_z, q_w])[:3, :3]

        for fused_person in msg.people:
            person = Person()
            person.name = f'Person_{fused_person.id}'

            # Extract person's position in the 'map' frame
            pos_map = np.array([
                fused_person.position.position.x,
                fused_person.position.position.y,
                fused_person.position.position.z
            ])

            vel_map = np.array([
                fused_person.velocity.x,
                fused_person.velocity.y,
                fused_person.velocity.z
            ])

            # Apply transformation: R * pos_map + T
            pos_base_link = rotation_matrix @ pos_map + np.array([t_x, t_y, t_z])

            vel_base_link = rotation_matrix @ vel_map

            # Set transformed position
            person.position = Point(
                x=pos_base_link[0],
                y=pos_base_link[1],
                z=pos_base_link[2]
            )

            person.velocity= Point(
                x=vel_base_link[0],
                y=vel_base_link[1],
                z=vel_base_link[2]
            )

            # person.velocity = fused_person.velocity
            person.reliability = 1.0  # Assuming perfect reliability
            person.tagnames = ["Tag1"]  # Placeholder
            person.tags = []  # Placeholder

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
