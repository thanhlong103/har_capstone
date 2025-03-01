import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
from people_msgs.msg import People, MyPerson, PeopleGroup, PeopleGroupArray
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
            People,
            '/people_fused',
            self.people_fused_callback,
            10)
        self.subscription = self.create_subscription(
            PeopleGroupArray,
            '/people_groups_detected',
            self.people_group_callback,
            10)
        self.publisher = self.create_publisher(People, '/people', 10)
        self.group_publisher = self.create_publisher(PeopleGroupArray, '/people_groups', 10)

        self.target_frame = 'map'  # Convert to this frame
        self.rotation_matrix = None
        self.t_x = None
        self.t_y = None
        self.t_z = None
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
        self.t_x = transform.transform.translation.x
        self.t_y = transform.transform.translation.y
        self.t_z = transform.transform.translation.z

        # Extract rotation (quaternion)
        q_x = transform.transform.rotation.x
        q_y = transform.transform.rotation.y
        q_z = transform.transform.rotation.z
        q_w = transform.transform.rotation.w

        # Convert quaternion to a rotation matrix
        self.rotation_matrix = tf_transformations.quaternion_matrix([q_x, q_y, q_z, q_w])[:3, :3]

        for fused_person in msg.people:
            person = MyPerson()

            # Extract person's position in the 'map' frame
            pos_map = np.array([
                fused_person.pose.position.x,
                fused_person.pose.position.y,
                fused_person.pose.position.z
            ])

            vel_map = np.array([
                fused_person.velocity.x,
                fused_person.velocity.y,
                fused_person.velocity.z
            ])

            # Apply transformation: R * pos_map + T
            pos_base_link = self.rotation_matrix @ pos_map + np.array([self.t_x, self.t_y, self.t_z])

            vel_base_link = self.rotation_matrix @ vel_map

            # Set transformed position
            person.pose.position = Point(
                x=pos_base_link[0],
                y=pos_base_link[1],
                z=pos_base_link[2]
            )

            person.velocity= Point(
                x=vel_base_link[0],
                y=vel_base_link[1],
                z=vel_base_link[2]
            )

            person.activity = fused_person.activity
            people_msg.people.append(person)
        
        self.publisher.publish(people_msg)

    def people_group_callback(self, msg):
        group_msg = PeopleGroupArray()
        group_msg.header.stamp = self.get_clock().now().to_msg()
        group_msg.header.frame_id = self.target_frame  # Change to target frame

        for detected_group in msg.groups:
            group = PeopleGroup()
            for person_group in detected_group.people:
                person = MyPerson()

                # Extract person's position in the 'map' frame
                pos_map = np.array([
                    person_group.pose.position.x,
                    person_group.pose.position.y,
                    person_group.pose.position.z
                ])

                vel_map = np.array([
                    person_group.velocity.x,
                    person_group.velocity.y,
                    person_group.velocity.z
                ])

                # Apply transformation: R * pos_map + T
                pos_base_link = self.rotation_matrix @ pos_map + np.array([self.t_x, self.t_y, self.t_z])

                vel_base_link = self.rotation_matrix @ vel_map

                # Set transformed position
                person.pose.position = Point(
                    x=pos_base_link[0],
                    y=pos_base_link[1],
                    z=pos_base_link[2]
                )

                person.velocity= Point(
                    x=vel_base_link[0],
                    y=vel_base_link[1],
                    z=vel_base_link[2]
                )

                group.people.append(person)
            
            centroid_map = np.array([
                group.centroid.x,
                group.centroid.y,
                group.centroid.z
            ])

            # Apply transformation: R * pos_map + T
            centroid_base_link = self.rotation_matrix @ centroid_map + np.array([self.t_x, self.t_y, self.t_z])

            # Set transformed position
            group.centroid = Point(
                x=centroid_base_link[0],
                y=centroid_base_link[1],
                z=centroid_base_link[2]
            )
            
            group.area = detected_group.area
            group.id = detected_group.id
            group.activity = detected_group.activity
            group.radius = detected_group.radius

            group_msg.groups.append(group)
        
        self.group_publisher.publish(group_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PeopleFusedConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()