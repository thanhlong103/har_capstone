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
        super().__init__("people_fused_converter")

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.subscription_people = self.create_subscription(
            People, "/people_fused", self.people_fused_callback, 10
        )

        self.subscription_groups = self.create_subscription(
            PeopleGroupArray, "/people_groups_detected", self.people_group_callback, 10
        )

        # Publishers
        self.publisher = self.create_publisher(People, "/people", 10)
        self.group_publisher = self.create_publisher(
            PeopleGroupArray, "/people_groups", 10
        )

        self.target_frame = "map"  # Convert to this frame
        self.rotation_matrix = np.eye(3)
        self.t_x, self.t_y, self.t_z = 0.0, 0.0, 0.0

        # Store latest messages
        self.latest_people_msg = None
        self.latest_group_msg = None

        # Dictionary to store groups (Persistent storage)
        self.stored_groups = {}

        # Create a timer to periodically publish the stored messages
        self.create_timer(0.03, self.publish_latest_data)  # Publish at 10 Hz

        self.get_logger().info("PeopleFusedConverter started.")

    def people_fused_callback(self, msg):
        self.latest_people_msg = self.transform_people(msg)

    def people_group_callback(self, msg):
        """Process detected groups, store those with >= 2 people, and update existing ones."""
        transformed_msg = self.transform_groups(msg)
        
        # Update stored groups
        for group in transformed_msg.groups:
            if len(group.people) >= 2:
                self.stored_groups[group.id] = group  # Store or update group

        # Keep publishing stored groups
        self.latest_group_msg = PeopleGroupArray()
        self.latest_group_msg.header.stamp = self.get_clock().now().to_msg()
        self.latest_group_msg.header.frame_id = self.target_frame
        self.latest_group_msg.groups = list(self.stored_groups.values())

    def transform_people(self, msg):
        """Transforms and returns a People message in the target frame, including orientation."""
        people_msg = People()
        people_msg.header.stamp = self.get_clock().now().to_msg()
        people_msg.header.frame_id = self.target_frame

        # Try to get the transform
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")
            return self.latest_people_msg  # Use previous data if transform fails

        # Extract translation
        self.t_x, self.t_y, self.t_z = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )

        # Extract and convert rotation
        q = transform.transform.rotation
        self.rotation_matrix = tf_transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w]
        )[:3, :3]

        for fused_person in msg.people:
            person = MyPerson()
            
            # Transform position
            pos_map = np.array([
                fused_person.pose.position.x,
                fused_person.pose.position.y,
                fused_person.pose.position.z,
            ])
            pos_base_link = self.rotation_matrix @ pos_map + np.array(
                [self.t_x, self.t_y, self.t_z]
            )
            person.pose.position = Point(
                x=pos_base_link[0], y=pos_base_link[1], z=pos_base_link[2]
            )

            # Transform orientation (quaternion)
            q_in = [
                fused_person.pose.orientation.x,
                fused_person.pose.orientation.y,
                fused_person.pose.orientation.z,
                fused_person.pose.orientation.w,
            ]
            q_rot = [q.x, q.y, q.z, q.w]

            # Apply rotation
            q_out = tf_transformations.quaternion_multiply(q_rot, q_in)

            person.pose.orientation.x = q_out[0]
            person.pose.orientation.y = q_out[1]
            person.pose.orientation.z = q_out[2]
            person.pose.orientation.w = q_out[3]

            # Transform velocity
            vel_map = np.array([
                fused_person.velocity.x,
                fused_person.velocity.y,
                fused_person.velocity.z,
            ])
            vel_base_link = self.rotation_matrix @ vel_map
            person.velocity = Point(
                x=vel_base_link[0], y=vel_base_link[1], z=vel_base_link[2]
            )

            person.activity = fused_person.activity
            people_msg.people.append(person)

        return people_msg

    def transform_groups(self, msg):
        """Transforms and returns a PeopleGroupArray message in the target frame."""
        group_msg = PeopleGroupArray()
        group_msg.header.stamp = self.get_clock().now().to_msg()
        group_msg.header.frame_id = self.target_frame

        for detected_group in msg.groups:
            group = PeopleGroup()
            group.area = detected_group.area
            group.id = detected_group.id
            group.activity = detected_group.activity
            group.radius = detected_group.radius

            for person_group in detected_group.people:
                person = MyPerson()
                pos_map = np.array(
                    [
                        person_group.pose.position.x,
                        person_group.pose.position.y,
                        person_group.pose.position.z,
                    ]
                )
                vel_map = np.array(
                    [
                        person_group.velocity.x,
                        person_group.velocity.y,
                        person_group.velocity.z,
                    ]
                )

                # Transform position and velocity
                pos_base_link = self.rotation_matrix @ pos_map + np.array(
                    [self.t_x, self.t_y, self.t_z]
                )
                vel_base_link = self.rotation_matrix @ vel_map

                person.pose.position = Point(
                    x=pos_base_link[0], y=pos_base_link[1], z=pos_base_link[2]
                )
                person.velocity = Point(
                    x=vel_base_link[0], y=vel_base_link[1], z=vel_base_link[2]
                )

                group.people.append(person)

            # Transform centroid
            centroid_map = np.array(
                [detected_group.centroid.x, detected_group.centroid.y, detected_group.centroid.z]
            )
            centroid_base_link = self.rotation_matrix @ centroid_map + np.array(
                [self.t_x, self.t_y, self.t_z]
            )
            group.centroid = Point(
                x=centroid_base_link[0],
                y=centroid_base_link[1],
                z=centroid_base_link[2],
            )

            group_msg.groups.append(group)

        return group_msg

    def publish_latest_data(self):
        """Publishes the latest stored People and PeopleGroupArray messages."""
        if self.latest_people_msg:
            self.publisher.publish(self.latest_people_msg)

        if self.latest_group_msg:
            self.group_publisher.publish(self.latest_group_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PeopleFusedConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
