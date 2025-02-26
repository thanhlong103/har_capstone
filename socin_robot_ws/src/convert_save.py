import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
from people_msgs.msg import People, MyPerson, PeopleGroup, PeopleGroupArray
import tf2_ros
import tf_transformations
import numpy as np
from collections import deque
from rclpy.time import Time, Duration

class PeopleFusedConverter(Node):
    def __init__(self):
        super().__init__('people_fused_converter')

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(People, '/people_fused', self.people_fused_callback, 10)
        self.create_subscription(PeopleGroupArray, '/people_groups_detected', self.people_group_callback, 10)

        # Publishers
        self.publisher = self.create_publisher(People, '/people', 10)
        self.group_publisher = self.create_publisher(PeopleGroupArray, '/people_groups', 10)

        # Data storage (retain at least 10 minutes)
        self.people_data = deque()  # Stores tuples of (timestamp, MyPerson)
        self.group_data = deque()  # Stores tuples of (timestamp, PeopleGroup)
        self.retention_duration = Duration(seconds=30)  # 10 minutes

        # Timer to clean up outdated data every 30 seconds
        self.create_timer(1.5, self.cleanup_old_data)

        self.target_frame = 'map'
        self.rotation_matrix = None
        self.t_x = self.t_y = self.t_z = None

    def people_fused_callback(self, msg):
        current_time = self.get_clock().now()
        people_msg = People()
        people_msg.header.stamp = current_time.to_msg()
        people_msg.header.frame_id = self.target_frame  

        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f"Transform from 'base_link' to '{self.target_frame}' not available: {e}")
            return  

        self.t_x, self.t_y, self.t_z = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
        q_x, q_y, q_z, q_w = transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w
        self.rotation_matrix = tf_transformations.quaternion_matrix([q_x, q_y, q_z, q_w])[:3, :3]

        # Convert deque to a dictionary for faster lookup
        people_dict = {person.id: (timestamp, person) for timestamp, person in self.people_data}

        for fused_person in msg.people:
            person = MyPerson()
            person.id = fused_person.id  # Ensure ID is stored

            pos_map = np.array([fused_person.pose.position.x, fused_person.pose.position.y, fused_person.pose.position.z])
            vel_map = np.array([fused_person.velocity.x, fused_person.velocity.y, fused_person.velocity.z])

            pos_base_link = self.rotation_matrix @ pos_map + np.array([self.t_x, self.t_y, self.t_z])
            vel_base_link = self.rotation_matrix @ vel_map

            person.pose.position = Point(x=pos_base_link[0], y=pos_base_link[1], z=pos_base_link[2])
            person.velocity = Point(x=vel_base_link[0], y=vel_base_link[1], z=vel_base_link[2])

            if person.id in people_dict:
                # Update existing person
                people_dict[person.id] = (current_time, person)
            else:
                # Add new person
                self.people_data.append((current_time, person))

        # Convert back to deque
        self.people_data = deque(people_dict.values(), maxlen=1000)

        self.publish_people()

    def people_group_callback(self, msg):
        current_time = self.get_clock().now()
        group_msg = PeopleGroupArray()
        group_msg.header.stamp = current_time.to_msg()
        group_msg.header.frame_id = self.target_frame  

        # Convert deque to a dictionary for fast lookup
        group_dict = {group.id: (timestamp, group) for timestamp, group in self.group_data}

        for detected_group in msg.groups:
            group = PeopleGroup()
            group.id = detected_group.id  # Ensure ID is stored

            for person_group in detected_group.people:
                person = MyPerson()
                person.id = person_group.id  # Ensure ID is stored

                pos_map = np.array([person_group.pose.position.x, person_group.pose.position.y, person_group.pose.position.z])
                vel_map = np.array([person_group.velocity.x, person_group.velocity.y, person_group.velocity.z])

                pos_base_link = self.rotation_matrix @ pos_map + np.array([self.t_x, self.t_y, self.t_z])
                vel_base_link = self.rotation_matrix @ vel_map

                person.pose.position = Point(x=pos_base_link[0], y=pos_base_link[1], z=pos_base_link[2])
                person.velocity = Point(x=vel_base_link[0], y=vel_base_link[1], z=vel_base_link[2])

                group.people.append(person)
            
            centroid_map = np.array([detected_group.centroid.x, detected_group.centroid.y, detected_group.centroid.z])
            centroid_base_link = self.rotation_matrix @ centroid_map + np.array([self.t_x, self.t_y, self.t_z])

            group.centroid = Point(x=centroid_base_link[0], y=centroid_base_link[1], z=centroid_base_link[2])
            group.area, group.id, group.activity, group.radius = detected_group.area, detected_group.id, detected_group.activity, detected_group.radius

            if group.id in group_dict:
                group_dict[group.id] = (current_time, group)
            else:
                self.group_data.append((current_time, group))

        self.group_data = deque(group_dict.values(), maxlen=500)

        self.publish_groups()

    def cleanup_old_data(self):
        """Remove people and groups older than 10 minutes"""
        current_time = self.get_clock().now()
        # Remove old people
        while self.people_data and (current_time - self.people_data[0][0]) > self.retention_duration:
            self.people_data.popleft()
        # Remove old groups
        while self.group_data and (current_time - self.group_data[0][0]) > self.retention_duration:
            self.group_data.popleft()

        self.publish_people()
        self.publish_groups()

    def publish_people(self):
        """Publish the filtered people data"""
        people_msg = People()
        people_msg.header.stamp = self.get_clock().now().to_msg()
        people_msg.header.frame_id = self.target_frame
        people_msg.people = [entry[1] for entry in self.people_data]  # Extract MyPerson objects
        self.publisher.publish(people_msg)

    def publish_groups(self):
        """Publish the filtered people group data"""
        group_msg = PeopleGroupArray()
        group_msg.header.stamp = self.get_clock().now().to_msg()
        group_msg.header.frame_id = self.target_frame
        group_msg.groups = [entry[1] for entry in self.group_data]  # Extract PeopleGroup objects
        self.group_publisher.publish(group_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PeopleFusedConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
