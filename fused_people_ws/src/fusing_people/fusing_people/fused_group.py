import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from fused_people_msgs.msg import FusedPersonArray, FusedPerson

from shapely.geometry import LineString, Point, Polygon
import numpy as np
from sklearn.cluster import DBSCAN


class FusedPeopleSubscriber(Node):
    def __init__(self):
        super().__init__("fused_people_subscriber")
        self.subscription = self.create_subscription(
            FusedPersonArray,
            "/people_fused",  # Topic name
            self.fused_people_callback,
            10,  # QoS
        )

        # Apply DBSCAN clustering
        # eps is the maximum distance between two points to be considered as neighbors
        # min_samples is the minimum number of points to form a cluster
        self.dbscan = DBSCAN(eps=4, min_samples=2)

        self.pose_array = []
        self.line_length = 10.0
        self.interest_area = 0.5
        self.area_near_threshold = 3.0

        self.get_logger().info("Fused People Subscriber Node has started.")

    def pose_process(self, pose):
        # Extract quaternion components
        qx = pose.position.orientation.x
        qy = pose.position.orientation.y
        qz = pose.position.orientation.z
        qw = pose.position.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, orientation = euler_from_quaternion([qx, qy, qz, qw])

        x = pose.position.position.x
        y = pose.position.position.y

        self.pose_array.append([x, y, orientation])

    def create_line(self, point):
        """
        Create a line extending from a point in the direction of orientation.
        :param point: [x, y, theta] where theta is in radians
        :param length: Length of the line segment
        :return: LineString object
        """
        x, y, theta = point
        dx = np.cos(theta) * self.line_length  # Delta x
        dy = np.sin(theta) * self.line_length  # Delta y
        start = (x, y)  # Line starts at the input point
        end = (x + dx, y + dy)  # Line ends in the direction of orientation
        return LineString([start, end])

    def find_intersections(self, lines):
        """
        Find intersections between all pairs of lines.
        :param lines: List of LineString objects
        :return: List of intersection points
        """
        intersections = []
        for i, line1 in enumerate(lines):
            for j, line2 in enumerate(lines):
                if i < j:  # Avoid duplicate pairs
                    intersection = line1.intersection(line2)
                    if isinstance(
                        intersection, Point
                    ):  # Only consider point intersections
                        intersections.append(intersection)
        return intersections

    def detect_group(self):
        self.pose_array = np.array(self.pose_array)

        # print(self.pose_array)

        labels = self.dbscan.fit_predict(self.pose_array)

        # Filter out the points labeled as outliers (label = -1)
        self.pose_array = self.pose_array[labels != -1]

        print(self.pose_array)
        # Create lines
        lines = [self.create_line(p) for p in self.pose_array]

        # Find intersections
        intersection_points = self.find_intersections(lines)

        # Create a polygon from intersection points
        polygon = None
        if len(intersection_points) == 1:
            return True
        elif len(intersection_points) > 1:
            # Sort points to form a convex hull
            coords = [(pt.x, pt.y) for pt in intersection_points]
            polygon = Polygon(coords).convex_hull

            # Calculate area if the polygon is valid
            area = polygon.area

            if area < self.interest_area:
                # Calculate the distance from each point to the centroid
                distances = []

                if area < 0.05:
                    for pt in self.pose_array:
                        point = Point(pt[0], pt[1])
                        distance = intersection_points[0].distance(point)
                        distances.append(distance)
                else:
                    # Calculate the centroid (center) of the polygon
                    centroid = polygon.centroid

                    if centroid:
                        for pt in self.pose_array:
                            point = Point(pt[0], pt[1])
                            distance = centroid.distance(point)
                            distances.append(distance)
                if np.mean(distance) < self.area_near_threshold:
                    return True
                else:
                    return False
            else:
                return False

        else:
            return False

    def fused_people_callback(self, msg):
        """
        Callback function to handle messages from the fused_people topic.
        """
        self.pose_array = []
        for i, pose in enumerate(msg.people):
            self.pose_process(pose)

        if self.detect_group():
            print("GROUP!")
        else:
            return


def main(args=None):
    rclpy.init(args=args)
    node = FusedPeopleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
