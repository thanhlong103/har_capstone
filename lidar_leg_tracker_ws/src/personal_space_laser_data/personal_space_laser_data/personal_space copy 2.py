#!/usr/bin/python3

import rclpy
from rclpy.node import Node

# Custom messages
from leg_detector_msgs.msg import Person, PersonArray, Leg, LegArray

import tf2_ros
from geometry_msgs.msg import PointStamped

# ROS messages
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from rclpy.qos import QoSProfile
from rclpy.clock import Clock

from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np, math, copy, random, time, datetime

### implement a version does not synchronize the two topics, the callback /people_tracked will update the person location,
### the callback for scan always works taking the updated or old data of /people_tracked

class FakeCoordinate():
    def __init__(self, point_x, point_y):
        self.x = point_x
        self.y = point_y
class FakePosition():
    def __init__(self, point_x, point_y):
        self.position = FakeCoordinate(point_x, point_y)

class FakePerson():
    def __init__(self, point_x, point_y, vel_x, vel_y):
        self.pose = FakePosition(point_x, point_y)
        self.velocity = FakeCoordinate(vel_x, vel_y)


class PersonalSpaceLaserData(Node):
    def __init__(self):
        super().__init__("PersonalSpaceLaserData")
        self.scan_subscription = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.person_subscription = self.create_subscription(PersonArray, "/people_tracked", self.people_tracked_callback, 10)
        self.scan_modified_pub = self.create_publisher(LaserScan, "/scan_modified", 10)
        
        self.processed_people = [[0,0,0,0],[0,0,0,0]] # maximum detected person is 2
        self.latest_update = False
        self.update_time = 0
        self.holding_seconds = 3
        self.tracked_people_count = 0

        self.num_simulated_people_array = [1 for i in range(1000000)]
        self.i = 0
    
    def simulate_transformed_person_array(self, num_person):
        person_array = []
        simulated_person = [[1, 1.5, 1, 0], [1, 0.2, 0, 1], [2, 1, 0, 0], [0.5, 3, 0, 0]]

        # Create a TransformListener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            transform = tf_buffer.lookup_transform("base_laser", "odom", rclpy.time.Time())

            for i in range(num_person):
                point_x = simulated_person[i][0]
                point_y = simulated_person[i][1]
                vel_x = simulated_person[i][2]
                vel_y = simulated_person[i][3]

                point_odom = PointStamped()
                point_odom.header.frame_id = "odom"
                point_odom.point.x = point_x
                point_odom.point.y = point_y
                point_odom.point.z = 0.0

                point_laser = tf_buffer.transform(point_odom, "base_laser")
                transformed_x = point_laser.point.x
                transformed_y = point_laser.point.y
                person_array.append(FakePerson(transformed_x, transformed_y, vel_x, vel_y))

        except tf2_ros.LookupException:
            self.get_logger().error("Transform not found: odom -> base_laser")
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"Failed to transform: {e}")

        return person_array

    def simulate_person_array(self, num_person):
        person_array = []
        simulated_person = [ [2, 0, 1, 0.5],  [1, 0.2, 1, 0], [2, 1, 0, 0], [0.5, 3, 0, 0]]
        for i in range(num_person):
            point_x = simulated_person[i][0]
            point_y = simulated_person[i][1]
            vel_x = simulated_person[i][2]
            vel_y = simulated_person[i][3]
            person_array.append(FakePerson(point_x, point_y, vel_x, vel_y))
        return person_array

    def people_tracked_callback(self, people_tracked):
        person_array = people_tracked.people

        simulation = False
        if simulation:
            if self.i < len(self.num_simulated_people_array):
                num_simulated_people = self.num_simulated_people_array[self.i]
                self.i += 1
                person_array = self.simulate_person_array(num_simulated_people)
                if num_simulated_people == 0:
                    print("Lost signal", len(person_array))                    
            else:
                num_simulated_people = random.randint(0,4) # simulate number of people
                person_array = self.simulate_person_array(num_simulated_people)
        
        if 0 <= len(person_array) < 3:
            if self.latest_update == True:
                diff_time = time.time() - self.update_time
                if len(person_array) == 0 and diff_time < self.holding_seconds: 
                    print(f"Lost signal! Hold previous location {self.holding_seconds} seconds")
                else:  
                    self.latest_update = False
            elif self.latest_update == False:
                print("Update ", len(person_array), self.latest_update)
                if len(person_array) == 0:
                    self.processed_people = [[0, 0], [0,0]]
                    self.tracked_people_count = 0
                else:
                    for i in range(len(person_array)):
                        point_x = person_array[i].pose.position.x
                        point_y = person_array[i].pose.position.y
                        vel_x = person_array[i].velocity.x
                        vel_y = person_array[i].velocity.y
                        self.processed_people[i] = [point_x, point_y, vel_x, vel_y]
                self.tracked_people_count = len(person_array)
                self.latest_update = True if len(person_array) > 0 else False
                self.update_time = time.time()
        elif len(person_array) >= 3:
            print("Update > 3")
            person_dist = lambda x:math.sqrt(x.pose.position.x ** 2 + x.pose.position.y ** 2)
            sorted_person_array = sorted(person_array, key=lambda a:person_dist(a)) # nearest to farthest
            
            person_1_information = [sorted_person_array[0].pose.position.x, sorted_person_array[0].pose.position.y, 
                                    sorted_person_array[0].velocity.x, sorted_person_array[0].velocity.y]
            person_2_information = [sorted_person_array[1].pose.position.x, sorted_person_array[1].pose.position.y,
                                    sorted_person_array[1].velocity.x, sorted_person_array[1].velocity.y]
            self.processed_people = [person_1_information, person_2_information]
            self.tracked_people_count = 2
            self.latest_update = True
            self.frame_count = 0

    def scan_callback(self, scan):
        copied_scan = copy.deepcopy(scan)
        scan_ranges = copied_scan.ranges
        processed_people = self.processed_people
        angle_incre = copied_scan.angle_increment
        person_radius = 0.35  # Radius for the circle (or semi-minor axis for the ellipse)
        time_scaling_factor = 1  # Scale factor for the semi-major axis of the ellipse

        if self.tracked_people_count > 0:
            print("Laser scan", processed_people)
            for k in range(self.tracked_people_count):
                person = processed_people[k]
                point_x = person[0]
                point_y = person[1]
                vel_x = 0
                vel_y = 0

                if abs(vel_x) > 0 or abs(vel_y) > 0:
                     # Moving human: represent as an ellipse
                    semi_major_axis = time_scaling_factor * math.sqrt(vel_x**2 + vel_y**2)
                    semi_minor_axis = person_radius
                    velocity_angle = math.atan2(vel_y, vel_x)

                    for i in range(len(scan_ranges)):
                        theta = i * angle_incre  # Angle corresponding to this index
                        if theta > math.pi / 2 or theta < -math.pi / 2:
                            continue  
                        line_slope = math.tan(theta)  # Slope of the line (y = slope * x)

                        # Coefficients of the quadratic equation for intersection
                        cos_v = math.cos(velocity_angle)
                        sin_v = math.sin(velocity_angle)
                        x_shift = point_x
                        y_shift = point_y

                        a = ((cos_v + line_slope * sin_v)**2 / semi_major_axis**2) + \
                            ((-sin_v + line_slope * cos_v)**2 / semi_minor_axis**2)

                        b = -2 * ((x_shift * (cos_v + line_slope * sin_v) / semi_major_axis**2) + 
                                (y_shift * (-sin_v + line_slope * cos_v) / semi_minor_axis**2))

                        c = (x_shift**2 / semi_major_axis**2) + (y_shift**2 / semi_minor_axis**2) - 1

                        # Solve the quadratic equation
                        discriminant = b**2 - 4 * a * c
                        if discriminant < 0:
                            continue  # No intersection
                        elif discriminant == 0:
                            # One intersection
                            x_intersection = -b / (2 * a)
                            y_intersection = line_slope * x_intersection
                            distance_to_intersection = math.sqrt(x_intersection**2 + y_intersection**2)
                            scan_ranges[i] = distance_to_intersection
                        else:
                            # Two intersections
                            x_intersection1 = (-b + math.sqrt(discriminant)) / (2 * a)
                            y_intersection1 = line_slope * x_intersection1
                            distance1 = math.sqrt(x_intersection1**2 + y_intersection1**2)

                            x_intersection2 = (-b - math.sqrt(discriminant)) / (2 * a)
                            y_intersection2 = line_slope * x_intersection2
                            distance2 = math.sqrt(x_intersection2**2 + y_intersection2**2)

                            scan_ranges[i] = min(distance1, distance2)
                else:
                    # Stationary human: represent as circle
                    distance = math.sqrt(point_x**2 + point_y**2)
                    m = round(math.atan2(point_y, point_x) / angle_incre)
                    scan_ranges[m] = distance - person_radius

                    count = 0
                    inflation = True
                    sign_ar = [-1, 1]
                    indexes_laser_ar = []
                    while inflation:
                        count += 1
                        for sign in sign_ar:
                            index_to_change = int(m + sign * count)
                            indexes_laser_ar.append(index_to_change)
                            theta = math.tan(index_to_change * angle_incre)
                            d_person_line = abs(point_y - theta * point_x) / math.sqrt(theta**2 + 1)
                            a = 1 + theta**2
                            b = -2 * (point_x + theta * point_y)
                            c = point_x**2 + point_y**2 - person_radius**2
                            delta = b**2 - 4 * a * c
                            if delta >= 0:
                                data_to_change = 0
                                if delta == 0:
                                    intersect_x = -b / (2 * a)
                                    intersect_y = theta * intersect_x
                                    data_to_change = abs(math.sqrt(intersect_x**2 + intersect_y**2))
                                else:
                                    intersect_x_1 = (-b + math.sqrt(delta)) / (2 * a)
                                    intersect_y_1 = theta * intersect_x_1
                                    intersect_x_2 = (-b - math.sqrt(delta)) / (2 * a)
                                    intersect_y_2 = theta * intersect_x_2
                                    d1, d2 = abs(math.sqrt(intersect_x_1**2 + intersect_y_1**2)), abs(
                                        math.sqrt(intersect_x_2**2 + intersect_y_2**2)
                                    )
                                    if d1 < d2:
                                        intersect_x = intersect_x_1
                                        intersect_y = intersect_y_1
                                        data_to_change = d1
                                    else:
                                        intersect_x = intersect_x_2
                                        intersect_y = intersect_y_2
                                        data_to_change = d2
                                scan_ranges[index_to_change] = data_to_change
                            else:
                                inflation = False
                                break

        self.scan_modified_pub.publish(copied_scan)


    def process_scan_ranges(self, arr, target_size):
        arr = np.array(arr, dtype=np.float64)
        nan_indices = np.where(np.isnan(arr))[0]
        
        for idx in nan_indices:
            if idx == 0:
                arr[idx] = arr[idx + 1]
            elif idx == len(arr) - 1:
                arr[idx] = arr[idx - 1]
            else:
                arr[idx] = (arr[idx - 1] + arr[idx + 1]) / 2
        current_size = len(arr)
        if target_size < current_size:
            drop_size = (current_size - target_size) // 2
            start_idx = drop_size
            end_idx = current_size - drop_size if (current_size - target_size) % 2 == 0 else current_size - drop_size - 1
            arr = arr[start_idx:end_idx]
        return arr
        


def main(args=None):
    rclpy.init(args=args)
    personal_space_lidar_data= PersonalSpaceLaserData()
    rclpy.spin(personal_space_lidar_data)
    personal_space_lidar_data.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()