#!/usr/bin/python3

import rclpy
from rclpy.node import Node

# Custom messages
from leg_detector_msgs.msg import Person, PersonArray, Leg, LegArray

# ROS messages
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from rclpy.qos import QoSProfile
from rclpy.clock import Clock

from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np, math, copy

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

def simulate_person_array(num_person):
    person_array = []
    simulated_person = [[1, 0.2, 0, 0], [1, -0.2, 0, 0], [1, -0.5, 0, 0], [0.5, 3, 0, 0]]
    for i in range(num_person):
        point_x = simulated_person[i][0]
        point_y = simulated_person[i][1]
        vel_x = simulated_person[i][2]
        vel_y = simulated_person[i][3]
        person_array.append(FakePerson(point_x, point_y, vel_x, vel_y))
    return person_array


class PersonalSpaceLaserData(Node):
    def __init__(self):
        super().__init__("PersonalSpaceLaserData")
        self.scan_subscription = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.person_subscription = self.create_subscription(PersonArray, "/people_tracked", self.people_tracked_callback, 10)
        self.scan_modified_pub = self.create_publisher(LaserScan, "/scan_modified", 10)
        
        self.processed_people = [0,0] # maximum detected person is 2
        self.latest_update = 0
        self.tracked_people_count = 0

    def people_tracked_callback(self, people_tracked):
        person_array = people_tracked.people
        simulation = True
        num_simulated_people = 2
        if simulation:
            person_array = simulate_person_array(num_simulated_people)
        print("Length person array", len(person_array))
        
        if 0 <= len(person_array) < 3:
            for i in range(len(person_array)):
                point_x = person_array[i].pose.position.x
                point_y = person_array[i].pose.position.y
                self.processed_people[i] = [point_x, point_y]
            self.tracked_people_count = len(person_array)
        elif len(person_array) >= 3:
            person_dist = lambda x:math.sqrt(x.pose.position.x ** 2 + x.pose.position.y ** 2)
            sorted_person_array = sorted(person_array, key=lambda a,b:person_dist(a) < person_dist(b))
            print("Sorted ", len(sorted_person_array))
            person_1_information = [sorted_person_array[-1].pose.position.x, sorted_person_array[-1].pose.position.x]
            person_2_information = [sorted_person_array[-2].pose.position.x, sorted_person_array[-2].pose.position.x]
            self.processed_people = [person_1_information, person_2_information]
            self.tracked_people_count = 2          

    def scan_callback(self, scan):
        print(f"There are {self.tracked_people_count} people tracked ")
        # copied_scan.ranges = self.process_scan_ranges(copied_scan.ranges, 360)
        copied_scan = copy.deepcopy(scan)
        scan_ranges = copied_scan.ranges
        # person_array = people_tracked.people
        processed_people = self.processed_people
        angle_incre = copied_scan.angle_increment 
        person_radius = 0.4
        
        if self.tracked_people_count > 0:
            for person in processed_people:
                point_x = person[0]
                point_y = person[1]
                print("Coordinate", point_x, point_y)

                distance = math.sqrt(point_x**2 + point_y**2)
                m = round(math.atan2(point_y, point_x) / angle_incre)
                i = 0
                inflation = True
                sign_ar = [-1,1]
                indexes_laser_ar = []
                while(inflation):
                    i += 1
                    for sign in sign_ar:
                        index_to_change = int(m + sign*i)
                        indexes_laser_ar.append(index_to_change)
                        theta = math.tan(i*angle_incre)
                        d_person_line = abs(point_y-theta*point_x)/math.sqrt(theta**2+1)
                        a = 1 + theta**2
                        b = -2*(point_x + theta*point_y)
                        c = point_x**2 + point_y**2 - person_radius**2
                        delta = b**2 - 4*a*c
                        if delta >= 0:
                            data_to_change = 0
                            if delta == 0:
                                intersect_x = -b/(2*a)
                                intersect_y = theta*intersect_x
                                data_to_change = abs(math.sqrt(intersect_x**2 + intersect_y**2))
                            else:    
                                intersect_x_1 = (-b+math.sqrt(delta))/(2*a)
                                intersect_y_1 = theta*intersect_x_1
                                intersect_x_2 = (-b-math.sqrt(delta))/(2*a)
                                intersect_y_2 = theta*intersect_x_2
                                d1, d2 = abs(math.sqrt(intersect_x_1**2 + intersect_y_1**2)), abs(math.sqrt(intersect_x_2**2 + intersect_y_2**2))
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
                            print("Change scan indexes", indexes_laser_ar)
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