#!/usr/bin/python3

import rclpy
from rclpy.node import Node

# Custom messages
from fused_people_msgs.msg import FusedPerson, FusedPersonArray

# ROS messages
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from rclpy.qos import QoSProfile
from rclpy.clock import Clock

from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Temperature, FluidPressure

import numpy as np, math, copy


class PersonalSpaceLaserData(Node):
    def __init__(self):
        super().__init__("PersonalSpaceLaserData")
        self.laser_sub = Subscriber(self, LaserScan, "scan")
        self.people_fused_sub = Subscriber(self, FusedPersonArray, "/people_fused")
        self.scan_modified_pub = self.create_publisher(LaserScan, "/scan_modified", 10)

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.laser_sub, self.people_fused_sub],
                                                     queue_size, max_delay)
        self.time_sync.registerCallback(self.SyncCallback)

    def SyncCallback(self, scan, people_fused):
        # copied_scan.ranges = self.process_scan_ranges(copied_scan.ranges, 360)
        copied_scan = copy.deepcopy(scan)
        scan_ranges = copied_scan.ranges
        person_array = people_fused.people
        print(f"There are {len(person_array)} people tracked ")
        angle_incre = copied_scan.angle_increment 
        person_radius = 0.4
        # person_array = [[1, -1.5], [2, 0]] # simulation
        if len(person_array) > 0:
            for person in person_array:
                point_x = person.position.x
                point_y = person.position.y
                # simulation
                # point_x = person[0]
                # point_y = person[1]
                distance = math.sqrt(point_x**2 + point_y**2)
                m = math.atan2(point_y, point_x) // angle_incre
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
                        a = 1+theta**2
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

    personal_space_lidar_data = PersonalSpaceLaserData()

    rclpy.spin(personal_space_lidar_data)

if __name__ == '__main__':
    main()
