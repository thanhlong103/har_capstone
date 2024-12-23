#!/usr/bin/env python3
import tensorflow as tf
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan, CameraInfo
import numpy as np
import message_filters 
import pandas as pd 
import pyrealsense2 as rs2
from matplotlib import pyplot as plt
# from realsense_depth import *
from std_msgs.msg import Header
# from people_msgs.msg import People, Person # should implement the custom_message

offset_x = 150.0
offset_y = 37
offset_z = 0.0
offset = [offset_x, offset_y, offset_z]
real_width = 220.0  # (mm) of the box
threshold = 300  # (mm)
intrinsics = None

class VisionTrackerNode(Node):
    def __init__(self):
        super().__init__('vision_tracker')
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.depth_sub = message_filters.Subscriber(self, Image, "/camera/aligned_depth_to_color/image_raw")
        self.color_sub = message_filters.Subscriber(self, Image, "/camera/color/image_raw")
        self.color_info_sub = message_filters.Subscriber(self, CameraInfo, "/camera/aligned_depth_to_color/camera_info")
        # self.scan_sub = message_filters.Subscriber(self, LaserScan, "/scan") # add later in the self.ats
        
        # Publishers
        # self.image_pub = self.create_publisher(Image, "/marked_image", 10)
        # self.people_pub = self.create_publisher(People, "/people_detected_vision", 10)
        
        # Load the model
        self.interpreter = tf.lite.Interpreter(model_path='/home/vision_leg_tracker_ws/src/vision_leg_tracker/vision_leg_tracker/model.tflite')
        self.interpreter.allocate_tensors()
        print("Model loaded!")

        # Synchronize messages
        self.ats = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub, self.color_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ats.registerCallback(self.depth_scan_callback)

    def draw_keypoints(self, frame, keypoints, confidence_threshold):
        y, x, c = frame.shape
        shaped = np.squeeze(np.multiply(keypoints, [y, x, 1]))

        needkey = []
        for i in range(11, 17):
            kp = shaped[i]
            ky, kx, kp_conf = kp
            if kp_conf >= confidence_threshold:
                needkey.append([kx, ky])
                cv2.circle(frame, (int(kx), int(ky)), 7, (0, 255, 0), -1)
            else:
                needkey.append([0, 0])

    def draw_connections(self, frame, keypoints, edges, confidence_threshold):
        y, x, c = frame.shape
        shaped = np.squeeze(np.multiply(keypoints, [y, x, 1]))
        
        for edge, color in edges.items():
            p1, p2 = edge
            y1, x1, c1 = shaped[p1]
            y2, x2, c2 = shaped[p2]
            
            if (c1 > confidence_threshold) & (c2 > confidence_threshold):
                cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

    def get_needed_keypoint(self, frame, keypoints, confidence):
        y, x, c = frame.shape
        shaped = np.squeeze(np.multiply(keypoints, [y, x, 1]))

        needkey = []
        legs_pixel = []

        for i in range(11, 17):
            kp = shaped[i]
            ky, kx, kp_conf = kp
            if kp_conf >= confidence:
                needkey.append([kx, ky, kp_conf])
                cv2.circle(frame, (int(kx), int(ky)), 7, (0, 255, 0), -1)
            else:
                needkey.append([0, 0, 0])

        if needkey[0][2] > confidence and needkey[1][2] > confidence:
            legs_pixel = [[needkey[0][0], needkey[0][1]], [needkey[1][0], needkey[1][1]]]
            return legs_pixel
        elif needkey[2][2] > confidence and needkey[3][2] > confidence:
            legs_pixel = [[needkey[2][0], needkey[2][1]], [needkey[3][0], needkey[3][1]]]
            return legs_pixel
        elif needkey[4][2] > confidence and needkey[5][2] > confidence:
            legs_pixel = [[needkey[4][0], needkey[4][1]], [needkey[5][0], needkey[5][1]]]
            return legs_pixel

        return [[0, 0], [0, 0]]

    def estimator(self, frame):
        EDGES = {
            (0, 1): 'm', (0, 2): 'c', (1, 3): 'm', (2, 4): 'c',
            (0, 5): 'm', (0, 6): 'c', (5, 7): 'm', (7, 9): 'm',
            (6, 8): 'c', (8, 10): 'c', (5, 6): 'y', (5, 11): 'm',
            (6, 12): 'c', (11, 12): 'y', (11, 13): 'm', (13, 15): 'm',
            (12, 14): 'c', (14, 16): 'c'
        }

        img = frame.copy()
        img = tf.image.resize_with_pad(np.expand_dims(img, axis=0), 192, 192)
        input_image = tf.cast(img, dtype=tf.float32)

        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        self.interpreter.set_tensor(input_details[0]['index'], np.array(input_image))
        self.interpreter.invoke()
        keypoints_with_scores = self.interpreter.get_tensor(output_details[0]['index'])

        leg_pixels = self.get_needed_keypoint(frame, keypoints_with_scores, 0.4)
        self.draw_connections(frame, keypoints_with_scores, EDGES, 0.4)
        self.draw_keypoints(frame, keypoints_with_scores, 0.4)

        return cv2.flip(frame, 1), leg_pixels

    def talker(self, people_array):
        pubPeople = People()
        pubPeople.header = Header()
        pubPeople.header.stamp = self.get_clock().now().to_msg()
        pubPeople.header.frame_id = "base_scan"
        pubPeople.people = []
        
        if people_array:
            for person in people_array:
                pubPerson = Person()
                pubPerson.position.x = person[0]
                pubPerson.position.y = person[1]
                pubPerson.position.z = person[2]
                pubPerson.velocity.x = 0.0
                pubPerson.velocity.y = 0.0
                pubPerson.velocity.z = 0.0
                pubPerson.reliability = 0.8
                pubPeople.people.append(pubPerson)
        
        self.people_pub.publish(pubPeople)
    def leg2person(self, leg_worlds):
        """Convert legs coordinate to person coordinate. Return an array of detected persons"""
        person_array = []
        if leg_worlds != []:
            # Code to threshold legs to person
            dis_legs = abs(leg_worlds[0][1] - leg_worlds[1][1])
            if dis_legs <= threshold:
                x = (leg_worlds[0][0] + leg_worlds[1][0])/2000
                y = (leg_worlds[0][1] + leg_worlds[1][1])/2000
                z = 0.0
                person_array.append([x, y, z])

        return person_array

    def localization(self, offset, intrinsics, legs, depth_array):
        legs_world = []
        if legs != [[0,0],[0,0]]:
            for leg in legs:
                x_pixel = int(leg[0])
                y_pixel = int(leg[1])
                depth = depth_array[y_pixel, x_pixel] # row index, column index
                coordinate_camera = rs2.rs2_deproject_pixel_to_point(intrinsics, [x_pixel, y_pixel], depth)
                # print(coordinate_camera)
                # Remapping from camera to world 
                x_world = coordinate_camera[2] + offset[0]
                y_world = -((coordinate_camera[0]) - offset[1])
                z_world = coordinate_camera[1] + offset[2]
                leg_world = [x_world, y_world, z_world]
                legs_world.append(leg_world)
            return legs_world
        else: 
            return legs_world

    def depth_scan_callback(self, depth, color, color_info):
        try:
            print("Receive depth, color, color camera info, scan topics")
            depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
            depth_image3 = cv2.merge([depth_image, depth_image, depth_image])

            color_image = self.bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
            depth_array = np.array(depth_image, dtype=np.float32)
            plt.imshow(color_image)
            plt.axis('off')  # Hide the axes
            plt.show()

            # intrinsics = rs2.intrinsics()
            # intrinsics.width = color_info.width
            # intrinsics.height = color_info.height
            # intrinsics.ppx = color_info.k[2]
            # intrinsics.ppy = color_info.k[5]
            # intrinsics.fx = color_info.k[0]
            # intrinsics.fy = color_info.k[4]

            # estimated_frame, legs_pixel = self.estimator(color_image)
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(estimated_frame, "bgr8"))

            # legs_world = self.localization(offset, intrinsics, legs_pixel, depth_array)
            # person_array = self.leg2person(legs_world)
            # self.talker(person_array)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

def main(args=None):
    print("Intialize vision  tracker node")
    rclpy.init(args=args)
    node = VisionTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
