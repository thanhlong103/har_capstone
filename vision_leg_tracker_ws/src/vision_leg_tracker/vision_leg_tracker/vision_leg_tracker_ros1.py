#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo
import numpy as np
import message_filters
import pandas as pd
import pyrealsense2 as rs2
import tensorflow as tf
from matplotlib import pyplot as plt
from roboflow import Roboflow
import supervision as sv
from realsense_depth import *
from std_msgs.msg import Header
from people_msgs.msg import People, Person

# This version track the human but does not record the data

offset_x = 150.0
offset_y = 37
offset_z = 0.0
offset = [offset_x, offset_y, offset_z]
real_width = 220.0  # (mm) of the box
threshold = 300  # (mm)
intrinsics = None

depth_sub = message_filters.Subscriber(
    "/camera/aligned_depth_to_color/image_raw", Image)
color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
color_info = message_filters.Subscriber(
    "/camera/aligned_depth_to_color/camera_info", CameraInfo)
scan_sub = message_filters.Subscriber("/scan", LaserScan)

image_pub = rospy.Publisher("/marked_image", Image, queue_size=10)
people_pub = rospy.Publisher("/people_detected_vision", People, queue_size=10)

# Load the model
interpreter = tf.lite.Interpreter(
    model_path='/home/turtlebot/catkin_ws/src/vision_leg_tracker/src/3.tflite')
interpreter.allocate_tensors()
print("Load the model!")


def draw_keypoints(frame, keypoints, confidence_threshold):
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


def draw_connections(frame, keypoints, edges, confidence_threshold):
    y, x, c = frame.shape
    shaped = np.squeeze(np.multiply(keypoints, [y, x, 1]))

    for edge, color in edges.items():
        p1, p2 = edge
        y1, x1, c1 = shaped[p1]
        y2, x2, c2 = shaped[p2]

        if (c1 > confidence_threshold) & (c2 > confidence_threshold):
            cv2.line(frame, (int(x1), int(y1)),
                     (int(x2), int(y2)), (0, 0, 255), 2)


def get_needed_keypoint(frame, keypoints, confidence):
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

    if (needkey[0][2] > confidence and needkey[1][2] > confidence):
        # print(needkey[0][1], needkey[1][1])
        legs_pixel = [[needkey[0][0], needkey[0][1]],
                      [needkey[1][0], needkey[1][1]]]
        return legs_pixel

    if (needkey[2][2] > confidence and needkey[3][2] > confidence):
        # print(needkey[2][1], needkey[3][1])
        legs_pixel = [[needkey[2][0], needkey[2][1]],
                      [needkey[3][0], needkey[3][1]]]
        return legs_pixel

    if (needkey[4][2] > confidence and needkey[5][2] > confidence):
        # print(needkey[4][1], needkey[5][1])
        legs_pixel = [[needkey[4][0], needkey[4][1]],
                      [needkey[5][0], needkey[5][1]]]
        return legs_pixel

    return [[0, 0], [0, 0]]


def estimator(frame, interpreter):
    """Return an image with marked keypoints and their pixel coordinates"""
    EDGES = {
        (0, 1): 'm',
        (0, 2): 'c',
        (1, 3): 'm',
        (2, 4): 'c',
        (0, 5): 'm',
        (0, 6): 'c',
        (5, 7): 'm',
        (7, 9): 'm',
        (6, 8): 'c',
        (8, 10): 'c',
        (5, 6): 'y',
        (5, 11): 'm',
        (6, 12): 'c',
        (11, 12): 'y',
        (11, 13): 'm',
        (13, 15): 'm',
        (12, 14): 'c',
        (14, 16): 'c'
    }
    # Reshape image
    img = frame.copy()
    img = tf.image.resize_with_pad(np.expand_dims(img, axis=0), 192, 192)
    input_image = tf.cast(img, dtype=tf.float32)

    # Setup input and output
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Make predictions
    interpreter.set_tensor(input_details[0]['index'], np.array(input_image))
    interpreter.invoke()
    keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])
    # print(keypoints_with_scores)
    leg_pixels = get_needed_keypoint(frame, keypoints_with_scores, 0.4)

    # Rendering
    draw_connections(frame, keypoints_with_scores, EDGES, 0.4)
    draw_keypoints(frame, keypoints_with_scores, 0.4)

    frame = cv2.flip(frame, 1)

    return frame, leg_pixels


def leg2person(leg_worlds):
    """Convert legs coordinate to person coordinate. Return an array of detected persons"""
    person_array = []
    if leg_worlds != []:
        # Code to threshold legs to person
        dis_legs = abs(leg_worlds[0][1] - leg_worlds[1][1])
        if dis_legs <= threshold:
            x = (leg_worlds[0][0] + leg_worlds[1][0]) / 2000
            y = (leg_worlds[0][1] + leg_worlds[1][1]) / 2000
            z = 0.0
            person_array.append([x, y, z])

    return person_array


def talker(people_array, people_pub):
    pubPeople = People()
    pubPeople.header = Header()
    pubPeople.header.stamp = rospy.Time.now()
    pubPeople.header.frame_id = "base_scan"
    pubPeople.people = []

    if people_array != []:
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

    people_pub.publish(pubPeople)


def localization(offset, intrinsics, legs, depth_array):
    legs_world = []
    if legs != [[0, 0], [0, 0]]:
        for leg in legs:
            x_pixel = int(leg[0])
            y_pixel = int(leg[1])
            depth = depth_array[y_pixel, x_pixel]  # row index, column index
            coordinate_camera = rs2.rs2_deproject_pixel_to_point(
                intrinsics, [x_pixel, y_pixel], depth)
            # print(x_pixel, y_pixel, depth, coordinate_camera)
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


def depth_scan_callback(depth, color, color_info, scan):
    # assert image.header.stamp == scan.header.stamp
    try:
        # Convert Image message type from ROS to CV2 image
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(
            depth, desired_encoding="passthrough")
        depth_image3 = cv2.merge([depth_image, depth_image, depth_image])

        color_image = bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")

        # Compute the depth
        depth_array = np.array(depth_image, dtype=np.float32)

        intrinsics = rs2.intrinsics()
        intrinsics.width = color_info.width
        intrinsics.height = color_info.height
        intrinsics.ppx = color_info.K[2]
        intrinsics.ppy = color_info.K[5]
        intrinsics.fx = color_info.K[0]
        intrinsics.fy = color_info.K[4]

        estimated_frame, legs_pixel = estimator(color_image, interpreter)
        # print("Legs pixel: ",legs_pixel)

        # Camera info
        if color_info.distortion_model == 'plumb_bob':
            intrinsics.model = rs2.distortion.brown_conrady
        elif color_info.distortion_model == 'equidistant':
            intrinsics.model = rs2.distortion.kannala_brandt4
        intrinsics.coeffs = [i for i in color_info.D]

        # Compute a person world coordinate
        # person_pixel = np.array(depth_array.shape) / 2 # fake pixel at the
        # center of the image (row, column)
        legs_world = localization(offset, intrinsics, legs_pixel, depth_array)
        person_array = leg2person(legs_world)
        # print("Leg worlds: ", legs_world)
        print("People array: ", person_array)

        color_pub = bridge.cv2_to_imgmsg(estimated_frame, "bgr8")

        image_pub.publish(color_pub)

        talker(person_array, people_pub)

    except CvBridgeError as e:
        print(e)


def main():
    ats = message_filters.ApproximateTimeSynchronizer(
        [depth_sub, color_sub, color_info, scan_sub], queue_size=10, slop=0.1)
    ats.registerCallback(depth_scan_callback)


if __name__ == '__main__':
    rospy.init_node('vision_tracker')
    main()
    rospy.spin()
