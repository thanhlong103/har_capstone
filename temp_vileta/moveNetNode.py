import rclpy
from rclpy.node import Node
import cv2
import tensorflow as tf
import numpy as np
import pyrealsense2 as rs
from tensorflow.keras.models import load_model
import math
import time
import os 
import pandas as pd
from matplotlib import pyplot as plt
from tflite_runtime.interpreter import load_delegate

class VisionLegTracker(Node):
    def __init__(self):
        super().__init__('vision_leg_tracker_node')
        self.pipe = rs.pipeline()
        self.cfg  = rs.config()

        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30) 

        self.label = "Warmup...."
        self.n_time_steps = 10
        self.lm_list = [[],[],[],[],[],[]]
        self.prevAct = time.time()
        self.i = 0
        self.warmup_frames = 60

        #cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.pipe.start(self.cfg)
        # Initialize prevtime for runtime calculation
        self.prevtime = time.time()

        # used to record the time when we processed last frame 
        self.prev_frame_time = 0

        # used to record the time at which we processed current frame 
        self.new_frame_time = 0

        self.fpsArr = []
        self.sumfps = 0

        self.confidence_threshold = 0.4

        self.EDGES = {
            (0, 1): 'm',(0, 2): 'c',(1, 3): 'm',
            (2, 4): 'c',(0, 5): 'm',(0, 6): 'c',
            (5, 7): 'm',(7, 9): 'm',(6, 8): 'c',
            (8, 10): 'c',(5, 6): 'y',(5, 11): 'm',
            (6, 12): 'c',(11, 12): 'y',(11, 13): 'm',
            (13, 15): 'm',(12, 14): 'c',(14, 16): 'c'
        }

        self.input_size = 256

        self.offset_x = 150.0
        self.offset_y = 37
        self.offset_z = 0.0
        self.offset = [self.offset_x, self.offset_y, self.offset_z]
        self.real_width = 220.0  # (mm) of the box
        self.threshold = 300  # (mm)
        self.intrinsics = None

        try:
        # self.interpreter = tf.lite.Interpreter(
        #     model_path="1.tflite",
        #     experimental_delegates=[
        #         load_delegate("libnvdsinfer_custom_impl.so")
        #     ]
        # )
        # self.interpreter.allocate_tensors()

            self.interpreter = tf.lite.Interpreter(model_path='1.tflite')
            self.interpreter.allocate_tensors()
            self.get_logger().info('MoveNet loaded!')
        except:
            print("Can not access the model!")

        self.get_logger().info('Vision Leg Tracker Node has started.')

    def draw(self, frame, keypoints, bbox):
        y, x, _ = frame.shape
        if bbox[4] > self.confidence_threshold:
            startpoint = (int(bbox[1]*x), int(bbox[0]*y))
            endpoint = (int(bbox[3]*x), int(bbox[2]*y))
            thickness = 2

            # Blue color in BGR
            color = (255, 0, 0)
            cv2.rectangle(frame, startpoint, endpoint, color, thickness)

        if keypoints.shape[0] == 51:
            keypoints_k = keypoints.reshape(-1, 3)

        for kp in keypoints_k:
            ky, kx, kp_conf = kp

            # Draw the keypoint if confidence is above the threshold
            if kp_conf >= self.confidence_threshold:
                cv2.circle(frame, (int(kx*x), int(ky*y)), 7, (0, 255, 0), -1)

        for edge, _ in self.EDGES.items():
            p1, p2 = edge
            kp1 = keypoints_k[p1]
            kp2 = keypoints_k[p2]

            # Unpack the coordinates and confidence for both keypoints
            y1, x1, c1 = kp1
            y2, x2, c2 = kp2

            # Draw the connection if both points have a confidence above the threshold
            if c1 > self.confidence_threshold and c2 > self.confidence_threshold:
                cv2.line(frame, (int(x1 * x), int(y1 * y)), (int(x2 * x), int(y2 * y)), (255, 0, 0), 2)

    def estimator(self, frame, interpreter, prevtime):
        img = frame.copy()
        img = tf.expand_dims(img, axis=0)
        resized_image, _ = self.keep_aspect_ratio_resizer(img, self.input_size)
        image_tensor = tf.cast(resized_image, dtype=tf.uint8)

        # Make predictions
        currenttime = time.time() 
        keypoints_with_scores = self.detect(interpreter, image_tensor)
        runtime = currenttime - prevtime
        print("Runtime: ", runtime)
        prevtime = currenttime

        return keypoints_with_scores, prevtime

    def keep_aspect_ratio_resizer(self, image, target_size):
        _, height, width, _ = image.shape
        scale = float(target_size / width)
        target_width = target_size
        scaled_height = math.ceil(height * scale)
        image = tf.image.resize(image, [scaled_height, target_width])
        target_height = int(math.ceil(scaled_height / 32) * 32)
        image = tf.image.pad_to_bounding_box(image, 0, 0, target_height, target_width)
        return (image, (target_height, target_width))

    def detect(self, interpreter, input_tensor):
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        is_dynamic_shape_model = input_details[0]['shape_signature'][2] == -1

        if is_dynamic_shape_model:
            input_tensor_index = input_details[0]['index']
            input_shape = input_tensor.shape
            interpreter.resize_tensor_input(input_tensor_index, input_shape, strict=True)

        interpreter.allocate_tensors()
        interpreter.set_tensor(input_details[0]['index'], input_tensor.numpy())

        interpreter.invoke()

        keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])

        return keypoints_with_scores

    def detectEachPerson(self, keypoints_with_scores, img):
        for i in range(6):
            bbox = keypoints_with_scores[0][i][51:57]
            keypoints = keypoints_with_scores[0][i][:51]
        
            # Rendering 
            self.draw(img, keypoints, bbox)
        return img
    
    def localization(self, frame, offset, intrinsics, bbox, depth_array):
        y, x, _ = frame.shape
        if bbox[4] > self.confidence_threshold:
            startpoint = (int(bbox[1]*x), int(bbox[0]*y))
            endpoint = (int(bbox[3]*x), int(bbox[2]*y))
            x_pixel = int((startpoint[0]+endpoint[0])/2)
            y_pixel = int((startpoint[1]+endpoint[1])/2)
            depth = depth_array[y_pixel, x_pixel]  # row index, column index
            coordinate_camera = rs.rs2_deproject_pixel_to_point(
                intrinsics, [x_pixel, y_pixel], depth)
            
            print(x_pixel, y_pixel, depth, coordinate_camera)
            # print(coordinate_camera)

            # Remapping from camera to world
            x_world = coordinate_camera[0]
            y_world = (coordinate_camera[1])
            z_world = coordinate_camera[2] 
            person_coor = [x_world, y_world, z_world]
            return person_coor
        return []

    def processImage(self):
        frame = self.pipe.wait_for_frames()
        color_frame = frame.get_color_frame()
        depth_frame = frame.get_depth_frame()
        img = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Initialize camera intrinsics if not done
        if self.intrinsics is None:
            self.intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        keypoints_with_scores, self.prevtime = self.estimator(img, self.interpreter, self.prevtime)

        # Detect and process keypoints for each person
        self.detectEachPerson(keypoints_with_scores, img)

        # Localization to get the world coordinates
        person_world_coords = []
        for i in range(6):
            bbox = keypoints_with_scores[0][i][51:57]
            if bbox[4] > self.confidence_threshold:  # Only proceed if confidence is above threshold
                y, x, _ = img.shape
                # Get the 3D world coordinates of the person
                world_coords = self.localization(img, self.offset, self.intrinsics, bbox, depth_image)
                startpoint = (int(bbox[1]*x), int(bbox[0]*y))
                endpoint = (int(bbox[3]*x), int(bbox[2]*y))
                x_pixel = int((startpoint[0]+endpoint[0])/2)
                y_pixel = int((startpoint[1]+endpoint[1])/2)
                cv2.circle(img, (x_pixel,y_pixel), 5, (0, 0, 255), -1)  # Red circle
                if world_coords:
                    person_world_coords.append(world_coords)

        # Log the world coordinates for debugging
        if person_world_coords:
            for i, coords in enumerate(person_world_coords):
                print(f"Person {i+1} World Coordinates: {coords}")

        # Display FPS
        font = cv2.FONT_HERSHEY_SIMPLEX
        self.new_frame_time = time.time()
        fps = 1 / (self.new_frame_time - self.prev_frame_time)
        self.prev_frame_time = self.new_frame_time
        fps_str = "FPS: " + str(round(fps, 2))

        # Update fpsArr and sumfps
        self.fpsArr.append(fps)
        sumfps = sum(self.fpsArr)
        fpsAvg = sumfps / len(self.fpsArr)

        if len(self.fpsArr) == 10:  # Reset every 10 frames
            print(f"Avg FPS: {fpsAvg}")
            self.fpsArr = []
            sumfps = 0

        cv2.putText(img, fps_str, (455, 30), font, 1, (100, 255, 0), 3, cv2.LINE_AA)
        cv2.imshow("Image", img)
        cv2.imshow('depth', depth_image)

        if cv2.waitKey(1) == ord('q'):
            self.pipe.stop()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def run(self):
        while rclpy.ok():
            self.processImage()

def main(args=None):
    rclpy.init(args=args)
    node = VisionLegTracker()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()