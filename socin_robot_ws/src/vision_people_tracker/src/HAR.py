import rclpy
from rclpy.node import Node
import cv2
import tensorflow as tf
import numpy as np
# import pyrealsense2 as rs
from tensorflow.keras.models import load_model
import math
import time
import pandas as pd
import os
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseArray, Pose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# from sklearn.decomposition import PCA


class VisionLegTracker(Node):
    def __init__(self):
        super().__init__("HAR_node")

        self.label = "Warmup...."
        self.n_time_steps = 10
        self.lm_list = [[], [], [], [], [], []]
        self.label = [None, None, None, None, None, None]
        self.prevAct = time.time()
        self.i = 0
        self.warmup_frames = 60

        self.cap = cv2.VideoCapture("walking_phone_test.mp4")

        # Initialize prevtime for runtime calculation
        self.prevtime = time.time()

        # used to record the time when we processed last frame
        self.prev_frame_time = 0

        # used to record the time at which we processed current frame
        self.new_frame_time = 0

        self.fpsArr = []
        self.sumfps = 0

        self.confidence_threshold = 0.25
        self.bbox_threshold = 0.125

        self.EDGES = {
            (0, 1): "m",
            (0, 2): "c",
            (1, 3): "m",
            (2, 4): "c",
            (0, 5): "m",
            (0, 6): "c",
            (5, 7): "m",
            (7, 9): "m",
            (6, 8): "c",
            (8, 10): "c",
            (5, 6): "y",
            (5, 11): "m",
            (6, 12): "c",
            (11, 12): "y",
            (11, 13): "m",
            (13, 15): "m",
            (12, 14): "c",
            (14, 16): "c",
        }

        self.input_size = 128

        try:
            self.interpreter = tf.lite.Interpreter(model_path="1.tflite")
            # self.interpreter = tf.lite.Interpreter(model_path="1.tflite")
            self.interpreter.allocate_tensors()
            self.get_logger().info("MoveNet loaded!")
        except:
            print("Can not access the MoveNet!")

        # Replace the HAR model loading section with:
        try:
            # Load quantized HAR model
            self.har_interpreter = tf.lite.Interpreter(model_path="model_quantized_custom.tflite")
            self.har_interpreter.allocate_tensors()
            self.har_input_details = self.har_interpreter.get_input_details()
            self.har_output_details = self.har_interpreter.get_output_details()
            self.get_logger().info("HAR model loaded!")
        except Exception as e:
            self.get_logger().error(f"Failed to load HAR model: {e}")
            raise

        self.get_logger().info("Vision Leg Tracker Node has started.")

        ret, frame = self.cap.read()

        # self.HEIGHT, self.WIDTH, _ = frame.shape
        self.WIDTH = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.HEIGHT = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def draw(self, frame, keypoints, bbox):
        if bbox[4] > self.bbox_threshold:
            startpoint = (int(bbox[1] * self.WIDTH), int(bbox[0] * self.HEIGHT))
            endpoint = (int(bbox[3] * self.WIDTH), int(bbox[2] * self.HEIGHT))
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
                cv2.circle(
                    frame,
                    (int(kx * self.WIDTH), int(ky * self.HEIGHT)),
                    7,
                    (0, 255, 0),
                    -1,
                )

        for edge, _ in self.EDGES.items():
            p1, p2 = edge
            kp1 = keypoints_k[p1]
            kp2 = keypoints_k[p2]

            # Unpack the coordinates and confidence for both keypoints
            y1, x1, c1 = kp1
            y2, x2, c2 = kp2

            # Draw the connection if both points have a confidence above the threshold
            if c1 > self.confidence_threshold and c2 > self.confidence_threshold:
                cv2.line(
                    frame,
                    (int(x1 * self.WIDTH), int(y1 * self.HEIGHT)),
                    (int(x2 * self.WIDTH), int(y2 * self.HEIGHT)),
                    (255, 0, 0),
                    2,
                )

    def estimator(self, frame):
        img = frame.copy()
        img = tf.expand_dims(img, axis=0)
        resized_image, _ = self.keep_aspect_ratio_resizer(img, self.input_size)
        image_tensor = tf.cast(resized_image, dtype=tf.uint8)

        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        is_dynamic_shape_model = input_details[0]["shape_signature"][2] == -1

        if is_dynamic_shape_model:
            input_tensor_index = input_details[0]["index"]
            input_shape = image_tensor.shape
            self.interpreter.resize_tensor_input(
                input_tensor_index, input_shape, strict=True
            )

        self.interpreter.allocate_tensors()
        self.interpreter.set_tensor(input_details[0]["index"], image_tensor.numpy())

        self.interpreter.invoke()

        keypoints_with_scores = self.interpreter.get_tensor(output_details[0]["index"])

        return keypoints_with_scores

    def keep_aspect_ratio_resizer(self, image, target_size):
        _, height, width, _ = image.shape
        scale = float(target_size / width)
        target_width = target_size
        scaled_height = math.ceil(height * scale)
        image = tf.image.resize(image, [scaled_height, target_width])
        target_height = int(math.ceil(scaled_height / 32) * 32)
        image = tf.image.pad_to_bounding_box(image, 0, 0, target_height, target_width)
        return (image, (target_height, target_width))

    def make_landmark_timestep(self, keypoints_with_scores):
        c_lm = []  # Start with an empty list
        keypoints_k = keypoints_with_scores.reshape(-1, 3)

        for i in range(0, 17):
            c_lm.append(keypoints_k[i])  # Append each keypoint as a list

        c_lm = np.concatenate(c_lm)
        return c_lm

    def detectAct(self, lm_list):
        lm_list = np.array(lm_list, dtype=np.float32)  # Ensure correct dtype
        lm_list = np.expand_dims(lm_list, axis=0)  # Shape: (1, 10, 51)

        # Set input tensor
        self.har_interpreter.set_tensor(
            self.har_input_details[0]['index'], lm_list
        )
        
        # Run inference
        self.har_interpreter.invoke()
        
        # Get output
        output = self.har_interpreter.get_tensor(
            self.har_output_details[0]['index']
        )
        results = np.argmax(output)

        print(output)

        # Label handling remains unchanged
        if results == 0:
            label = "Walking"
        elif results == 1:
            label = "Talking"
        elif results == 2:
            label = "Walking Phone"
        elif results == 3:
            label = "JOGGING"
        elif results == 4:
            label = "RUNNING"
        else:
            label = "WALKING"
        return label
    
    def draw_class_on_image(self, label, img, bbox):
        y, x, _ = img.shape
        font = cv2.FONT_HERSHEY_SIMPLEX
        position = (int(bbox[1] * x), int(bbox[0] * y - 10))
        fontScale = 1
        fontColor = (0, 255, 0)
        thickness = 2
        lineType = 2
        # print(position)
        cv2.putText(
            img, label, (10,30), font, fontScale, fontColor, thickness, lineType
        )

    def processImage(self):
        ret, img = self.cap.read()

        if ret:
            keypoints_with_scores = self.estimator(img)

            for i in range(6):
                bbox = keypoints_with_scores[0][i][51:57]
                keypoints = keypoints_with_scores[0][i][:51]

                self.draw(img, keypoints, bbox)

                # print(keypoints)

                if bbox[4] > self.bbox_threshold:
                    # print(left_shoulder, right_shoulder)

                    c_lm = self.make_landmark_timestep(keypoints)
                    if len(c_lm) > 0:
                        self.lm_list[i].append(c_lm)
                    if len(self.lm_list[i]) == self.n_time_steps:
                        label = self.detectAct(self.lm_list[i])
                        self.label[i] = label
                        self.lm_list[i] = []

                    if self.label[i] != None:
                        self.draw_class_on_image(self.label[i], img, bbox)

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
                # print(f"Avg FPS: {fpsAvg}")
                self.fpsArr = []
                sumfps = 0

            cv2.putText(img, fps_str, (455, 30), font, 1, (100, 255, 0), 3, cv2.LINE_AA)

            cv2.imshow("People Detected", img)

            if cv2.waitKey(1) == ord("q"):
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


if __name__ == "__main__":
    main()
