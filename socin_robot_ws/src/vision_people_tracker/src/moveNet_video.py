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

cap = cv2.VideoCapture("/home/irs/har_capstone/socin_robot_ws/src/vision_people_tracker/src/walking_test.mp4")

label = "Warmup...."
n_time_steps = 10
lm_list = [[],[],[],[],[],[]]
prevAct = time.time()
i = 0
warmup_frames = 60

# Initialize prevtime for runtime calculation
prevtime = time.time()

# used to record the time when we processed last frame 
prev_frame_time = 0

# used to record the time at which we processed current frame 
new_frame_time = 0

fpsArr = []
sumfps = 0

def draw(frame, keypoints, edges, confidence_threshold, bbox):
    y, x, _ = frame.shape
    if bbox[4] > 0.15:
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
        if kp_conf >= confidence_threshold:
            cv2.circle(frame, (int(kx*x), int(ky*y)), 7, (0, 255, 0), -1)

    for edge, _ in edges.items():
        p1, p2 = edge
        kp1 = keypoints_k[p1]
        kp2 = keypoints_k[p2]

        # Unpack the coordinates and confidence for both keypoints

        y1, x1, c1 = kp1

        y2, x2, c2 = kp2



        # Draw the connection if both points have a confidence above the threshold

        if c1 > confidence_threshold and c2 > confidence_threshold:

            cv2.line(frame, (int(x1 * x), int(y1 * y)), (int(x2 * x), int(y2 * y)), (255, 0, 0), 2)



def estimator(frame, interpreter, prevtime):
    EDGES = {
        (0, 1): 'm',(0, 2): 'c',(1, 3): 'm',
        (2, 4): 'c',(0, 5): 'm',(0, 6): 'c',
        (5, 7): 'm',(7, 9): 'm',(6, 8): 'c',
        (8, 10): 'c',(5, 6): 'y',(5, 11): 'm',
        (6, 12): 'c',(11, 12): 'y',(11, 13): 'm',
        (13, 15): 'm',(12, 14): 'c',(14, 16): 'c'
    }

    img = frame.copy()
    input_size = 160
    img = tf.expand_dims(img, axis=0)
    resized_image, _ = keep_aspect_ratio_resizer(img, input_size)
    image_tensor = tf.cast(resized_image, dtype=tf.uint8)

    # Make predictions
    currenttime = time.time() 
    keypoints_with_scores = detect(interpreter, image_tensor)
    runtime = currenttime - prevtime
    print("Runtime: ", runtime)
    prevtime = currenttime

    return EDGES, keypoints_with_scores, prevtime

def keep_aspect_ratio_resizer(image, target_size):
    _, height, width, _ = image.shape
    scale = float(target_size / width)
    target_width = target_size
    scaled_height = math.ceil(height * scale)
    image = tf.image.resize(image, [scaled_height, target_width])
    target_height = int(math.ceil(scaled_height / 32) * 32)
    image = tf.image.pad_to_bounding_box(image, 0, 0, target_height, target_width)
    return (image, (target_height, target_width))

def detect(interpreter, input_tensor):
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

def detectEachPerson(keypoints_with_scores, img):
    for i in range(6):
        bbox = keypoints_with_scores[0][i][51:57]
        keypoints = keypoints_with_scores[0][i][:51]
    
        # Rendering 
        draw(img, keypoints, EDGES, 0.2, bbox)
    return img

try:
    interpreter = tf.lite.Interpreter(model_path='1.tflite')
    interpreter.allocate_tensors()
except:
    print("Can not access the model!")

while True:

    ret, img = cap.read()

    if ret:

        if True:
            EDGES, keypoints_with_scores, prevtime = estimator(img, interpreter, prevtime)

            detectEachPerson(keypoints_with_scores, img)

            # font which we will be using to display FPS 
            font = cv2.FONT_HERSHEY_SIMPLEX 

            # time when we finish processing for this frame 
            new_frame_time = time.time() 

            fps = 1/(new_frame_time-prev_frame_time) 
            prev_frame_time = new_frame_time 
            fps_str = "FPS: " + str(round(fps,2))

            # Update fpsArr and sumfps
            fpsArr.append(fps)
            sumfps = sum(fpsArr)
            fpsAvg = sumfps / len(fpsArr)

            if len(fpsArr) == 10:  # Reset every 10 frames
                print(f"Avg FPS: {fpsAvg}")
                fpsArr = []
                sumfps = 0

            cv2.putText(img, fps_str, (455, 30), font, 1, (100, 255, 0), 3, cv2.LINE_AA) 

            cv2.imshow("Image", img)

        if cv2.waitKey(1) == ord('q'):
            break
cv2.destroyAllWindows()
