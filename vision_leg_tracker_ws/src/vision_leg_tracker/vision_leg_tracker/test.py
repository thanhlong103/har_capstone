#!/usr/bin/env python3
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
import numpy as np
import cv2
import tensorflow as tf

def main(args=None):
    interpreter = tf.lite.Interpreter(model_path='/home/vision_leg_tracker_ws/src/vision_leg_tracker/vision_leg_tracker/model.tflite')
    interpreter.allocate_tensors()
    print("Model loaded!")

if __name__ == '__main__':
    main()
