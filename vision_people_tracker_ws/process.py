import cv2
import tensorflow as tf
import numpy as np
import os
import math

def keep_aspect_ratio_resizer(image, target_size):
    """Resizes and pads the image to maintain aspect ratio, as per MoveNet's requirement."""
    _, height, width, _ = image.shape
    scale = float(target_size / width)
    scaled_height = math.ceil(height * scale)
    resized_image = tf.image.resize(image, [scaled_height, target_size])
    target_height = int(math.ceil(scaled_height / 32) * 32)
    resized_image = tf.image.pad_to_bounding_box(resized_image, 0, 0, target_height, target_size)
    return resized_image, (target_height, target_size)

def detect(interpreter, input_tensor):
    """Runs inference using the TensorFlow Lite model."""
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    if input_details[0]['shape_signature'][2] == -1:  # Check for dynamic shape
        input_shape = input_tensor.shape
        interpreter.resize_tensor_input(input_details[0]['index'], input_shape)
    
    interpreter.allocate_tensors()
    interpreter.set_tensor(input_details[0]['index'], input_tensor.numpy())
    interpreter.invoke()
    keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])
    return keypoints_with_scores

def process_video(video_path, interpreter, output_file):
    """Processes each video to extract and save keypoints."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error opening video file {video_path}")
        return

    input_size = 128  # MoveNet's input size

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert BGR to RGB and preprocess
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = tf.expand_dims(img, axis=0)
        resized_img, _ = keep_aspect_ratio_resizer(img, input_size)
        input_tensor = tf.cast(resized_img, dtype=tf.uint8)

        # Run pose estimation
        keypoints_with_scores = detect(interpreter, input_tensor)

        # Select person with highest bbox confidence
        max_conf = 0.0
        selected_idx = -1
        for i in range(6):
            if keypoints_with_scores[0, i, 55] > max_conf:  # bbox confidence at index 55
                max_conf = keypoints_with_scores[0, i, 55]
                selected_idx = i

        # Check if valid detection
        if selected_idx != -1 and max_conf > 0.15:
            keypoints = keypoints_with_scores[0, selected_idx, :51].flatten()
            line = ','.join(map(str, keypoints)) + '\n'
            output_file.write(line)
        else:
            continue  # Skip frame if no valid person

    cap.release()

def main():
    interpreter = tf.lite.Interpreter(model_path='1.tflite')  # Load MoveNet model
    interpreter.allocate_tensors()

    actions = ['walking', 'talking', 'walking_phone']  # Action categories
    # actions = ['walking_phone']  # Action categories

    for action in actions:
        output_filename = f"{action}.txt"
        with open(output_filename, 'w') as output_file:
            video_dir = os.path.join(os.getcwd(), action)
            if not os.path.isdir(video_dir):
                print(f"Skipping missing directory: {video_dir}")
                continue

            video_files = [f for f in os.listdir(video_dir) if f.endswith('.mp4')]
            for video_file in video_files:
                video_path = os.path.join(video_dir, video_file)
                print(f"Processing {video_path}")
                process_video(video_path, interpreter, output_file)

if __name__ == "__main__":
    main()