import cv2
import tensorflow as tf
import numpy as np
import pyrealsense2 as rs
import math
import time

class ActivityDetector:
    def __init__(self, model_path, tflite_path, input_size=128, n_time_steps=10, confidence_threshold=0.2):
        self.label = "Warmup...."
        self.model_path = model_path
        self.tflite_path = tflite_path
        self.input_size = input_size
        self.n_time_steps = n_time_steps
        self.confidence_threshold = confidence_threshold
        self.lm_list = [[], [], [], [], [], []]
        self.prevAct = time.time()
        self.fpsArr = []
        self.sumfps = 0
        self.prevtime = time.time()
        self.warmup_frames = 60
        self.frame_count = 0

        # Initialize RealSense pipeline
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipe.start(cfg)

        # Load models
        self.model = tf.saved_model.load(self.model_path)
        self.infer = self.model.signatures["serving_default"]
        self.interpreter = self._load_tflite_model()

        # FPS calculation
        self.prev_frame_time = 0
        self.new_frame_time = 0

        # Edge connections for skeleton drawing
        self.EDGES = {
            (0, 1): 'm', (0, 2): 'c', (1, 3): 'm', (2, 4): 'c',
            (0, 5): 'm', (0, 6): 'c', (5, 7): 'm', (7, 9): 'm',
            (6, 8): 'c', (8, 10): 'c', (5, 6): 'y', (5, 11): 'm',
            (6, 12): 'c', (11, 12): 'y', (11, 13): 'm', (13, 15): 'm',
            (12, 14): 'c', (14, 16): 'c'
        }

    def _load_tflite_model(self):
        try:
            interpreter = tf.lite.Interpreter(model_path=self.tflite_path)
            interpreter.allocate_tensors()
            return interpreter
        except Exception as e:
            print("Cannot access the model:", e)
            return None

    def keep_aspect_ratio_resizer(self, image, target_size):
        _, height, width, _ = image.shape
        scale = float(target_size / width)
        target_width = target_size
        scaled_height = math.ceil(height * scale)
        image = tf.image.resize(image, [scaled_height, target_width])
        target_height = int(math.ceil(scaled_height / 32) * 32)
        image = tf.image.pad_to_bounding_box(image, 0, 0, target_height, target_width)
        return image, (target_height, target_width)

    def detect_keypoints(self, input_tensor):
        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        # Dynamic shape handling
        is_dynamic = input_details[0]['shape_signature'][2] == -1
        if is_dynamic:
            self.interpreter.resize_tensor_input(input_details[0]['index'], input_tensor.shape, strict=True)
        self.interpreter.allocate_tensors()

        self.interpreter.set_tensor(input_details[0]['index'], input_tensor.numpy())
        self.interpreter.invoke()
        keypoints = self.interpreter.get_tensor(output_details[0]['index'])
        return keypoints

    def draw_skeleton(self, frame, keypoints, bbox):
        y, x, _ = frame.shape
        keypoints_k = keypoints.reshape(-1, 3)

        # Draw bounding box
        if bbox[4] > self.confidence_threshold:
            startpoint = (int(bbox[1]*x), int(bbox[0]*y))
            endpoint = (int(bbox[3]*x), int(bbox[2]*y))
            cv2.rectangle(frame, startpoint, endpoint, (255, 0, 0), 2)

        # Draw keypoints and edges
        for kp in keypoints_k:
            ky, kx, kp_conf = kp
            if kp_conf >= self.confidence_threshold:
                cv2.circle(frame, (int(kx*x), int(ky*y)), 7, (0, 255, 0), -1)

        for edge in self.EDGES:
            p1, p2 = edge
            kp1, kp2 = keypoints_k[p1], keypoints_k[p2]
            if kp1[2] > self.confidence_threshold and kp2[2] > self.confidence_threshold:
                cv2.line(frame, (int(kp1[1]*x), int(kp1[0]*y)), (int(kp2[1]*x), int(kp2[0]*y)), (255, 0, 0), 2)

    def detect_activity(self, lm_list):
        lm_list = np.expand_dims(np.array(lm_list), axis=0)
        results = np.argmax(self.infer(keras_tensor=tf.constant(lm_list, dtype=tf.float32))["output_0"].numpy())
        return ["HANDCLAPPING", "BOXING", "HANDWAVING", "JOGGING", "RUNNING", "WALKING"][results]

    def process_frame(self, frame):
        img = tf.expand_dims(frame, axis=0)
        resized_img, _ = self.keep_aspect_ratio_resizer(img, self.input_size)
        keypoints = self.detect_keypoints(tf.cast(resized_img, dtype=tf.uint8))

        for i in range(6):
            bbox = keypoints[0][i][51:57]
            if bbox[4] > self.confidence_threshold:
                lm = keypoints[0][i][:51].reshape(-1, 3).flatten()
                self.lm_list[i].append(lm)
                if len(self.lm_list[i]) == self.n_time_steps:
                    self.label = self.detect_activity(self.lm_list[i])
                    self.lm_list[i] = []
                self.draw_skeleton(frame, keypoints[0][i][:51], bbox)
        return frame

    def run(self):
        while True:
            frames = self.pipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            frame = np.asanyarray(color_frame.get_data())

            self.frame_count += 1
            if self.frame_count > self.warmup_frames:
                frame = self.process_frame(frame)

            # FPS Calculation
            self.new_frame_time = time.time()
            fps = 1 / (self.new_frame_time - self.prev_frame_time)
            self.prev_frame_time = self.new_frame_time
            self.fpsArr.append(fps)
            avg_fps = sum(self.fpsArr) / len(self.fpsArr) if len(self.fpsArr) < 10 else 0

            cv2.putText(frame, f"FPS: {round(avg_fps, 2)}", (455, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 3)
            cv2.imshow("Activity Detection", frame)

            if cv2.waitKey(1) == ord('q'):
                break

        self.pipe.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = ActivityDetector(model_path='rs/mymodel', tflite_path='rs/1.tflite')
    detector.run()
