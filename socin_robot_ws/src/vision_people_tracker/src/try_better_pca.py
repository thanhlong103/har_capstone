import rclpy
from rclpy.node import Node
import cv2
import tensorflow as tf
import numpy as np
import pyrealsense2 as rs
from tensorflow.keras.models import load_model
import math
import time
import pandas as pd
import os
from tf_transformations import quaternion_from_euler

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from people_msgs.msg import People, MyPerson

class VisionLegTracker(Node):
    def __init__(self):
        super().__init__("HAR_node")
        self.pipe = rs.pipeline()
        self.cfg = rs.config()

        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipe)
        self.pipeline_profile = self.cfg.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(
            self.device.get_info(rs.camera_info.product_line)
        )

        self.image_publisher_ = self.create_publisher(Image, 'robot_vision', 10)
        self.bridge = CvBridge()

        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.label = "Warmup...."
        self.n_time_steps = 10
        self.lm_list = [[], [], [], [], [], []]
        self.label = [None, None, None, None, None, None]
        self.results = [0, 0, 0, 0, 0, 0]
        self.prevAct = time.time()
        self.i = 0
        self.warmup_frames = 60

        # cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.pipe.start(self.cfg)
        # Initialize prevtime for runtime calculation
        self.prevtime = time.time()

        # used to record the time when we processed last frame
        self.prev_frame_time = 0

        # used to record the time at which we processed current frame
        self.new_frame_time = 0

        self.fpsArr = []
        self.sumfps = 0

        self.confidence_threshold = 0.2
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

        self.offset_x = 150.0
        self.offset_y = 37.0
        self.offset_z = 0.0
        self.offset = [self.offset_x, self.offset_y, self.offset_z]
        self.intrinsics = None

        self.prev_person_pos = [0.0, 0.0, 0.0]

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
            self.har_interpreter = tf.lite.Interpreter(
                model_path="model_quantized.tflite"
            )
            self.har_interpreter.allocate_tensors()
            self.har_input_details = self.har_interpreter.get_input_details()
            self.har_output_details = self.har_interpreter.get_output_details()
            self.get_logger().info("HAR model loaded!")
        except Exception as e:
            self.get_logger().error(f"Failed to load HAR model: {e}")
            raise

        self.get_logger().info("Vision Leg Tracker Node has started.")

        # Add a publisher for person coordinates
        self.coord_publisher = self.create_publisher(People, "/people_vision", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, "/human_markers", 10)
        self.pose_array_publisher = self.create_publisher(PoseArray, "/ps_vision", 10)

        # Define the static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Define the camera frame's static transform
        self.broadcast_camera_frame()

        # Frame width and height
        frame = self.pipe.wait_for_frames()
        color_frame = frame.get_color_frame()
        img = np.asanyarray(color_frame.get_data())
        self.HEIGHT, self.WIDTH, _ = img.shape

        # Align object - aligning depth to color
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Multi-person tracking state
        self.people_states = {}  # Dict: ID -> {prev_theta, prev_centroid, last_seen}
        self.next_id = 0  # Incremental ID for new people
        self.alpha = 0.4  # EMA smoothing factor (0 < alpha < 1)
        self.max_distance = 0.5  # Max centroid distance (meters) for matching
        self.timeout = 1.0  # Seconds before removing inactive person

    def match_people(self, current_centroids):
        """
        Matches current centroids to previous people based on Euclidean distance.

        Parameters:
            current_centroids (list): List of [x, y, z] centroids for current frame.

        Returns:
            dict: Mapping of current index to matched person ID.
        """
        matches = {}  # Current index -> person ID
        used_ids = set()

        # Convert current centroids to numpy for efficiency
        current_centroids = np.array(current_centroids, dtype=np.float32)

        # If no previous people, assign new IDs
        if not self.people_states:
            for idx in range(len(current_centroids)):
                matches[idx] = self.next_id
                self.people_states[self.next_id] = {
                    'prev_theta': None,
                    'prev_centroid': current_centroids[idx],
                    'last_seen': time.time()
                }
                self.next_id += 1
            return matches

        # Compute distances between current and previous centroids
        prev_centroids = np.array([state['prev_centroid'] for state in self.people_states.values()])
        prev_ids = list(self.people_states.keys())

        for curr_idx, curr_centroid in enumerate(current_centroids):
            if len(prev_centroids) == 0:
                matches[curr_idx] = self.next_id
                self.next_id += 1
                continue

            distances = np.linalg.norm(prev_centroids - curr_centroid, axis=1)
            closest_idx = np.argmin(distances)
            min_distance = distances[closest_idx]

            if min_distance < self.max_distance and prev_ids[closest_idx] not in used_ids:
                # Match found
                matches[curr_idx] = prev_ids[closest_idx]
                used_ids.add(prev_ids[closest_idx])
            else:
                # New person
                matches[curr_idx] = self.next_id
                self.next_id += 1

        return matches

    def broadcast_camera_frame(self):
        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"  # Parent frame
        t.child_frame_id = "camera_frame"  # Camera frame

        # Define translation (position) of the camera
        t.transform.translation.x = -0.3
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2

        # Define rotation (orientation) of the camera
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the static transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Camera frame transform broadcasted.")

    def publish_human_marker(self, marker_array, id, x, y):
        # Create a marker for the sphere (human's head)
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "camera_frame"
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = "humans"
        sphere_marker.id = id * 2
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = x
        sphere_marker.pose.position.y = -y
        sphere_marker.pose.position.z = 0.4  # Sphere above the cylinder
        sphere_marker.scale.x = 0.15
        sphere_marker.scale.y = 0.15
        sphere_marker.scale.z = 0.15
        sphere_marker.color.a = 1.0  # Alpha (transparency)
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.0  # Green color

        # Create a marker for the cylinder (human's body)
        cylinder_marker = Marker()
        cylinder_marker.header.frame_id = "camera_frame"
        cylinder_marker.header.stamp = self.get_clock().now().to_msg()
        cylinder_marker.ns = "humans"
        cylinder_marker.id = id * 2 + 1
        cylinder_marker.type = Marker.CYLINDER
        cylinder_marker.action = Marker.ADD
        cylinder_marker.pose.position.x = x
        cylinder_marker.pose.position.y = -y
        cylinder_marker.pose.position.z = 0.2  # Cylinder's center
        cylinder_marker.scale.x = 0.1  # Diameter
        cylinder_marker.scale.y = 0.1  # Diameter
        cylinder_marker.scale.z = 0.4  # Height of the cylinder
        cylinder_marker.color.a = 1.0  # Alpha (transparency)
        cylinder_marker.color.r = 0.0
        cylinder_marker.color.g = 0.5
        cylinder_marker.color.b = 1.0  # Blue color

        marker_array.markers.append(sphere_marker)
        marker_array.markers.append(cylinder_marker)

        return marker_array

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
    
    def process_keypoints(self, intrinsics, keypoints, depth_array, depth_frame):
        """
        Transforms 2D keypoints to 3D world coordinates using depth data with improved robustness.

        Parameters:
            intrinsics (rs.intrinsics): Camera intrinsics for deprojection.
            keypoints (numpy.ndarray): Array of shape (51,) with [y, x, conf, y, x, conf, ...].
            depth_array (numpy.ndarray): 2D depth image array (HEIGHT, WIDTH).
            depth_frame (rs.depth_frame): RealSense depth frame for direct depth access.

        Returns:
            numpy.ndarray: Array of shape (N, 3) with 3D coordinates [x, y, z] for valid keypoints.
        """
        # Ensure keypoints is a NumPy array and reshape to (17, 3) for [y, x, conf]
        keypoints = np.array(keypoints, dtype=np.float32).reshape(-1, 3)
        num_keypoints = keypoints.shape[0]

        # Pre-allocate output array with zeros (invalid points stay as [0, 0, 0])
        keypoints_3d = np.zeros((num_keypoints, 3), dtype=np.float32)

        # Extract 2D coordinates and confidences
        y_coords = keypoints[:, 0] * self.HEIGHT  # Scale to image height
        x_coords = keypoints[:, 1] * self.WIDTH   # Scale to image width
        confidences = keypoints[:, 2]

        # Validate depth array dimensions
        if depth_array.shape != (self.HEIGHT, self.WIDTH):
            self.get_logger().error("Depth array size mismatch. Returning zeros.")
            return keypoints_3d

        # Process each keypoint
        for i in range(num_keypoints):
            if confidences[i] < self.confidence_threshold:
                continue  # Skip low-confidence keypoints

            # Convert to integer pixel coordinates
            col = int(x_coords[i])  # x
            row = int(y_coords[i])  # y

            # Check bounds
            if not (0 <= row < self.HEIGHT and 0 <= col < self.WIDTH):
                self.get_logger().debug(f"Keypoint {i} out of bounds: ({row}, {col})")
                continue

            # Adaptive patch size based on approximate depth (or fixed if unknown)
            # Assume initial depth from center of depth array for simplicity
            center_depth = depth_frame.get_distance(self.WIDTH // 2, self.HEIGHT // 2)
            patch_size = max(3, min(11, int(5 * (1 + center_depth))))  # 3 to 11 based on depth

            half_size = patch_size // 2
            row_min = max(0, row - half_size)
            row_max = min(self.HEIGHT, row + half_size + 1)
            col_min = max(0, col - half_size)
            col_max = min(self.WIDTH, col + half_size + 1)

            # Extract depth patch
            depth_patch = depth_array[row_min:row_max, col_min:col_max]
            if depth_patch.size == 0:
                self.get_logger().debug(f"Empty depth patch for keypoint {i}")
                continue

            # Use median depth to reduce noise (non-zero values only)
            valid_depths = depth_patch[depth_patch > 0]
            if valid_depths.size < patch_size:  # Require at least some valid points
                self.get_logger().debug(f"Insufficient valid depths for keypoint {i}")
                continue

            depth = np.median(valid_depths) / 1000.0  # Convert mm to meters (if needed)

            # Depth filtering
            if not (0.2 < depth < 3.0):  # Same range as original
                self.get_logger().debug(f"Depth {depth} out of range for keypoint {i}")
                continue

            # Deproject to 3D world coordinates
            try:
                coord_3d = rs.rs2_deproject_pixel_to_point(
                    intrinsics, [x_coords[i], y_coords[i]], depth
                )
                # Assign in standard [x, y, z] order
                keypoints_3d[i] = [coord_3d[0], coord_3d[1], coord_3d[2]]
            except Exception as e:
                self.get_logger().warning(f"Deprojection failed for keypoint {i}: {e}")
                continue

        return keypoints_3d

    def estimate_plane_pca(self, points, confidences=None, person_id=None):
        """
        Estimates a plane from a set of 3D points using PCA with confidence weighting, tailored for
        keypoints from process_keypoints, with robustness checks for degenerate cases.

        Parameters:
            points (numpy.ndarray): A (N, 3) array of 3D points [x, y, z], where N <= 17 (MoveNet keypoints).
                                    Invalid points may be [0, 0, 0].
            confidences (numpy.ndarray, optional): A (N,) array of confidence scores for each point.
                                                If None, uses equal weighting for valid points.
            person_id (int, optional): Unique ID of the person for accessing previous state.

        Returns:
            normal_vector (numpy.ndarray): A (3,) vector representing the plane's normal, or fallback.
            centroid (numpy.ndarray): A (3,) vector representing the weighted centroid of valid points.
        """
        # Ensure points is a NumPy array with expected shape
        points = np.array(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            self.get_logger().warning("Invalid points shape for PCA. Expected (N, 3). Returning defaults.")
            return self._get_fallback_normal(person_id), None

        # Filter out invalid points ([0, 0, 0]) from process_keypoints
        valid_mask = np.linalg.norm(points, axis=1) > 1e-6  # Non-zero norm indicates valid point
        if np.sum(valid_mask) < 3:
            self.get_logger().warning("Fewer than 3 valid points for PCA. Using fallback.")
            return self._get_fallback_normal(person_id), None

        valid_points = points[valid_mask]

        # Handle confidences
        if confidences is None:
            confidences = np.ones(valid_points.shape[0], dtype=np.float32)
        else:
            confidences = np.array(confidences, dtype=np.float32)[valid_mask]
            if confidences.shape[0] != valid_points.shape[0]:
                self.get_logger().warning("Confidence array size mismatch after filtering. Using equal weights.")
                confidences = np.ones(valid_points.shape[0], dtype=np.float32)
            confidences = np.clip(confidences, 0.0, 1.0)
            if np.sum(confidences) < 1e-6:
                self.get_logger().warning("Sum of confidences too low. Using equal weights.")
                confidences = np.ones(valid_points.shape[0], dtype=np.float32)

        try:
            # Compute weighted centroid using only valid points
            weight_sum = np.sum(confidences)
            centroid = np.average(valid_points, axis=0, weights=confidences)

            # Center the valid points around the weighted centroid
            centered_points = valid_points - centroid

            # Compute weighted covariance matrix
            weighted_points = centered_points * confidences[:, np.newaxis]
            covariance_matrix = np.dot(weighted_points.T, centered_points) / weight_sum

            # Ensure symmetry and numerical stability
            covariance_matrix = (covariance_matrix + covariance_matrix.T) / 2

            # Perform eigen decomposition
            eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)

            # Robustness check: Eigenvalue ratios to detect degeneracy
            eigenvalues = np.abs(eigenvalues)  # Ensure non-negative
            sorted_idx = np.argsort(eigenvalues)  # Smallest to largest
            lambda1, lambda2, lambda3 = eigenvalues[sorted_idx]  # Smallest, middle, largest

            # Define thresholds for degeneracy
            degeneracy_threshold = 1e-4  # Ratio below which points are nearly coplanar/collinear
            if lambda3 < 1e-6:  # All eigenvalues tiny, data too noisy
                self.get_logger().warning("All eigenvalues too small. Using fallback.")
                return self._get_fallback_normal(person_id), centroid
            elif lambda2 / lambda3 < degeneracy_threshold:  # Middle eigenvalue too small
                self.get_logger().warning("Points nearly collinear/coplanar. Using fallback.")
                return self._get_fallback_normal(person_id), centroid

            # Normal vector is eigenvector with smallest eigenvalue
            min_eigen_idx = sorted_idx[0]
            normal_vector = eigenvectors[:, min_eigen_idx]

            # Normalize the normal vector
            norm = np.linalg.norm(normal_vector)
            if norm > 1e-6:
                normal_vector /= norm
            else:
                self.get_logger().warning("Normal vector norm too small. Using fallback.")
                return self._get_fallback_normal(person_id), centroid

        except np.linalg.LinAlgError:
            self.get_logger().error("Eigen decomposition failed. Using fallback.")
            return self._get_fallback_normal(person_id), centroid
        except Exception as e:
            self.get_logger().error(f"Unexpected error in PCA: {e}. Using fallback.")
            return self._get_fallback_normal(person_id), centroid

        return normal_vector, centroid

    def _get_fallback_normal(self, person_id):
        """
        Helper function to retrieve a fallback normal vector based on person state.

        Parameters:
            person_id (int or None): Unique ID of the person for accessing previous state.

        Returns:
            numpy.ndarray: Fallback normal vector (previous or default).
        """
        if person_id is not None and person_id in self.people_states:
            prev_theta = self.people_states[person_id].get('prev_theta')
            if prev_theta is not None:
                # Reconstruct normal from previous theta (approximation)
                # Assuming normal lies in XZ plane (simplified)
                normal = np.array([np.cos(prev_theta), 0.0, np.sin(prev_theta)], dtype=np.float32)
                norm = np.linalg.norm(normal)
                if norm > 1e-6:
                    return normal / norm
        # Default fallback if no previous data
        return None

    def facing_direction(self, normal_vector, centroid, person_id):
        """
        Computes facing direction with EMA smoothing for a specific person.

        Parameters:
            normal_vector (numpy.ndarray): Plane normal from PCA.
            centroid (numpy.ndarray): Person's centroid.
            person_id (int): Unique ID for the person.

        Returns:
            float: Smoothed theta (facing direction in radians).
        """
        # PCA-based theta calculation
        A, B, C = normal_vector
        dot_product = np.dot(normal_vector, [1, 0, 0])
        magnitude_estimated = np.linalg.norm(normal_vector)
        cos_angle = dot_product / (magnitude_estimated * 1.0)
        theta = np.arccos(cos_angle)
        if B < 0:
            theta = theta
        else:
            theta = 6.28 - theta

        # Get or initialize person state
        if person_id not in self.people_states:
            self.people_states[person_id] = {
                'prev_theta': None,
                'prev_centroid': centroid.copy(),
                'last_seen': time.time()
            }

        person_state = self.people_states[person_id]
        person_state['prev_centroid'] = centroid.copy()
        person_state['last_seen'] = time.time()

        # Apply EMA smoothing
        if person_state['prev_theta'] is not None:
            theta = self.alpha * theta + (1 - self.alpha) * person_state['prev_theta']
        person_state['prev_theta'] = theta

        return theta

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
        self.har_interpreter.set_tensor(self.har_input_details[0]["index"], lm_list)

        # Run inference
        self.har_interpreter.invoke()

        # Get output
        output = self.har_interpreter.get_tensor(self.har_output_details[0]["index"])
        results = np.argmax(output)

        # Label handling remains unchanged
        if results == 0:
            label = "Walking"
        elif results == 1:
            label = "Discussing"
        elif results == 2:
            label = "Walking Phone"
        # elif results == 3:
        #     label = "Sitting"
        elif results == 3:
            label = "Sit Work"
        # elif results == 5:
        #     label = "Standing"
        # elif results == 4:
        #     label = "Wave Hi"
        else:
            label = "Drilling"
            
        return results, label

    def draw_class_on_image(self, label, img, bbox):
        y, x, _ = img.shape
        font = cv2.FONT_HERSHEY_SIMPLEX
        position = (int((bbox[3] + bbox[1]) / 2 * x), int((bbox[0] + bbox[2]) / 2 * y))
        fontScale = 1
        fontColor = (0, 255, 0)
        thickness = 2
        lineType = 2
        # print(position)
        cv2.putText(
            img, label, position, font, fontScale, fontColor, thickness, lineType
        )

    def transform_keypoints(self, keypoints, original_frame, aligned_frame):
        """
        Transforms keypoints from the unaligned frame to the aligned frame.
        Maintains the same keypoints structure and dtype (numpy.float32).
        """

        # Get intrinsics of both frames
        intrinsics_orig = (
            original_frame.get_profile().as_video_stream_profile().intrinsics
        )
        intrinsics_aligned = (
            aligned_frame.get_profile().as_video_stream_profile().intrinsics
        )

        # Create an empty array with the same shape and dtype
        transformed_keypoints = np.zeros_like(keypoints, dtype=np.float32)

        for i in range(keypoints.shape[0]):  # Iterate over all persons
            for j in range(
                0, keypoints.shape[1], 3
            ):  # Iterate over keypoints (x, y, confidence)
                y, x, confidence = (
                    keypoints[i, j],
                    keypoints[i, j + 1],
                    keypoints[i, j + 2],
                )

                # Convert (x, y) from unaligned to aligned frame
                aligned_x = x * (intrinsics_aligned.width / intrinsics_orig.width)
                aligned_y = y * (intrinsics_aligned.height / intrinsics_orig.height)

                transformed_keypoints[i, j] = aligned_y  # Preserve y first
                transformed_keypoints[i, j + 1] = aligned_x  # Then x
                transformed_keypoints[i, j + 2] = confidence

        return transformed_keypoints

    def processImage(self):
        frame = self.pipe.wait_for_frames()

        aligned_frames = self.align.process(frame)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        color_frame_har = frame.get_color_frame()

        img = np.asanyarray(color_frame.get_data())
        img_har = np.asanyarray(color_frame_har.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert depth image to 3-channel grayscale for visualization
        depth_visual = cv2.normalize(
            depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        depth_visual = cv2.cvtColor(depth_visual, cv2.COLOR_GRAY2BGR)

        # Initialize camera intrinsics if not done
        if self.intrinsics is None:
            self.intrinsics = (
                aligned_frames.profile.as_video_stream_profile().intrinsics
            )

        kp_har = self.estimator(img_har)

        keypoints_with_scores = self.transform_keypoints(kp_har, frame, aligned_frames)

        current_centroids = []
        person_world_coords = []
        poses = People()
        poses_array = PoseArray()

        for i in range(6):
            bbox = keypoints_with_scores[0][i][51:57]

            if bbox[4] < self.bbox_threshold:
                continue

            keypoints_draw = keypoints_with_scores[0][i][:51]
            kp_har_detect = kp_har[0][i][:51]

            # if (keypoints_draw[17] > self.confidence_threshold):
            right_shoulder = tuple(
                [
                    int(keypoints_draw[16] * self.WIDTH),
                    int(keypoints_draw[15] * self.HEIGHT),
                ]
            )
            left_shoulder = tuple(
                [
                    int(keypoints_draw[19] * self.WIDTH),
                    int(keypoints_draw[18] * self.HEIGHT),
                ]
            )

            # Rendering
            self.draw(depth_visual, kp_har_detect, bbox)
            self.draw(img, kp_har_detect, bbox)

            keypoints = self.process_keypoints(
                self.intrinsics, keypoints_draw, depth_image, depth_frame
            )
            
            confidences = keypoints_draw[2::3]  # Confidence is every 3rd value (y, x, conf)
            
            normal, centroid = self.estimate_plane_pca(keypoints, confidences)
            
            print(centroid)
            
            # Skip if PCA failed
            if normal is None or centroid is None:
                self.get_logger().warning(f"Skipping person {i} due to PCA failure.")
                continue

            current_centroids.append(centroid)
            
            c_lm = self.make_landmark_timestep(kp_har_detect)
            if len(c_lm) > 0:
                self.lm_list[i].append(c_lm)
            if len(self.lm_list[i]) == self.n_time_steps:
                results, label = self.detectAct(self.lm_list[i])
                self.results[i] = results
                self.label[i] = label
                self.lm_list[i] = []

            if self.label[i] != None:
                self.draw_class_on_image(self.label[i], img, bbox)
        
        # Match current people to previous ones
        matches = self.match_people(current_centroids)
        
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = "camera_frame"
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        # Process matched people
        for i, person_id in matches.items():
            bbox = keypoints_with_scores[0][i][51:57]
            keypoints_draw = keypoints_with_scores[0][i][:51]
            kp_har_detect = kp_har[0][i][:51]

            keypoints_3d = self.process_keypoints(self.intrinsics, keypoints_draw, depth_image, depth_frame)
            normal, centroid = self.estimate_plane_pca(keypoints_3d, keypoints_draw[2::3])
            
            if normal is None or centroid is None:
                self.get_logger().warning(f"Skipping person {i} due to PCA failure.")
                continue

            # Compute smoothed theta for this person
            theta = self.facing_direction(normal, centroid, person_id)

            # Shoulder adjustment
            right_shoulder = tuple([int(keypoints_draw[16] * self.WIDTH), int(keypoints_draw[15] * self.HEIGHT)])
            left_shoulder = tuple([int(keypoints_draw[19] * self.WIDTH), int(keypoints_draw[18] * self.HEIGHT)])
            if left_shoulder[0] - right_shoulder[0] > 20:
                theta = theta + 3.14

            # Activity detection (unchanged)
            c_lm = self.make_landmark_timestep(kp_har_detect)
            if len(c_lm) > 0:
                self.lm_list[i].append(c_lm)
            if len(self.lm_list[i]) == self.n_time_steps:
                results, label = self.detectAct(self.lm_list[i])
                self.results[i] = results
                self.label[i] = label
                self.lm_list[i] = []

            if self.label[i] is not None:
                self.draw_class_on_image(self.label[i], img, bbox)

            # Store and publish
            x, y = float(centroid[2]), float(centroid[0])
            person_world_coords.append([x, y, theta])

            pose = MyPerson()
            pose.pose.position.x = x
            pose.pose.position.y = -y
            pose.pose.position.z = 0.03
            q = quaternion_from_euler(0, 0, theta + 3.14)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            pose.activity = int(self.results[i])
            poses.people.append(pose)

            pose_array = Pose()
            pose_array.position.x = x
            pose_array.position.y = -y
            pose_array.position.z = 0.03
            q = quaternion_from_euler(0, 0, theta + 3.14)
            pose_array.orientation.x = q[0]
            pose_array.orientation.y = q[1]
            pose_array.orientation.z = q[2]
            pose_array.orientation.w = q[3]

            poses_array.poses.append(pose_array)

            marker_array = self.publish_human_marker(marker_array, person_id, x, y)

        # Publish the PoseArray if there are valid coordinates
        if poses.people:
            poses.header.frame_id = "camera_frame"
            poses.header.stamp = self.get_clock().now().to_msg()
            poses_array.header.frame_id = "camera_frame"
            poses_array.header.stamp = self.get_clock().now().to_msg()
            self.coord_publisher.publish(poses)
            self.marker_publisher.publish(marker_array)
            self.pose_array_publisher.publish(poses_array)

        # Clean up old people states (optional)
        current_time = time.time()
        self.people_states = {
            pid: state for pid, state in self.people_states.items()
            if current_time - state['last_seen'] < self.timeout
        }

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

        # cv2.putText(img, fps_str, (455, 30), font, 1, (100, 255, 0), 3, cv2.LINE_AA)

        # Stack both images horizontally
        # images = np.hstack((img, depth_visual))

        cv2.imshow("People Detected", img)
        # img_msg = self.bridge.cv2_to_imgmsg(images, encoding='bgr8')
        # self.image_publisher_.publish(img_msg)

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
