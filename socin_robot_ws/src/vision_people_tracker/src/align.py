import pyrealsense2 as rs
import numpy as np
import cv2

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Get camera intrinsics and extrinsics
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))

color_intr = color_profile.get_intrinsics()
depth_intr = depth_profile.get_intrinsics()
extrinsics = depth_profile.get_extrinsics_to(color_profile)

# Extrinsics: Rotation (R) and Translation (T)
R = np.array(extrinsics.rotation).reshape(3, 3)  # 3x3 rotation matrix
T = np.array(extrinsics.translation).reshape(3, 1)  # 3x1 translation vector

# Intrinsics: Camera matrices
depth_camera_matrix = np.array([
    [depth_intr.fx, 0, depth_intr.ppx],
    [0, depth_intr.fy, depth_intr.ppy],
    [0, 0, 1]
])

color_camera_matrix = np.array([
    [color_intr.fx, 0, color_intr.ppx],
    [0, color_intr.fy, color_intr.ppy],
    [0, 0, 1]
])

# Distortion coefficients
depth_dist_coeffs = np.array(depth_intr.coeffs)
color_dist_coeffs = np.array(color_intr.coeffs)

count = 0

try:
    while True: 
        count = count + 1 
        # Wait for frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame :
            continue

        if count <= 10 : 
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert depth image to meters
        depth_image_meters = depth_image * depth_scale

        # Get pixel coordinates for the depth image
        height, width = depth_image.shape
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        u = u.flatten()
        v = v.flatten()
        z = depth_image_meters.flatten()

        # Filter valid depth points
        valid = z > 0
        u = u[valid]
        v = v[valid]
        z = z[valid]

        # Convert depth pixels to 3D points (depth camera coordinates)
        x = (u - depth_intr.ppx) * z / depth_intr.fx
        y = (v - depth_intr.ppy) * z / depth_intr.fy
        depth_points = np.vstack((x, y, z))  # Shape: (3, N)

        # Transform points from depth to color camera coordinates
        color_points = R @ depth_points + T  # Shape: (3, N)

        # Project 3D points onto the color image plane
        uv_homog = color_camera_matrix @ color_points  # Shape: (3, N)
        u_color = (uv_homog[0] / uv_homog[2]).astype(int)
        v_color = (uv_homog[1] / uv_homog[2]).astype(int)

        # Filter points inside the color image bounds
        valid = (u_color >= 0) & (u_color < color_intr.width) & \
                (v_color >= 0) & (v_color < color_intr.height)

        u_color = u_color[valid]
        v_color = v_color[valid]
        valid_points = color_points[:, valid].T  # Shape: (N, 3)

        # Get corresponding RGB colors
        valid_colors = color_image[v_color, u_color]

        # Convert BGR to RGB
        valid_colors = valid_colors[:, [2, 1, 0]]

        # Save as point cloud
        point_cloud = np.hstack((valid_points, valid_colors))
        np.savetxt("point_cloud.xyz", point_cloud, fmt="%f %f %f %d %d %d")

        print("Point cloud saved as point_cloud.xyz")
        break

finally:
    pipeline.stop()
