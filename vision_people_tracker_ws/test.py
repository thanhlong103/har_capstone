import pyrealsense2 as rs
import numpy as np
import time
import cv2

def get_intrinsics():
    # Create a pipeline
    pipeline = rs.pipeline()
    
    # Configure the pipeline to stream color and depth
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Start the pipeline
    pipeline.start(config)

    # Wait for a valid frame
    frames = pipeline.wait_for_frames()

    # Get the color frame
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Get the intrinsics for the color stream
    color_intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
    
    # Extract intrinsic parameters
    fx = color_intrinsics.fx
    fy = color_intrinsics.fy
    cx = color_intrinsics.ppx
    cy = color_intrinsics.ppy

    # Distortion coefficients
    distortion_coeffs = color_intrinsics.coeffs

    # Print the intrinsic parameters
    print("FX: {}, FY: {}, CX: {}, CY: {}".format(fx, fy, cx, cy))
    print("Distortion Coefficients:", distortion_coeffs)

    return color_intrinsics, pipeline

def deproject_pixel_to_point(intrinsics, pixel, depth_value):
    # Deproject the pixel to a 3D point using the intrinsics
    point = rs.rs2_deproject_pixel_to_point(intrinsics, pixel, depth_value)
    return point

def main():
    color_intrinsics, pipeline = get_intrinsics()

    try:
        while True:
            # Wait for a frame
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            # Convert the color frame to an OpenCV format (numpy array)
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Get the depth value at the center pixel
            pixel = [200, 200]
            depth_value = depth_frame.get_distance(int(color_intrinsics.ppx), int(color_intrinsics.ppy))

            # Deproject the pixel to a 3D point
            point_3d = deproject_pixel_to_point(color_intrinsics, pixel, depth_value)

            # Print the 3D point
            print("3D Point: {}".format(point_3d))

            # Annotate the image with the selected pixel (center pixel)
            center_pixel = (pixel[0], pixel[1])
            cv2.circle(color_image, center_pixel, 5, (0, 0, 255), -1)  # Red circle

            # Display the frame with the highlighted pixel
            cv2.imshow("Color Frame", color_image)
            cv2.imshow("Depth Frame", depth_image)

            # Wait for a key press to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                break

            # Sleep briefly to reduce CPU load
            time.sleep(0.1)

    finally:
        # Stop the pipeline after exiting the loop
        pipeline.stop()
        cv2.destroyAllWindows()  # Close the OpenCV window

if __name__ == "__main__":
    main()
