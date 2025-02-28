import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Initialize RealSense pipeline and configure color stream

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)

# Set up video writer for color stream with MP4 format
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' codec for MP4 format
current_time = time.strftime("%Y%m%d-%H%M%S")
color_out = cv2.VideoWriter(f'sitwork_5.mp4', fourcc, 30.0, (640, 480))

try:
    while True:
    
        # Get frames
        frames = pipe.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Show the color frame
        cv2.imshow('RGB', color_image)

        # Write color frame to the video file
        color_out.write(color_image)

        # Press 'q' to stop
        if cv2.waitKey(1) == ord('q'):
            break
finally:
    # Release resources
    pipe.stop()
    color_out.release()
    cv2.destroyAllWindows()
