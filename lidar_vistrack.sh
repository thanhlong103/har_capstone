#!/bin/bash

# Launch the LiDAR
(
  cd /home/ntlong/har_capstone/lidar_leg_tracker_ws || exit
  source install/setup.bash
  ros2 launch ldlidar_stl_ros2 ld19.launch.py
) &

# Launch the vision_people_tracker
(
  cd /home/ntlong/har_capstone/vision_people_tracker_ws || exit
  source tf/bin/activate
  python3 HAR.py
) &

# Launch Rviz2
rviz2 &

# Wait for all background processes to finish
wait
