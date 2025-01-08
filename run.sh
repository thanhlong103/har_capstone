#!/bin/bash

# Launch the LiDAR
(
  cd /home/ntlong/har_capstone/lidar_ws || exit
  source install/setup.bash
  ros2 launch ldlidar_stl_ros2 ld19.launch.py
) &

# Launch the vision_people_tracker
(
  cd /home/ntlong/har_capstone/temp_vileta || exit
  source tf/bin/activate
  python3 moveNetNode.py
) &

# Launch Rviz2
rviz2 &

# Wait for all background processes to finish
wait
