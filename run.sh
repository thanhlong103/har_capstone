#!/bin/bash

# Launch the LiDAR
(
  cd /home/ntlong/har_capstone/lidar_leg_tracker_ws || exit
  source install/setup.bash
  ros2 launch ldlidar_stl_ros2 ld19.launch.py
) &

# (
#   ros2 launch turtlebot3_bringup robot.launch.py || exit
# ) &

# (
#   docker start quirky_gould || exit
#   docker exec -it quirky_gould bash
#   cd
#   cd ros2_ws
#   source install/setup.bash
#   ros2 launch leg_detector lidar_leg_tracker.py
# ) &

# (
#   docker exec -it quirky_gould bash || exit
#   cd
#   python3 ros2_ws/src/ros2_leg_detector/src/leg_detector/scripts/publish_marker.py
# ) &

(
  cd /home/ntlong/har_capstone/fused_people_ws || exit
  source install/setup.bash
  ros2 run fusing_people fusing_people
) &

(
  cd /home/ntlong/har_capstone/fused_people_ws || exit
  source install/setup.bash
  ros2 run fusing_people fused_group
) &

# Launch the vision_people_tracker
(
  cd /home/ntlong/har_capstone/vision_people_tracker_ws || exit
  source tf/bin/activate
  python3 vision_people_tracker.py
) &

# Launch Rviz2
rviz2 &

# Wait for all background processes to finish
wait
