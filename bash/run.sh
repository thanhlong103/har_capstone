#!/bin/bash

# =========== LIDAR ==============
(
  cd /home/ntlong/har_capstone/lidar_leg_tracker_ws || exit
  source install/setup.bash
  ros2 launch ldlidar_stl_ros2 ld19.launch.py
) &

#========== ROBOT BRINGUP ============
# (
#   cd /home/ntlong/har_capstone/socin_robot_ws || exit
#   source install/setup.bash
#   ros2 launch turtlebot3_bringup robot.launch.py
# ) &

#========== FUSING DATA AND GROUP DETECTION ===========
(
  cd /home/ntlong/har_capstone/socin_robot_ws || exit
  source install/setup.bash
  ros2 run fusing_people fusing_people
) &

(
  cd /home/ntlong/har_capstone/socin_robot_ws || exit
  source install/setup.bash
  ros2 run fusing_people fused_group
) &

#========== CAMERA & HAR & TRACKER ============
(
  cd ~/har_capstone/socin_robot_ws || exit
  source install/setup.bash
  cd /home/ntlong/har_capstone/socin_robot_ws/src/vision_people_tracker/src 
  source tf/bin/activate
  python3 try_better_pca.py
) &

(
  cd ~/har_capstone || exit
  python3 filter_lidar.py 
) &

# Launch Rviz2
rviz2 -d ~/har_capstone/rviz2/fused.rviz &

# Wait for all background processes to finish
wait