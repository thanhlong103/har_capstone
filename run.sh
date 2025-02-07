#!/bin/bash

# =========== LIDAR ==============
(
  cd /home/ntlong/har_capstone/lidar_leg_tracker_ws || exit
  source install/setup.bash
  ros2 launch ldlidar_stl_ros2 ld19.launch.py
) &

# ========== DOCKER ==============
# (
#   ros2 launch turtlebot3_bringup robot.launch.py || exit
# ) &

# (
#   docker start quirky_gould || exit
#   docker exec -it quirky_gould bash
#   cd
#   cd ros2_ws
#   source install/setup.bash
#   ros2 launch leg_detector lidar_leg_tracker.launch.py
# ) &

# (
#   docker exec -it quirky_gould bash || exit
#   cd
#   python3 ros2_ws/src/ros2_leg_detector/src/leg_detector/scripts/publish_marker.py
# ) &

#========== ROBOT BRINGUP ============
# (
#   cd /home/ntlong/har_capstone/social_costmap_ws || exit
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

# Launch the vision_people_tracker
(
  cd /home/ntlong/har_capstone/socin_robot_ws/src/vision_people_tracker/src || exit
  source tf/bin/activate
  python3 vision_people_tracker.py
) &

# Launch the vision_people_tracker
(
  cd /home/ntlong/har_capstone/fused_people_ws || exit
  source install/setup.bash
  cd src
  # python3 convert.py
) &


# Launch Rviz2
rviz2 &

# Wait for all background processes to finish

wait
