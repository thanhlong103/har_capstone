#!/bin/bash

export USER=ntlong

# =========== LIDAR ==============
(
  cd /home/$USER/har_capstone/lidar_leg_tracker_ws || exit
  source install/setup.bash
  ros2 launch ldlidar_stl_ros2 ld19.launch.py
) &

#========== FUSING DATA AND GROUP DETECTION ===========
# (
#   cd /home/$USER/har_capstone/socin_robot_ws || exit
#   source install/setup.bash
#   ros2 run fusing_people fusing_people
# ) &

(
  cd /home/$USER/har_capstone/socin_robot_ws || exit
  source install/setup.bash
  ros2 run fusing_people fused_group
) &

Launch the vision_people_tracker
(
  cd /home/ntlong/har_capstone/socin_robot_ws/src/vision_people_tracker/experiment || exit
  source tf/bin/activate
  python3 vision_people_tracker.py
) &

# (
#     cd /home/$USER/har_capstone/socin_robot_ws/src/vision_people_tracker/src || exit
#     python3 publish_people_vision.py
# )&

rviz2 -d /home/$USER/har_capstone/rviz2/group_detection_experiment.rviz

# Wait for all background processes to finish

wait
