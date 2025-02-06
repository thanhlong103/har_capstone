#!/bin/bash

(
  cd /home/ntlong/har_capstone/social_costmap_ws || exit
  source install/setup.bash
  ros2 launch turtlebot3_bringup robot.launch.py
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

# Launch the vision_people_tracker
(
  cd /home/ntlong/har_capstone/fused_people_ws || exit
  source install setup.bash
  cd src
  python3 convert.py
) &

# Wait for all background processes to finish

wait
