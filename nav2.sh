    #!/bin/bash

export USER=irs
# =========== LIDAR ==============
(
  cd /home/$USER/har_capstone/socin_robot_ws || exit
  source install/setup.bash
  ros2 launch turtlebot3_sim robot_launch.py
) &

( 
  source /home/$USER/har_capstone/socin_robot_ws/install/setup.bash || exit
  python3  /home/$USER/har_capstone/socin_robot_ws/src/convert.py
) &

# (
#     source /home/$USER/har_capstone/socin_robot_ws/install/setup.bash || exit
#     python3  /home/$USER/har_capstone/socin_robot_ws/src/fusing_people/fusing_people/publish_people_group.py
# ) &
#========== FUSING DATA AND GROUP DETECTION ===========
# (
#   cd /home/ntlong/har_capstone/socin_robot_ws || exit
#   source install/setup.bash
#   ros2 run fusing_people fusing_people
# ) &

(
  cd /home/$USER/har_capstone/socin_robot_ws || exit
  source install/setup.bash
  ros2 run fusing_people fused_group
) &

# (
#   cd /home/$USER/har_capstone/socin_robot_ws || exit
#   source install/setup.bash
#   sleep 10
#   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=src/turtlebot3_navigation2/map/sim_map.yaml 
# ) &

# Wait for all background processes to finish

wait
