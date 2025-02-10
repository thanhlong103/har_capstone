#!/bin/bash

# Wait for the system to fully start
# sleep 5

# Launch ROS 2 bridge
(
    source /opt/ros/foxy/setup.bash || exit
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
) &
# Start the React web app
# export DISPLAY=:0
# (
#     cd /home/irs/har_capstone/monitor-ui || exit
#     npm start 
# ) &
# Start the Node.js server
(
    cd /home/irs/har_capstone/monitor_ui/src || exit
    node server.js 
) & 

(
    cd /home/irs/har_capstone/socin_robot_ws || exit
    source install/setup.bash
    ros2 launch turtlebot3_sim robot_launch.py
) &

(
    sleep 10 || exit
    google-chrome --start-fullscreen  http://localhost:3000
)

wait