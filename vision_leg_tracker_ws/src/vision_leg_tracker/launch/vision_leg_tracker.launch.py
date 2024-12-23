#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import os

vision_leg_tracker_path = get_package_share_directory('vision_leg_tracker')
rviz2_config_path = vision_leg_tracker_path + "/vision_leg_tracker_rviz.rviz"

def generate_launch_description():

    ld = LaunchDescription([
	# launch.actions.ExecuteProcess(
        #    cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz2_config_path],
        #    output='screen'
        # )
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 'enable_rgbd:=true', 'enable_sync:=true', 'align_depth.enable:=true', 'enable_color:=true', 'enable_depth:=true'],
            output='screen'
        )
    ])

    vision_leg_tracker = Node(
        package='vision_leg_tracker',
        executable='vision_leg_tracker',
        name='vision_leg_tracker',
        output='screen'
    )

    
    ld.add_action(vision_leg_tracker)
    
    return ld 
 
