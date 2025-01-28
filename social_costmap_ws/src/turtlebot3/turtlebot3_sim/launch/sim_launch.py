#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_gazebo = get_package_share_directory('turtlebot3_sim')
    pkg_moving_cylinder = get_package_share_directory('turtlebot3_sim')  # Replace 'my_package' with your package name

    # Gazebo world file
    world_file = os.path.join(pkg_robot_gazebo, 'worlds', 'sim_world.worlds')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Python node to control moving cylinder
    moving_cylinder_node = Node(
        package='turtlebot3_sim',  # Replace with your package name
        executable='moving_cylinder.py',  # Python script to move the cylinder
        name='moving_cylinder',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='SDF world file to load in Gazebo'
        ),
        gazebo,
        # moving_cylinder_node  # Add the moving cylinder node
    ])
