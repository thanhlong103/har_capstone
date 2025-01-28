import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_name = 'turtlebot3_sim'
    pkg_path = get_package_share_directory(pkg_name)

    # Start Gazebo world
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim_launch.py'),
        )
    )

    # Spawn robot node
    spawn_robot_node = Node(
        package='turtlebot3_sim',
        executable='robot_spawn.py',  # Refers to the installed script
        name='robot_spawner',
        output='screen',
        parameters=[{
            'urdf_path': "/home/irs/har_capstone/social_costmap_ws/src/turtlebot3/turtlebot3_sim/descriptions/robot.urdf",
            'robot_name': 'turtlebot3',
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
        }]
    )

    return LaunchDescription([
        start_world,
        spawn_robot_node,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=["0", "0", "0.085", "0", "0", "0", "base_link", "laser_frame"],
            output="screen",
       ),
    ])
