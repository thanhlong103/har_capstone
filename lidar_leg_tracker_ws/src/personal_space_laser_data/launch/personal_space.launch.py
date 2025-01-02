from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  personal_space = Node(
      package='personal_space_laser_data',
      executable='personal_space',
      name='personal_space',
      output='screen'   
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(personal_space)

  return ld