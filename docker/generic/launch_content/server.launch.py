import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  # Launch File Arguments
  use_fleet_control = LaunchConfiguration('use_fleet_control')
  use_fleet_control_arg = DeclareLaunchArgument('use_fleet_control', default_value='True')

  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value='eduard')

  # Bringing Up Server
  virtual_joy_server = Node(
    package='edu_virtual_joy',
    executable='virtual_joy',
    name='virtual_joy_server',
    # parameters=[parameter_file],
    namespace=edu_robot_namespace,
    # prefix=['gdbserver localhost:3000'],
    output='screen'    
  )

  return LaunchDescription([
    use_fleet_control_arg,
    edu_robot_namespace_arg,
    virtual_joy_server
  ])
