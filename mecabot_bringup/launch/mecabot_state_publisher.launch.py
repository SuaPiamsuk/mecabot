#!/usr/bin/env python3
# Author: Addison Sears-Collins
# Date: August 31, 2021
# Description: Launch a basic mobile robot
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
  pkg_create3_description = get_package_share_directory('mecabot_description')
  xacro_file = PathJoinSubstitution([pkg_create3_description, 'urdf', 'mecabot.urdf'])

  robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_description':
             Command(
                  ['xacro', ' ', xacro_file, ' ',
                   ])},
        ],
     
    )
  
  joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

  # ld = LaunchDescription(ARGUMENTS)
  ld = LaunchDescription()
 
  # Add nodes to LaunchDescription
  ld.add_action(joint_state_publisher)
  ld.add_action(robot_state_publisher)


  
  return ld
