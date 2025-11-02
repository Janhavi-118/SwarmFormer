#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(
            get_package_share_directory('swarm_data'),
            'worlds',
            'swarm.world'
        ),
        description='Path to world file'
    )
    
    # Gazebo server launch [2]
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', 
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world_file')],
        output='screen'
    )
    
    # Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    return LaunchDescription([
        world_file_arg,
        gazebo_server,
        gazebo_client,
    ])
