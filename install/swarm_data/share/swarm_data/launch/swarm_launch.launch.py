#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    '''
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='5',
        description='Number of robots to spawn'
    )
    '''
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
    '''
    # Dynamic robot spawner [3]
    robot_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package='swarm_data',
            executable='robot_spawner.py',
            parameters=[{
                'num_robots': LaunchConfiguration('num_robots'),
                'spawn_formation': 'grid'
            }],
            output='screen'
        )]
    )
    
    # Delay obstacle spawner by 30 seconds (if desired)
    obstacle_spawner = TimerAction(
        period=30.0,
        actions=[Node(
            package='swarm_data',
            executable='obstacle_spawner.py',
            parameters=[{
                'obstacle_density': 0.3,
                'spawn_interval': 300.0,
                'arena_size': 20.0
            }],
            output='screen'
        )]
    )
    
    '''
    return LaunchDescription([
        #num_robots_arg,
        world_file_arg,
        gazebo_server,
        gazebo_client,
        #robot_spawner,
        #obstacle_spawner
    ])
