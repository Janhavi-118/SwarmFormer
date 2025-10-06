#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for dynamic configuration
    obstacle_density_arg = DeclareLaunchArgument(
        'obstacle_density',
        default_value='0.45',
        description='Density of obstacles in the arena'
    )
    spawn_interval_arg = DeclareLaunchArgument(
        'spawn_interval',
        default_value='300.0',
        description='Interval (seconds) between respawning obstacles'
    )
    arena_size_arg = DeclareLaunchArgument(
        'arena_size',
        default_value='20.0',
        description='Size of the (square) arena in meters'
    )

    # Launch the obstacle spawner node with parameters
    obstacle_spawner_node = Node(
        package='swarm_data',
        executable='obstacle_spawner.py',
        name='obstacle_spawner',
        parameters=[
            {'obstacle_density': LaunchConfiguration('obstacle_density')},
            {'spawn_interval': LaunchConfiguration('spawn_interval')},
            {'arena_size': LaunchConfiguration('arena_size')}
        ],
        output='screen'
    )

    return LaunchDescription([
        obstacle_density_arg,
        spawn_interval_arg,
        arena_size_arg,
        obstacle_spawner_node
    ])
