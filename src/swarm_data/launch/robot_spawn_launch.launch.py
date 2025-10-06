#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import random

def generate_launch_description():
    # You can adjust these values or make them arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='7',
        description='Number of robots to spawn'
    )
    arena_size_arg = DeclareLaunchArgument(
        'arena_size',
        default_value='20.0',
        description='Arena size for spawn positions'
    )

    num_robots = LaunchConfiguration('num_robots')
    arena_size = LaunchConfiguration('arena_size')

    # Find the default TurtleBot3 burger model
    try:
        tb3_path = get_package_share_directory('turtlebot3_gazebo')
        model_sdf = os.path.join(tb3_path, 'models', 'turtlebot3_burger', 'model.sdf')
    except Exception:
        model_sdf = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'

    ld = LaunchDescription()
    ld.add_action(num_robots_arg)
    ld.add_action(arena_size_arg)

    spawn_pos = [
      [
        -8.08248571612879,
        -1.8293487266511796,
        0.026981919216495922
      ],
      [
        -7.422376750657204,
        -1.8293487266511796,
        4.640100816709798
      ],
      [
        -6.762267785185619,
        -1.8293487266511796,
        6.129882096049467
      ],
      [
        -8.08248571612879,
        -1.1692397611795946,
        2.0814976880034832
      ],
      [
        -7.422376750657204,
        -1.1692397611795946,
        4.432481598634157
      ],
      [
        -6.762267785185619,
        -1.1692397611795946,
        2.977668581767639
      ],
      [
        -8.08248571612879,
        -0.5091307957080092,
        1.8210378957264524
      ]    ]
    for i in range(len(spawn_pos)):
        robot_ns = f'tb3_{i}'
        # Use random positions in the arena, but you may adjust as needed
        spawn_x = spawn_pos[i][0]
        spawn_y = spawn_pos[i][1]

        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_ns,
                '-file', model_sdf,
                '-robot_namespace', robot_ns,
                '-x', str(spawn_x),
                '-y', str(spawn_y),
                '-z', '0.01',
                '-Y', str(spawn_pos[i][2])
            ],
            output='screen',
            condition=IfCondition(PythonExpression([num_robots, '>=', str(i + 1)])),
        )
        ld.add_action(spawn_node)

    return ld
