#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='11',
        description='Number of robots to launch'
    )
    
    data_output_dir_arg = DeclareLaunchArgument(
        'data_output_dir', 
        default_value='./training_data/episode1',
        description='Directory to save training data'
    )
    
    num_robots = LaunchConfiguration('num_robots')
    data_output_dir = LaunchConfiguration('data_output_dir')

    ld = LaunchDescription()
    ld.add_action(num_robots_arg)
    ld.add_action(data_output_dir_arg)
    
    for i in range(11):  # Maximum robots possible
        robot_id = f'tb3_{i}'

        condition = IfCondition(
            PythonExpression([num_robots, '>=', str(i + 1)])
        )

        # Expert Controller Process
        expert_process = ExecuteProcess(
            cmd=[
                'python3', '/home/janhavi/swarm_ws/src/swarm_data/src/expert_controller.py',
                '--ros-args',
                '-p', f'robot_id:={robot_id}',
                '-p', ['data_output_dir:=', f'{data_output_dir_arg}/{robot_id}'],
                '-p', 'episode_timeout:=60.0',
                '-p', 'max_linear_speed:=0.2',
                '-p', 'max_angular_speed:=1.0',
                '-p', 'goal_tolerance:=0.25',
                '-p', 'safety_distance:=0.8'
            ],
            output='screen',
            condition=condition
        )

        # Gossip Manager Process
        gossip_process = ExecuteProcess(
            cmd=[
                'python3', '/home/janhavi/swarm_ws/src/swarm_data/src/gossip_manager.py', 
                '--ros-args',
                '-p', f'robot_id:={robot_id}',
                '-p', 'arena_size:=20.0',
                '-p', 'max_neighbors:=5',
                '-p', 'heartbeat_interval:=1.0',
                '-p', 'failure_timeout:=3.0',
                '-p', 'gossip_interval:=2.0'
            ],
            output='screen', 
            condition=condition
        )

        ld.add_action(expert_process)
        ld.add_action(gossip_process)

    return ld
