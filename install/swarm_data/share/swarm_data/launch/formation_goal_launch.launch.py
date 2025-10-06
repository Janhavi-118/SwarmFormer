#!/usr/bin/env python3

import os
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def send_formation_goals(context, *args, **kwargs):
    """Send formation goals from JSON scenario or parameters"""
    
    # Try to read from current scenario first
    formation_goals = []
    
    if os.path.exists('current_scenario.json'):
        try:
            with open('current_scenario.json', 'r') as f:
                scenario = json.load(f)
            formation_goals = scenario['formation_goals']
            print(f"📋 Using formation goals from scenario: {scenario['scenario_id']}")
        except Exception as e:
            print(f"⚠️ Error reading scenario: {e}")
    
    # If no scenario or failed to read, use default goals
    if not formation_goals:
        # Default line formation goals
        formation_goals = [     
      [
        5.684495969843876,
        6.418101032791164
      ],
      [
        7.814958999713429,
        6.418101032791164
      ],
      [
        9.945422029582982,
        6.418101032791164
      ],
      [
        5.684495969843876,
        8.548564062660716
      ],
      [
        7.814958999713429,
        8.548564062660716
      ],
      [
        9.945422029582982,
        8.548564062660716
      ],
      [
        5.684495969843876,
        10
      ]        ]
        print("📍 Using default line formation goals")
    
    # Create goal publishing processes
    actions = []
    
    for i, (x, y) in enumerate(formation_goals):
        robot_id = f'tb3_{i}'
        print(f"   {robot_id}: goal ({x}, {y})")
        
        goal_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub',
                f'/formation/goal_{robot_id}',
                'geometry_msgs/msg/Point',
                f'{{x: {x}, y: {y}, z: 0.0}}',
                '--once'
            ],
            output='screen',
            name=f'send_goal_{robot_id}'
        )
        actions.append(goal_cmd)
    
    return actions

def generate_launch_description():
    ld = LaunchDescription()
    
    # Add formation goal sender
    goals_opaque = OpaqueFunction(function=send_formation_goals)
    ld.add_action(goals_opaque)
    
    return ld
