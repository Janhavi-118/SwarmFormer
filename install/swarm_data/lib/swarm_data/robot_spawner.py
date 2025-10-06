#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import random
import os
import numpy as np
from collections import deque


class SimpleSerialRobotSpawner(Node):
    def __init__(self):
        super().__init__('simple_serial_robot_spawner')

        # Declare parameters with defaults
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('arena_size', 20.0)
        self.declare_parameter('spawn_delay_sec', 1.0)
        self.declare_parameter(
            'robot_sdf_path',
            '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'
        )

        self.num_robots = self.get_parameter('num_robots').value
        self.arena_size = self.get_parameter('arena_size').value
        self.spawn_delay_sec = self.get_parameter('spawn_delay_sec').value
        self.robot_sdf_path = self.get_parameter('robot_sdf_path').value

        # Load robot SDF file contents only once
        if not os.path.exists(self.robot_sdf_path):
            self.get_logger().error(f"Robot SDF file not found: {self.robot_sdf_path}")
            rclpy.shutdown()
            return

        with open(self.robot_sdf_path, 'r') as f:
            self.robot_sdf = f.read()

        # Queue of robot namespaces to spawn e.g., tb3_0, tb3_1, ...
        self.robot_queue = deque([f'tb3_{i}' for i in range(self.num_robots)])

        # Create spawn_entity service client
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_logger().info('Waiting for /spawn_entity service...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        self.get_logger().info(f'Starting serial spawn of {self.num_robots} robots')

        self.spawning = False
        self.spawn_timer = self.create_timer(self.spawn_delay_sec, self.spawn_next_robot)

    def spawn_next_robot(self):
        if self.spawning:
            return

        if not self.robot_queue:
            self.get_logger().info('All robots spawned.')
            self.spawn_timer.cancel()
            return

        self.spawning = True
        robot_ns = self.robot_queue.popleft()

        # Sample random spawn pose (x,y) in arena bounds, ignoring collisions
        half_arena = self.arena_size / 2.0
        x = random.uniform(-half_arena + 1, half_arena - 1)
        y = random.uniform(-half_arena + 1, half_arena - 1)

        self.get_logger().info(f'Spawning robot {robot_ns} at x={x:.2f}, y={y:.2f}')

        req = SpawnEntity.Request()
        req.name = robot_ns
        req.xml = self.robot_sdf
        req.robot_namespace = robot_ns
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.01  # small elevation above ground

        # Random initial yaw orientation
        yaw = random.uniform(0, 2 * np.pi)
        req.initial_pose.orientation.z = float(np.sin(yaw / 2.0))
        req.initial_pose.orientation.w = float(np.cos(yaw / 2.0))

        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda future: self.spawn_response_callback(future, robot_ns))

    def spawn_response_callback(self, future, robot_ns):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f'Robot {robot_ns} spawned successfully.')
            else:
                self.get_logger().warn(f'Robot {robot_ns} failed to spawn: {res.status_message}')
                # Optionally retry spawn by re-enqueueing
                # self.robot_queue.append(robot_ns)
        except Exception as e:
            self.get_logger().error(f'Exception during spawn of {robot_ns}: {e}')
            # Optionally retry spawn by re-enqueueing
            # self.robot_queue.append(robot_ns)
        finally:
            self.spawning = False


def main(args=None):
    rclpy.init(args=args)
    spawner = SimpleSerialRobotSpawner()
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        spawner.get_logger().info('SimpleSerialRobotSpawner shutting down...')
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
