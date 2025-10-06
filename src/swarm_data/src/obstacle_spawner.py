#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import Odometry
import numpy as np
import random
import math
from string import Template
from std_msgs.msg import Bool

def get_obstacle_footprint(obstacle_type):
    # Returns (half-length_x, half-width_y) in meters for AABB check
    if obstacle_type == 'cylinder':
        return 0.3, 0.3   # radius, radius
    elif obstacle_type == 'box':
        return 0.3, 0.3   # 0.6x0.6
    elif obstacle_type == 'wall':
        return 1.0, 0.1   # 2.0 long, 0.2 thick
    elif obstacle_type == 'convex':
        return 0.4, 0.4   # fat cylinder
    return 0.3, 0.3


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')

        # Declare parameters with defaults
        self.declare_parameter('robot_namespaces', [f'tb3_{i}' for i in range(10)])
        self.declare_parameter('obstacle_density', 0.3)
        self.declare_parameter('spawn_interval', 3.0)
        self.declare_parameter('arena_size', 20.0)
        self.declare_parameter('min_obstacle_count', 5)
        self.declare_parameter('min_delete_pct', 0.20)
        self.declare_parameter('max_delete_pct', 0.40)
        self.declare_parameter('min_spawn_pct', 0.10)
        self.declare_parameter('max_spawn_pct', 0.30)
        self.declare_parameter('min_robot_clearance', 1.2)   # min allowed distance robot↔obstacle
        self.declare_parameter('min_obstacle_clearance', 0.8) # min spacing between obstacles

        # Get parameter values
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        self.density = self.get_parameter('obstacle_density').value
        self.spawn_interval = self.get_parameter('spawn_interval').value
        self.arena_size = self.get_parameter('arena_size').value
        self.min_obstacle_count = self.get_parameter('min_obstacle_count').value
        self.min_delete_pct = self.get_parameter('min_delete_pct').value
        self.max_delete_pct = self.get_parameter('max_delete_pct').value
        self.min_spawn_pct = self.get_parameter('min_spawn_pct').value
        self.max_spawn_pct = self.get_parameter('max_spawn_pct').value
        self.min_robot_clearance = self.get_parameter('min_robot_clearance').value
        self.min_obstacle_clearance = self.get_parameter('min_obstacle_clearance').value

        # Track robots’ live positions
        self.robot_positions = {ns: (None, None) for ns in self.robot_namespaces}
        for ns in self.robot_namespaces:
            topic = f"/{ns}/odom"
            self.create_subscription(Odometry, topic, lambda msg, ns=ns: self.on_robot_odom(msg, ns), 10)

        self.autopilot_pause_pub = self.create_publisher(Bool, '/autopilot_pause', 10)
        # Initialize service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity service...')

        # Obstacles info and counters
        self.obstacle_types = [
            {'type': 'cylinder', 'weight': 0.4},
            {'type': 'box', 'weight': 0.3},
            {'type': 'wall', 'weight': 0.2},
            {'type': 'convex', 'weight': 0.1}
        ]
        self.spawned_obstacles = []
        self.obstacle_counter = 0

        self._num_to_spawn = int(self.density * (self.arena_size ** 2) / 4)
        self._spawned_count = 0
        self._max_attempts_per_obstacle = 10

        # Respawn control variables
        self.new_obstacles_to_spawn = 0
        self.obstacles_to_remove = []
        self.deletions_completed = 0
        self.deletions_needed = 0
        self._respawn_spawn_count = 0
        self._recovery_spawn_count = 0
        self._recovery_spawn_target = 0

        self._initial_spawn_started = False
        self._initial_spawn_timer = self.create_timer(0.1, self._start_initial_spawn_once)
        self.create_timer(self.spawn_interval, self.respawn_obstacles)

    # Live robot odometry callback
    def on_robot_odom(self, msg, ns):
        self.robot_positions[ns] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # Position safety check - for robot and other obstacles proximity
    def is_position_safe_for_obstacle(self, x, y):
        # Check robot clearance
        for rpos in self.robot_positions.values():
            if rpos[0] is None or rpos[1] is None:
                continue
            d = np.hypot(x - rpos[0], y - rpos[1])
            if d < self.min_robot_clearance + self.min_obstacle_clearance:
                return False
        # Check obstacle clearance
        for o in self.spawned_obstacles:
            d = np.hypot(x - o['x'], y - o['y'])
            if d < 2 * self.min_obstacle_clearance:
                return False
        return True

    # Sample random spawn pos with safety check
    def sample_safe_obstacle_position(self, max_attempts=20):
        half = self.arena_size / 2.0
        for _ in range(max_attempts):
            x = np.random.uniform(-half + self.min_obstacle_clearance, half - self.min_obstacle_clearance)
            y = np.random.uniform(-half + self.min_obstacle_clearance, half - self.min_obstacle_clearance)
            # Avoid central 2x2m no spawn zone
            if abs(x) < 2.0 and abs(y) < 2.0:
                continue
            if self.is_position_safe_for_obstacle(x, y):
                return x, y
        return None, None

    # Initial spawn kick-off
    def _start_initial_spawn_once(self):
        if not self._initial_spawn_started:
            self._initial_spawn_started = True
            self._initial_spawn_timer.cancel()
            self.get_logger().info(f'Starting initial spawn of {self._num_to_spawn} obstacles...')
            self._spawn_next_obstacle()

    # Spawn helpers with safety checks
    def _spawn_next_obstacle(self):
        if self._spawned_count >= self._num_to_spawn:
            self.get_logger().info(f'Initial spawning complete: {self._spawned_count} obstacles spawned.')
            return
        obstacle_type = self._weighted_choice(self.obstacle_types)
        self._attempt_spawn_obstacle(obstacle_type, attempt=1)

    def _attempt_spawn_obstacle(self, obstacle_type, attempt):
        if attempt > self._max_attempts_per_obstacle:
            self.get_logger().warn(
                f'No safe spot for {obstacle_type} after {self._max_attempts_per_obstacle} attempts, skipping obstacle.'
            )
            self._spawned_count += 1
            self._spawn_next_obstacle()
            return

        x, y = self.sample_safe_obstacle_position()
        if x is None or y is None:
            self._attempt_spawn_obstacle(obstacle_type, attempt + 1)
            return

        sdf = self._generate_obstacle_sdf(obstacle_type)
        req = SpawnEntity.Request()
        req.name = f'obstacle_{self.obstacle_counter}'
        req.xml = sdf
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.0

        future = self.spawn_client.call_async(req)
        future.add_done_callback(
            lambda fut: self._on_spawn_result(fut, x, y, obstacle_type, attempt)
        )

    def _on_spawn_result(self, future, x, y, obstacle_type, last_attempt):
        try:
            res = future.result()
            if res.success:
                self.spawned_obstacles.append({
                    'name': f'obstacle_{self.obstacle_counter}',
                    'x': x,
                    'y': y,
                    'type': obstacle_type
                })
                self.get_logger().info(
                    f'Spawned obstacle_{self.obstacle_counter} ({obstacle_type}) at ({x:.1f}, {y:.1f})'
                )
                self.obstacle_counter += 1
                self._spawned_count += 1
                self._spawn_next_obstacle()
            else:
                self.get_logger().warn(
                    f'Spawn failed on attempt {last_attempt} for obstacle_{self.obstacle_counter}: {res.status_message}'
                )
                self._attempt_spawn_obstacle(obstacle_type, last_attempt + 1)
        except Exception as e:
            self.get_logger().error(f'Exception in spawn callback: {e}')
            self._attempt_spawn_obstacle(obstacle_type, last_attempt + 1)

    # Weighted random choice for obstacle type
    def _weighted_choice(self, choices):
        weights = [c['weight'] for c in choices]
        return np.random.choice([c['type'] for c in choices], p=weights)

    def _generate_obstacle_sdf(self, obstacle_type):
        geometries = {
            'cylinder': '<cylinder><radius>0.3</radius><length>1.0</length></cylinder>',
            'box': '<box><size>0.6 0.6 1.0</size></box>',
            'wall': '<box><size>2.0 0.2 1.0</size></box>',
            'convex': '<cylinder><radius>0.4</radius><length>0.8</length></cylinder>'
        }
        geometry = geometries.get(obstacle_type, geometries['cylinder'])
        height = 0.5 if obstacle_type != 'convex' else 0.4

        sdf_template = Template("""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="obstacle_${counter}">
    <static>true</static>
    <link name="link">
      <pose>0 0 ${height} 0 0 0</pose>
      <collision name="collision">
        <geometry>${geometry}</geometry>
      </collision>
      <visual name="visual">
        <geometry>${geometry}</geometry>
        <material>
          <ambient>0.8 0.3 0.1 1</ambient>
          <diffuse>0.8 0.3 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")
        return sdf_template.substitute(
            counter=self.obstacle_counter,
            height=height,
            geometry=geometry
        )

    # ====== Respawn logic with multi deletions and spawns =======

    def respawn_obstacles(self):
        if not self.spawned_obstacles:
            self.get_logger().info("No obstacles to respawn yet.")
            pause_msg.data = False
            self.autopilot_pause_pub.publish(pause_msg)
            return
        self.get_logger().info("Pausing autopilot for obstacle respawn.")
        pause_msg = Bool()
        pause_msg.data = True
        self.autopilot_pause_pub.publish(pause_msg)

        n_obs = len(self.spawned_obstacles)
        MIN_OBS = self.min_obstacle_count

        if n_obs <= MIN_OBS:
            self.get_logger().info("At or below minimum obstacles; skipping deletion phase.")
            obstacles_to_remove_count = 0
        else:
            delete_pct = random.uniform(self.min_delete_pct, self.max_delete_pct)
            max_removable = n_obs - MIN_OBS
            obstacles_to_remove_count = min(max(1, int(delete_pct * n_obs)), max_removable)

        spawn_pct = random.uniform(self.min_spawn_pct, self.max_spawn_pct)
        self.new_obstacles_to_spawn = max(1, int(spawn_pct * n_obs))

        if obstacles_to_remove_count > 0:
            self.obstacles_to_remove = random.sample(self.spawned_obstacles, k=obstacles_to_remove_count)
        else:
            self.obstacles_to_remove = []

        self.deletions_completed = 0
        self.deletions_needed = len(self.obstacles_to_remove)

        self.get_logger().info(
            f"Deleting {self.deletions_needed} obstacles, will spawn {self.new_obstacles_to_spawn} new obstacles."
        )

        if self.deletions_needed > 0:
            self._delete_next_obstacle()
        else:
            self._respawn_spawn_count = 0
            self._spawn_next_respawn_obstacle()

    def _delete_next_obstacle(self):
        if not self.obstacles_to_remove:
            self.get_logger().info(f'All deletions complete. Spawning {self.new_obstacles_to_spawn} new obstacles.')
            self._respawn_spawn_count = 0
            self._spawn_next_respawn_obstacle()
            return

        obstacle = self.obstacles_to_remove.pop()
        self._delete_obstacle_for_respawn(obstacle['name'])

    def _delete_obstacle_for_respawn(self, obstacle_name):
        self.get_logger().info(f'Deleting obstacle {obstacle_name} for respawn')
        req = DeleteEntity.Request()
        req.name = obstacle_name
        future = self.delete_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_respawn_delete_result(fut, obstacle_name))

    def _on_respawn_delete_result(self, future, obstacle_name):
        try:
            res = future.result()
            if res.success:
                self.spawned_obstacles = [o for o in self.spawned_obstacles if o['name'] != obstacle_name]
                self.get_logger().info(f'Successfully deleted {obstacle_name} for respawn')
            else:
                self.get_logger().warn(f'Failed to delete {obstacle_name} for respawn: {res.status_message}')
        except Exception as e:
            self.get_logger().error(f'Exception in respawn delete callback: {e}')
        finally:
            self.deletions_completed += 1
            self._delete_next_obstacle()

    def _spawn_next_respawn_obstacle(self):
        if self._respawn_spawn_count >= self.new_obstacles_to_spawn:
            self.get_logger().info(f'Respawn cycle complete: spawned {self._respawn_spawn_count} new obstacles.')
            if len(self.spawned_obstacles) < 10:
                self.get_logger().info(f'Obstacle count ({len(self.spawned_obstacles)}) low! Recovering to 30 obstacles...')
                self._recover_obstacle_count(target=30)
            else:
                # Unpause autopilot after full respawn cycle
                self._unpause_autopilot()
            return

        # Respawn only cylinder or box for simplicity
        respawn_obstacle_types = [
            {'type': 'cylinder', 'weight': 0.5},
            {'type': 'box', 'weight': 0.5}
        ]
        obstacle_type = self._weighted_choice(respawn_obstacle_types)
        self._attempt_respawn_spawn_obstacle(obstacle_type, attempt=1)

    def _attempt_respawn_spawn_obstacle(self, obstacle_type, attempt):
        if attempt > self._max_attempts_per_obstacle:
            self.get_logger().warn(
                f'No safe spot for respawn obstacle after {self._max_attempts_per_obstacle} attempts, skipping.'
            )
            self._respawn_spawn_count += 1
            self._spawn_next_respawn_obstacle()
            return

        x, y = self.sample_safe_obstacle_position()
        if x is None or y is None:
            self._attempt_respawn_spawn_obstacle(obstacle_type, attempt + 1)
            return

        sdf = self._generate_obstacle_sdf(obstacle_type)
        req = SpawnEntity.Request()
        req.name = f'obstacle_{self.obstacle_counter}'
        req.xml = sdf
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.0

        future = self.spawn_client.call_async(req)
        future.add_done_callback(
            lambda fut: self._on_respawn_spawn_result(fut, x, y, obstacle_type, attempt)
        )

    def _on_respawn_spawn_result(self, future, x, y, obstacle_type, last_attempt):
        try:
            res = future.result()
            if res.success:
                self.spawned_obstacles.append({
                    'name': f'obstacle_{self.obstacle_counter}',
                    'x': x,
                    'y': y,
                    'type': obstacle_type
                })
                self.get_logger().info(f'Respawn spawned obstacle_{self.obstacle_counter} ({obstacle_type}) at ({x:.1f}, {y:.1f})')
                self.obstacle_counter += 1
                self._respawn_spawn_count += 1
                self._spawn_next_respawn_obstacle()
            else:
                self.get_logger().warn(
                    f'Respawn spawn failed on attempt {last_attempt} for obstacle_{self.obstacle_counter}: {res.status_message}')
                self._attempt_respawn_spawn_obstacle(obstacle_type, last_attempt + 1)
        except Exception as e:
            self.get_logger().error(f'Exception in respawn spawn callback: {e}')
            self._attempt_respawn_spawn_obstacle(obstacle_type, last_attempt + 1)

    # --- Recovery spawning to hit a target obstacle count  ---
    def _recover_obstacle_count(self, target):
        current_count = len(self.spawned_obstacles)
        needed = target - current_count
        if needed <= 0:
            self.get_logger().info(f"No recovery needed; current count: {current_count}")
            return
        self.get_logger().info(f"Recovering {needed} obstacles to reach target {target}")
        self._recovery_spawn_count = 0
        self._recovery_spawn_target = needed
        self._spawn_next_recovery_obstacle()

    def _spawn_next_recovery_obstacle(self):
        if self._recovery_spawn_count >= self._recovery_spawn_target:
            self.get_logger().info(f"Recovery spawning complete: spawned {self._recovery_spawn_count} obstacles.")
            self._unpause_autopilot()  # Unpause autopilot at recovery completion
            return
        obstacle_type = self._weighted_choice(self.obstacle_types)
        self._attempt_recovery_spawn(obstacle_type, attempt=1)

    def _attempt_recovery_spawn(self, obstacle_type, attempt):
        if attempt > self._max_attempts_per_obstacle:
            self.get_logger().warn(
                f"Failed to find safe spot for recovery after {self._max_attempts_per_obstacle} attempts, skipping."
            )
            self._recovery_spawn_count += 1
            self._spawn_next_recovery_obstacle()
            return

        x, y = self.sample_safe_obstacle_position()
        if x is None or y is None:
            self._attempt_recovery_spawn(obstacle_type, attempt + 1)
            return

        sdf = self._generate_obstacle_sdf(obstacle_type)
        req = SpawnEntity.Request()
        req.name = f'obstacle_{self.obstacle_counter}'
        req.xml = sdf
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.0

        future = self.spawn_client.call_async(req)
        future.add_done_callback(
            lambda fut: self._on_recovery_spawn_result(fut, x, y, obstacle_type, attempt)
        )

    def _on_recovery_spawn_result(self, future, x, y, obstacle_type, last_attempt):
        try:
            res = future.result()
            if res.success:
                self.spawned_obstacles.append({
                    'name': f'obstacle_{self.obstacle_counter}',
                    'x': x,
                    'y': y,
                    'type': obstacle_type
                })
                self.get_logger().info(f"Recovery spawned obstacle_{self.obstacle_counter} ({obstacle_type}) at ({x:.1f}, {y:.1f})")
                self.obstacle_counter += 1
                self._recovery_spawn_count += 1
                self._spawn_next_recovery_obstacle()
            else:
                self.get_logger().warn(f"Recovery spawn failed on attempt {last_attempt} for obstacle_{self.obstacle_counter}: {res.status_message}")
                self._attempt_recovery_spawn(obstacle_type, last_attempt + 1)
        except Exception as e:
            self.get_logger().error(f"Exception in recovery spawn callback: {e}")
            self._attempt_recovery_spawn(obstacle_type, last_attempt + 1)

    # Delete obstacle wrapper
    def _delete_obstacle(self, obstacle_name):
        self.get_logger().info(f'Deleting obstacle {obstacle_name}')
        req = DeleteEntity.Request()
        req.name = obstacle_name
        future = self.delete_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_delete_result(fut, obstacle_name))

    def _on_delete_result(self, future, obstacle_name):
        try:
            res = future.result()
            if res.success:
                self.spawned_obstacles = [o for o in self.spawned_obstacles if o['name'] != obstacle_name]
                self.get_logger().info(f'Successfully deleted {obstacle_name}')
            else:
                self.get_logger().warn(f'Failed to delete {obstacle_name}: {res.status_message}')
        except Exception as e:
            self.get_logger().error(f'Exception in delete callback: {e}')

    def _unpause_autopilot(self):
        self.get_logger().info("Unpausing autopilot after obstacle respawn.")
        pause_msg = Bool()
        pause_msg.data = False
        self.autopilot_pause_pub.publish(pause_msg)

def main(args=None):
    rclpy.init(args=args)
    spawner = ObstacleSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
