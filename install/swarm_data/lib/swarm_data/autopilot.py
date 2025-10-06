#!/usr/bin/env python3
'''
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import numpy as np
import math
import time
import random

class ExplorationAutopilot(Node):
    def __init__(self):
        super().__init__('exploration_autopilot')

        # Declare and read robot_id parameter
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value

        # Internal state
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_max = None
        self.lidar_angle_increment = None
        self.pose = np.zeros(3)  # [x, y, yaw]
        self.paused = False

        # For random turning when without formation goal
        self.random_turn = 0.0
        self.last_turn_change_time = 0.0
        self.turn_change_interval = 5.0  # seconds

        # Formation goal
        self.formation_goal = None

        # State machine states: 'turning', 'escaping', 'goal_finding'
        self.state = 'goal_finding'

        # Gains for formation control (tunable)
        self.k_formation_lin = 0.2
        self.k_formation_ang = 1.0

        # Subscribers
        self.create_subscription(LaserScan, f'/{self.robot_id}/scan', self.lidar_cb, 10)
        self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/autopilot_pause', self.pause_cb, 10)
        self.create_subscription(Point, f'/formation/goal_{self.robot_id}', self.formation_goal_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)

        # Control loop timer (~10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"Exploration Autopilot started for robot: {self.robot_id}")

    def lidar_cb(self, msg: LaserScan):
        # Store angles and ranges to use in obstacle avoidance logic
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_max = msg.angle_max
        self.lidar_angle_increment = msg.angle_increment

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        # Yaw angle from quaternion
        yaw = math.atan2(
            2.0 * (orient.w * orient.z + orient.x * orient.y),
            1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        )

        self.pose[0] = pos.x
        self.pose[1] = pos.y
        self.pose[2] = yaw

    def pause_cb(self, msg: Bool):
        if msg.data and not self.paused:
            self.paused = True
            self.cmd_pub.publish(Twist())  # Stop immediately
            self.get_logger().info('Autopilot paused')
        elif not msg.data and self.paused:
            self.paused = False
            self.get_logger().info('Autopilot resumed')

    def formation_goal_cb(self, msg: Point):
        self.formation_goal = [msg.x, msg.y]

    def normalize_angle(self, angle: float) -> float:
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if self.paused:
            return  # Do nothing when paused

        twist = Twist()
        current_time = time.time()

        # Parameters for obstacle avoidance
        safe_close_call = 0.3  # meters, very close obstacle threshold
        safe_slow_down = 0.6   # meters, slow down threshold

        # Check LiDAR front sector if data is available
        if self.lidar_ranges is not None and len(self.lidar_ranges) > 0 and \
                self.lidar_angle_min is not None and self.lidar_angle_increment is not None:

            # Compute angles for each lidar measurement
            angles = self.lidar_angle_min + np.arange(len(self.lidar_ranges)) * self.lidar_angle_increment

            # Indices for -30 to +30 degrees (in radians)
            front_sector_indices = np.where((angles >= -math.radians(45)) & (angles <= math.radians(45)))[0]

            front_sector_distances = self.lidar_ranges[front_sector_indices]
            # Filter out invalid or NaN ranges
            front_sector_distances = front_sector_distances[np.isfinite(front_sector_distances)]
            if len(front_sector_distances) == 0:
                min_front_dist = float('inf')
            else:
                min_front_dist = np.min(front_sector_distances)
        else:
            min_front_dist = float('inf')

        # State machine handling for obstacle avoidance

        if self.state == 'turning':
            # Keep turning until front sector clear
            if min_front_dist < safe_close_call:
                twist.linear.x = 0.0  # no forward motion while turning
                # Maintain or assign random turning direction
                if self.random_turn == 0.0 or (current_time - self.last_turn_change_time) > 2.0:
                    self.random_turn = 0.5 if random.random() > 0.5 else -0.5
                    self.last_turn_change_time = current_time
                twist.angular.z = self.random_turn
                self.get_logger().debug(f"Turning to clear obstacle, min front dist: {min_front_dist:.2f}m")
            else:
                # Sector clear, switch to escaping state (move forward a bit)
                self.state = 'escaping'
                self.escape_start_time = current_time
                self.get_logger().debug("Front sector clear, switching to escaping state")
                twist.linear.x = 0.5
                twist.angular.z = 0.0
        elif self.state == 'escaping':
            # Move forward slowly for a bit (e.g. 1 second)
            escape_duration = 1.0  # seconds
            if current_time - self.escape_start_time < escape_duration:
                twist.linear.x = 0.5
                twist.angular.z = 0.0
                self.get_logger().debug("Escaping obstacle by moving forward")
            else:
                # After escape, switch back to goal_finding
                self.state = 'goal_finding'
                self.get_logger().debug("Escape completed, switching to goal_finding")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:  # goal_finding state
            # Normal behavior: check distances and adjust

            if min_front_dist < safe_close_call:
                # Obstacle too close, switch to turning state
                self.state = 'turning'
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().debug(f"Obstacle too close ({min_front_dist:.2f} m), switching to turning")
            elif min_front_dist < safe_slow_down:
                # Slow down and gentle turning
                twist.linear.x = 0.1
                # Occasionally change turn direction
                if current_time - self.last_turn_change_time > self.turn_change_interval:
                    self.random_turn = random.uniform(-0.3, 0.3)
                    self.last_turn_change_time = current_time
                twist.angular.z = self.random_turn
                self.get_logger().debug(f"Near obstacle ({min_front_dist:.2f} m): slowing & turning")
            else:
                # Clear path: track formation goal if available
                if self.formation_goal is not None:
                    dx = self.formation_goal[0] - self.pose[0]
                    dy = self.formation_goal[1] - self.pose[1]
                    dist = math.hypot(dx, dy)
                    angle_to_goal = math.atan2(dy, dx)
                    angle_error = self.normalize_angle(angle_to_goal - self.pose[2])

                    # Proportional control for linear and angular velocities
                    lin_vel = self.k_formation_lin * dist
                    ang_vel = self.k_formation_ang * angle_error

                    # Clamp speeds
                    lin_vel = min(lin_vel, 0.25)
                    ang_vel = max(min(ang_vel, 1.0), -1.0)

                    twist.linear.x = lin_vel
                    twist.angular.z = ang_vel
                    self.get_logger().debug(f"Formation control: dist={dist:.2f}, ang_err={angle_error:.2f}")
                else:
                    # No goal — random walk: move forward slowly and turn randomly occasionally
                    twist.linear.x = 0.0
                    if current_time - self.last_turn_change_time > self.turn_change_interval:
                        self.random_turn = random.uniform(-0.5, 0.5)
                        self.last_turn_change_time = current_time
                    twist.angular.z = 0.0
                    self.get_logger().debug("Clear path: random walk")

        # Publish velocity commands
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import math
import time


class ExplorationAutopilot(Node):
    def __init__(self):
        super().__init__('exploration_autopilot')

        # Params
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value

        # States: FORWARD, TURNING, ESCAPE, STUCK
        self.state = 'FORWARD'
        self.turn_direction = 1   # 1 = CCW, -1 = CW
        self.turn_angle_deg = 30  # degrees per incremental turn
        self.turn_speed = 0.4     # rad/s
        self.forward_speed = 0.15
        self.close_call_thresh = 0.3
        self.safe_sector_width = 90    # degrees (±45°)
        self.escape_time = 1.0         # seconds
        self.max_turn_retries = 12

        # Robot state
        self.pose = np.zeros(3)  # [x, y, yaw]
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None
        self.formation_goal = None
        self.paused = False

        # Internals
        self.turn_start_heading = None
        self.turn_retries = 0
        self.escape_start_time = None

        # Subscriptions
        self.create_subscription(LaserScan, f'/{self.robot_id}/scan', self.lidar_cb, 10)
        self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/autopilot_pause', self.pause_cb, 10)
        self.create_subscription(Point, f'/formation/goal_{self.robot_id}', self.goal_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)

        # Loop timer
        self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info(f"ExplorationAutopilot started for {self.robot_id}")

    # ------------ Callbacks ------------
    def lidar_cb(self, msg: LaserScan):
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orient.w * orient.z + orient.x * orient.y),
            1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z),
        )
        self.pose[:] = [pos.x, pos.y, yaw]

    def pause_cb(self, msg: Bool):
        if msg.data and not self.paused:
            self.paused = True
            self.stop_robot()
            self.get_logger().info("Autopilot paused")
        elif not msg.data and self.paused:
            self.paused = False
            self.get_logger().info("Autopilot resumed")

    def goal_cb(self, msg: Point):
        self.formation_goal = [msg.x, msg.y]

    # ------------ Utilities ------------
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def yaw_deg(self):
        return math.degrees(self.pose[2]) % 360

    def forward_sector_clear(self):
        """Check if ±(safe_sector_width/2) deg sector is clear."""
        if self.lidar_ranges is None:
            return True

        half_width_rad = math.radians(self.safe_sector_width / 2)
        angles = self.lidar_angle_min + np.arange(len(self.lidar_ranges)) * self.lidar_angle_increment
        in_sector = (angles >= -half_width_rad) & (angles <= half_width_rad)
        sector_vals = self.lidar_ranges[in_sector]
        sector_vals = sector_vals[np.isfinite(sector_vals)]
        if len(sector_vals) == 0:
            return False
        return np.all(sector_vals > self.close_call_thresh)

    # ------------ State Machine ------------
    def control_loop(self):
        if self.paused:
            return
        if self.lidar_ranges is None:
            return

        twist = Twist()

        if self.state == 'FORWARD':
            if not self.forward_sector_clear():
                self.get_logger().info("Obstacle ahead → TURNING")
                self.state = 'TURNING'
                self.turn_start_heading = self.yaw_deg()
                self.turn_retries = 0
            else:
                # Track formation goal if exists
                if self.formation_goal is not None:
                    dx = self.formation_goal[0] - self.pose[0]
                    dy = self.formation_goal[1] - self.pose[1]
                    goal_dist = math.hypot(dx, dy)
                    goal_ang = math.atan2(dy, dx)
                    ang_err = self.normalize_angle(goal_ang - self.pose[2])
                    twist.linear.x = min(0.25, 0.2 * goal_dist)
                    twist.angular.z = max(min(ang_err, 1.0), -1.0)
                else:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = 0.0

        elif self.state == 'TURNING':
            if self.forward_sector_clear():
                self.get_logger().info("Path clear → ESCAPE")
                self.state = 'ESCAPE'
                self.escape_start_time = time.time()
            else:
                # Turn in place
                twist.angular.z = self.turn_direction * self.turn_speed

                # Heading change check
                current_heading = self.yaw_deg()
                delta = abs((current_heading - self.turn_start_heading + 540) % 360 - 180)
                if delta >= self.turn_angle_deg:
                    self.turn_start_heading = current_heading
                    self.turn_retries += 1
                    if self.turn_retries >= self.max_turn_retries:
                        self.get_logger().error("No path found → STUCK")
                        self.state = 'STUCK'

        elif self.state == 'ESCAPE':
            twist.linear.x = self.forward_speed
            if time.time() - self.escape_start_time >= self.escape_time:
                self.get_logger().info("Escape done → FORWARD")
                self.state = 'FORWARD'

        elif self.state == 'STUCK':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish command
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import math
import time
import functools


class ExplorationAutopilot(Node):
    def __init__(self):
        super().__init__('exploration_autopilot')

        # Parameters
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('swarm_size', 10)
        self.robot_id = self.get_parameter('robot_id').value
        swarm_size = self.get_parameter('swarm_size').value

        # Teammates
        self.teammates = [f"tb3_{i}" for i in range(swarm_size) if f"tb3_{i}" != self.robot_id]

        # Movement config
        self.forward_speed = 0.12
        self.turn_speed = 0.3
        self.turn_angle_deg = 20
        self.close_call_thresh = 0.4
        self.safe_sector_width = 90
        self.safety_gap = 0.8
        self.escape_time = 0.8
        self.max_turn_retries = 8

        # Goal handling
        self.goal_block_radius = 0.5
        self.safe_goal_search_radius = 1.2
        self.safe_goal_angle_step = 20
        self.goal_offset_distance = 0.6

        # Stuck detection
        self.last_pos = (0.0, 0.0)
        self.stuck_counter = 0
        self.stuck_threshold = 25
        self.recovery_mode = False
        self.recovery_start_time = 0.0

        # State
        self.state = 'FORWARD'
        self.turn_direction = 1
        self.turn_start_heading = 0.0
        self.turn_retries = 0
        self.escape_start_time = 0.0
        self.yield_start_time = None
        self.yield_timeout = 3.0

        # Pose and sensors
        self.pose = np.array([0.0, 0.0, 0.0])
        self.pose_initialized = False
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None

        # Goals and swarm
        self.formation_goal = None
        self.active_goal = None
        self.paused = False
        self.teammate_poses = {}

        # Subscriptions
        self.create_subscription(LaserScan, f'/{self.robot_id}/scan', self.lidar_cb, 10)
        self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_cb, 10)
        self.create_subscription(Point, f'/formation/goal_{self.robot_id}', self.goal_cb, 10)
        self.create_subscription(Bool, '/autopilot_pause', self.pause_cb, 10)

        for mate in self.teammates:
            self.create_subscription(
                Odometry,
                f'/{mate}/odom',
                functools.partial(self.teammate_odom_cb, robot_id=mate),
                10
            )

        # Publisher and timer
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.create_timer(0.05, self.control_loop)

    # Callbacks
    def lidar_cb(self, msg: LaserScan):
        if len(msg.ranges) > 0:
            self.lidar_ranges = np.array(msg.ranges)
            self.lidar_angle_min = msg.angle_min
            self.lidar_angle_increment = msg.angle_increment

    def odom_cb(self, msg: Odometry):
        try:
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orient.w * orient.z + orient.x * orient.y),
                1.0 - 2.0 * (orient.y**2 + orient.z**2)
            )
            self.pose[:] = [pos.x, pos.y, yaw]
            self.pose_initialized = True
        except Exception as e:
            self.get_logger().error(f"odom_cb error: {e}")

    def teammate_odom_cb(self, msg: Odometry, robot_id: str):
        try:
            pos = msg.pose.pose.position
            if math.isfinite(pos.x) and math.isfinite(pos.y):
                self.teammate_poses[robot_id] = (pos.x, pos.y)
        except Exception as e:
            self.get_logger().error(f"teammate_odom_cb[{robot_id}] error: {e}")

    def goal_cb(self, msg: Point):
        try:
            if math.isfinite(msg.x) and math.isfinite(msg.y):
                self.formation_goal = [msg.x, msg.y]
                self.active_goal = self.formation_goal
        except Exception as e:
            self.get_logger().error(f"goal_cb error: {e}")

    def pause_cb(self, msg: Bool):
        if msg.data and not self.paused:
            self.paused = True
            self.stop_robot()
        elif not msg.data and self.paused:
            self.paused = False

    # Utils
    def get_priority(self):
        try:
            return int(self.robot_id.split('_')[-1])
        except:
            return 9999

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def yaw_deg(self):
        return math.degrees(self.pose[2]) % 360.0

    def lidar_sector_clear(self, center_angle, width_deg, dist_thresh):
        if (self.lidar_ranges is None or
            self.lidar_angle_increment is None or abs(self.lidar_angle_increment) < 1e-6):
            return True
        try:
            half_w = math.radians(width_deg / 2)
            angles = self.lidar_angle_min + np.arange(len(self.lidar_ranges)) * self.lidar_angle_increment
            rel = (angles - center_angle + math.pi) % (2 * math.pi) - math.pi
            mask = (rel >= -half_w) & (rel <= half_w)
            vals = self.lidar_ranges[mask]
            vals = vals[np.isfinite(vals) & (vals > 0)]
            return len(vals) > 0 and np.all(vals > dist_thresh)
        except:
            return True

    def forward_sector_clear(self):
        return self.lidar_sector_clear(0.0, self.safe_sector_width, self.close_call_thresh)

    def any_robot_too_close(self):
        if not self.pose_initialized or not self.teammate_poses:
            return False, None, float('inf')
        closest, min_d = None, float('inf')
        for tid, (x, y) in self.teammate_poses.items():
            dx, dy = x - self.pose[0], y - self.pose[1]
            d = math.hypot(dx, dy)
            if d < min_d:
                min_d, closest = d, tid
        if min_d < self.safety_gap and closest:
            return True, int(closest.split('_')[-1]), min_d
        return False, None, min_d

    def robot_in_direction(self, angle_rad, width_deg=60):
        if not self.pose_initialized or not self.teammate_poses:
            return False, None
        half_w = math.radians(width_deg / 2)
        for tid, (x, y) in self.teammate_poses.items():
            dx, dy = x - self.pose[0], y - self.pose[1]
            d = math.hypot(dx, dy)
            if d < self.safety_gap:
                bearing = math.atan2(dy, dx)
                rel = self.normalize_angle(bearing - angle_rad)
                if -half_w <= rel <= half_w:
                    return True, int(tid.split('_')[-1])
        return False, None

    def goal_area_blocked(self, goal_xy):
        if (goal_xy is None or not self.pose_initialized or
            self.lidar_ranges is None or
            self.lidar_angle_increment is None or abs(self.lidar_angle_increment) < 1e-6):
            return False
        gx, gy = goal_xy
        dx, dy = gx - self.pose[0], gy - self.pose[1]
        dist_goal = math.hypot(dx, dy)
        bearing = self.normalize_angle(math.atan2(dy, dx) - self.pose[2])
        valid = self.lidar_ranges[np.isfinite(self.lidar_ranges) & (self.lidar_ranges > 0)]
        if len(valid) == 0 or dist_goal > np.max(valid):
            return False
        idx = (bearing - self.lidar_angle_min) / self.lidar_angle_increment
        idx = int(max(0, min(len(self.lidar_ranges) - 1, idx)))
        bd = self.lidar_ranges[idx]
        if not np.isfinite(bd) or bd <= 0:
            return False
        return bd < dist_goal and (dist_goal - bd) < self.goal_block_radius

    def goal_conflict_with_teammates(self):
        if not self.pose_initialized or self.formation_goal is None:
            return False, None
        gx, gy = self.formation_goal
        for tid, (x, y) in self.teammate_poses.items():
            if math.hypot(x - gx, y - gy) < self.goal_offset_distance:
                return True, int(tid.split('_')[-1])
        return False, None

    def find_nearest_accessible_point(self):
        if not self.pose_initialized or self.formation_goal is None:
            return None
        gx, gy = self.formation_goal
        candidates = []
        for r in np.linspace(0.4, self.safe_goal_search_radius, 5):
            for deg in range(0, 360, self.safe_goal_angle_step):
                ang = math.radians(deg)
                cx, cy = gx + r * math.cos(ang), gy + r * math.sin(ang)
                if not self.goal_area_blocked((cx, cy)):
                    conflict = any(
                        math.hypot(cx - tx, cy - ty) < self.goal_offset_distance
                        for tx, ty in self.teammate_poses.values()
                    )
                    if not conflict:
                        rd = math.hypot(cx - self.pose[0], cy - self.pose[1])
                        gd = math.hypot(cx - gx, cy - gy)
                        candidates.append(((cx, cy), 0.7 * rd + 0.3 * gd))
        return min(candidates, key=lambda c: c[1])[0] if candidates else None


    def is_safe_to_move(self, twist):
        ok, prio, dist = self.any_robot_too_close()
        if ok and prio is not None:
            myp = self.get_priority()
            if myp <= prio:
                return False, 'yield'
            if dist < 0.4:
                return False, 'emergency_stop'
        return True, 'safe'

    def control_loop(self):
        if not self.pose_initialized or self.paused or self.lidar_ranges is None:
            return
        t = Twist()

        # Recovery
        if self.recovery_mode:
            if time.time() - self.recovery_start_time < 2.0:
                t.linear.x = -0.15
                t.angular.z = 0.5 if self.get_priority() % 2 == 0 else -0.5
                self.cmd_pub.publish(t)
                return
            self.recovery_mode = False
            self.state = 'FORWARD'
            self.stuck_counter = 0
            self.yield_start_time = None

        # Goal handling
        if self.formation_goal:
            conflict, cp = self.goal_conflict_with_teammates()
            if conflict:
                if self.get_priority() <= cp:
                    alt = self.find_nearest_accessible_point()
                    self.active_goal = alt or self.formation_goal
                else:
                    self.active_goal = self.formation_goal
            else:
                if self.goal_area_blocked(self.formation_goal):
                    alt = self.find_nearest_accessible_point()
                    self.active_goal = alt or self.formation_goal
                else:
                    self.active_goal = self.formation_goal

        # State machine
        if self.state == 'FORWARD':
            safe, reason = self.is_safe_to_move(t)
            if not safe:
                if reason == 'yield':
                    if self.yield_start_time is None:
                        self.yield_start_time = time.time()
                    if time.time() - self.yield_start_time < self.yield_timeout:
                        t.linear.x = -0.2
                        self.cmd_pub.publish(t)
                        return
                    self.yield_start_time = None
                    self.state = 'TURNING'
                    self.turn_start_heading = self.yaw_deg()
                    self.turn_retries = 0
                else:
                    t.linear.x = 0.0
                    t.angular.z = 0.0
                    self.cmd_pub.publish(t)
                    return
            else:
                self.yield_start_time = None
                if (not self.forward_sector_clear() or
                    (self.active_goal and self.goal_area_blocked(self.active_goal))):
                    self.state = 'TURNING'
                    self.turn_start_heading = self.yaw_deg()
                    self.turn_retries = 0
                else:
                    if self.active_goal:
                        dx = self.active_goal[0] - self.pose[0]
                        dy = self.active_goal[1] - self.pose[1]
                        dist = math.hypot(dx, dy)
                        if dist < 0.15:
                            t.linear.x = 0.0
                            t.angular.z = 0.0
                            self.cmd_pub.publish(t)
                            return
                        
                        err = self.normalize_angle(math.atan2(dy, dx) - self.pose[2])
                        if abs(err) < math.radians(45):  # Within ±45° of goal
                            t.linear.x = 0.05 if dist < 0.2 else min(0.2, 0.15 * dist)
                        else:
                            t.linear.x = 0.0  # Stop and turn in place first
                        
                        t.angular.z = max(min(err * 1.5, 0.8), -0.8)
                    else:
                        t.linear.x = self.forward_speed

        elif self.state == 'TURNING':
            safe, reason = self.is_safe_to_move(t)
            if not safe:
                t.linear.x = 0.0
                t.angular.z = 0.0
                self.cmd_pub.publish(t)
                return
            if self.forward_sector_clear():
                front, _ = self.robot_in_direction(self.pose[2], 60)
                if not front:
                    self.state = 'ESCAPE'
                    self.escape_start_time = time.time()
                else:
                    t.angular.z = self.turn_direction * self.turn_speed
                    delta = abs((self.yaw_deg() - self.turn_start_heading + 540) % 360 - 180)
                    if delta >= self.turn_angle_deg:
                        self.turn_start_heading = self.yaw_deg()
                        self.turn_retries += 1
                        if self.turn_retries % 2 == 0:
                            self.turn_direction *= -1
                        if self.turn_retries >= self.max_turn_retries:
                            self.state = 'STUCK'
            else:
                t.angular.z = self.turn_direction * self.turn_speed
                delta = abs((self.yaw_deg() - self.turn_start_heading + 540) % 360 - 180)
                if delta >= self.turn_angle_deg:
                    self.turn_start_heading = self.yaw_deg()
                    self.turn_retries += 1
                    if self.turn_retries % 2 == 0:
                        self.turn_direction *= -1
                    if self.turn_retries >= self.max_turn_retries:
                        self.state = 'STUCK'

        elif self.state == 'ESCAPE':
            safe, reason = self.is_safe_to_move(t)
            if not safe and reason == 'emergency_stop':
                t.linear.x = 0.0
                t.angular.z = 0.0
            elif not safe:
                t.linear.x = -0.1
            else:
                t.linear.x = self.forward_speed * 0.8
            if time.time() - self.escape_start_time >= self.escape_time:
                self.state = 'FORWARD'

        elif self.state == 'STUCK':
            self.recovery_mode = True
            self.recovery_start_time = time.time()
            self.state = 'FORWARD'

        # Publish and stuck detect
        self.cmd_pub.publish(t)
        dx = self.pose[0] - self.last_pos[0]
        dy = self.pose[1] - self.last_pos[1]
        if math.hypot(dx, dy) < 0.03:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            self.last_pos = (float(self.pose[0]), float(self.pose[1]))
        if self.stuck_counter > self.stuck_threshold and not self.recovery_mode:
            self.recovery_mode = True
            self.recovery_start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import math
import time
import random

class ExplorationAutopilot(Node):
    def __init__(self):
        super().__init__('exploration_autopilot')
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value
        self.declare_parameter('swarm_size', 10)
        swarm_size = self.get_parameter('swarm_size').value
        # Teammates
        self.teammates = [f"tb3_{i}" for i in range(swarm_size)
                          if f"tb3_{i}" != self.robot_id]

        # Config
        self.forward_speed = 0.15
        self.turn_speed = 0.4
        self.min_obstacle_dist = 0.5
        self.goal_tolerance = 0.2
        self.yield_time = 3.0  # seconds to back off

        # State
        self.pose = np.zeros(3)
        self.pose_ready = False
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None
        self.formation_goal = None
        self.paused = False

        # Yield state
        self.yielding = False
        self.yield_start = 0.0

        # Subscriptions
        self.create_subscription(LaserScan, f'/{self.robot_id}/scan', self.lidar_cb, 10)
        self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_cb, 10)
        self.create_subscription(Point, f'/formation/goal_{self.robot_id}', self.goal_cb, 10)
        self.create_subscription(Bool, '/autopilot_pause', self.pause_cb, 10)
        for mate in self.teammates:
            self.create_subscription(
                Odometry, f'/{mate}/odom',
                lambda msg, m=mate: self.teammate_odom_cb(msg, m), 10)

        self.teammate_poses = {}
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

    def lidar_cb(self, msg):
        if len(msg.ranges) > 0:
            self.lidar_ranges = np.array(msg.ranges)
            self.lidar_angle_min = msg.angle_min
            self.lidar_angle_increment = msg.angle_increment

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(
            2*(o.w*o.z + o.x*o.y),
            1 - 2*(o.y*o.y + o.z*o.z))
        self.pose[:] = [pos.x, pos.y, yaw]
        self.pose_ready = True

    def teammate_odom_cb(self, msg, robot_id):
        pos = msg.pose.pose.position
        self.teammate_poses[robot_id] = (pos.x, pos.y)

    def goal_cb(self, msg):
        self.formation_goal = [msg.x, msg.y]

    def pause_cb(self, msg):
        self.paused = msg.data
        if self.paused:
            self.cmd_pub.publish(Twist())

    def normalize_angle(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def get_front_distance(self):
        if self.lidar_ranges is None: return float('inf')
        angles = self.lidar_angle_min + np.arange(len(self.lidar_ranges)) * self.lidar_angle_increment
        mask = (angles >= -math.pi/6) & (angles <= math.pi/6)
        vals = self.lidar_ranges[mask]
        vals = vals[np.isfinite(vals)&(vals>0)]
        return np.min(vals) if len(vals)>0 else float('inf')

    def teammate_in_front(self):
        # same front cone ±30°, check for any teammate
        for (x,y) in self.teammate_poses.values():
            dx = x - self.pose[0]; dy = y - self.pose[1]
            d = math.hypot(dx,dy)
            if d< self.min_obstacle_dist:
                bearing = math.atan2(dy,dx)
                rel = self.normalize_angle(bearing - self.pose[2])
                if abs(rel) <= math.pi/6:
                    return True
        return False

    def control_loop(self):
        if self.paused or not self.pose_ready: return
        now = time.time()
        t = Twist()

        # If currently yielding/backing off
        if self.yielding:
            if now - self.yield_start < self.yield_time:
                t.linear.x = -self.forward_speed  # back off
                self.cmd_pub.publish(t)
                return
            else:
                self.yielding = False  # Done yielding

        # If a teammate is in front AND priority lower, start yielding
        if self.teammate_in_front():
            my_id = int(self.robot_id.split('_')[-1])
            # pick the smallest id to go, larger yields
            front_ids = [int(r.split('_')[-1]) for r,pos in self.teammate_poses.items()
                         if math.hypot(pos[0]-self.pose,pos[1]-self.pose[1])<self.min_obstacle_dist]
            lowest = min(front_ids+[my_id])
            if my_id != lowest:
                self.yielding = True
                self.yield_start = now
                t.linear.x = -self.forward_speed
                self.cmd_pub.publish(t)
                return

        # Obstacle avoidance
        if self.get_front_distance() < self.min_obstacle_dist:
            t.linear.x = 0.0
            t.angular.z = self.turn_speed
            self.cmd_pub.publish(t)
            return

        # Goal following
        if self.formation_goal:
            dx = self.formation_goal[0]-self.pose
            dy = self.formation_goal[1]-self.pose[1]
            dist = math.hypot(dx,dy)
            if dist < self.goal_tolerance:
                t.linear.x=0; t.angular.z=0
            else:
                ang = math.atan2(dy,dx)
                err = self.normalize_angle(ang - self.pose[2])
                # drive only if roughly facing
                if abs(err)<math.pi/4:
                    t.linear.x = min(self.forward_speed, 0.3*dist)
                else:
                    t.linear.x = 0.0
                t.angular.z = max(min(1.5*err,1.0),-1.0)
        else:
            # no goal: slow forward
            t.linear.x = 0.05

        self.cmd_pub.publish(t)

def main():
    rclpy.init()
    node=ExplorationAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    rclpy.shutdown()

if __name__=='__main__':
    main()
