#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from swarm_msgs.msg import Gossip, RobotState as RobotStateMsg, GMMParams
from builtin_interfaces.msg import Time

import math
import time
import numpy as np
from sklearn.mixture import GaussianMixture
from dataclasses import dataclass
from typing import List, Dict


@dataclass
class RobotState:
    robot_id: str
    timestamp: float
    position: List[float]  # [x, y, theta]
    velocity: List[float]  # [vx, vy, w]
    neighbors: List[str]
    local_obstacle_count: int
    battery_level: float
    task_status: str
    error_flags: List[str]
    heartbeat_count: int


class GossipManager(Node):
    def __init__(self):
        super().__init__('gossip_manager')

        # Declare parameters and get robot_id
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value

        self.declare_parameter('max_neighbors', 5)
        self.declare_parameter('heartbeat_interval', 1.0)   # seconds
        self.declare_parameter('failure_timeout', 3.0)      # seconds
        self.declare_parameter('gossip_interval', 2.0)      # seconds
        self.declare_parameter('gmm_components', 5)         # max GMM clusters

        self.max_neighbors = self.get_parameter('max_neighbors').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.failure_timeout = self.get_parameter('failure_timeout').value
        self.gossip_interval = self.get_parameter('gossip_interval').value
        self.gmm_components = self.get_parameter('gmm_components').value
        
        # Initialize current robot state
        self.current_state = RobotState(
            robot_id=self.robot_id,
            timestamp=time.time(),
            position=[0.0, 0.0, 0.0],
            velocity=[0.0, 0.0, 0.0],
            neighbors=[],
            local_obstacle_count=0,
            battery_level=100.0,
            task_status="exploring",
            error_flags=[],
            heartbeat_count=0
        )

        # Neighbors' states and failure tracking
        self.neighbor_states: Dict[str, RobotState] = {}
        self.failed_robots = set()
        self.suspected_failures: Dict[str, float] = {}

        # ROS2 pub/sub
        self.gossip_pub = self.create_publisher(Gossip, '/swarm/gossip', 10)

        self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, f'/{self.robot_id}/scan', self.scan_callback, 10)
        self.create_subscription(Gossip, '/swarm/gossip', self.gossip_callback, 10)

        # Timers
        self.create_timer(self.heartbeat_interval, self.publish_heartbeat)
        self.create_timer(self.gossip_interval, self.publish_gossip)
        #self.create_timer(1.0, self.check_for_failures)

        self.get_logger().info(f"GossipManager started for robot_id: {self.robot_id}")

    def to_ros_time(self, ts: float) -> Time:
        sec = int(ts)
        nanosec = int((ts - sec) * 1e9)
        ros_time = Time(sec=sec, nanosec=nanosec)
        return ros_time

    def robot_state_to_msg(self, state: RobotState) -> RobotStateMsg:
        msg = RobotStateMsg()
        msg.robot_id = state.robot_id
        msg.timestamp = self.to_ros_time(state.timestamp)
        msg.position = state.position
        msg.velocity = state.velocity
        msg.neighbors = state.neighbors
        msg.local_obstacle_count = state.local_obstacle_count
        msg.battery_level = state.battery_level
        msg.task_status = state.task_status
        msg.error_flags = state.error_flags
        msg.heartbeat_count = state.heartbeat_count
        return msg

    def gmm_params_to_msg(self, gmm_params) -> GMMParams:
        msg = GMMParams()
        if gmm_params is not None:
            # Flatten the list of means from [[x1, y1], [x2, y2], ...] to [x1, y1, x2, y2, ...]
            msg.means = [float(item) for sublist in gmm_params['means'] for item in sublist]

            # Flatten covariances similarly if required by GMMParams (check the message structure)
            msg.covariances = []
            for cov in gmm_params['covs']:
                msg.covariances.extend([float(val) for row in cov for val in row])

            # Weights is likely already a flat list, but ensure float values
            msg.weights = [float(w) for w in gmm_params['weights']]
        return msg
    
    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        vel = msg.twist.twist

        # Yaw from quaternion
        yaw = math.atan2(
            2.0 * (orient.w * orient.z + orient.x * orient.y),
            1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        )

        self.current_state.position = [pos.x, pos.y, yaw]
        self.current_state.velocity = [vel.linear.x, vel.linear.y, vel.angular.z]
        self.current_state.timestamp = time.time()

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        self.current_state.local_obstacle_count = int((valid < 2.0).sum())

    def publish_heartbeat(self):
        self.current_state.heartbeat_count += 1
        self.current_state.timestamp = time.time()
        # Heartbeat info can be embedded or published separately if needed

    def update_neighbor_list(self):
        mypos = np.array(self.current_state.position[:2])
        distances = []
        for nid, st in self.neighbor_states.items():
            if nid in self.failed_robots:
                continue
            pos = np.array(st.position[:2])
            distances.append((nid, np.linalg.norm(mypos - pos)))
        distances.sort(key=lambda x: x[1])
        self.current_state.neighbors = [nid for nid, _ in distances[:self.max_neighbors]]

    def compute_gmm(self):
        positions = [s.position[:2] for s in self.neighbor_states.values()
                     if s.robot_id not in self.failed_robots]
        if len(positions) < 2:
            return None
        X = np.array(positions)
        n_components = min(self.gmm_components, len(X))
        try:
            gmm = GaussianMixture(n_components=n_components, covariance_type='full').fit(X)
            means = gmm.means_.tolist()
            covs = [cov.tolist() for cov in gmm.covariances_]
            weights = gmm.weights_.tolist()
            return {'means': means, 'covs': covs, 'weights': weights}
        except Exception as e:
            self.get_logger().warn(f"GMM fitting failed: {e}")
            return None

    def publish_gossip(self):
        self.update_neighbor_list()
        gmm_params = self.compute_gmm()

        gossip_msg = Gossip()
        gossip_msg.sender = self.robot_id
        gossip_msg.timestamp = self.to_ros_time(time.time())
        gossip_msg.robot_state = self.robot_state_to_msg(self.current_state)
        gossip_msg.known_neighbors = [self.robot_state_to_msg(s) for s in self.neighbor_states.values()]
        gossip_msg.failed_robots = list(self.failed_robots)
        gossip_msg.gmm_params = self.gmm_params_to_msg(gmm_params)
        gossip_msg.message_type = "gossip_update"

        self.gossip_pub.publish(gossip_msg)

    def gossip_callback(self, msg: Gossip):
        sender = msg.sender
        if sender == self.robot_id:
            return  # Ignore own messages

        try:
            rs = msg.robot_state
            state = RobotState(
                robot_id=rs.robot_id,
                timestamp=rs.timestamp.sec + rs.timestamp.nanosec * 1e-9,
                position=rs.position,
                velocity=rs.velocity,
                neighbors=rs.neighbors,
                local_obstacle_count=rs.local_obstacle_count,
                battery_level=rs.battery_level,
                task_status=rs.task_status,
                error_flags=rs.error_flags,
                heartbeat_count=rs.heartbeat_count
            )
            self.neighbor_states[sender] = state
            self.suspected_failures.pop(sender, None)
            self.failed_robots.discard(sender)

            # Incorporate known neighbors (optional)
            for kn in msg.known_neighbors:
                if kn.robot_id not in self.neighbor_states:
                    kn_state = RobotState(
                        robot_id=kn.robot_id,
                        timestamp=kn.timestamp.sec + kn.timestamp.nanosec * 1e-9,
                        position=kn.position,
                        velocity=kn.velocity,
                        neighbors=kn.neighbors,
                        local_obstacle_count=kn.local_obstacle_count,
                        battery_level=kn.battery_level,
                        task_status=kn.task_status,
                        error_flags=kn.error_flags,
                        heartbeat_count=kn.heartbeat_count
                    )
                    self.neighbor_states[kn.robot_id] = kn_state

            for failed in msg.failed_robots:
                if failed != self.robot_id:
                    self.failed_robots.add(failed)

        except Exception as e:
            self.get_logger().error(f"Error processing gossip message: {e}")

    def check_for_failures(self):
        now = time.time()
        to_remove = []
        for nid, st in list(self.neighbor_states.items()):
            if now - st.timestamp > self.failure_timeout:
                if nid not in self.suspected_failures:
                    self.get_logger().warn(f"Suspecting failure of {nid}")
                    self.suspected_failures[nid] = now
                else:
                    if now - self.suspected_failures[nid] > self.failure_timeout:
                        self.get_logger().error(f"Confirmed failure of {nid}")
                        self.failed_robots.add(nid)
                        to_remove.append(nid)
                        self.suspected_failures.pop(nid, None)
        for nid in to_remove:
            self.neighbor_states.pop(nid, None)


def main(args=None):
    rclpy.init(args=args)
    node = GossipManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
