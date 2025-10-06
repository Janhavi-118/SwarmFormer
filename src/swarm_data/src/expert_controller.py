#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from swarm_msgs.msg import Gossip, RobotState as RobotStateMsg
import numpy as np
import math
import time
import json
import os
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional

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

class ExpertFormationController(Node):
    def __init__(self):
        super().__init__('expert_controller')
        
        # Parameters
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('data_output_dir', './training_data/')
        self.declare_parameter('episode_timeout', 60.0)  # 60 seconds per episode
        
        self.robot_id = self.get_parameter('robot_id').value
        self.data_output_dir = self.get_parameter('data_output_dir').value
        self.episode_timeout = self.get_parameter('episode_timeout').value
        
        self.episode_timeout = 60.0
        # Expert behavior parameters (tuned for good demonstrations)
        self.max_linear_speed = 0.2
        self.max_angular_speed = 1.0
        self.goal_attraction_gain = 0.4
        self.obstacle_repulsion_gain = 2.0
        self.neighbor_avoidance_gain = 1.5
        self.formation_maintenance_gain = 0.3
        self.safety_distance = 0.5
        self.obstacle_safety_distance = 0.5
        self.goal_tolerance = 0.25
        
        # State variables
        self.pose = np.zeros(3)  # [x, y, theta]
        self.pose_initialized = False
        self.velocity = np.zeros(3)  # [vx, vy, w]
        
        # Swarm state (from gossip)
        self.neighbor_states = {}
        self.failed_robots = set()
        
        # Sensor data
        self.lidar_ranges = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None
        self.local_obstacles = []
        
        # Goals and formation
        self.formation_goal = None
        self.formation_type = "unknown"
        self.paused = False
        
        # Data collection
        self.collecting_data = False
        self.episode_data = {
            'episode_id': '',
            'robot_id': self.robot_id,
            'scenario_config': {},
            'timesteps': [],
            'start_time': 0.0,
            'success_metrics': {}
        }
        
        # ROS interfaces
        self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, f'/{self.robot_id}/scan', self.lidar_callback, 10)
        self.create_subscription(Gossip, '/swarm/gossip', self.gossip_callback, 10)
        self.create_subscription(Point, f'/formation/goal_{self.robot_id}', self.goal_callback, 10)
        self.create_subscription(Bool, '/autopilot_pause', self.pause_callback, 10)
        self.create_subscription(String, '/data_collection/start', self.start_collection_callback, 10)
        self.create_subscription(String, '/data_collection/stop', self.stop_collection_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.debug_pub = self.create_publisher(String, f'/{self.robot_id}/expert_debug', 10)
        
        # Control and data collection timers
        self.create_timer(0.1, self.expert_control_loop)  # 10Hz control
        self.create_timer(0.1, self.data_logging_loop)    # 10Hz data logging
        
        self.get_logger().info(f"ExpertFormationController started for {self.robot_id}")
    
    def odom_callback(self, msg):
        try:
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            vel = msg.twist.twist
            
            # Convert quaternion to yaw
            yaw = math.atan2(
                2.0 * (orient.w * orient.z + orient.x * orient.y),
                1.0 - 2.0 * (orient.y**2 + orient.z**2)
            )
            
            self.pose = np.array([pos.x, pos.y, yaw])
            self.velocity = np.array([vel.linear.x, vel.linear.y, vel.angular.z])
            self.pose_initialized = True
            
        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {e}")
    
    def lidar_callback(self, msg):
        try:
            if len(msg.ranges) > 0:
                self.lidar_ranges = np.array(msg.ranges)
                self.lidar_angle_min = msg.angle_min
                self.lidar_angle_increment = msg.angle_increment
                self.update_local_obstacles()
        except Exception as e:
            self.get_logger().error(f"Error in lidar_callback: {e}")
    
    def gossip_callback(self, msg):
        try:
            sender = msg.sender
            if sender == self.robot_id:
                return
            
            # Update neighbor state
            rs = msg.robot_state
            state = RobotState(
                robot_id=rs.robot_id,
                timestamp=rs.timestamp.sec + rs.timestamp.nanosec * 1e-9,
                position=list(rs.position),
                velocity=list(rs.velocity),
                neighbors=list(rs.neighbors),
                local_obstacle_count=rs.local_obstacle_count,
                battery_level=rs.battery_level,
                task_status=rs.task_status
            )
            self.neighbor_states[sender] = state
            
            # Update failed robots list
            for failed_id in msg.failed_robots:
                if failed_id != self.robot_id:
                    self.failed_robots.add(failed_id)
                    
        except Exception as e:
            self.get_logger().error(f"Error in gossip_callback: {e}")
    
    def goal_callback(self, msg):
        try:
            if math.isfinite(msg.x) and math.isfinite(msg.y):
                self.formation_goal = np.array([msg.x, msg.y])
        except Exception as e:
            self.get_logger().error(f"Error in goal_callback: {e}")
    
    def pause_callback(self, msg):
        self.paused = msg.data
        if self.paused:
            self.cmd_pub.publish(Twist())
    
    def start_collection_callback(self, msg):
        try:
            config_data = json.loads(msg.data)
            self.start_data_collection(config_data)
        except Exception as e:
            self.get_logger().error(f"Error starting data collection: {e}")
    
    def stop_collection_callback(self, msg):
        self.stop_data_collection()
    
    def update_local_obstacles(self):
        """Extract obstacle positions from LiDAR data"""
        if self.lidar_ranges is None or not self.pose_initialized:
            return
        
        self.local_obstacles = []
        angles = self.lidar_angle_min + np.arange(len(self.lidar_ranges)) * self.lidar_angle_increment
        
        for i, (distance, angle) in enumerate(zip(self.lidar_ranges, angles)):
            if np.isfinite(distance) and 0.1 < distance < 3.0:  # Valid obstacle
                # Convert to global coordinates
                global_angle = self.pose[2] + angle
                obstacle_x = self.pose[0] + distance * math.cos(global_angle)
                obstacle_y = self.pose[1] + distance * math.sin(global_angle)
                self.local_obstacles.append([obstacle_x, obstacle_y, 0.1])  # [x, y, size]
    
    def expert_control_loop(self):
        """Main expert control logic - 4 layer behavioral system"""
        if self.paused or not self.pose_initialized:
            return
        
        # Get expert action using 4-layer approach
        action = self.compute_expert_action()
        
        # Convert to ROS Twist message
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        
        # Publish command
        self.cmd_pub.publish(cmd)
        
        # Debug info
        debug_msg = String()
        debug_msg.data = json.dumps({
            'action': action.tolist(),
            'goal_distance': float(np.linalg.norm(self.formation_goal - self.pose[:2])) if self.formation_goal is not None else 0.0,
            'neighbor_count': len(self.neighbor_states),
            'obstacle_count': len(self.local_obstacles)
        })
        self.debug_pub.publish(debug_msg)
    
    def compute_expert_action(self):
        """4-Layer Expert Decision Making"""
        if self.formation_goal is None:
            return np.array([0.05, 0.0])  # Default slow forward
        
        # Layer 1: Goal Attraction
        goal_force = self.compute_goal_attraction()
        
        # Layer 2: Obstacle Avoidance  
        obstacle_force = self.compute_obstacle_avoidance()
        
        # Layer 3: Neighbor Avoidance
        neighbor_force = self.compute_neighbor_avoidance()
        
        # Layer 4: Formation Maintenance
        formation_force = self.compute_formation_maintenance()
        
        # Combine forces
        total_force = (goal_force + obstacle_force + neighbor_force + formation_force)
        
        # Convert to action [linear_velocity, angular_velocity]
        desired_heading = math.atan2(total_force[1], total_force[0])
        heading_error = self.normalize_angle(desired_heading - self.pose[2])
        
        # Calculate velocities
        force_magnitude = np.linalg.norm(total_force)
        linear_vel = min(self.max_linear_speed, force_magnitude)
        angular_vel = heading_error * 2.0
        
        # Apply constraints and smoothing
        if abs(heading_error) > math.pi/3:  # If facing wrong direction, turn in place
            linear_vel *= 0.3
        
        # Clamp values
        linear_vel = max(0.0, min(linear_vel, self.max_linear_speed))
        angular_vel = max(-self.max_angular_speed, min(angular_vel, self.max_angular_speed))
        
        return np.array([linear_vel, angular_vel])
    
    def compute_goal_attraction(self):
        """Layer 1: Attract to formation goal"""
        if self.formation_goal is None:
            return np.zeros(2)
        
        goal_vector = self.formation_goal - self.pose[:2]
        goal_distance = np.linalg.norm(goal_vector)
        
        if goal_distance < self.goal_tolerance:
            return np.zeros(2)  # At goal, no attraction
        
        goal_direction = goal_vector / goal_distance
        attraction_strength = min(goal_distance, 2.0) * self.goal_attraction_gain
        
        return goal_direction * attraction_strength
    
    def compute_obstacle_avoidance(self):
        """Layer 2: Repel from static obstacles"""
        if not self.local_obstacles:
            return np.zeros(2)
        
        total_repulsion = np.zeros(2)
        
        for obstacle in self.local_obstacles:
            obs_pos = np.array(obstacle[:2])
            to_obstacle = obs_pos - self.pose[:2]
            distance = np.linalg.norm(to_obstacle)
            
            if distance < self.obstacle_safety_distance and distance > 0.01:
                # Repulsion inversely proportional to distance
                repulsion_strength = self.obstacle_repulsion_gain / (distance + 0.1)
                repulsion_direction = -to_obstacle / distance
                total_repulsion += repulsion_direction * repulsion_strength
        
        return total_repulsion
    
    def compute_neighbor_avoidance(self):
        """Layer 3: Avoid other robots"""
        if not self.neighbor_states:
            return np.zeros(2)
        
        total_repulsion = np.zeros(2)
        
        for neighbor_id, neighbor_state in self.neighbor_states.items():
            if neighbor_id in self.failed_robots:
                continue
            
            neighbor_pos = np.array(neighbor_state.position[:2])
            to_neighbor = neighbor_pos - self.pose[:2]
            distance = np.linalg.norm(to_neighbor)
            
            if distance < self.safety_distance and distance > 0.01:
                # Priority-based avoidance
                my_priority = int(self.robot_id.split('_')[-1])
                neighbor_priority = int(neighbor_id.split('_')[-1])
                
                if my_priority > neighbor_priority:  # I yield to higher priority (lower number)
                    repulsion_strength = self.neighbor_avoidance_gain / (distance + 0.1)
                    repulsion_direction = -to_neighbor / distance
                    total_repulsion += repulsion_direction * repulsion_strength
        
        return total_repulsion
    
    def compute_formation_maintenance(self):
        """Layer 4: Maintain formation structure"""
        if not self.neighbor_states or self.formation_goal is None:
            return np.zeros(2)
        
        # Simple formation maintenance - stay roughly aligned with neighbors heading to their goals
        alignment_force = np.zeros(2)
        count = 0
        
        for neighbor_id, neighbor_state in self.neighbor_states.items():
            if neighbor_id in self.failed_robots:
                continue
            
            neighbor_pos = np.array(neighbor_state.position[:2])
            neighbor_vel = np.array(neighbor_state.velocity[:2])
            distance = np.linalg.norm(neighbor_pos - self.pose[:2])
            
            if 0.5 < distance < 2.0:  # Neighbors in formation range
                # Align with neighbor's movement direction
                if np.linalg.norm(neighbor_vel) > 0.01:
                    alignment_force += neighbor_vel[:2] / np.linalg.norm(neighbor_vel)
                    count += 1
        
        if count > 0:
            alignment_force = alignment_force / count * self.formation_maintenance_gain
        
        return alignment_force
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    # Data collection methods
    def start_data_collection(self, scenario_config):
        """Start collecting training data for current episode"""
        self.collecting_data = True
        episode_id = f"{scenario_config.get('formation_type', 'unknown')}_{scenario_config.get('swarm_size', 0)}_{int(time.time())}"
        
        self.episode_data = {
            'episode_id': episode_id,
            'robot_id': self.robot_id,
            'scenario_config': scenario_config,
            'timesteps': [],
            'start_time': time.time(),
            'success_metrics': {}
        }
        
        self.get_logger().info(f"Started data collection for episode: {episode_id}")
    
    def stop_data_collection(self):
        """Stop data collection and save episode"""
        if not self.collecting_data:
            return
        
        self.collecting_data = False
        
        # Calculate success metrics
        self.episode_data['success_metrics'] = self.calculate_success_metrics()
        self.episode_data['end_time'] = time.time()
        self.episode_data['duration'] = time.time() - self.episode_data['start_time']
        
        # Save episode data
        self.save_episode_data()
        
        self.get_logger().info(f"Stopped data collection for episode: {self.episode_data['episode_id']}")
    
    def data_logging_loop(self):
        """Log current timestep data (called at 10Hz)"""
        if not self.collecting_data or not self.pose_initialized:
            return
        
        timestep_data = {
            'timestamp': time.time() - self.episode_data['start_time'],
            'robot_state': {
                'position': self.pose.tolist(),
                'velocity': self.velocity.tolist(),
                'goal': self.formation_goal.tolist() if self.formation_goal is not None else None
            },
            'neighbors': {
                neighbor_id: {
                    'position': state.position,
                    'velocity': state.velocity
                } for neighbor_id, state in self.neighbor_states.items()
            },
            'obstacles': self.local_obstacles.copy(),
            'expert_action': self.get_last_action(),
            'formation_metrics': self.calculate_instant_metrics()
        }
        
        self.episode_data['timesteps'].append(timestep_data)
    
    def get_last_action(self):
        """Get the most recent action (for logging purposes)"""
        # This would store the last computed action
        # For now, recompute (slight inefficiency but cleaner)
        action = self.compute_expert_action()
        return action.tolist()
    
    def calculate_instant_metrics(self):
        """Calculate instantaneous performance metrics"""
        metrics = {}
        
        if self.formation_goal is not None:
            goal_distance = np.linalg.norm(self.formation_goal - self.pose[:2])
            metrics['goal_distance'] = float(goal_distance)
            metrics['at_goal'] = goal_distance < self.goal_tolerance
        
        # Collision risk (minimum distance to neighbors)
        min_neighbor_distance = float('inf')
        for neighbor_state in self.neighbor_states.values():
            neighbor_pos = np.array(neighbor_state.position[:2])
            distance = np.linalg.norm(neighbor_pos - self.pose[:2])
            min_neighbor_distance = min(min_neighbor_distance, distance)
        
        metrics['min_neighbor_distance'] = float(min_neighbor_distance) if min_neighbor_distance != float('inf') else None
        metrics['collision_risk'] = min_neighbor_distance < 0.4 if min_neighbor_distance != float('inf') else False
        
        return metrics
    
    def calculate_success_metrics(self):
        """Calculate overall episode success metrics"""
        if not self.episode_data['timesteps']:
            return {}
        
        # Formation achievement
        goal_distances = [ts['formation_metrics'].get('goal_distance', float('inf')) 
                         for ts in self.episode_data['timesteps'] 
                         if ts['formation_metrics'].get('goal_distance') is not None]
        
        formation_achieved = any(d < self.goal_tolerance for d in goal_distances)
        avg_goal_distance = np.mean(goal_distances) if goal_distances else float('inf')
        
        # Collision detection
        collision_risks = [ts['formation_metrics'].get('collision_risk', False) 
                          for ts in self.episode_data['timesteps']]
        collision_occurred = any(collision_risks)
        
        # Time to goal
        time_to_goal = None
        for ts in self.episode_data['timesteps']:
            if ts['formation_metrics'].get('at_goal', False):
                time_to_goal = ts['timestamp']
                break
        
        return {
            'formation_achieved': formation_achieved,
            'collision_occurred': collision_occurred,
            'time_to_goal': time_to_goal,
            'average_goal_distance': float(avg_goal_distance),
            'episode_length': len(self.episode_data['timesteps'])
        }
    
    def save_episode_data(self):
        """Save episode data to JSON file"""
        try:
            os.makedirs(self.data_output_dir, exist_ok=True)
            filename = os.path.join(self.data_output_dir, f"{self.episode_data['episode_id']}.json")
            
            with open(filename, 'w') as f:
                json.dump(self.episode_data, f, indent=2)
            
            self.get_logger().info(f"Saved episode data to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save episode data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ExpertFormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_data_collection()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
