#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import subprocess
import time
import json
import os
import signal
import psutil
from scenario_generator import ScenarioGenerator, ScenarioConfig
from typing import List, Dict, Optional

class DatasetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')
        
        # Parameters
        self.declare_parameter('scenarios_file', './training_data/scenarios.json')
        self.declare_parameter('output_dir', './training_data/episodes/')
        self.declare_parameter('parallel_episodes', 1)  # Start with 1, scale later
        
        self.scenarios_file = self.get_parameter('scenarios_file').value
        self.output_dir = self.get_parameter('output_dir').value
        self.parallel_episodes = self.get_parameter('parallel_episodes').value
        
        # Load scenarios
        self.scenario_gen = ScenarioGenerator()
        if os.path.exists(self.scenarios_file):
            self.scenario_gen.load_scenarios(self.scenarios_file)
        
        # Publishers for coordination
        self.data_start_pub = self.create_publisher(String, '/data_collection/start', 10)
        self.data_stop_pub = self.create_publisher(String, '/data_collection/stop', 10)
        
        # Goal publishers (will be created dynamically)
        self.goal_pubs = {}
        
        # Collection state
        self.current_scenario = None
        self.current_episode = 0
        self.running_processes = []
        self.collection_active = False
        self.episode_start_time = 0.0
        
        # Statistics
        self.completed_episodes = 0
        self.failed_episodes = 0
        self.total_episodes = sum(s.episodes_needed for s in self.scenario_gen.scenarios)
        
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info(f"DatasetCollector initialized")
        self.get_logger().info(f"Total scenarios: {len(self.scenario_gen.scenarios)}")
        self.get_logger().info(f"Total episodes to collect: {self.total_episodes}")
    
    def run_full_collection(self):
        """Run complete dataset collection"""
        self.get_logger().info("🚀 Starting full dataset collection...")
        start_time = time.time()
        
        try:
            for scenario_idx, scenario in enumerate(self.scenario_gen.scenarios):
                self.get_logger().info(f"\n📊 Scenario {scenario_idx + 1}/{len(self.scenario_gen.scenarios)}: {scenario.scenario_id}")
                self.get_logger().info(f"   Config: {scenario.swarm_size} robots, {scenario.formation_type}, {scenario.environment_type}")
                
                for episode in range(scenario.episodes_needed):
                    episode_success = self.run_single_episode(scenario, episode)
                    
                    if episode_success:
                        self.completed_episodes += 1
                    else:
                        self.failed_episodes += 1
                    
                    # Progress update
                    if (self.completed_episodes + self.failed_episodes) % 10 == 0:
                        self.print_progress()
                    
                    # Brief pause between episodes
                    time.sleep(2)
        
        except KeyboardInterrupt:
            self.get_logger().warn("Collection interrupted by user")
        finally:
            self.cleanup_processes()
            
        total_time = time.time() - start_time
        self.get_logger().info(f"\n✅ Collection completed!")
        self.get_logger().info(f"   Completed: {self.completed_episodes}")
        self.get_logger().info(f"   Failed: {self.failed_episodes}")
        self.get_logger().info(f"   Total time: {total_time/3600:.2f} hours")
    
    def run_single_episode(self, scenario: ScenarioConfig, episode_num: int) -> bool:
        """Run a single data collection episode"""
        episode_id = f"{scenario.scenario_id}_ep_{episode_num:03d}"
        
        try:
            # 1. Setup Gazebo environment
            if not self.setup_gazebo_scenario(scenario):
                self.get_logger().error(f"Failed to setup Gazebo for {episode_id}")
                return False
            
            # 2. Launch robot nodes (gossip + expert controllers)
            if not self.launch_robot_nodes(scenario):
                self.get_logger().error(f"Failed to launch robot nodes for {episode_id}")
                return False
            
            # 3. Send formation goals
            self.send_formation_goals(scenario, episode_id)
            
            # 4. Run episode
            success = self.run_episode_data_collection(scenario, episode_id)
            
            # 5. Cleanup
            self.cleanup_current_episode()
            
            return success
            
        except Exception as e:
            self.get_logger().error(f"Episode {episode_id} failed: {e}")
            self.cleanup_current_episode()
            return False
    
    def setup_gazebo_scenario(self, scenario: ScenarioConfig) -> bool:
        """Launch Gazebo with appropriate world"""
        try:
            # Kill any existing Gazebo
            self.kill_gazebo()
            time.sleep(2)
            
            # Launch Gazebo with world
            launch_cmd = [
                'ros2', 'launch', 'turtlebot3_gazebo', scenario.world_file
            ]
            
            gazebo_process = subprocess.Popen(launch_cmd)
            self.running_processes.append(gazebo_process)
            
            # Wait for Gazebo to start
            time.sleep(10)
            
            # Spawn robots at specified positions
            for i, (x, y, yaw) in enumerate(scenario.spawn_positions):
                robot_name = f"tb3_{i}"
                self.spawn_robot(robot_name, x, y, yaw)
            
            time.sleep(5)  # Wait for spawning
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup Gazebo: {e}")
            return False
    
    def spawn_robot(self, robot_name: str, x: float, y: float, yaw: float):
        """Spawn a single robot in Gazebo"""
        spawn_cmd = [
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', str(x), '-y', str(y), '-z', '0.01',
            '-Y', str(yaw)
        ]
        
        try:
            subprocess.run(spawn_cmd, check=True, timeout=30)
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as e:
            self.get_logger().error(f"Failed to spawn {robot_name}: {e}")
    
    def launch_robot_nodes(self, scenario: ScenarioConfig) -> bool:
        """Launch gossip managers and expert controllers for all robots"""
        try:
            for i in range(scenario.swarm_size):
                robot_id = f"tb3_{i}"
                
                # Launch gossip manager
                gossip_cmd = [
                    'python3', 'gossip_manager.py',
                    '--ros-args', '-p', f'robot_id:={robot_id}',
                    '-p', 'swarm_size:=' + str(scenario.swarm_size)
                ]
                gossip_process = subprocess.Popen(gossip_cmd)
                self.running_processes.append(gossip_process)
                
                # Launch expert controller
                expert_cmd = [
                    'python3', 'expert_controller.py',
                    '--ros-args', '-p', f'robot_id:={robot_id}',
                    '-p', f'data_output_dir:={self.output_dir}'
                ]
                expert_process = subprocess.Popen(expert_cmd)
                self.running_processes.append(expert_process)
                
                time.sleep(1)  # Stagger launches
            
            # Wait for nodes to initialize
            time.sleep(8)
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to launch robot nodes: {e}")
            return False
    
    def send_formation_goals(self, scenario: ScenarioConfig, episode_id: str):
        """Send formation goals to all robots and start data collection"""
        
        # Create goal publishers if needed
        for i in range(scenario.swarm_size):
            robot_id = f"tb3_{i}"
            topic_name = f"/formation/goal_{robot_id}"
            if topic_name not in self.goal_pubs:
                self.goal_pubs[topic_name] = self.create_publisher(Point, topic_name, 10)
        
        time.sleep(1)  # Let publishers initialize
        
        # Start data collection
        start_msg = String()
        start_msg.data = json.dumps({
            'episode_id': episode_id,
            'scenario_config': scenario.__dict__
        })
        self.data_start_pub.publish(start_msg)
        
        time.sleep(0.5)
        
        # Send formation goals
        for i, (goal_x, goal_y) in enumerate(scenario.formation_goals):
            robot_id = f"tb3_{i}"
            topic_name = f"/formation/goal_{robot_id}"
            
            goal_msg = Point()
            goal_msg.x = float(goal_x)
            goal_msg.y = float(goal_y)
            goal_msg.z = 0.0
            
            self.goal_pubs[topic_name].publish(goal_msg)
        
        self.get_logger().info(f"Formation goals sent for {episode_id}")
    
    def run_episode_data_collection(self, scenario: ScenarioConfig, episode_id: str) -> bool:
        """Run episode and collect data"""
        episode_timeout = 60.0  # 60 seconds max per episode
        self.episode_start_time = time.time()
        
        self.get_logger().info(f"Running episode {episode_id} (timeout: {episode_timeout}s)")
        
        # Monitor episode progress
        while time.time() - self.episode_start_time < episode_timeout:
            time.sleep(1)
            
            # Could add early termination logic here:
            # - Check if formation achieved
            # - Check if robots are stuck
            # For now, just run full timeout
        
        # Stop data collection
        stop_msg = String()
        stop_msg.data = episode_id
        self.data_stop_pub.publish(stop_msg)
        
        time.sleep(2)  # Let data be saved
        
        return True  # For now, consider all episodes successful
    
    def cleanup_current_episode(self):
        """Clean up current episode"""
        # Kill all running processes
        for process in self.running_processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except (subprocess.TimeoutExpired, psutil.NoSuchProcess):
                try:
                    process.kill()
                except psutil.NoSuchProcess:
                    pass
        
        self.running_processes.clear()
        
        # Kill Gazebo
        self.kill_gazebo()
        time.sleep(2)
    
    def kill_gazebo(self):
        """Kill all Gazebo processes"""
        try:
            subprocess.run(['pkill', '-f', 'gazebo'], check=False)
            subprocess.run(['pkill', '-f', 'gzserver'], check=False)
            subprocess.run(['pkill', '-f', 'gzclient'], check=False)
        except Exception:
            pass
    
    def cleanup_processes(self):
        """Cleanup all processes"""
        self.cleanup_current_episode()
        
        # Kill any remaining ROS nodes
        try:
            subprocess.run(['pkill', '-f', 'python3.*expert_controller'], check=False)
            subprocess.run(['pkill', '-f', 'python3.*gossip_manager'], check=False)
        except Exception:
            pass
    
    def print_progress(self):
        """Print collection progress"""
        total_collected = self.completed_episodes + self.failed_episodes
        progress_pct = (total_collected / self.total_episodes) * 100
        
        self.get_logger().info(f"📈 Progress: {total_collected}/{self.total_episodes} ({progress_pct:.1f}%)")
        self.get_logger().info(f"   ✅ Success: {self.completed_episodes}")
        self.get_logger().info(f"   ❌ Failed: {self.failed_episodes}")

def main():
    rclpy.init()
    collector = DatasetCollector()
    
    try:
        collector.run_full_collection()
    except KeyboardInterrupt:
        collector.get_logger().info("Collection stopped by user")
    finally:
        collector.cleanup_processes()
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
