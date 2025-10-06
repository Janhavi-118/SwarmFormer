#!/usr/bin/env python3
# fast_robot_spawner.py (UPDATED with random positions)

import rclpy
from rclpy.node import Node
import subprocess
import time
import threading
from std_msgs.msg import String
import os
import random
import math

class FastRobotSpawner(Node):
    def __init__(self):
        super().__init__('fast_robot_spawner')
        self.setup_robot_description()
        
    def generate_random_positions(self, num_robots, arena_size=8.0, spread_type='random', min_distance=1.0):
        """Generate collision-free random positions"""
        positions = []
        max_attempts = 100
        
        print(f"🎲 Generating {num_robots} random positions (spread: {spread_type}, arena: {arena_size})")
        
        for i in range(num_robots):
            attempts = 0
            while attempts < max_attempts:
                # Generate position based on spread type
                if spread_type == 'tight':
                    x = random.uniform(-2.0, 2.0) 
                    y = random.uniform(-2.0, 2.0)
                elif spread_type == 'spread':
                    x = random.uniform(-arena_size/2, arena_size/2)
                    y = random.uniform(-arena_size/2, arena_size/2)
                elif spread_type == 'circle':
                    # Spawn in circle formation with noise
                    angle = 2 * math.pi * i / num_robots
                    radius = arena_size/3 + random.uniform(-1, 1)
                    x = radius * math.cos(angle) + random.uniform(-0.5, 0.5)
                    y = radius * math.sin(angle) + random.uniform(-0.5, 0.5)
                else:  # 'random'
                    x = random.uniform(-arena_size, arena_size)
                    y = random.uniform(-arena_size, arena_size)
                
                yaw = random.uniform(0, 2 * math.pi)
                
                # Check collision with existing positions
                collision = False
                for existing_x, existing_y, _ in positions:
                    distance = math.sqrt((x - existing_x)**2 + (y - existing_y)**2)
                    if distance < min_distance:
                        collision = True
                        break
                
                if not collision:
                    positions.append((x, y, yaw))
                    print(f"  🤖 tb3_{i}: ({x:.1f}, {y:.1f}, {yaw:.2f})")
                    break
                    
                attempts += 1
            
            # Fallback if we can't find good position
            if attempts >= max_attempts:
                x, y, yaw = (i * 1.5, 0.0, random.uniform(0, 2*math.pi))
                positions.append((x, y, yaw))
                print(f"  ⚠️  tb3_{i}: FALLBACK position ({x:.1f}, {y:.1f})")
        
        return positions
    
    def setup_robot_description(self):
        """Setup robot description publisher for fast spawning"""
        from ament_index_python.packages import get_package_share_directory
        
        try:
            description_pkg = get_package_share_directory('turtlebot3_description')
            urdf_file = os.path.join(description_pkg, 'urdf', 'turtlebot3_burger.urdf')
            
            if os.path.exists(urdf_file):
                # Start robot state publisher
                cmd = [
                    'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                    '--ros-args', '-p', f'robot_description:={open(urdf_file).read()}'
                ]
                self.rsp_process = subprocess.Popen(cmd)
                time.sleep(3)  # Let it start properly
                print("✅ Robot description publisher started")
            else:
                print(f"❌ URDF file not found: {urdf_file}")
                
        except Exception as e:
            print(f"❌ Failed to setup robot description: {e}")
    
    def spawn_robots_random(self, num_robots, arena_size=8.0, spread_type='random'):
        """Spawn robots at random positions with fast parallel spawning"""
        
        # Generate random positions
        positions = self.generate_random_positions(num_robots, arena_size, spread_type)
        
        print(f"🚀 Fast spawning {num_robots} robots with random positions...")
        
        # Create spawn commands
        spawn_commands = []
        for i in range(num_robots):
            x, y, yaw = positions[i]
            robot_ns = f'tb3_{i}'
            
            cmd = [
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-topic', 'robot_description',  # Fast topic-based spawning
                '-entity', robot_ns,
                '-robot_namespace', robot_ns,
                '-x', str(round(x, 2)), '-y', str(round(y, 2)), '-z', '0.01', 
                '-Y', str(round(yaw, 3)),
                '-timeout', '20'
            ]
            spawn_commands.append((robot_ns, cmd))
        
        # Spawn robots with controlled parallelism
        batch_size = 2  # 2 robots at once for speed + reliability
        
        for i in range(0, len(spawn_commands), batch_size):
            batch = spawn_commands[i:i + batch_size]
            threads = []
            
            print(f"📦 Spawning batch {i//batch_size + 1}: {[cmd[0] for cmd in batch]}")
            
            # Start batch
            for robot_name, cmd in batch:
                thread = threading.Thread(target=self.spawn_single_robot, args=(robot_name, cmd))
                thread.start()
                threads.append(thread)
            
            # Wait for batch to complete
            for thread in threads:
                thread.join()
            
            print(f"✅ Batch {i//batch_size + 1} completed")
            time.sleep(2)  # Brief gap between batches
        
        print(f"🎉 All {num_robots} robots spawned at random positions!")
        
        # Verify spawning
        time.sleep(3)
        return self.verify_all_spawned(num_robots)
    
    def spawn_single_robot(self, robot_name, cmd):
        """Spawn a single robot (called in thread)"""
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
            if result.returncode == 0:
                print(f"✅ {robot_name} spawned successfully")
            else:
                print(f"❌ {robot_name} spawn failed: {result.stderr}")
        except subprocess.TimeoutExpired:
            print(f"⏰ {robot_name} spawn timeout")
        except Exception as e:
            print(f"❌ {robot_name} spawn error: {e}")
    
    def verify_all_spawned(self, expected_count):
        """Verify all robots spawned successfully"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=15)
            topics = result.stdout
            
            spawned_robots = []
            for i in range(expected_count):
                if f'/tb3_{i}/odom' in topics:
                    spawned_robots.append(f'tb3_{i}')
            
            print(f"📊 Verification: {len(spawned_robots)}/{expected_count} robots confirmed")
            print(f"   ✅ Spawned: {spawned_robots}")
            
            return len(spawned_robots) == expected_count
            
        except Exception as e:
            print(f"❌ Verification failed: {e}")
            return False

def main():
    rclpy.init()
    spawner = FastRobotSpawner()
    
    try:
        # Test different spawn strategies
        success = spawner.spawn_robots_random(
            num_robots=5, 
            arena_size=8.0, 
            spread_type='random'  # 'tight', 'spread', 'random', 'circle'
        )
        
        if success:
            print("🎉 SUCCESS: Fast random spawning works!")
        else:
            print("❌ FAILED: Some robots didn't spawn")
            
    except KeyboardInterrupt:
        print("Spawning interrupted")
    finally:
        spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
