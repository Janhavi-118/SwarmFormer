import sqlite3
import numpy as np
import os
import subprocess
import tempfile
import shutil
import pickle
import json
from pathlib import Path

import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist

# Parameters
SCAN_STALE_THRESHOLD_NS = int(200e6)  # 200 ms in nanoseconds
GOSSIP_STALE_THRESHOLD_NS = int(500e6)  # 500 ms for neighbor data

def decompress_zstd_file(zstd_file_path):
    """Decompress a .zstd file and return path to decompressed file."""
    temp_dir = tempfile.mkdtemp()
    output_file = os.path.join(temp_dir, os.path.basename(zstd_file_path).replace('.zstd', ''))
    
    try:
        subprocess.run(['zstd', '-d', zstd_file_path, '-o', output_file], 
                      check=True, capture_output=True)
        return output_file, temp_dir
    except subprocess.CalledProcessError as e:
        print(f"Error decompressing {zstd_file_path}: {e}")
        shutil.rmtree(temp_dir)
        return None, None
    except FileNotFoundError:
        print("zstd command not found. Please install zstd: sudo apt-get install zstd")
        shutil.rmtree(temp_dir)
        return None, None

def find_episode_files(root_directory):
    """Find all episode .db3.zstd files in the directory structure."""
    episode_files = []
    root_path = Path(root_directory)
    
    for episode_folder in root_path.iterdir():
        if episode_folder.is_dir():
            for file_path in episode_folder.iterdir():
                if file_path.suffix == '.zstd' and '.db3' in file_path.name:
                    episode_files.append({
                        'episode_folder': episode_folder.name,
                        'file_path': str(file_path),
                        'episode_name': file_path.stem.replace('.db3', '')
                    })
    
    return sorted(episode_files, key=lambda x: x['episode_name'])

def get_message_type_map():
    """Map topic types to ROS message classes."""
    return {
        'nav_msgs/msg/Odometry': Odometry,
        'sensor_msgs/msg/LaserScan': LaserScan,
        'geometry_msgs/msg/Twist': Twist,
    }

def load_topic_stream(db_path, topic_name):
    """Load timestamps and deserialized data for a given topic."""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    cursor.execute("SELECT id, type FROM topics WHERE name = ?", (topic_name,))
    row = cursor.fetchone()
    if not row:
        conn.close()
        return np.array([]), [], None
    
    topic_id, topic_type = row[0], row[1]
    
    type_map = get_message_type_map()
    if topic_type not in type_map:
        print(f"Warning: Unknown message type {topic_type} for topic {topic_name}")
        conn.close()
        return np.array([]), [], topic_type
    
    message_class = type_map[topic_type]
    
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    )
    records = cursor.fetchall()
    conn.close()
    
    if not records:
        return np.array([]), [], topic_type
    
    timestamps = np.array([r[0] for r in records], dtype=np.int64)
    
    try:
        data = []
        for i, r in enumerate(records):
            try:
                msg = deserialize_message(r[1], message_class)
                data.append(msg)
            except Exception as e:
                print(f"Error deserializing message {i} for {topic_name}: {e}")
                continue
                
    except Exception as e:
        print(f"Error deserializing messages for {topic_name}: {e}")
        return np.array([]), [], topic_type
    
    return timestamps, data, topic_type

def get_all_robot_names(db_path):
    """Extract all robot names from topic names."""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT name FROM topics WHERE name LIKE '%/odom'")
    odom_topics = [row[0] for row in cursor.fetchall()]
    conn.close()
    
    robot_names = [topic.split('/')[1] for topic in odom_topics if len(topic.split('/')) >= 2]
    return robot_names

def forward_fill_with_threshold(target_ts, source_ts, source_data, threshold_ns):
    """Forward-fill source data onto target timeline with staleness threshold."""
    n = len(target_ts)
    filled = [None] * n
    mask = np.zeros(n, dtype=np.int8)
    
    if len(source_ts) == 0:
        return filled, mask
    
    source_idx = 0
    last_data = None
    last_data_ts = None
    
    for j, t in enumerate(target_ts):
        while source_idx < len(source_ts) and source_ts[source_idx] <= t:
            last_data = source_data[source_idx]
            last_data_ts = source_ts[source_idx]
            source_idx += 1
        
        if last_data is not None and last_data_ts is not None:
            if (t - last_data_ts) <= threshold_ns:
                filled[j] = last_data
                mask[j] = 1
            else:
                filled[j] = None
                mask[j] = 0
        else:
            filled[j] = None
            mask[j] = 0
    
    return filled, mask

def extract_pose_from_odom(odom_msg):
    """Extract position and orientation from Odometry message."""
    if odom_msg is None:
        return None
    
    # Handle case where odom_msg is a list containing one message
    if isinstance(odom_msg, list):
        if len(odom_msg) > 0:
            odom_msg = odom_msg[0]  # Extract the actual message from the list
        else:
            return None
    
    # Check if it's a proper Odometry message
    if not hasattr(odom_msg, 'pose'):
        print(f"Error: odom_msg is of type {type(odom_msg)}, expected Odometry message")
        return None
    
    try:
        pose = odom_msg.pose.pose
        return {
            'position': [pose.position.x, pose.position.y, pose.position.z],
            'orientation': [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            'timestamp': odom_msg.header.stamp.sec * 1e9 + odom_msg.header.stamp.nanosec
        }
    except Exception as e:
        print(f"Error extracting pose from odom message: {e}")
        return None

def reconstruct_gossip(target_ts, all_robot_data, current_robot):
    """Reconstruct gossip (neighbor states) for current robot at target timestamps."""
    neighbors = [robot for robot in all_robot_data.keys() if robot != current_robot]
    gossip_filled = []
    gossip_mask = np.zeros(len(target_ts), dtype=np.int8)
    
    for j, t in enumerate(target_ts):
        neighbor_states = {}
        has_any_neighbor = False
        
        for neighbor in neighbors:
            if neighbor in all_robot_data:
                neighbor_odom_ts = all_robot_data[neighbor]['odom_ts']
                neighbor_odom_data = all_robot_data[neighbor]['odom_data']
                
                neighbor_filled, neighbor_mask = forward_fill_with_threshold(
                    [t], neighbor_odom_ts, neighbor_odom_data, GOSSIP_STALE_THRESHOLD_NS
                )
                
                if neighbor_mask[0] == 1 and neighbor_filled is not None:
                    neighbor_pose = extract_pose_from_odom(neighbor_filled)
                    if neighbor_pose:
                        neighbor_states[neighbor] = neighbor_pose
                        has_any_neighbor = True
        
        gossip_filled.append(neighbor_states)
        gossip_mask[j] = 1 if has_any_neighbor else 0
    
    return gossip_filled, gossip_mask

def build_per_robot_dataset(db_path, robot_name, all_robot_data=None):
    """Build aligned streams for one robot with proper forward-filling and masking."""
    print(f"    Processing robot: {robot_name}")
    
    try:
        odom_ts, odom_data, _ = load_topic_stream(db_path, f"/{robot_name}/odom")
        if len(odom_ts) == 0:
            print(f"    No odom data found for {robot_name}")
            return None
        
        cmd_ts, cmd_data, _ = load_topic_stream(db_path, f"/{robot_name}/cmd_vel")
        if len(cmd_ts) == 0:
            print(f"    No cmd_vel data for {robot_name}, skipping this robot")
            return None
        
        first_cmd_ts = cmd_ts[0]
        valid_odom_mask = odom_ts >= first_cmd_ts
        odom_ts = odom_ts[valid_odom_mask]
        odom_data = [odom_data[i] for i, valid in enumerate(valid_odom_mask) if valid]
        
        if len(odom_ts) == 0:
            print(f"    No odom data after first cmd_vel for {robot_name}")
            return None
        
        cmd_filled, cmd_mask = forward_fill_with_threshold(
            odom_ts, cmd_ts, cmd_data, float('inf')
        )
        
        scan_ts, scan_data, _ = load_topic_stream(db_path, f"/{robot_name}/scan")
        scan_filled, scan_mask = forward_fill_with_threshold(
            odom_ts, scan_ts, scan_data, SCAN_STALE_THRESHOLD_NS
        )
        
        if all_robot_data is not None:
            gossip_filled, gossip_mask = reconstruct_gossip(odom_ts, all_robot_data, robot_name)
        else:
            gossip_filled = [{}] * len(odom_ts)
            gossip_mask = np.zeros(len(odom_ts), dtype=np.int8)
        
        print(f"      Found {len(odom_ts)} timesteps, scan coverage: {np.sum(scan_mask)}/{len(scan_mask)} ({np.sum(scan_mask)/len(scan_mask)*100:.1f}%)")
        
        return {
            'robot_name': robot_name,
            'odom_ts': odom_ts,
            'odom_data': odom_data,
            'cmd_vel_filled': cmd_filled,
            'cmd_vel_mask': cmd_mask,
            'scan_filled': scan_filled,
            'scan_mask': scan_mask,
            'gossip_filled': gossip_filled,
            'gossip_mask': gossip_mask,
        }
        
    except Exception as e:
        print(f"    Error processing robot {robot_name}: {e}")
        import traceback
        traceback.print_exc()
        return None

def build_episode_dataset(db_path, episode_name):
    """Build dataset for all robots in one episode."""
    print(f"  Processing episode: {episode_name}")
    
    try:
        robot_names = get_all_robot_names(db_path)
        print(f"    Found robots: {robot_names}")
        
        if len(robot_names) == 0:
            print(f"    No robots found in {episode_name}")
            return None
        
        all_robot_basic_data = {}
        for robot in robot_names:
            odom_ts, odom_data, _ = load_topic_stream(db_path, f"/{robot}/odom")
            if len(odom_ts) > 0:
                all_robot_basic_data[robot] = {
                    'odom_ts': odom_ts,
                    'odom_data': odom_data
                }
        
        complete_datasets = {}
        for robot in robot_names:
            dataset = build_per_robot_dataset(db_path, robot, all_robot_basic_data)
            if dataset is not None:
                complete_datasets[robot] = dataset
        
        return complete_datasets
        
    except Exception as e:
        print(f"  Error processing episode {episode_name}: {e}")
        import traceback
        traceback.print_exc()
        return None

def save_dataset(dataset, output_path, episode_name):
    """Save dataset to files."""
    os.makedirs(output_path, exist_ok=True)
    
    pickle_path = os.path.join(output_path, f"{episode_name}_dataset.pkl")
    with open(pickle_path, 'wb') as f:
        pickle.dump(dataset, f)
    
    metadata = {}
    for robot_name, robot_data in dataset.items():
        metadata[robot_name] = {
            'timesteps': len(robot_data['odom_ts']),
            'time_span_seconds': float((robot_data['odom_ts'][-1] - robot_data['odom_ts'][0]) / 1e9),
            'scan_coverage_percent': float(np.sum(robot_data['scan_mask']) / len(robot_data['scan_mask']) * 100),
            'gossip_coverage_percent': float(np.sum(robot_data['gossip_mask']) / len(robot_data['gossip_mask']) * 100)
        }
    
    json_path = os.path.join(output_path, f"{episode_name}_metadata.json")
    with open(json_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"    Saved dataset to {pickle_path}")

def process_all_episodes(root_directory, output_directory):
    """Process all episodes in the directory structure."""
    print("=== Processing All Episodes ===")
    
    try:
        rclpy.init()
    except:
        pass
    
    try:
        episode_files = find_episode_files(root_directory)
        print(f"Found {len(episode_files)} episodes to process")
        
        for i, episode_info in enumerate(episode_files):
            print(f"\n[{i+1}/{len(episode_files)}] Processing {episode_info['episode_name']} from {episode_info['episode_folder']}")
            
            db_path, temp_dir = decompress_zstd_file(episode_info['file_path'])
            if db_path is None:
                print(f"  Failed to decompress {episode_info['file_path']}")
                continue
            
            try:
                dataset = build_episode_dataset(db_path, episode_info['episode_name'])
                
                if dataset is not None and len(dataset) > 0:
                    save_dataset(dataset, output_directory, episode_info['episode_name'])
                    total_timesteps = sum(len(robot_data['odom_ts']) for robot_data in dataset.values())
                    print(f"  Completed {episode_info['episode_name']}: {len(dataset)} robots, {total_timesteps} total timesteps")
                else:
                    print(f"  No valid data found in {episode_info['episode_name']}")
                    
            except Exception as e:
                print(f"  Error processing {episode_info['episode_name']}: {e}")
                    
            finally:
                if temp_dir:
                    shutil.rmtree(temp_dir)
        
        print(f"\n=== Processing Complete ===")
        
    except Exception as e:
        print(f"Error in process_all_episodes: {e}")
        
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    root_directory = "/home/janhavi/swarm_ws/training_bags"
    output_directory = "/home/janhavi/swarm_ws/processed_datasets"
    
    process_all_episodes(root_directory, output_directory)
