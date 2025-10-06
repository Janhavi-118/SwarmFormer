#!/usr/bin/env python3
"""
process_rosbags_leftovers.py

Parse ONLY specified leftover episode folders from root directory and output dataset.
"""

import os
import json
import argparse
import glob
import numpy as np
import subprocess
import tempfile
import shutil
from collections import defaultdict

import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from swarm_msgs.msg import Gossip

def extract_obstacle_angles(ranges, angle_min, angle_increment, max_angles=5):
    threshold = 1.0
    angles = []
    for i, r in enumerate(ranges):
        if r < threshold:
            angles.append(angle_min + i * angle_increment)
    return angles[:max_angles] + [0.0] * (max_angles - len(angles))

def decompress_zstd_if_needed(bag_path):
    if bag_path.endswith('.zstd'):
        temp_dir = tempfile.mkdtemp(prefix='rosbag_decomp_')
        decompressed_path = os.path.join(temp_dir, os.path.basename(bag_path)[:-5])
        try:
            subprocess.run(['zstd', '-d', bag_path, '-o', decompressed_path],
                           check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return decompressed_path, temp_dir
        except Exception as e:
            print(f"❌ Failed to decompress {bag_path}: {e}")
            shutil.rmtree(temp_dir)
            return None, None
    else:
        return bag_path, None

def parse_bag(bag_path):
    # (Same as previous parse_bag function, unchanged)
    actual_path, temp_dir = decompress_zstd_if_needed(bag_path)
    if actual_path is None:
        return None

    try:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=actual_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)

        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        ts_map = defaultdict(list)
        while reader.has_next():
            topic, data, ts = reader.read_next()
            ts_map[ts / 1e9].append((topic, data))

        episode = {'episode_id': os.path.basename(bag_path), 'timesteps': []}
        for t in sorted(ts_map.keys()):
            step = {'timestamp': t, 'odom': {}, 'scan': {}, 'cmd_vel': {}, 'goal': {}, 'gossip': {}}
            for topic, raw in ts_map[t]:
                mtype = topic_types.get(topic, '')

                if mtype == 'nav_msgs/msg/Odometry' and topic.endswith('/odom'):
                    msg = deserialize_message(raw, Odometry)
                    rid = topic.split('/')[1]
                    step['odom'][rid] = {
                        'position': [msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.orientation.z],
                        'velocity': [msg.twist.twist.linear.x,
                                     msg.twist.twist.linear.y,
                                     msg.twist.twist.angular.z]
                    }

                elif mtype == 'sensor_msgs/msg/LaserScan' and topic.endswith('/scan'):
                    msg = deserialize_message(raw, LaserScan)
                    rid = topic.split('/')[1]
                    ranges = np.array(msg.ranges)
                    ranges[np.isinf(ranges)] = msg.range_max
                    step['scan'][rid] = {
                        'min_distance': float(np.min(ranges)),
                        'mean_distance': float(np.mean(ranges)),
                        'front_clear': float(np.min(ranges[len(ranges)//4:3*len(ranges)//4])),
                        'obstacle_angles': extract_obstacle_angles(
                            ranges, msg.angle_min, msg.angle_increment)
                    }

                elif mtype == 'geometry_msgs/msg/Twist' and topic.endswith('/cmd_vel'):
                    msg = deserialize_message(raw, Twist)
                    rid = topic.split('/')[1]
                    step['cmd_vel'][rid] = [msg.linear.x, msg.angular.z]

                elif mtype == 'geometry_msgs/msg/Point' and '/formation/goal_' in topic:
                    msg = deserialize_message(raw, Point)
                    rid = topic.split('_')[-1]
                    step['goal'][f'robot_{rid}'] = {
                        'target_x': msg.x,
                        'target_y': msg.y,
                        'formation_type': 0.0
                    }

                elif mtype == 'swarm_msgs/msg/Gossip' and topic == '/swarm/gossip':
                    msg = deserialize_message(raw, Gossip)
                    rs = msg.robot_state
                    gossip = {
                        'sender': msg.sender,
                        'robot_state': {
                            'robot_id': rs.robot_id,
                            'position': list(rs.position),
                            'velocity': list(rs.velocity),
                            'neighbors': rs.neighbors,
                            'local_obstacle_count': rs.local_obstacle_count,
                            'battery_level': rs.battery_level,
                            'task_status': rs.task_status,
                            'error_flags': rs.error_flags,
                            'heartbeat_count': rs.heartbeat_count
                        },
                        'known_neighbors': [
                            {
                                'robot_id': nbr.robot_id,
                                'position': list(nbr.position),
                                'velocity': list(nbr.velocity)
                            }
                            for nbr in msg.known_neighbors
                        ],
                        'failed_robots': list(msg.failed_robots)
                    }
                    step['gossip'] = gossip

            if step['odom']:
                episode['timesteps'].append(step)

        return episode

    except Exception as e:
        print(f"❌ Error parsing {bag_path}: {e}")
        return None

    finally:
        if temp_dir and os.path.isdir(temp_dir):
            shutil.rmtree(temp_dir)

def find_bag(folder):
    for ext in ('db3.zstd', 'db3'):
        matches = glob.glob(os.path.join(folder, f'*.{ext}'))
        if matches:
            return matches[0]
    return None

def main():
    rclpy.init()
    root_dir = '/home/janhavi/swarm_ws/training_bags'
    output = '/home/janhavi/swarm_ws/training_data.json'
    
    # Specify only leftover episodes here:
    leftovers = [1]

    episode_dirs = sorted([
        os.path.join(root_dir, f"episode{num}")
        for num in leftovers
    ])

    with open(output, 'w') as fout:
        fout.write('[\n')
        first = True
        for i, epdir in enumerate(episode_dirs, 1):
            if not os.path.isdir(epdir):
                print(f"⚠️ Episode folder not found: {epdir}, skipping")
                continue
            bag = find_bag(epdir)
            if not bag:
                print(f"⚠️ No bag file found in {epdir}, skipping")
                continue
            print(f"[{i}/{len(episode_dirs)}] ➡️ Parsing {bag}")
            ep = parse_bag(bag)
            if not ep:
                print(f"⚠️ Failed to parse {bag}")
                continue
            if not first:
                fout.write(',\n')
            first = False
            json.dump(ep, fout)
            del ep
        fout.write('\n]\n')
    print(f"✅ Wrote leftover episodes dataset to {output}")

if __name__ == '__main__':
    main()
