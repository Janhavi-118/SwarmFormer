#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from sklearn.mixture import GaussianMixture
from scipy.spatial.distance import cdist
import os
import json
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class SwarmDatasetProcessor:
    def __init__(self, bag_dir, output_dir):
        self.bag_dir = bag_dir
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Data storage
        self.robot_data = {}
        self.gossip_data = {}
        
    def load_bag_data(self):
        """Load and parse ROS 2 bag data"""
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_dir, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            # Extract robot ID from topic name
            robot_id = topic.split('/')[1] if len(topic.split('/')) > 1 else 'global'
            
            # Process based on message type
            if isinstance(msg, LaserScan):
                self._process_scan(robot_id, msg, timestamp)
            elif isinstance(msg, Odometry):
                self._process_odom(robot_id, msg, timestamp)
            elif 'gossip' in topic:
                self._process_gossip(msg, timestamp)
    
    def _process_scan(self, robot_id, msg, timestamp):
        if robot_id not in self.robot_data:
            self.robot_data[robot_id] = {}
        if timestamp not in self.robot_data[robot_id]:
            self.robot_data[robot_id][timestamp] = {}
        self.robot_data[robot_id][timestamp]['lidar'] = np.array(msg.ranges)
    
    def _process_odom(self, robot_id, msg, timestamp):
        if robot_id not in self.robot_data:
            self.robot_data[robot_id] = {}
        if timestamp not in self.robot_data[robot_id]:
            self.robot_data[robot_id][timestamp] = {}
            
        pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.robot_data[robot_id][timestamp]['pose'] = [
            pose.x, pose.y, pose.z,
            orientation.x, orientation.y, orientation.z, orientation.w
        ]
    
    def _process_gossip(self, msg, timestamp):
        try:
            data = json.loads(msg.data)
            robot_id = data['robot_id']
            if robot_id not in self.gossip_data:
                self.gossip_data[robot_id] = {}
            self.gossip_data[robot_id][timestamp] = data['gmm_params']
        except json.JSONDecodeError:
            pass
    
    def compute_optimal_trajectories(self):
        """Generate MPC-based optimal waypoints for each robot"""
        for robot_id, timestamps in self.robot_data.items():
            for ts, data in timestamps.items():
                if 'pose' not in data or 'lidar' not in data:
                    continue
                    
                # Get GMM parameters (use latest gossip before this timestamp)
                gmm_params = self._get_gmm_params(robot_id, ts)
                
                # Compute optimal waypoint using MPC
                waypoint = self.mpc_optimize(
                    current_pose=data['pose'],
                    lidar=data['lidar'],
                    gmm_params=gmm_params
                )
                
                # Compute collision risk
                risk = self.compute_cvar_risk(data['lidar'])
                
                # Store results
                data['target_waypoint'] = waypoint
                data['risk_score'] = risk
                data['gmm_params'] = gmm_params
    
    def _get_gmm_params(self, robot_id, timestamp):
        """Get nearest GMM parameters before given timestamp"""
        if robot_id not in self.gossip_data:
            return None
            
        # Find most recent gossip before current timestamp
        prev_timestamps = [ts for ts in self.gossip_data[robot_id].keys() if ts <= timestamp]
        if not prev_timestamps:
            return None
            
        nearest_ts = max(prev_timestamps)
        return self.gossip_data[robot_id][nearest_ts]
    
    def mpc_optimize(self, current_pose, lidar, gmm_params):
        """Model Predictive Control optimization"""
        # Simplified MPC implementation
        def objective(waypoint):
            # Energy cost (distance to waypoint)
            energy_cost = np.linalg.norm(waypoint - np.array(current_pose[:2]))
            
            # Formation cost (Wasserstein distance to GMM)
            if gmm_params:
                formation_cost = self.wasserstein_distance(
                    np.array([waypoint]), 
                    gmm_params['means'],
                    gmm_params['covs'],
                    gmm_params['weights']
                )
            else:
                formation_cost = 0
                
            # Safety cost (inverse of min lidar distance)
            safety_cost = 1.0 / (np.min(lidar) + 1e-5)
            
            return 0.5 * energy_cost + 0.3 * formation_cost + 0.2 * safety_cost
        
        # Constraints: velocity limits
        constraints = [
            {'type': 'ineq', 'fun': lambda x: 0.26 - np.linalg.norm(x - np.array(current_pose[:2]))},
            {'type': 'ineq', 'fun': lambda x: 1.5 - np.abs(np.arctan2(x[1], x[0]) - current_pose[5])}
        ]
        
        # Initial guess (current velocity direction)
        x0 = [current_pose[0] + 0.1, current_pose[1]]
        
        # Solve optimization
        result = minimize(objective, x0, method='SLSQP', constraints=constraints)
        return result.x
    
    def wasserstein_distance(self, points, means, covs, weights):
        """Compute Wasserstein distance to GMM"""
        # Simplified 1D Wasserstein for efficiency
        D = cdist(points, means, 'euclidean')
        return np.sum(weights * np.min(D, axis=0))
    
    def compute_cvar_risk(self, lidar_data, alpha=0.05):
        """Conditional Value-at-Risk for collision probability"""
        # Get worst 5% of readings
        sorted_dists = np.sort(lidar_data)
        worst_5pct = sorted_dists[:int(alpha * len(sorted_dists))]
        return np.mean(worst_5pct) if len(worst_5pct) > 0 else 0.0
    
    def save_dataset(self):
        """Save processed dataset to Parquet format"""
        records = []
        for robot_id, timestamps in self.robot_data.items():
            for ts, data in timestamps.items():
                if 'target_waypoint' not in data:
                    continue
                    
                record = {
                    'robot_id': robot_id,
                    'timestamp': ts,
                    'lidar': data['lidar'].tobytes(),
                    'pose': data['pose'],
                    'target_waypoint': data['target_waypoint'],
                    'risk_score': data['risk_score']
                }
                
                if 'gmm_params' in data and data['gmm_params']:
                    record.update({
                        'gmm_means': np.array(data['gmm_params']['means']).tobytes(),
                        'gmm_covs': np.array(data['gmm_params']['covs']).tobytes(),
                        'gmm_weights': np.array(data['gmm_params']['weights']).tobytes()
                    })
                
                records.append(record)
        
        df = pd.DataFrame(records)
        df.to_parquet(os.path.join(self.output_dir, 'swarm_dataset.parquet'))
        
    def process(self):
        self.load_bag_data()
        self.compute_optimal_trajectories()
        self.save_dataset()

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Swarm Dataset Processor')
    parser.add_argument('--bag-dir', required=True, help='Path to ROS bag directory')
    parser.add_argument('--output-dir', required=True, help='Output directory for processed data')
    args = parser.parse_args()
    
    processor = SwarmDatasetProcessor(args.bag_dir, args.output_dir)
    processor.process()
    print(f"Dataset saved to {args.output_dir}/swarm_dataset.parquet")

if __name__ == '__main__':
    main()
