#!/usr/bin/env python3
"""
process_rosbags_stream.py

Like process_rosbags.py, but streams episodes directly to disk to avoid high memory usage.
"""

import os, json, argparse, glob, numpy as np, subprocess, tempfile, shutil
from collections import defaultdict
import rosbag2_py, rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from swarm_msgs.msg import Gossip

def extract_obstacle_angles(ranges, amin, ainc, max_angles=5):
    th=1.0; angles=[]
    for i,r in enumerate(ranges):
        if r<th: angles.append(amin + i*ainc)
    return angles[:max_angles] + [0.0]*(max_angles-len(angles))

def decompress(bag_path):
    if bag_path.endswith('.zstd'):
        td=tempfile.mkdtemp(prefix='rosbag_decomp_')
        out=os.path.join(td,os.path.basename(bag_path)[:-5])
        try:
            subprocess.run(['zstd','-d',bag_path,'-o',out],
                           check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            return out,td
        except:
            shutil.rmtree(td); return None,None
    return bag_path,None

def parse_bag(bag_path):
    ap,td=decompress(bag_path)
    if ap is None: return None
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=ap,storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('',''))
    tmap=defaultdict(list)
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic,data,ts=reader.read_next()
        tmap[ts/1e9].append((topic,data))
    # build episode
    ep={'episode_id':os.path.basename(bag_path),'timesteps':[]}
    for t in sorted(tmap):
        step={'timestamp':t,'odom':{},'scan':{},'cmd_vel':{},'goal':{},'gossip':{}}
        for topic,raw in tmap[t]:
            mtype=types.get(topic,'')
            if mtype=='nav_msgs/msg/Odometry' and topic.endswith('/odom'):
                msg=deserialize_message(raw,Odometry); rid=topic.split('/')[1]
                step['odom'][rid]={'position':[msg.pose.pose.position.x,
                                               msg.pose.pose.position.y,
                                               msg.pose.pose.orientation.z],
                                   'velocity':[msg.twist.twist.linear.x,
                                               msg.twist.twist.linear.y,
                                               msg.twist.twist.angular.z]}
            elif mtype=='sensor_msgs/msg/LaserScan' and topic.endswith('/scan'):
                msg=deserialize_message(raw,LaserScan); rid=topic.split('/')[1]
                rg=np.array(msg.ranges); rg[np.isinf(rg)]=msg.range_max
                step['scan'][rid]={'min_distance':float(rg.min()),
                                  'mean_distance':float(rg.mean()),
                                  'front_clear':float(rg[len(rg)//4:3*len(rg)//4].min()),
                                  'obstacle_angles':extract_obstacle_angles(
                                      rg,msg.angle_min,msg.angle_increment)}
            elif mtype=='geometry_msgs/msg/Twist' and topic.endswith('/cmd_vel'):
                msg=deserialize_message(raw,Twist); rid=topic.split('/')[1]
                step['cmd_vel'][rid]=[msg.linear.x,msg.angular.z]
            elif mtype=='geometry_msgs/msg/Point' and '/formation/goal_' in topic:
                msg=deserialize_message(raw,Point); rid=topic.split('_')[-1]
                step['goal'][f'robot_{rid}']={'target_x':msg.x,'target_y':msg.y,'formation_type':0.0}
            elif mtype=='swarm_msgs/msg/Gossip' and topic=='/swarm/gossip':
                msg=deserialize_message(raw,Gossip); rs=msg.robot_state
                step['gossip']={'sender':msg.sender,
                                'robot_state':{'robot_id':rs.robot_id,
                                               'position':list(rs.position),
                                               'velocity':list(rs.velocity),
                                               'neighbors':rs.neighbors,
                                               'local_obstacle_count':rs.local_obstacle_count,
                                               'battery_level':rs.battery_level,
                                               'task_status':rs.task_status,
                                               'error_flags':rs.error_flags,
                                               'heartbeat_count':rs.heartbeat_count},
                                'known_neighbors':[
                                    {'robot_id':n.robot_id,
                                     'position':list(n.position),
                                     'velocity':list(n.velocity)}
                                    for n in msg.known_neighbors],
                                'failed_robots':list(msg.failed_robots)}
        if step['odom']:
            ep['timesteps'].append(step)
    if td: shutil.rmtree(td)
    return ep

def find_bag(folder):
    for ext in ('db3.zstd','db3'):
        f=glob.glob(os.path.join(folder,f'*.{ext}'))
        if f: return f[0]
    return None

def main():
    rclpy.init()
    root_dir = '/home/janhavi/swarm_ws/training_bags'
    output = '/home/janhavi/swarm_ws/training_data.json'
    dirs=[d for d in sorted(os.listdir(root_dir))
          if os.path.isdir(os.path.join(root_dir,d))]
    with open(output,'w') as fout:
        fout.write('[\n')
        first=True
        for i,d in enumerate(dirs,1):
            folder=os.path.join(root_dir,d)
            bag=find_bag(folder)
            if not bag: continue
            print(f"[{i}/{len(dirs)}] ➡️ Parsing {bag}")
            ep=parse_bag(bag)
            if not ep: continue
            if not first: fout.write(',\n')
            first=False
            json.dump(ep,fout)
            # free memory
            del ep
        fout.write('\n]\n')
    print(f"✅ Wrote streamed dataset to {output}")

if __name__=='__main__':
    main()
