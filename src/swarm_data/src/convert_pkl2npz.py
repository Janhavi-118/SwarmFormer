# convert_ros_pickles.py
import pickle
import numpy as np
from pathlib import Path
from tqdm import tqdm

SRC = Path("/home/janhavi/swarm_ws/processed_datasets")
DST = Path("/home/janhavi/swarm_ws/converted_datasets")
DST.mkdir(exist_ok=True, parents=True)

for pkl in tqdm(list(SRC.glob("*_dataset.pkl"))):
    data = pickle.load(open(pkl, "rb"))
    plain = {}
    for robot, rd in data.items():
        # Odom
        odom_arr = []
        for m in rd['odom_data']:
            msg = m[0] if isinstance(m, list) else m
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            lin = msg.twist.twist.linear
            ang = msg.twist.twist.angular
            odom_arr.append([
                pos.x, pos.y, pos.z,
                ori.x, ori.y, ori.z, ori.w,
                lin.x, lin.y, lin.z,
                ang.x, ang.y, ang.z
            ])
        odom_arr = np.array(odom_arr, dtype=np.float32)

        # Cmd_vel
        cmd_arr = []
        for c in rd['cmd_vel_filled']:
            msg = c[0] if isinstance(c, list) else c
            cmd_arr.append([msg.linear.x, msg.angular.z])
        cmd_arr = np.array(cmd_arr, dtype=np.float32)

        # Scan ranges
        scan_list = []
        for s in rd['scan_filled']:
            if s and isinstance(s, list):
                s = s
            if s is None:
                scan_list.append(None)
            else:
                scan_list.append(np.array(s.ranges, dtype=np.float32))
        # Gossip: list of dicts of positions+orientations
        gossip_list = []
        for g in rd['gossip_filled']:
            nn = []
            for _, nb in g.items():
                pos = nb['position']
                ori = nb['orientation']
                nn.append(pos[:3] + ori[:4])
            gossip_list.append(np.array(nn, dtype=np.float32))

        plain[robot] = {
            'odom': odom_arr,
            'odom_ts': np.array(rd['odom_ts'], dtype=np.int64),
            'scan': scan_list,
            'scan_mask': np.array(rd['scan_mask'], dtype=np.bool_),
            'scan_range_max': rd.get('scan_range_max', None),
            'gossip': gossip_list,
            'gossip_mask': np.array(rd['gossip_mask'], dtype=np.bool_),
            'cmd_vel': cmd_arr
        }

    out_path = DST / (pkl.stem + ".npz")
    np.savez_compressed(out_path, **plain)
