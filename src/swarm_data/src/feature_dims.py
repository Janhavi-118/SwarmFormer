import pickle
from pathlib import Path

data_dir = "/home/janhavi/swarm_ws/processed_datasets"
# Pick the first dataset file
pkl_path = next(Path(data_dir).glob("*_dataset.pkl"))
with open(pkl_path, "rb") as f:
    dataset = pickle.load(f)

robot_name = list(dataset.keys())[0]
robot_data = dataset[robot_name]

print("=== Dimension Inspection ===")

# Helper function to extract odometry message from possible list
def get_odom_msg(odom_entry):
    # If it's a list, take the first element
    if isinstance(odom_entry, list):
        if len(odom_entry) > 0:
            return odom_entry[0]
        return None
    return odom_entry

# 1. Odom feature dimension
odom_vec = get_odom_msg(robot_data['odom_data'])
if odom_vec is not None and hasattr(odom_vec, 'pose'):
    odom_dim = len([
        odom_vec.pose.pose.position.x,
        odom_vec.pose.pose.position.y,
        odom_vec.pose.pose.position.z,
        odom_vec.pose.pose.orientation.x,
        odom_vec.pose.pose.orientation.y,
        odom_vec.pose.pose.orientation.z,
        odom_vec.pose.pose.orientation.w,
        odom_vec.twist.twist.linear.x,
        odom_vec.twist.twist.linear.y,
        odom_vec.twist.twist.linear.z,
        odom_vec.twist.twist.angular.x,
        odom_vec.twist.twist.angular.y,
        odom_vec.twist.twist.angular.z
    ])
    print(f"Odom feature dimension: {odom_dim}")
else:
    print("Failed to extract odom features.")

# 2. Scan feature dimension
scan_msg = robot_data['scan_filled'][0]
if isinstance(scan_msg, list) and len(scan_msg) > 0:
    scan_msg = scan_msg
scan_dim = len(scan_msg.ranges) if scan_msg is not None and hasattr(scan_msg, 'ranges') else None
print(f"Scan feature dimension (#ranges): {scan_dim}")

# 3. Maximum neighbors for gossip
gossip_keys = [len(g) for g in robot_data['gossip_filled']]
max_neighbors = max(gossip_keys)
print(f"Max neighbors observed: {max_neighbors}")

# 4. Cmd_vel dimension
cmd = robot_data['cmd_vel_filled'][0]
if isinstance(cmd, list) and len(cmd) > 0:
    cmd = cmd
cmd_dim = 2 if cmd is not None else None
print(f"Cmd_vel (target) dimension: {cmd_dim}")
