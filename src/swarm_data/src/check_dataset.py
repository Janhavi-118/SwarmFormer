import pickle
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict
import pandas as pd

def load_episode_data(data_dir, episode_name):
    """Load a single episode dataset and metadata."""
    pkl_path = Path(data_dir) / f"{episode_name}_dataset.pkl"
    json_path = Path(data_dir) / f"{episode_name}_metadata.json"
    
    with open(pkl_path, 'rb') as f:
        dataset = pickle.load(f)
    
    with open(json_path, 'r') as f:
        metadata = json.load(f)
    
    return dataset, metadata

def validate_single_episode(dataset, metadata, episode_name):
    """Validate data structure and quality for one episode."""
    issues = []
    stats = {}
    
    print(f"\n=== Validating Episode: {episode_name} ===")
    
    # 1. Check basic structure
    if not isinstance(dataset, dict):
        issues.append("Dataset is not a dictionary")
        return issues, stats
    
    robots = list(dataset.keys())
    stats['num_robots'] = len(robots)
    print(f"Found {len(robots)} robots: {robots}")
    
    # 2. Check each robot's data
    robot_stats = {}
    for robot in robots:
        robot_data = dataset[robot]
        robot_issues = []
        
        # Check required fields
        required_fields = ['robot_name', 'odom_ts', 'odom_data', 'cmd_vel_filled', 
                          'cmd_vel_mask', 'scan_filled', 'scan_mask', 
                          'gossip_filled', 'gossip_mask']
        
        for field in required_fields:
            if field not in robot_data:
                robot_issues.append(f"Missing field: {field}")
        
        if robot_issues:
            issues.extend([f"Robot {robot}: {issue}" for issue in robot_issues])
            continue
        
        # Check data lengths consistency
        odom_len = len(robot_data['odom_ts'])
        lengths = {
            'odom_data': len(robot_data['odom_data']),
            'cmd_vel_filled': len(robot_data['cmd_vel_filled']),
            'scan_filled': len(robot_data['scan_filled']),
            'gossip_filled': len(robot_data['gossip_filled']),
            'cmd_vel_mask': len(robot_data['cmd_vel_mask']),
            'scan_mask': len(robot_data['scan_mask']),
            'gossip_mask': len(robot_data['gossip_mask'])
        }
        
        for field, length in lengths.items():
            if length != odom_len:
                robot_issues.append(f"Length mismatch: {field} ({length}) vs odom ({odom_len})")
        
        # Check for None values in critical fields
        none_counts = {
            'cmd_vel': sum(1 for x in robot_data['cmd_vel_filled'] if x is None),
            'odom': sum(1 for x in robot_data['odom_data'] if x is None)
        }
        
        if none_counts['cmd_vel'] > 0:
            robot_issues.append(f"Found {none_counts['cmd_vel']} None cmd_vel entries")
        if none_counts['odom'] > 0:
            robot_issues.append(f"Found {none_counts['odom']} None odom entries")
        
        # Check timestamp ordering
        timestamps = robot_data['odom_ts']
        if not np.all(timestamps[:-1] <= timestamps[1:]):
            robot_issues.append("Timestamps are not ordered")
        
        # Check mask statistics
        scan_coverage = np.mean(robot_data['scan_mask'])
        gossip_coverage = np.mean(robot_data['gossip_mask'])
        
        robot_stats[robot] = {
            'timesteps': odom_len,
            'time_span': (timestamps[-1] - timestamps[0]) / 1e9,
            'scan_coverage': scan_coverage,
            'gossip_coverage': gossip_coverage,
            'none_cmd_vel': none_counts['cmd_vel'],
            'none_odom': none_counts['odom'],
            'issues': robot_issues
        }
        
        print(f"  {robot}: {odom_len} timesteps, scan {scan_coverage:.2%}, gossip {gossip_coverage:.2%}")
        
        if robot_issues:
            issues.extend([f"Robot {robot}: {issue}" for issue in robot_issues])
    
    stats['robots'] = robot_stats
    return issues, stats

def analyze_dataset_statistics(data_dir, sample_size=20):
    """Analyze statistics across multiple episodes."""
    print("=== Dataset Statistics Analysis ===")
    
    # Get all episode files
    data_path = Path(data_dir)
    episode_files = list(data_path.glob("*_dataset.pkl"))
    episode_names = [f.stem.replace('_dataset', '') for f in episode_files]
    
    print(f"Found {len(episode_names)} episodes")
    
    # Sample episodes for analysis
    if len(episode_names) > sample_size:
        import random
        sampled_episodes = random.sample(episode_names, sample_size)
        print(f"Analyzing random sample of {sample_size} episodes")
    else:
        sampled_episodes = episode_names
        print(f"Analyzing all {len(episode_names)} episodes")
    
    # Collect statistics
    all_stats = []
    all_issues = []
    
    for episode_name in sampled_episodes:
        try:
            dataset, metadata = load_episode_data(data_dir, episode_name)
            issues, stats = validate_single_episode(dataset, metadata, episode_name)
            
            stats['episode_name'] = episode_name
            all_stats.append(stats)
            all_issues.extend(issues)
            
        except Exception as e:
            print(f"Error loading {episode_name}: {e}")
            all_issues.append(f"Failed to load {episode_name}: {e}")
    
    return all_stats, all_issues

def create_summary_report(all_stats, all_issues):
    """Create comprehensive summary report."""
    print("\n" + "="*60)
    print("DATASET VALIDATION SUMMARY REPORT")
    print("="*60)
    
    # Overall statistics
    total_episodes = len(all_stats)
    total_robots = sum(stats['num_robots'] for stats in all_stats)
    
    print(f"\n📊 OVERALL STATISTICS:")
    print(f"  Total Episodes Analyzed: {total_episodes}")
    print(f"  Total Robot-Episodes: {total_robots}")
    print(f"  Average Robots per Episode: {total_robots/total_episodes:.1f}")
    
    # Collect robot-level stats
    robot_stats = []
    for episode_stats in all_stats:
        for robot, stats in episode_stats['robots'].items():
            stats['episode'] = episode_stats['episode_name']
            stats['robot'] = robot
            robot_stats.append(stats)
    
    df = pd.DataFrame(robot_stats)
    
    print(f"\n⏱️  TEMPORAL STATISTICS:")
    print(f"  Average Timesteps per Robot: {df['timesteps'].mean():.0f}")
    print(f"  Min/Max Timesteps: {df['timesteps'].min()}/{df['timesteps'].max()}")
    print(f"  Average Episode Duration: {df['time_span'].mean():.1f} seconds")
    print(f"  Total Training Time: {df['time_span'].sum()/3600:.1f} hours")
    
    print(f"\n📡 DATA COVERAGE:")
    print(f"  Average Scan Coverage: {df['scan_coverage'].mean():.1%}")
    print(f"  Average Gossip Coverage: {df['gossip_coverage'].mean():.1%}")
    print(f"  Min Scan Coverage: {df['scan_coverage'].min():.1%}")
    print(f"  Min Gossip Coverage: {df['gossip_coverage'].min():.1%}")
    
    # Data quality issues
    print(f"\n🚨 DATA QUALITY ISSUES:")
    if all_issues:
        print(f"  Total Issues Found: {len(all_issues)}")
        issue_types = defaultdict(int)
        for issue in all_issues:
            if "None cmd_vel" in issue:
                issue_types["None cmd_vel entries"] += 1
            elif "Length mismatch" in issue:
                issue_types["Length mismatches"] += 1
            elif "Missing field" in issue:
                issue_types["Missing fields"] += 1
            else:
                issue_types["Other"] += 1
        
        for issue_type, count in issue_types.items():
            print(f"    {issue_type}: {count}")
    else:
        print("  ✅ No critical issues found!")
    
    # Training readiness assessment
    print(f"\n🎯 TRAINING READINESS ASSESSMENT:")
    
    # Check minimum requirements
    min_timesteps = df['timesteps'].min()
    avg_scan_coverage = df['scan_coverage'].mean()
    avg_gossip_coverage = df['gossip_coverage'].mean()
    critical_issues = len([i for i in all_issues if "None cmd_vel" in i or "Missing field" in i])
    
    readiness_score = 0
    readiness_issues = []
    
    if min_timesteps >= 100:
        readiness_score += 25
        print("  ✅ Sufficient timesteps per episode (min: 100)")
    else:
        readiness_issues.append("Some episodes have very few timesteps")
    
    if avg_scan_coverage >= 0.3:  # 30% coverage
        readiness_score += 25
        print("  ✅ Adequate scan data coverage")
    else:
        readiness_issues.append("Low scan data coverage")
    
    if avg_gossip_coverage >= 0.5:  # 50% coverage
        readiness_score += 25
        print("  ✅ Good gossip data coverage")
    else:
        readiness_issues.append("Low gossip data coverage")
    
    if critical_issues == 0:
        readiness_score += 25
        print("  ✅ No critical data integrity issues")
    else:
        readiness_issues.append("Critical data integrity issues found")
    
    print(f"\n📈 OVERALL READINESS SCORE: {readiness_score}/100")
    
    if readiness_score >= 75:
        print("  🎉 EXCELLENT! Dataset is ready for training!")
    elif readiness_score >= 50:
        print("  ⚠️  GOOD. Some minor issues but ready for training.")
    else:
        print("  ❌ NEEDS WORK. Address issues before training.")
    
    if readiness_issues:
        print("\n🔧 RECOMMENDATIONS:")
        for issue in readiness_issues:
            print(f"    • {issue}")
    
    return df, readiness_score

def plot_data_distributions(df):
    """Create visualization plots for data analysis."""
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Dataset Analysis Visualizations', fontsize=16)
    
    # Timesteps distribution
    axes[0,0].hist(df['timesteps'], bins=20, alpha=0.7, color='blue')
    axes[0,0].set_title('Distribution of Timesteps per Robot-Episode')
    axes[0,0].set_xlabel('Number of Timesteps')
    axes[0,0].set_ylabel('Frequency')
    
    # Episode duration distribution
    axes[0,1].hist(df['time_span'], bins=20, alpha=0.7, color='green')
    axes[0,1].set_title('Distribution of Episode Durations')
    axes[0,1].set_xlabel('Duration (seconds)')
    axes[0,1].set_ylabel('Frequency')
    
    # Scan coverage distribution
    axes[1,0].hist(df['scan_coverage'], bins=20, alpha=0.7, color='orange')
    axes[1,0].set_title('Distribution of Scan Data Coverage')
    axes[1,0].set_xlabel('Coverage Ratio')
    axes[1,0].set_ylabel('Frequency')
    axes[1,0].axvline(x=0.3, color='red', linestyle='--', label='Min Threshold')
    axes[1,0].legend()
    
    # Gossip coverage distribution
    axes[1,1].hist(df['gossip_coverage'], bins=20, alpha=0.7, color='purple')
    axes[1,1].set_title('Distribution of Gossip Data Coverage')
    axes[1,1].set_xlabel('Coverage Ratio')
    axes[1,1].set_ylabel('Frequency')
    axes[1,1].axvline(x=0.5, color='red', linestyle='--', label='Min Threshold')
    axes[1,1].legend()
    
    plt.tight_layout()
    plt.savefig('dataset_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

def detailed_episode_inspection(data_dir, episode_name):
    """Detailed inspection of a specific episode."""
    print(f"\n=== DETAILED INSPECTION: {episode_name} ===")
    
    dataset, metadata = load_episode_data(data_dir, episode_name)
    
    for robot_name, robot_data in dataset.items():
        print(f"\nRobot: {robot_name}")
        
        # Sample some data points
        n_samples = min(5, len(robot_data['odom_ts']))
        
        for i in range(n_samples):
            odom = robot_data['odom_data'][i]
            cmd_vel = robot_data['cmd_vel_filled'][i]
            scan = robot_data['scan_filled'][i]
            gossip = robot_data['gossip_filled'][i]
            
            print(f"  Timestep {i}:")
            print(f"    Timestamp: {robot_data['odom_ts'][i]}")
            print(f"    Odom: {type(odom)} - {'Valid' if odom else 'None'}")
            print(f"    Cmd_vel: {type(cmd_vel)} - {'Valid' if cmd_vel else 'None'}")
            print(f"    Scan: {type(scan)} - {'Valid' if scan else 'None'} (mask: {robot_data['scan_mask'][i]})")
            print(f"    Gossip: {len(gossip)} neighbors (mask: {robot_data['gossip_mask'][i]})")

# Main validation script
def run_full_validation(data_dir, sample_size=20):
    """Run complete dataset validation."""
    print("🚀 Starting Comprehensive Dataset Validation...")
    
    # Run statistical analysis
    all_stats, all_issues = analyze_dataset_statistics(data_dir, sample_size)
    
    # Create summary report
    df, readiness_score = create_summary_report(all_stats, all_issues)
    
    # Create visualizations
    plot_data_distributions(df)
    
    # Detailed inspection of one episode
    if all_stats:
        sample_episode = all_stats[0]['episode_name']
        detailed_episode_inspection(data_dir, sample_episode)
    
    return df, readiness_score, all_issues

# Usage
if __name__ == "__main__":
    data_directory = "/home/janhavi/swarm_ws/processed_datasets"  # Update this path
    
    df, score, issues = run_full_validation(data_directory, sample_size=488)
    
    print(f"\n✅ Validation complete! Readiness score: {score}/100")
    
    if score >= 75:
        print("🎉 Your data looks great! Ready to proceed with transformer architecture!")
    else:
        print("⚠️  Some issues found. Review the report above.")
