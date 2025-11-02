# SwarmFormer: Transformer-Based Multi-Robot Formation Control with Risk-Aware Fusion

> **Official Implementation** of the paper accepted at **RCAE 2025**  
> *SwarmFormer: Transformer-Based Multi-Robot Formation Control with Risk-Aware Fusion*

**Authors:** Janhavi Chaurasia, Saanvi Shetty, Sakshi Ahuja, Harini Sriraman  
**Institution:** Vellore Institute of Technology Chennai Campus

## Overview

SwarmFormer introduces a **goal-conditioned transformer-based policy** for decentralized multi-robot formation control and collision avoidance. The framework integrates:

- **Transformer architecture** with cross-modal attention for sensor fusion
- **Gaussian Mixture Model (GMM)** output for multimodal action distributions
- **Risk-aware perception** using CVaR-augmented LiDAR data
- **Gossip protocol** for decentralized inter-robot communication

The system addresses critical challenges in swarm robotics: partial observability, restricted communication, and dynamic formation adaptation in cluttered environments.

### Applications
- Autonomous exploration and surveillance
- Search and rescue operations
- Warehouse logistics and coordination
- Environmental monitoring with robot teams

---

## Key Features

### Goal-Conditioned Policy
- Dynamic formation objectives provided as explicit network inputs
- Real-time adaptation to changing target geometries
- Flexible mission reconfiguration without retraining

### Three-Stage Cross-Modal Attention
1. **State-Goal Alignment**: Aligns robot state with formation objectives
2. **Environment Integration**: Fuses risk-aware LiDAR and odometry data
3. **Communication Fusion**: Incorporates distributed gossip messages

### Probabilistic Action Modeling
- GMM with 5 components models multimodal velocity distributions
- Explicit uncertainty quantification for safer decision-making
- Superior performance in ambiguous scenarios with multiple valid paths

### Risk-Aware Perception
- CVaR-augmented LiDAR scans for principled risk assessment
- Enhanced collision avoidance in noisy, cluttered environments
- Robust to sensor uncertainty and partial observability

### Decentralized Coordination
- Gossip-based communication protocol for scalability
- Asynchronous information sharing without central coordination
- Fault-tolerant operation with intermittent connectivity

---

## Architecture

### Model Components

**Input Modalities:**
- **Odometry** (13D): Robot pose, linear/angular velocities, kinematic state
- **LiDAR + CVaR** (361D): 360° range scans + Conditional Value at Risk features
- **Gossip Messages** (70D): Distributed inter-robot communication
- **Formation Goals**: Target odometry vectors for desired formations

**Network Architecture:**
- 4 transformer layers per modality encoder
- 8 attention heads with 256-dimensional embeddings
- Dropout (0.3) and layer normalization for stability
- GMM head with 5 mixture components for action prediction

**Training Details:**
- AdamW optimizer (lr=1e-5, weight decay=1e-4)
- Gradient clipping (max norm=1.0)
- Cosine annealing schedule with early stopping
- Mixed precision training for efficiency

---

## Installation

### Prerequisites
- Ubuntu 20.04 or 22.04
- ROS2 Humble
- Gazebo 11
- Python 3.8+
- CUDA 11.0+ (for GPU training)

### Step 1: Clone Repository
```bash
git clone https://github.com/Janhavi-118/SwarmFormer.git
cd SwarmFormer
```

### Step 2: Install ROS2 Dependencies
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Install required ROS2 packages
sudo apt-get update
sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations
```

### Step 3: Create Python Environment
```bash
# Create conda environment
conda create -n swarmformer python=3.8
conda activate swarmformer
```

### Step 4: Build Workspace
```bash
# Build ROS2 workspace
colcon build --symlink-install
source install/setup.bash
```

---

## Dataset

### Dataset Structure

The dataset contains **488 unique simulation episodes** with diverse multi-robot scenarios:

- **Formation Types:** Line, Wedge, Circle, Diamond, Grid (5 types)
- **Swarm Sizes:** 3, 5, 7, 11 robots (4 sizes)
- **Obstacle Densities:** 0.0 (empty), 0.15 (light), 0.3 (medium), 0.45 (heavy)
- **Formation Tightness:** Tight, spread, random spacing

### Data Format

Each timestep contains:
- Odometry data (pose, velocities)
- 360° LiDAR scans with CVaR features
- Gossip communication messages from neighbors
- Ground truth velocity commands (linear, angular)
- Formation goal reference

### Dataset Generation

Perform each of these in separate terminals.

```bash
# Launch Gazebo environment
ros2 launch swarm_data swarm_launch.launch.py

# Change the value of obstacle density as required by respective scenario in obstacle_spawn_launch.launch.py
# Launch the obstacle spawner
ros2 launch swarm_data obstacle_spawn_launch.launch.py

# Change the number of robots and the initial positions in robot_spawn_launch.launch.py
# Launch the robot spawner
ros2 launch swarm_data robot_spawn_launch.launch.py

# Change the number of robots in movement_launch.launch.py
# Launch the movement code
ros2 launch swarm_data movement_launch.launch.py

# Change the formation goals and number of robots in formation_goal_launch.launch.py
# Launch the formation goal code
ros2 launch swarm_data formation_goal_launch.launch.py

# COLLECT THE DATA USING ROSBAGS
# (THE SPECIFIC COMMANDS FOR THE SAME CAN BE FOUND IN THE DATASET REPOSITORY.)

```

### Download Pre-collected Dataset

Go to the official GitHub repository of the dataset [https://github.com/Janhavi-118/SwarmDataset.git] and follow the instrcutions.

## Usage

After preparing the dataset:
- Compress the converted_dataset folder to a zip file.
- Upload the zip file into your drive.
- Go through the ipynb file codes provided and change paths wherever needed.
- It is suggested that the ipynb file is opened and run on google colab.

## Results

### Quantitative Performance

| Model | Test MAE ↓ | Test RMSE ↓ | Test NLL ↓ | Test MSE ↓ |
|-------|-----------|------------|-----------|-----------|
| **SwarmFormer** | **0.0767** | 0.2899 | **-3.6712** | - |
| MLP Baseline | 0.0968 | 0.2696 | - | 0.0781 |
| RNN Baseline | 0.1069 | **0.2632** | - | 0.0743 |

**Key Findings:**
- **Best MAE**: SwarmFormer achieves 21% improvement over MLP and 28% over RNN
- **Superior Uncertainty Modeling**: NLL of -3.67 demonstrates probabilistic confidence
- **Balanced Performance**: Best combination of accuracy and uncertainty quantification
- **Complex Scenario Excellence**: Outperforms baselines significantly in dense obstacles + large swarms

---

## Citation

If you use SwarmFormer in your research, please cite our paper:

```bibtex
@inproceedings{chaurasia2025swarmformer,
  title={SwarmFormer: Transformer-Based Multi-Robot Formation Control with Risk-Aware Fusion},
  author={Chaurasia, Janhavi and Shetty, Saanvi and Ahuja, Sakshi and Sriraman, Harini},
  booktitle={Proceedings of the International Conference on Recent Advances in Control Engineering (RCAE)},
  year={2025},
  organization={IEEE}
}
```

**Note:** The research paper is copyrighted by IEEE (RCAE 2025). This repository contains the implementation code only.

---

### References

This work builds upon research in:
- Transformer architectures for robotics
- Decentralized multi-agent coordination
- Probabilistic modeling with GMMs
- Gossip protocols for swarm communication

---
