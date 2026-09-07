# NaviGrid Challenge: Presentation & Evaluation Guide

Use this guide when presenting the project to judges, evaluators, or team members. It is structured directly according to the official **100-mark scoring rubric**.

---

## 🎤 Presentation Pitch Outline (3 - 5 Minutes)

### 1. Introduction (30 seconds)
> *"Hello judges. Our project implements a complete, production-grade autonomous mobile robot (AMR) navigation stack for the NaviGrid Adaptive Path Challenge. It runs on Ubuntu with ROS 2 and Gazebo Harmonic, achieving zero-error Colcon compilation, modular package separation, and compliance with Google C++ and PEP 8 standards."*

### 2. Stage 1: Arena & System Integration (30 Marks)
* **What to Show**: The 30m × 30m world in Gazebo and the AMR model in RViz.
* **Key Talking Points**:
  - *"We designed an exact 30m × 30m arena with perimeter LIDAR-reflective boundary walls."*
  - *"It features the dual-path topology: an elevated 3D incline ramp rising to 1.4m summit and a longer flat 2D zig-zag corridor with warehouse racks, structural pillars, and dynamic pedestrian actors."*
  - *"The AMR model is modeled in modular Xacro with differential drive kinematics, passive casters for ramp stability, GPU LIDAR, IMU, and clean `ros_gz_bridge` integration."*

### 3. Stage 2: Mapping & Adaptive Navigation (35 Marks)
* **What to Show**: Costmap in RViz and Planner Terminal Logs.
* **Key Talking Points**:
  - *"Our global costmap node builds a coherent occupancy grid with dynamic obstacle inflation and a 3D elevation slope layer."*
  - *"Our cost-aware global planner dynamically balances travel distance (`L`) against elevation slope cost `S(s)` using `J = integral(1 + alpha * S(s)) ds`."*
  - *"When the robot carries a light payload, it selects the direct ramp (34m vs 55m). Under heavy payloads or steep slope penalties, it autonomously diverts to the flat zig-zag corridor to prevent motor stall or tipping."*

### 4. Stage 3: Dynamic Obstacle Avoidance & Safety Override (35 Marks)
* **What to Show**: Crossing pedestrian in Gazebo, reactive detour in RViz, and Safety Marker switching to RED when breached.
* **Key Talking Points**:
  - *"Stage 3 tackles dynamic environments. Our local controller uses Artificial Potential Fields (APF) to seamlessly detour around crossing pedestrians without getting trapped in stall loops."*
  - *"For critical safety, we implemented a dedicated high-priority safety override node running at 50 Hz. It enforces the exact competition formula:"*
    ```text
    d_safe = k * v² + d_min
    ```
  - *"If an actor suddenly cuts into the robot's dynamic safety envelope, the override instantly clamps `/cmd_vel` to zero, guaranteeing collision prevention (avoiding the +10.0s penalty)."*

---

## 📋 Evaluation Checklist & Command Cheatsheet

### Build & Validate Workspace (Colcon Rule Compliance)
```bash
source /opt/ros/lyrical/setup.bash
cd ~/navigrid_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Entire System with One Command
```bash
source ~/navigrid_ws/install/setup.bash
ros2 launch navigrid_bringup navigrid_all.launch.py
```

### Launch Gazebo Arena & Robot Only (Stage 1 Demo)
```bash
ros2 launch navigrid_gazebo sim.launch.py
```

### Launch Navigation Stack & Safety Override (Stages 2 & 3 Demo)
```bash
ros2 launch navigrid_navigation navigation.launch.py
```

### Trigger New Goal in RViz or Terminal
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 12.0, y: 12.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

### Check Active Topics
```bash
ros2 topic list | grep -E "scan|odom|plan|costmap|cmd_vel|safety"
```
