# NaviGrid: The Adaptive Path Challenge

Autonomous Mobile Robot (AMR) navigation stack, Gazebo Harmonic simulation world, cost-aware dual-path global planner, and velocity-dependent safety override node.

Built strictly in adherence to the official **NaviGrid Challenge Rulebook, FAQs, and Specifications**.

---

## 📚 Complete Learning & Presentation Documentation

All comprehensive documentation has been organized inside the [`docs/`](docs/) directory:

| Document | Description |
| :--- | :--- |
| 📖 [**FILE_BY_FILE_EXPLANATION.md**](docs/FILE_BY_FILE_EXPLANATION.md) | **Line-by-line breakdown** of every file, class, function, parameter, and coordinate frame in the workspace. |
| 🔬 [**LIBRARIES_AND_ALTERNATIVES.md**](docs/LIBRARIES_AND_ALTERNATIVES.md) | Complete encyclopedia of **every library used, its purpose, and 2–4 industry alternatives** (with comparison tables). |
| 🎓 [**CODEBASE_LEARNING_GUIDE.md**](docs/CODEBASE_LEARNING_GUIDE.md) | **Robotics masterclass** on diff-drive kinematics, TF2, costmaps, APF avoidance, and **judge Q&A cheatsheet**. |
| 🎤 [**PRESENTATION_GUIDE.md**](docs/PRESENTATION_GUIDE.md) | **3–5 minute pitch script**, slide outline, live demo commands, and rubric alignment for full marks. |
| 🏛️ [**ARCHITECTURE.md**](docs/ARCHITECTURE.md) | System block diagrams, Mermaid flowcharts, TF tree hierarchy, and kinematic derivations. |

---

## 📂 Workspace Repository Structure

```
navigrid_ws/
├── docs/
│   ├── ARCHITECTURE.md              # System architecture & Mermaid flowcharts
│   ├── PRESENTATION_GUIDE.md        # Pitch script & judge evaluation rubric
│   ├── FILE_BY_FILE_EXPLANATION.md  # Detailed functionality of every file
│   ├── LIBRARIES_AND_ALTERNATIVES.md# Guide to all libraries & industry alternatives
│   └── CODEBASE_LEARNING_GUIDE.md   # Robotics concepts masterclass & judge Q&A
├── src/
│   ├── navigrid_description/        # AMR URDF/Xacro model, sensors, RViz configuration
│   │   ├── urdf/amr.urdf.xacro      # Chassis, wheels, casters, LIDAR, IMU, diff-drive
│   │   ├── rviz/navigrid.rviz       # Pre-configured RViz visualizer
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── navigrid_gazebo/             # Gazebo simulation world & ROS-GZ bridge
│   │   ├── worlds/navigrid_arena.sdf# 30m x 30m arena, ramp + zig-zag, dynamic actors
│   │   ├── config/ros_gz_bridge.yaml# Topic bridge configuration (scan, imu, odom, cmd_vel)
│   │   ├── launch/sim.launch.py     # Simulation bringup, robot spawner & RViz
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── navigrid_navigation/         # Stages 2 & 3 navigation algorithms
│   │   ├── navigrid_navigation/
│   │   │   ├── costmap_generator.py         # 2D/3D costmap with slope penalty layer
│   │   │   ├── adaptive_global_planner.py   # Cost-aware planner (Ramp vs. Zig-zag)
│   │   │   ├── local_reactive_controller.py # Real-time reactive APF path follower
│   │   │   └── safety_override_node.py      # Stage 3: d_safe = k * v^2 + d_min
│   │   ├── config/
│   │   │   ├── planner_params.yaml          # Grid resolution, payload, slope weights
│   │   │   └── safety_params.yaml           # Safety equation parameters (k, d_min)
│   │   ├── launch/navigation.launch.py
│   │   ├── test/test_navigation_math.py     # Pytest unit tests (100% passing)
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── navigrid_bringup/            # Master orchestration package
│       ├── launch/navigrid_all.launch.py    # 1-command startup for entire stack
│       ├── CMakeLists.txt
│       └── package.xml
└── README.md
```

---

## 🚀 Quick Start Guide

### 1. Build the Workspace
```bash
source /opt/ros/lyrical/setup.bash
cd ~/navigrid_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Everything (Simulation + Navigation + Safety Override + RViz)
```bash
ros2 launch navigrid_bringup navigrid_all.launch.py
```

### 3. Run Automated Math & Formula Tests
```bash
pytest ~/navigrid_ws/src/navigrid_navigation/test/test_navigation_math.py
```

---

## 🏆 Competition Stages Overview

| Stage | Marks | Module | Key Features |
| :--- | :---: | :--- | :--- |
| **Stage 1** | 30 | Arena & Robot Setup | 30m × 30m world, 3D incline ramp, flat zig-zag corridor, static racks/pillars, dynamic crossing pedestrians, Xacro AMR model. |
| **Stage 2** | 35 | Cost-Aware Planning | 2D/3D costmap with slope penalty layer, autonomous trajectory selection balancing ramp elevation cost vs flat corridor distance. |
| **Stage 3** | 35 | Dynamic Avoidance & Safety | Reactive APF controller avoiding pedestrians, dedicated 50Hz safety node enforcing `d_safe = k * v² + d_min`. |
