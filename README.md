# NaviGrid: The Adaptive Path Challenge

Autonomous Mobile Robot (AMR) navigation stack featuring a Gazebo Harmonic simulation arena, cost-aware dual-path global planner, local reactive collision avoidance, and a high-frequency velocity-dependent safety override system.

Compatible with modern ROS 2 distributions: **Jazzy Jalisco**, **Humble Hawksbill**, **Iron Irwini**, and **Rolling Ridley**.

---

## 🎯 Challenge Overview & What You Need To Do

**NaviGrid: The Adaptive Path Challenge** is a mobile robotics autonomy competition and benchmark designed to evaluate how an Autonomous Mobile Robot (AMR) navigates complex industrial environments with terrain variations and dynamic hazards.

### The Problem
An AMR must transport goods from **Start Zone A** `(-12.0, -12.0)` to **Goal Zone B** `(+12.0, +12.0)` across a 30m × 30m warehouse floor. Two route topologies connect Start and Goal:
1. **Route 1: Direct 3D Incline Ramp** (~34m Euclidean distance)
   * Climbs an elevated summit platform ($z = 1.4\text{ m}$) via an $11.3^\circ$ incline.
   * Shorter travel distance, but requires significant motor torque and energy to overcome gravitational resistance.
2. **Route 2: Flat 2D Zig-Zag Corridor** (~67m travel distance)
   * Navigates around warehouse storage racks and structural pillars on level ground ($z = 0\text{ m}$).
   * Zero elevation penalty, but twice the travel distance.

### The Autonomy Stages
| Stage | Marks | Module | Objective |
| :--- | :---: | :--- | :--- |
| **Stage 1** | 30 | **Arena & Robot Simulation** | Gazebo Harmonic 30m × 30m arena with ramp, storage racks, crossing dynamic obstacles, and diff-drive AMR model with GPU LIDAR, IMU, and wheel odometry. |
| **Stage 2** | 35 | **Cost-Aware Global Planning** | 2D/3D Costmap layer computing elevation penalties. Autonomous trajectory selection balancing distance vs. slope cost based on robot payload ($m_{\text{payload}}$). |
| **Stage 3** | 35 | **Dynamic Avoidance & Safety** | Reactive Artificial Potential Field (APF) controller navigating around dynamic obstacles, backed by a deterministic 50 Hz Safety Override Node enforcing $d_{\text{safe}} = k \cdot v^2 + d_{\text{min}}$. |

---

## 📦 System Requirements & Dependencies

### Prerequisites
* **Operating System**: Ubuntu 24.04 (Noble) or Ubuntu 22.04 (Jammy)
* **ROS 2 Distribution**: ROS 2 Jazzy, Humble, Iron, or Rolling
* **Simulation Engine**: Gazebo Harmonic (`gz-harmonic` / `gz-sim`)

### Required Packages
Install all ROS 2 and Gazebo integration dependencies via `apt`:

```bash
# Set your ROS 2 distro (e.g., jazzy or humble)
export ROS_DISTRO=jazzy  # change to humble, iron, or rolling if applicable

sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-ros-gz-interfaces \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-rviz2 \
  python3-pip \
  python3-pytest \
  python3-numpy \
  python3-yaml
```

### Automated Dependency Resolution via rosdep
```bash
cd ~/navigrid_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 📂 Workspace Repository Structure

```
navigrid_ws/
├── docs/
│   ├── ARCHITECTURE.md              # System architecture, Mermaid flowcharts & TF tree
│   ├── CODEBASE_LEARNING_GUIDE.md   # Robotics masterclass & evaluation Q&A cheatsheet
│   ├── FILE_BY_FILE_EXPLANATION.md  # Detailed breakdown of every file, class & function
│   ├── LIBRARIES_AND_ALTERNATIVES.md# Encyclopedia of libraries used & industry alternatives
│   └── PRESENTATION_GUIDE.md        # Pitch script, slide outline & live demo cheatsheet
├── src/
│   ├── navigrid_description/        # AMR robot model & visualization
│   │   ├── urdf/amr.urdf.xacro      # Differential-drive chassis, dual casters, LIDAR, IMU
│   │   ├── rviz/navigrid.rviz       # Pre-configured RViz layout (map frame, costmap, scans, path)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── navigrid_gazebo/             # Gazebo Harmonic world & simulation assets
│   │   ├── worlds/navigrid_arena.sdf# 30m x 30m arena, ramp, racks, pillars, crossing actors
│   │   ├── models/                  # Standalone Gazebo models
│   │   │   ├── warehouse_rack/      # Storage rack model (model.config, model.sdf)
│   │   │   ├── structural_pillar/   # Structural chokepoint pillar
│   │   │   ├── incline_ramp/        # Incline ramp structure
│   │   │   ├── dynamic_pedestrian/  # Crossing pedestrian model
│   │   │   └── dynamic_amr/         # Secondary cross-traffic AMR model
│   │   ├── config/ros_gz_bridge.yaml# Topic bridge configuration (clock, scan, imu, odom, cmd_vel)
│   │   ├── launch/sim.launch.py     # Gazebo launch, robot spawner, static TF map->odom & RViz
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── navigrid_navigation/         # Autonomy algorithms (Stages 2 & 3)
│   │   ├── navigrid_navigation/
│   │   │   ├── costmap_generator.py         # 2D/3D costmap with slope penalty layer
│   │   │   ├── adaptive_global_planner.py   # Cost-aware planner (Ramp vs. Zig-zag)
│   │   │   ├── local_reactive_controller.py # Real-time reactive APF path tracker
│   │   │   └── safety_override_node.py      # Stage 3 safety node: d_safe = k * v^2 + d_min
│   │   ├── config/
│   │   │   ├── planner_params.yaml          # Cost weights, grid resolution, payload limit
│   │   │   └── safety_params.yaml           # Safety equation parameters (k, d_min, FOV)
│   │   ├── launch/navigation.launch.py      # Standalone navigation stack launch
│   │   ├── test/test_navigation_math.py     # Automated math & formula unit tests
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── navigrid_bringup/            # Master orchestration package
│       ├── config/
│       │   └── navigrid_bringup.yaml        # Top-level orchestration parameters
│       ├── launch/navigrid_all.launch.py    # Master 1-command startup for entire system
│       ├── CMakeLists.txt
│       └── package.xml
└── README.md
```

---

## 🚀 Quick Start Guide

### 1. Build the Workspace
```bash
# Source your installed ROS 2 distribution
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

cd ~/navigrid_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Complete System (Simulation + Navigation + RViz)
```bash
ros2 launch navigrid_bringup navigrid_all.launch.py
```
*Optional launch arguments:*
* `headless:=true` — Run Gazebo without GUI (saves GPU/CPU resources).
* `rviz:=false` — Launch without RViz visualizer.

### 3. Run Automated Unit Tests
```bash
pytest ~/navigrid_ws/src/navigrid_navigation/test/test_navigation_math.py
```

---

## ⚙️ How to Test Each Challenge Task

### Task A: Verify Sensor Streams & Coordinate Frames
Open a new terminal, source the workspace, and verify active topics:

```bash
source ~/navigrid_ws/install/setup.bash

# 1. Check active ROS 2 topics
ros2 topic list

# 2. Check LIDAR scan returns (640 rays, ranges between 0.15m and 25.0m)
ros2 topic echo /scan --once

# 3. Check 100 Hz IMU orientation and angular velocity
ros2 topic echo /imu --once

# 4. Check Odometry pose and velocity
ros2 topic echo /odom --once

# 5. Check Costmap publication (published with Transient Local QoS in frame 'map')
ros2 topic echo /costmap --once

# 6. Check Active Global Plan (/plan)
ros2 topic echo /plan --once
```

### Task B: Test Stage 2 Autonomous Path Switching
The global planner evaluates the cost objective function:
$$J(\text{path}) = \int \left(1 + \alpha(m_{\text{payload}}) \cdot S(s)\right) ds$$

You can test autonomous path selection under different payload conditions:

1. **Light Payload ($m = 15\text{ kg}$)**:
   $$\text{Cost}_{\text{ramp}} = 34.0 + 11.2 = 45.2 < \text{Cost}_{\text{zigzag}} (67.6) \implies \textbf{Takes Direct Ramp}$$
2. **Heavy Payload ($m = 30\text{ kg}$)**:
   $$\text{Cost}_{\text{ramp}} = 34.0 + 22.4 = 56.4 \text{ vs } \text{Cost}_{\text{zigzag}} \implies \textbf{Switches to Flat Zig-Zag Corridor}$$
3. **Overloaded Payload ($m > 35\text{ kg}$)**:
   $$\text{Exceeds motor torque on } 11.3^\circ \text{ incline} \implies \textbf{Ramp strictly prohibited, selects Zig-Zag}$$

To run with a heavy payload:
```bash
ros2 launch navigrid_bringup navigrid_all.launch.py --ros-args -p payload_weight_kg:=32.0
```

### Task C: Test Stage 3 Dynamic Obstacle Avoidance & Safety Override
* The **dynamic pedestrian** patrols across the primary travel path `(-8.5, -8.5)` between `(-10.5, -6.5)` and `(-6.5, -10.5)`.
* As the obstacle crosses in front of the robot:
  1. The **Local Reactive Controller** uses Artificial Potential Fields (APF) to steer away from the obstacle.
  2. If the obstacle approaches within the braking threshold $d_{\text{safe}} = k \cdot v^2 + d_{\text{min}}$, the **Safety Override Node** triggers an emergency stop (`EMERGENCY STOP! Obstacle at ... < d_safe`).
  3. Once the dynamic obstacle moves clear, the safety node automatically disengages with hysteresis clearance and autonomous navigation resumes smoothly.

### Task D: Interactive Goal Setting in RViz
1. In the RViz toolbar at the top, click the **"2D Goal Pose"** tool.
2. Click and drag anywhere in the arena map to publish a new target pose to `/goal_pose`.
3. The `adaptive_global_planner` will immediately receive the goal, recompute the optimal route, and the robot will autonomously navigate to the new destination.

---

## 🔍 Technical Diagnosis & FAQ

### 1. Why was `/imu` null?
* **Root Cause**: Gazebo Sim requires the `gz-sim-imu-system` plugin loaded into the world SDF in order to simulate `<sensor type="imu">`. Without `<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>`, Gazebo never instantiates the IMU sensor. Additionally, the sensor in `amr.urdf.xacro` lacked `<frame_id>imu_link</frame_id>`.
* **Fix**: Added the `gz-sim-imu-system` plugin to `navigrid_arena.sdf` and declared `<frame_id>imu_link</frame_id>` in `amr.urdf.xacro`. The bridge now continuously streams 100 Hz IMU data.

### 2. Why did the Costmap show an error or `/costmap_updates` is null?
* **Root Cause**:
  1. **QoS Durability Mismatch**: RViz's Map plugin requires `TRANSIENT_LOCAL` durability. The original `costmap_generator.py` published with default `VOLATILE` QoS. ROS 2 DDS treats this as incompatible and drops the topic in RViz.
  2. **`/costmap_updates`**: RViz's Map display defaults `Update Topic` to `/costmap_updates`. Because the standalone costmap generator publishes full map grids on `/costmap` (rather than differential tiles), `/costmap_updates` receives no messages.
* **Fix**:
  1. Set `costmap_pub` in `costmap_generator.py` to `TRANSIENT_LOCAL` and `RELIABLE` QoS.
  2. Set `Update Topic: ""` (empty) in `navigrid.rviz` so RViz does not wait for differential updates.
  3. Costmap `frame_id` is set to `'map'` to anchor to the arena.

### 3. Why did `/scan` appear to output zeros?
* **Root Cause**:
  1. `sensor_msgs/msg/LaserScan` contains two arrays: `ranges` (distance readings) and `intensities` (reflective returns). Simulation lidar does not model surface reflectivity by default, so `intensities` is populated with `0.0`. When using `ros2 topic echo /scan`, the terminal scrolls through 640 ranges and ends on the `intensities` array, making it look like only zeros.
  2. The Gazebo sensor was missing `<frame_id>lidar_link</frame_id>`, which caused Gazebo to publish an unmapped frame name (`navigrid_amr/base_footprint/lidar_sensor`) missing from the TF tree.
* **Fix**: Added `<frame_id>lidar_link</frame_id>` to the URDF sensor, matching the TF tree from `robot_state_publisher`. In RViz, the laser scan is displayed with Flat Color based on distance.

### 4. Why was `/odom` reading zero?
* **Root Cause**: Gazebo's `DiffDrive` odometry plugin calculates distance traveled relative to the robot's spawn pose. At rest at time $t=0$, wheel displacement is $0.0$, so `/odom` position and velocity are naturally $(0, 0)$. Furthermore, the world arena was centered at $(0, 0)$, while the robot started at $(-12, -12)$.
* **Fix**: Added a static transform publisher (`map -> odom` with translation $[-12.0, -12.0, 0.0]$ and yaw $0.785\text{ rad}$). This cleanly links the global arena map to the robot's local odometry frame.

### 5. Why did `/goal_pose`, `/initialpose`, and `/clicked_point` have no messages?
* **Explanation**: These are interactive RViz GUI tool topics. They do not publish continuous streams; they only publish when a user interacts with the RViz toolbar (clicking **"2D Goal Pose"**, **"2D Pose Estimate"**, or **"Publish Point"**). The system operates autonomously using preconfigured start/goal coordinates, but dynamically accepts user goals from `/goal_pose` at any time.

### 6. Why did the bot stop in the middle with an emergency stop deadlock?
* **Root Causes**:
  1. **Artificial Potential Field (APF) Collinear Cancellation**: When facing a dynamic obstacle straight ahead, the repulsive vector directly opposed the attractive vector ($F_{\text{rep}} \approx -F_{\text{att}}$) with near-zero lateral component. The resulting heading error was $0.0$, causing the controller to command straight forward into the obstacle with $0$ angular steering instead of carving a lateral detour.
  2. **ROS 2 Jazzy Logger Crash**: `safety_override_node.py` called `self.get_logger().warn(...)`, which was removed in modern ROS 2 (Jazzy/Rolling/Lyrical) in favor of `.warning(...)`, causing the node to crash upon detecting an obstacle.
  3. **Mutating Re-Planning Loop**: `adaptive_global_planner.py` re-planned every 1.0 second using `self.current_pose`. Once the robot drove forward past `(-9.0, -9.0)`, re-planning inserted `(-9.0, -9.0)` as a waypoint *behind* the robot, causing the robot to turn around.
* **Fix**:
  1. Implemented **Deadlock-Free Lateral Evasion** in `local_reactive_controller.py`: forward obstacles compute a bounded lateral repulsive force ($+y$ or $-y$) with symmetry breaking for dead-center obstacles, smoothly steering the robot sideways around the hazard.
  2. Fixed `self.get_logger().warning(...)` in `safety_override_node.py` and required both `obs_dist >= clear_threshold` and cooldown expiration before disengaging emergency stop.
  3. Cached the active global path (`self.active_path_msg`) in `adaptive_global_planner.py` so the trajectory remains stable during navigation.

### 7. Why was the dynamic obstacle moving outside the perimeter?
* **Root Cause**:
  1. In SDFormat, `<trajectory><waypoint><pose>` tags within an `<actor>` are evaluated **relative to the actor's initial `<pose>`**, not the world origin.
  2. The actor had `<pose>-8.5 -8.5 0.85 0 0 -0.785</pose>` and waypoints at `(-10.5, -6.5)`. Gazebo computed the world coordinate as $(-8.5) + (-10.5) = -19.0\text{ m}$. Because the arena perimeter wall is at $x = \pm 15\text{ m}$, the cylinder was spawned 4 meters outside the arena boundary!
  3. Furthermore, `<actor>` without a skeletal `<skin>` mesh caused Gazebo Harmonic's `libgz-sim-scene-broadcaster-system` to search for `__default__` mesh and segfault.
* **Fix**: Converted dynamic agents to physical models with `gz-sim-trajectory-follower-system`, floating with `<gravity>false</gravity>`, moving strictly within the interior between `(-10.5, -6.5)` and `(-6.5, -10.5)` across the robot's diagonal at `(-8.5, -8.5)`.

### 8. Why did the rover keep moving in the same space near the start point?
* **Root Cause**: In `local_reactive_controller.py`, the Pure Pursuit target loop searched from index 0 (`self.path_points[0] = (-12.0, -12.0)`):
  ```python
  for pt in self.path_points:
      if dist(pt, robot) >= lookahead (1.0m):
          target_pt = pt; break
  ```
  As soon as the robot moved $1.0\text{ m}$ away from the start, `path_points[0]` satisfied `dist >= 1.0m`. Because index 0 was the first point in the list, the controller picked `path_points[0]` behind the robot! The robot turned $180^\circ$, drove back, got within 1m, picked a forward point, drove 1m, and turned around again—oscillating indefinitely in the same 1m space.
* **Fix**: Implemented monotonic progress index tracking:
  ```python
  # Search strictly forward from current_waypoint_idx
  closest_idx = argmin(dist(pt, robot))
  self.current_waypoint_idx = closest_idx
  # Target point is strictly ahead of closest_idx
  for i in range(closest_idx, len(self.path_points)):
      if dist(self.path_points[i], robot) >= self.lookahead:
          target_pt = self.path_points[i]; break
  ```
  The robot now maintains forward velocity along the path all the way to Goal Zone B.

---

## 📚 Further Reading & Documentation

* 📖 [**FILE_BY_FILE_EXPLANATION.md**](docs/FILE_BY_FILE_EXPLANATION.md) — Comprehensive technical reference of all files, classes, parameters, and coordinate systems.
* 🔬 [**LIBRARIES_AND_ALTERNATIVES.md**](docs/LIBRARIES_AND_ALTERNATIVES.md) — Architectural comparison of all ROS 2 & Gazebo packages against industry alternatives.
* 🎓 [**CODEBASE_LEARNING_GUIDE.md**](docs/CODEBASE_LEARNING_GUIDE.md) — In-depth robotics guide covering diff-drive kinematics, costmaps, APF avoidance, and evaluation Q&A.
* 🎤 [**PRESENTATION_GUIDE.md**](docs/PRESENTATION_GUIDE.md) — Presentation pitch script, live demonstration steps, and rubric scoring guide.
* 🏛️ [**ARCHITECTURE.md**](docs/ARCHITECTURE.md) — System block diagrams, TF trees, and mathematical formulations.
