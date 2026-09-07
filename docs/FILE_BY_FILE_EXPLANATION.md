# NaviGrid AMR: Complete File-by-File Technical Guide

This document provides a comprehensive, file-by-file breakdown of every folder, configuration, script, model, and launch file in the `navigrid_ws` workspace. It is intended to help you learn, master, and present the inner workings of each component to judges and peers.

---

## Table of Contents
1. [Workspace Structure Overview](#1-workspace-structure-overview)
2. [Package: navigrid_description](#2-package-navigrid_description)
   - [package.xml](#navigrid_descriptionpackagexml)
   - [CMakeLists.txt](#navigrid_descriptioncmakeliststxt)
   - [amr.urdf.xacro](#amrurdfxacro)
   - [navigrid.rviz](#navigridrviz)
3. [Package: navigrid_gazebo](#3-package-navigrid_gazebo)
   - [package.xml](#navigrid_gazebopackagexml)
   - [CMakeLists.txt](#navigrid_gazebocmakeliststxt)
   - [navigrid_arena.sdf](#navigrid_arenasdf)
   - [ros_gz_bridge.yaml](#ros_gz_bridgeyaml)
   - [sim.launch.py](#simlaunchpy)
4. [Package: navigrid_navigation](#4-package-navigrid_navigation)
   - [package.xml](#navigrid_navigationpackagexml)
   - [setup.py & setup.cfg](#setuppy--setupcfg)
   - [planner_params.yaml & safety_params.yaml](#planner_paramsyaml--safety_paramsyaml)
   - [costmap_generator.py](#costmap_generatorpy)
   - [adaptive_global_planner.py](#adaptive_global_plannerpy)
   - [local_reactive_controller.py](#local_reactive_controllerpy)
   - [safety_override_node.py](#safety_override_nodepy)
   - [navigation.launch.py](#navigationlaunchpy)
   - [test_navigation_math.py](#test_navigation_mathpy)
5. [Package: navigrid_bringup](#5-package-navigrid_bringup)
   - [navigrid_all.launch.py](#navigrid_alllaunchpy)
6. [Desktop Automation & Launchers](#6-desktop-automation--launchers)

---

## 1. Workspace Structure Overview

The workspace follows the standard ROS 2 multi-package architecture:
* **Separation of Concerns**: Physical description (`description`), simulation environment (`gazebo`), autonomy algorithms (`navigation`), and system coordination (`bringup`) are decoupled into independent packages.
* **Colcon Cleanliness**: Every package includes correct package manifests (`package.xml`) and build definitions (`CMakeLists.txt` or `setup.py`), compiling with 0 errors and 0 warnings.

---

## 2. Package: `navigrid_description`

### `navigrid_description/package.xml`
* **Purpose**: Declares metadata and build/runtime dependencies for the robot model.
* **Key Tags**:
  * `<buildtool_depend>ament_cmake</buildtool_depend>`: Uses `ament_cmake` as the build system.
  * `<exec_depend>robot_state_publisher</exec_depend>`: Needed at runtime to compute forward kinematics and broadcast TF frames.
  * `<exec_depend>xacro</exec_depend>`: Required to process XML macro macros into pure URDF format.
  * `<exec_depend>rviz2</exec_depend>`: The 3D visualization frontend.

### `navigrid_description/CMakeLists.txt`
* **Purpose**: Instructs CMake to install the `urdf/` and `rviz/` directories into the ROS 2 share directory (`install/navigrid_description/share/navigrid_description/`).
* **Why it matters**: Without the `install(...)` macro, launch files would not be able to find `amr.urdf.xacro` or `navigrid.rviz` via `get_package_share_directory()`.

### `navigrid_description/urdf/amr.urdf.xacro`
* **Purpose**: Defines the kinematic, dynamic, visual, collision, and sensor properties of the Autonomous Mobile Robot (AMR).
* **Detailed Breakdown**:
  1. **Properties & Dimensions**:
     * Chassis: 0.60m (length) × 0.45m (width) × 0.22m (height), mass = 18.0kg.
     * Wheels: Radius 0.10m, separation 0.48m, mass = 2.0kg.
  2. **Link Hierarchy**:
     * `base_footprint`: Ground-plane projection of the robot center (z = 0). Essential for navigation costmaps so the robot footprint is evaluated at floor level.
     * `base_link`: Main chassis housing payload, computer, and battery. Elevated at z = 0.10m (equal to wheel radius).
     * `left_wheel_link` & `right_wheel_link`: Actuated cylinders providing differential drive traction.
     * `front_caster_link` & `rear_caster_link`: Dual passive spherical casters (r = 0.05m) mounted front (x = +0.22) and rear (x = -0.22). They provide a stable 4-point contact polygon preventing tipping while climbing the 11.3° incline ramp!
     * `lidar_link`: Mounted forward at x = +0.18m, z = 0.245m for an unobstructed forward field of view.
     * `imu_link`: Mounted at the chassis center for measuring angular rates and gravitational tilt during ramp ascent.
  3. **Inertia Tensors**:
     * Implements precise analytical box and cylinder moments of inertia (Ixx, Iyy, Izz), preventing erratic physics bounce in Gazebo.
  4. **Gazebo Sim Plugins**:
     * `<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">`:
       Simulates realistic motor torques, wheel slip, and differential drive kinematics. Subscribes to `/cmd_vel` and publishes `/odom` and TF (`odom -> base_footprint`).
     * `<sensor name="lidar_sensor" type="gpu_lidar">`:
       Generates 640 horizontal range samples spanning 360° (-180° to +180°) up to 25m range at 15 Hz.
     * `<sensor name="imu_sensor" type="imu">`:
       Streams 3-axis angular velocity and linear acceleration at 100 Hz.

### `navigrid_description/rviz/navigrid.rviz`
* **Purpose**: Pre-configured RViz visualizer layout.
* **Configured Displays**:
  * `RobotModel`: Visualizes the 3D blue chassis and black wheels updating via TF.
  * `TF`: Shows the entire coordinate frame tree.
  * `LaserScan`: Displays red lidar point reflections.
  * `Costmap`: Visualizes the 2D grid map with obstacle inflation and slope penalties.
  * `Global Path`: Neon green line tracing the planned path from Start to Goal.
  * `Safety Zone Markers`: Dynamic semi-transparent cylinder that expands quadratically as robot speed increases (Green = Clear, Red = Emergency Stop triggered).

---

## 3. Package: `navigrid_gazebo`

### `navigrid_gazebo/package.xml` & `CMakeLists.txt`
* Manages dependencies on `ros_gz_sim`, `ros_gz_bridge`, and `navigrid_description`, exporting the world files, bridge configs, and launch files to the install space.

### `navigrid_gazebo/worlds/navigrid_arena.sdf`
* **Purpose**: Complete 3D simulation environment conforming to all competition rules.
* **Detailed Components**:
  1. **Bounding Walls (30m × 30m)**:
     * North (y = +15), South (y = -15), East (x = +15), West (x = -15).
     * 2.0m high with collision boxes ensuring full lidar reflection.
  2. **Start & Goal Markers**:
     * Start Zone A: Green circle centered at (-12, -12, 0).
     * Goal Zone B: Red circle centered at (+12, +12, 0).
  3. **Dual-Path Topology**:
     * **Path 1: Direct Incline Ramp (3D)**:
       * Ascending ramp from (-9, -9) to (-2, -2), tilted at theta ≈ 11.3° (0.2 rad) rising to 1.4m.
       * Summit platform (3.5m × 3.5m) at (0, 0, 1.4).
       * Descending ramp from (+2, +2) to (+9, +9) back to ground level.
       * Tests 3D terrain cost, motor torque under payload, and tilt stability.
     * **Path 2: Zig-Zag Ground Floor Corridor (2D)**:
       * Completely flat (z = 0). Winding chicanes through warehouse racks.
       * Distance is ~55m compared to 34m on the ramp.
  4. **Static Obstacles**:
     * 4 Warehouse Storage Racks (1.2m × 8.0m × 2.5m).
     * 3 Cylindrical Structural Pillars creating narrow chokepoints.
  5. **Dynamic Agents (Gazebo Actors)**:
     * `pedestrian_crossing`: Animated actor traversing back and forth between (-12, -4) and (-12, +4) across the zig-zag corridor.
     * `cross_traffic_amr`: Secondary robot traversing perpendicular to the east corridor.
  6. **Simulation Plugins**:
     * `Physics`, `UserCommands`, `SceneBroadcaster`, and `Sensors` (Ogre-2 rendering engine).

### `navigrid_gazebo/config/ros_gz_bridge.yaml`
* **Purpose**: Configures high-performance C++ topic bridging between Gazebo Transport Protobuf messages and ROS 2 middleware messages.
* **Topic Mappings**:
  * `/clock` (`GZ_TO_ROS`): Synchronizes ROS 2 nodes with simulation time.
  * `/scan` (`GZ_TO_ROS`): `gz.msgs.LaserScan` -> `sensor_msgs/msg/LaserScan`.
  * `/imu` (`GZ_TO_ROS`): `gz.msgs.IMU` -> `sensor_msgs/msg/Imu`.
  * `/odom` (`GZ_TO_ROS`): `gz.msgs.Odometry` -> `nav_msgs/msg/Odometry`.
  * `/cmd_vel` (`ROS_TO_GZ`): `geometry_msgs/msg/Twist` -> `gz.msgs.Twist`.
  * `/joint_states` (`GZ_TO_ROS`): `gz.msgs.Model` -> `sensor_msgs/msg/JointState`.

### `navigrid_gazebo/launch/sim.launch.py`
* **Purpose**: Orchestrates the startup of Gazebo Harmonic, AMR model spawning, robot state publisher, bridge, and RViz.
* **Step Sequence**:
  1. Compiles Xacro to URDF in memory via `Command(['xacro ', xacro_path])`.
  2. Launches `gz sim -r navigrid_arena.sdf`.
  3. Executes `ros_gz_sim create` to instantiate `navigrid_amr` at Start Zone (-12, -12, 0.2).
  4. Runs `parameter_bridge` loading `ros_gz_bridge.yaml`.
  5. Launches RViz2 with `navigrid.rviz`.

---

## 4. Package: `navigrid_navigation`

### `navigrid_navigation/package.xml`, `setup.py`, & `setup.cfg`
* Configures the Python package according to ROS 2 `ament_python` standards.
* Declares console script entry points:
  * `costmap_generator`
  * `adaptive_global_planner`
  * `local_reactive_controller`
  * `safety_override_node`

### Parameter Files (`config/*.yaml`)
* **`planner_params.yaml`**:
  * `grid_resolution: 0.15`: Grid cell size in meters (200 × 200 cells for the 30m × 30m world).
  * `robot_radius: 0.45`: Obstacle inflation buffer radius.
  * `slope_cost_multiplier: 4.0`: Penalty weight for elevation ascent.
  * `payload_weight_kg: 25.0`: Current robot payload weight.
  * `max_incline_payload_limit: 35.0`: Payload threshold above which the ramp is classified as impassable.
* **`safety_params.yaml`**:
  * `safety_k: 1.25`: Braking distance quadratic coefficient (s²/m).
  * `safety_d_min: 0.65`: Static safety clearance buffer at zero velocity (m).
  * `detection_fov_deg: 150.0`: Forward angular detection cone (±75°).

---

### `navigrid_navigation/costmap_generator.py`
* **Purpose**: Stage 2 deliverable generating a coherent 2D/3D costmap published on `/costmap` (`nav_msgs/msg/OccupancyGrid`).
* **Algorithmic Breakdown**:
  1. `_build_static_arena()`:
     * Rasterizes boundary walls, warehouse storage racks, and structural pillars into an internal NumPy matrix (200 × 200, 8-bit integers).
     * Values: `0 = Free Space`, `100 = Lethal Obstacle`.
  2. **3D Elevation Slope Layer**:
     * Calculates the diagonal corridor equation: `|x - y| / sqrt(2) <= 1.3m`.|corridor equation: `|x - y| / sqrt(2) <= 1.3m`.|corridor equation: `|x - y| / sqrt(2) <= 1.3m`.
     * Assigns a continuous elevation cost to cells spanning the ramp based on the slope multiplier. This gives the planner a 3D cost-aware surface without requiring heavy point cloud voxels.
  3. `scan_callback()`:
     * Integrates live laser returns into dynamic obstacle points.
  4. `publish_costmap()`:
     * Merges static obstacles, inflated obstacle shells, dynamic laser points, and elevation costs, outputting standard ROS OccupancyGrid messages at 2 Hz.

---

### `navigrid_navigation/adaptive_global_planner.py`
* **Purpose**: Stage 2 deliverable that autonomously computes the optimal trajectory balancing terrain slope cost vs flat ground distance.
* **Mathematical Cost Function**:
  ```text
  Cost(path) = L + Cost_slope
  ```
  * For the **Zig-Zag corridor**: `Cost_slope = 0` => `Cost = 55.0m`.
  * For the **Direct Incline Ramp**:
    ```text
    Cost_slope = Delta_z * alpha * (m_payload / 15.0kg)
    ```
    Where `Delta_z = 2.8m` (total ascent + descent), `alpha = 4.0`.
* **Autonomous Decision Logic**:
  * If `m_payload <= 15kg` (Light Payload):
    `Cost_ramp = 34.0 + (2.8 * 4.0 * 1.0) = 45.2 < 55.0` => **Direct Ramp Selected!**
  * If `m_payload = 30kg` (Heavy Payload):
    `Cost_ramp = 34.0 + (2.8 * 4.0 * 2.0) = 56.4 > 55.0` => **Flat Zig-Zag Selected!**
  * If `m_payload > 35kg` (Overload Limit):
    `Cost_ramp = 10^6` => **Direct Ramp Deemed Impermissible, Zig-Zag Selected!**
* **Publishing**:
  Interpolates smooth, equidistant waypoints (spacing 0.3m) and publishes `nav_msgs/msg/Path` on `/plan`.

---

### `navigrid_navigation/local_reactive_controller.py`
* **Purpose**: Stage 3 deliverable executing real-time trajectory tracking while reactively dodging dynamic crossing pedestrians without stall loops.
* **Control Loop (20 Hz)**:
  1. **Pure Pursuit Path Following**:
     * Finds the closest point ahead of the robot at lookahead distance `L_look = 1.0m`.
     * Calculates attractive vector `v_att` pointing to the goal waypoint in the robot's local body frame.
  2. **Artificial Potential Field (APF) Reactive Avoidance**:
     * When dynamic actors (pedestrians) appear inside the repulsion zone (`r < 1.8m`), it computes repulsive force vectors:
       ```text
       v_rep = sum_i [ k_rep * (1/r_i - 1/d_rep) * (-p_i / r_i) ]
       ```
     * Sums the vectors: `v_total = v_att + v_rep`.
  3. **Steering & Speed Smoothing**:
     * Determines heading error `theta_err = atan2(v_y, v_x)`.
     * Angular velocity: `omega = clip(2.0 * theta_err, -omega_max, omega_max)`.
     * Linear speed scales with heading alignment: `v = v_max * max(0.2, cos(theta_err))`.
     * Outputs nominal velocity commands on `/cmd_vel_nav`.

---

### `navigrid_navigation/safety_override_node.py`
* **Purpose**: Stage 3 deliverable enforcing the official velocity-dependent safety formula:
  ```text
  d_safe = k * v² + d_min
  ```
* **Why this node is critical**:
  In industrial AMRs, safety must NEVER depend on a complex planner or controller that might lag or stall. The safety node is an independent, lightweight, high-priority arbiter running at 50 Hz.
* **Inner Working**:
  1. Subscribes to `/odom` to read forward linear speed `v`.
  2. Computes the dynamic safety envelope radius `d_safe`.
  3. Scans the forward 150° FOV of `/scan` for the nearest obstacle `d_obs`.
  4. **Arbitration Condition**:
     * If `d_obs < d_safe`:
       - **EMERGENCY STOP TRIGGERED!**
       - Immediately clamps `/cmd_vel` to (0, 0).
       - Publishes a RED warning cylinder marker in RViz.
       - Logs a high-priority warning message.
     * If `d_obs >= d_safe`:
       - Safe. Forwards `/cmd_vel_nav` directly to `/cmd_vel`.
       - Publishes a GREEN safety cylinder marker in RViz.
  5. Completely prevents physical collisions, protecting against the +10.0s penalty!

---

### `navigrid_navigation/launch/navigation.launch.py`
* Starts all four navigation nodes (`costmap_generator`, `adaptive_global_planner`, `local_reactive_controller`, `safety_override_node`) with their parameter YAML files.

---

### `navigrid_navigation/test/test_navigation_math.py`
* **Purpose**: Automated Pytest suite verifying:
  1. `d_safe` formula at rest (`v = 0` => `d_safe = 0.65m`).
  2. `d_safe` formula at speed (`v = 1.0 m/s` => `d_safe = 1.90m`).
  3. Light payload path selection (Ramp cost < ZigZag cost).
  4. Heavy payload path selection (Ramp cost > ZigZag cost).
  5. Overload limit (>35kg) enforcing immediate ramp rejection.

---

## 5. Package: `navigrid_bringup`

### `navigrid_bringup/launch/navigrid_all.launch.py`
* **Purpose**: Master launch file that combines `sim.launch.py` (Gazebo world + AMR + Bridge + RViz) and `navigation.launch.py` (Costmap + Global Planner + Local Controller + Safety Override).
* Allows full system evaluation with one command:
  ```bash
  ros2 launch navigrid_bringup navigrid_all.launch.py
  ```

---

## 6. Desktop Automation & Launchers

* **`NaviGrid_Workspace`** (`/home/aryan/Desktop/NaviGrid_Workspace`):
  Symlink pointing directly to `/home/aryan/navigrid_ws`.
* **`run_navigrid.sh`** (`/home/aryan/Desktop/run_navigrid.sh`):
  Shell script that sources `/opt/ros/lyrical/setup.bash`, builds the workspace if unbuilt, sources `install/setup.bash`, and runs `navigrid_all.launch.py`.
* **`navigrid-simulation.desktop`**:
  GUI launcher on Desktop to start the simulation in `xfce4-terminal` with one click.
* **`navigrid-workspace.desktop`**:
  GUI launcher opening the complete project in VS Code.
* **`navigrid-presentation.desktop`**:
  GUI launcher opening the presentation guide in VS Code.
