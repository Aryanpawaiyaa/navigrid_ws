# NaviGrid Robotics Masterclass: Complete Learning & Concept Guide

This guide is designed to take you from foundational concepts to advanced robotics engineering so you can thoroughly understand, explain, and defend every design decision in the **NaviGrid AMR Navigation Stack**.

---

## 1. Differential Drive Kinematics & Physical Modeling

### How the Robot Moves
Our AMR uses a **Differential Drive** configuration with two independently driven wheels of radius R = 0.10 m separated by track width L = 0.48 m, plus two passive casters for 4-point stability.

Given commanded linear velocity `v` (m/s) and angular velocity `omega` (rad/s), the individual wheel speeds are:

```text
omega_left  = (v - (omega * L) / 2) / R
omega_right = (v + (omega * L) / 2) / R
```

### Why Passive Casters Matter on the Incline Ramp
On a flat plane, a 3-point contact (2 wheels + 1 caster) works. But when climbing a **11.3° incline ramp**, a single front or rear caster can cause the chassis to tip backwards or high-center at the summit transition!
* **Our Solution**: We placed **dual casters** (front at +0.22 m, rear at -0.22 m). As the robot climbs, the rear caster prevents backward tipping, and as it crowns the summit at (0, 0, 1.4), the front caster prevents bottoming out.

---

## 2. Coordinate Frames & The TF2 Tree

In ROS 2 robotics, every sensor reading and movement command lives in a coordinate frame:

| Frame Name | Description | Origin / Anchor |
| :--- | :--- | :--- |
| **`odom`** | World-fixed reference frame estimated via wheel encoders & IMU. Continuous, non-jumping. | Robot starting position at (-12, -12, 0). |
| **`base_footprint`** | 2D projection of the robot center onto the ground plane (z = 0). | Under the robot chassis on the floor. |
| **`base_link`** | Rigid center of mass of the robot chassis (z = +0.10 m). | Chassis center. |
| **`lidar_link`** | Origin of the laser scan emitter (x = +0.18 m, z = 0.245 m). | Top of front sensor bracket. |
| **`imu_link`** | Origin of the inertial measurement unit (z = 0.22 m). | Embedded in the chassis center. |

> **Interview / Judge Tip**: If asked *"Why do you need both `base_footprint` and `base_link`?"*, answer:
> *"Costmaps and 2D footprint collision checkers operate on the 2D floor projection (`base_footprint`), while 3D dynamics, sensor offsets, and inertia moments must be modeled at the physical center of mass (`base_link`). Decoupling them avoids projecting sensor heights erroneously into ground collision checks."*

---

## 3. Perception: Converting Polar Laser Scans to Cartesian Space

The LIDAR returns an array of ranges `r_i` at angle increments `delta_theta`:

```text
theta_i = theta_min + i * delta_theta
```

In [local_reactive_controller.py](file:///home/aryan/navigrid_ws/src/navigrid_navigation/navigrid_navigation/local_reactive_controller.py) and [safety_override_node.py](file:///home/aryan/navigrid_ws/src/navigrid_navigation/navigrid_navigation/safety_override_node.py), each return inside the detection window is converted to Cartesian coordinates in the robot's local frame:

```text
x_obs = r_i * cos(theta_i)
y_obs = r_i * sin(theta_i)
```

This allows direct Euclidean distance calculation and vector repulsion without heavy trigonometric loops.

---

## 4. Costmaps & Configuration Space (C-Space)

In [costmap_generator.py](file:///home/aryan/navigrid_ws/src/navigrid_navigation/navigrid_navigation/costmap_generator.py):
* **Lethal Obstacles (C = 100)**: Exact physical walls, racks, and pillars.
* **Inflation Zone**: Any cell within `robot_radius = 0.45m` of an obstacle is assigned a high cost. By inflating obstacles by the robot's bounding radius, the planning algorithm can treat the AMR as a single point, dramatically simplifying path computation.
* **Elevation Slope Layer**: The diagonal strip where the ramp resides is annotated with an additional slope cost `C_slope = 25 * alpha`.

---

## 5. Stage 2: Cost-Aware Global Path Planning

The central problem of Stage 2 is: **Which path is optimal — the direct incline ramp (34m) or the winding zig-zag corridor (55m)?**

The objective function is:

```text
J(path) = integral_path (1 + alpha(m_payload) * S(s)) ds
```

### The Trade-off Dynamics:
1. **Travel Distance (L)**:
   * Direct Ramp: `L_ramp ≈ 34 m` (straight Euclidean diagonal).
   * Zig-Zag: `L_zigzag ≈ 55 m` (corridors winding around storage racks).
2. **Elevation Cost**:
   * Zig-Zag: Entirely flat (`z = 0`), so `S(s) = 0` → `Cost = 55.0`.
   * Ramp: Involves an ascent of 1.4 m and descent of 1.4 m (`delta_z_total = 2.8 m`):
     ```text
     Cost_slope = delta_z_total * alpha * (m_payload / 15 kg)
     ```
3. **Autonomous Switching Behavior**:
   * **Light Payload (15 kg)**:
     `Cost = 34.0 + (2.8 * 4.0 * 1.0) = 45.2 < 55.0` → **Direct Ramp is chosen!**
   * **Heavy Payload (30 kg)**:
     `Cost = 34.0 + (2.8 * 4.0 * 2.0) = 56.4 > 55.0` → **Flat Zig-Zag is chosen!**
   * **Overloaded Payload (> 35 kg)**:
     Exceeds motor torque limits on an 11.3° slope; cost is set to `1,000,000` → **Ramp strictly prohibited!**

---

## 6. Stage 3: Reactive Obstacle Avoidance (No Stall Loops)

In dynamic warehouse environments, pedestrians cross paths unpredictably. Traditional global planners often trigger "recovery behaviors" (spinning in circles) that lead to stall loops.

### Our Solution: Artificial Potential Fields (APF)
In [local_reactive_controller.py](file:///home/aryan/navigrid_ws/src/navigrid_navigation/navigrid_navigation/local_reactive_controller.py):
1. **Attractive Force (F_att)**: Pulls the robot along the global path toward a waypoint 1.0 m ahead.
2. **Repulsive Force (F_rep)**: Any dynamic obstacle within 1.8 m pushes the robot perpendicularly away:
   ```text
   F_rep = k_rep * (1/r - 1/d_rep) * (-p_obs / r)
   ```
3. **Smooth Blended Trajectory**: The resultant vector `F = F_att + F_rep` smoothly steers the AMR around the moving actor. As soon as the actor crosses, `F_rep → 0`, and the AMR naturally converges back onto the nominal path without stopping!

---

## 7. Stage 3: High-Priority Safety Override Node

### The Competition Safety Formula

```text
d_safe = k * v² + d_min
```

* `k = 1.25`: Represents the inverse of twice the available braking deceleration (`k = 1 / (2 * a_max)`).
* `v`: Current forward linear velocity from wheel odometry.
* `d_min = 0.65 m`: Physical chassis safety margin + sensor latency buffer.

### Why Decoupling Safety into a Separate Node is Best Practice
* In ROS 2 navigation stacks, path planning and obstacle prediction algorithms can experience processing spikes (e.g. 50-200ms latency).
* If safety logic is embedded inside the planner, a sudden pedestrian intrusion during a latency spike could cause a collision (+10s penalty).
* **Our Architecture**: The `safety_override_node` runs as an independent, lightweight node at **50 Hz (20 ms cycle)**. It intercepts `/cmd_vel_nav` and directly controls `/cmd_vel`. If `d_obs < d_safe`, it overrides commands instantly and forces `v = 0`!

---

## 8. Judge Q&A Cheatsheet: How to Answer Like a Pro

### Q1: "How did you ensure you won't get disqualified under the Colcon build rule?"
> *"We strictly adhere to ROS 2 package guidelines. All dependencies are explicitly declared in `package.xml` and registered in `CMakeLists.txt` and `setup.py`. We eliminated all compiler warnings and verified clean builds using `colcon build --symlink-install` in under 1.5 seconds."*

### Q2: "How does the robot decide between the ramp and the zig-zag corridor?"
> *"Our `adaptive_global_planner` evaluates an objective functional J = integral (1 + alpha * S(s)) ds. While the ramp is 21 meters shorter, it incurs continuous elevation penalties scaled by the robot's payload weight. Under light payloads (15kg), the total ramp cost is 45.2 vs 55.0 for the zig-zag, so it takes the ramp. Under heavy payloads (30kg), the ramp cost rises to 56.4, so the planner autonomously switches to the flat ground floor."*

### Q3: "What prevents the robot from colliding with the dynamic pedestrian?"
> *"We implement defense-in-depth: Level 1 is our 20Hz reactive controller that computes artificial repulsive fields to steer around pedestrians in real time. Level 2 is our 50Hz high-priority safety override node enforcing d_safe = k * v² + d_min. If the pedestrian cuts into the safety envelope faster than the controller can steer, the safety node immediately commands an emergency stop."*

### Q4: "How is simulation time synchronized?"
> *"We bridge `/clock` from Gazebo Sim to ROS 2 via `ros_gz_bridge` and set `use_sim_time: true` on all nodes, ensuring deterministic timing regardless of CPU load."*
