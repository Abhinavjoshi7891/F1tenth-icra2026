# 🌐 RoboRacer Master System Reference

This document provides a complete breakdown of every ROS 2 node in the RoboRacer stack, explaining its purpose and the parameters available for tuning.

---

## 🏎️ Infrastructure & Communication

### 1. `/bridge` (autodrive_bridge)
The communication gateway between ROS and the AutoDRIVE simulator.
*   **`ego_namespace`**: The prefix for your car's topics (usually `car_state`).
*   **`ego_scan_topic`**: The name of the LiDAR topic from the sim (usually `scan`).
*   **`use_sim_time`**: (Crucial) Must be **True** to synchronize with the simulator's clock.

### 2. `/global_parameters`
A static data bank providing configuration variables to the entire stack.
*   **`racecar_version`**: Switches between `AUTODRIVE`, `SIM`, or hardware `NUC` profiles.
*   **`track_length`**: The total length of the raceline in meters. Used for lap percentage calculations.

### 3. `/map_server`
Provides the occupancy grid (white=track, black=wall) for collision checking and localization.
*   **`yaml_filename`**: Path to the map configuration file.
*   **`frame_id`**: Usually set to `map`.

---

## 📍 Localization & State Estimation

### 4. `/frenet_odom_republisher`
Converts standard World Coordinates (X, Y) into Track Coordinates (S, D).
*   **Why use it?** It's easier to say "I am 10cm to the left of the path" than "I am at X: 124.5, Y: 10.2".
*   **`s_offset`**: Used to calibrate the starting point of the track.

### 5. `/ego_robot_state_publisher`
Publishes the physical "skeleton" (TF tree) of the car.
*   **`robot_description`**: Contains the URDF data (dimensions of the car, wheel positions).

---

## 🧠 Planning & Strategy

### 6. `/state_machine`
The high-level brain that decides how to race.
*   **`mode`**: Toggle between `Solo` (Time Trial) and `HeadToHead` (Racing).
*   **`ot_params`**: Parameters defining when it is "safe" to try an overtake.
*   **`dist_to_opponent_threshold`**: How close the car must be before switching to `TRAILING` mode.

### 7. `/spline_planner`
Generates a smooth local path between the track boundaries.
*   **`max_curvature`**: Limits how sharply the car is allowed to turn.
*   **`replan_frequency`**: How often (in Hz) the planner calculates a new curve. Higher is safer but uses more CPU.

### 8. `/ot_interpolator` (Overtake Interpolator)
Calculates the "steering around" path when the state machine triggers an overtake.
*   **`ot_width`**: The lateral distance (in meters) to veer away from the racing line during an overtake.
*   **`transition_length`**: How many meters the car should take to merge into the overtake lane.

---

## ⚙️ Control (The "Feet")

### 9. `/controller` (controller_manager)
*See [CONTROLLER_TUNING.md](CONTROLLER_TUNING.md) for a deep dive into these.*
*   **`m_l1`, `q_l1`**: Steering aggression.
*   **`t_clip_min`**: Turning radius floor.
*   **`lat_err_coeff`**: Braking for stabilization.

---

## 👁️ Perception & Safety

### 10. `/tracking` & `/detect`
Finds and monitors the opponent car using LiDAR.
*   **`min_obstacle_radius`**: Filter out small noise (like poles or cones) and only track car-sized objects.
*   **`detection_range`**: How far ahead (in meters) the car "looks" for opponents.

### 11. `/collision_detector`
The final safety net.
*   **`collision_threshold`**: Distance (in meters) to any obstacle before the car triggers an emergency stop.
*   **`use_scan_cleaning`**: If enabled, the car ignores its own "ghost" reflections from the LiDAR.

---

## 📈 Analytics

### 12. `/lap_analyser`
*   **`finish_line_x/y`**: The coordinates of the timing gate.
*   **`total_laps`**: Set your race distance target.

### 13. `/sector_tuner`
Divides the track into "Sectors" for specific behavior.
*   **`sector_id`**: Map-specific segments. You can set different speed limits for a "Safe Sector" (tight turn) vs an "Attack Sector" (straightaway).

---
*Created for ICRA 2026 Sim Racing League.*
