# ROS 1 to ROS 2 Race Stack Comparison Report

This document provides a detailed comparison between the original ROS 1 stack at `/Data_Drive/ros1_race_stack` and the current ROS 2 implementation in `forzaeth/race_stack`.

## Executive Summary
The porting process is approximately **75-80% complete**. The core navigation, planning, and control logic has been successfully ported and reorganized. The remaining work primarily involves specialized utilities, specific state estimation modules (Particle Filter), and legacy perception trackers.

---

## 1. Planner Mapping
The ROS 2 stack has reorganized planners into `global_planner` and `local_planners`.

| Component | ROS 1 (Source) | ROS 2 (Target) | Status | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Frenet Planner** | `frenet-planner` | `local_planners/frenet_planner` | ✅ Ported | Clean structure in ROS 2. |
| **Graph Based Planner** | `graph_based_planner` | `local_planners/graph_based_planner`| ✅ Ported | Port plan documented in workspace. |
| **Predictive Spliner** | `predictive-spliner` | `local_planners/predictive_planner` | ✅ Ported | Renamed for consistency. |
| **Spline Planner** | `spliner` | `local_planners/spline_planner` | ✅ Ported | |
| **Global Path Opt** | `gb_optimizer` | `global_planner` | ✅ Ported | Consolidated into global_planner. |
| **Multi-Opponent** | `multiopponent-pspliner` | - | ❌ **Left to Port** | Nodes: `sqp_avoidance_node.py`, `gp_node_multiprocessing.py`. |
| **Misc Utils** | `misc` | `misc` | ✅ Ported | Common math/helper libs. |

---

## 2. Controller Mapping
The ROS 2 controller package consolidates multiple Python scripts that were previously in separate folders.

| Component | ROS 1 (Source) | ROS 2 (Target) | Status | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Manager** | `controller_manager.py` | `controller_manager.py` | ✅ Ported | Updated to rclpy. |
| **Follow the Gap** | `ftg` | `ftg.py` | ✅ Ported | Converted to single node file. |
| **Pure Pursuit** | `pp` | `pp.py` | ✅ Ported | |
| **Kinematic MPC** | `mpc` | `kinematic_mpc/` | ✅ Ported | Expanded into sub-package. |
| **Single Track MPC** | - | `single_track_mpc/` | 🆕 Ported | New high-fidelity controller. |
| **Tuning Server** | `online_tuning_server` | `online_tuning_server.py` | ✅ Ported | |

---

## 3. Perception Mapping
ROS 2 uses a consolidated `perception` package instead of multiple standalone packages.

| Component | ROS 1 (Source) | ROS 2 (Target) | Status | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Center Speed** | `TinyCenterSpeed` | `centerspeed_node.py` | ✅ Ported | Uses ONNX/TensorRT. |
| **Norfair Tracker** | `norfair_tracker` | `norfair_node.py` | ✅ Ported | Modernized logic. |
| **ABD Tracker** | `abd_tracker` | `tracking.py` | ⚠️ Partially Ported | `tracking.py` seems to consolidate ABD logic. |
| **Detection** | - | `detect.py` | 🆕 Ported | New unified detection node. |

---

## 4. State Estimation Mapping
This is the area with the most significant remaining work.

| Component | ROS 1 (Source) | ROS 2 (Target) | Status | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Car State Node** | `src/carstate.py` | `carstate_node.py` | ✅ Ported | |
| **Particle Filter** | `particle_filter` | - | ❌ **Left to Port** | Main node: `synPF.py`. Uses dynamic server. |
| **Cartographer** | `cartographer_pbl` | `autodrive_adapter/` | ⚠️ Integrated | Ported config to adapter logic. |

---

## 5. State Machine Mapping
The state machine has been significantly refactored for better maintainability in ROS 2.

| Component | ROS 1 (Source) | ROS 2 (Target) | Status | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Core Logic** | `src/` (Combined) | `state_machine.py` | ✅ Ported | Better separation of concerns. |
| **Transitions** | Hardcoded | `transitions.py` | ✅ Ported | Extracted logic. |
| **State Definitions** | - | `states.py` | ✅ Ported | Defined as distinct classes. |
| **Indicator** | `state_indicator` | - | ❌ **Left to Port** | Visual/LED indicator logic. |

---

## 6. Utilities & f110_utils
Many legacy ROS 1 utilities are being evaluated for necessity.

| Component | ROS 1 (Source) | ROS 2 (Target) | Status | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Frenet Conversion** | `frenet_conversion_server` | `libraries/frenet_conversion` | ✅ Ported | Converted from Server to Library. |
| **Lap Analyser** | `lap_analyser` | `nodes/lap_analyser` | ✅ Ported | |
| **Map/Race Editor** | `map_editor` | `nodes/map_editor` | ✅ Ported | Consolidated logic. |
| **Opponent Pub** | `obstacle_publisher` | `nodes/opponent_publisher`| ✅ Ported | Renamed for clarity. |
| **CPU Monitor** | `cpu_monitor` | - | ❌ **Left to Port** | Resource tracking. |
| **BayesOpt Tuning** | `bayesopt4ros` | - | ❌ **Left to Port** | Automatic tuning framework. |
| **TF Helpers** | `tf_transformer` | - | ❌ **Left to Port** | Likely handled by standard tf2. |

---

## 7. System Identification
Porting here is focused on the core models.

| Component | ROS 1 (Source) | ROS 2 (Target) | Status | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **ID Controller** | `id_controller` | `nodes/id_controller` | ✅ Ported | |
| **Steering Lookup** | `steering_lookup` | `steering_lookup/` | ✅ Ported | |
| **On-Track SysID** | `on_track_sys_id` | - | ❌ **Left to Port** | Real-time ID logic. |
| **Analyser** | `id_analyser` | - | ❌ **Left to Port** | Post-processing scripts. |

---

## Remaining Tasks Summary

### Priority 1: Localization & Estimation
*   [ ] **Port `particle_filter`**: Critical for real-world racing and some simulation scenarios.
*   [ ] **Finalize `cartographer` integration**: Ensure bridge/config is consistent with ROS 1 performance.

### Priority 2: Advanced Planners
*   [ ] **Port `multiopponent-pspliner`**: Required for multi-car aggressive overtaking.

### Priority 3: Tuning & Monitoring
*   [ ] **Port `bayesopt4ros` interface**: Critical for parameter optimization.
*   [ ] **Port `cpu_monitor`**: Needed for hardware performance validation.

### Priority 4: Visuals & Metadata
*   [ ] **Port `state_indicator`**: Minor but helpful for debugging vehicle status.
