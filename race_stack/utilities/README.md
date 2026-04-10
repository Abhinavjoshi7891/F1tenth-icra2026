# race_stack/utilities — ROS2 Port Documentation

**Repository:** `Abhinavjoshi7891/F1Tenth-icra2026`  
**Source (ROS1):** `ForzaETH/race_stack` → `f110_utils/`  
**Target (ROS2):** `race_stack/utilities/`  
**Platform:** F1Tenth 1/10-scale autonomous racing — Jetson Orin Nano, Hokuyo LiDAR, ROS2 Humble  
**Competition target:** Physical F1Tenth race (ICRA 2026)

---

## Directory Structure

```
race_stack/utilities/
├── libraries/
│   ├── f110_msgs/          # Custom ROS2 message definitions
│   ├── frenet_conversion/  # Cartesian ↔ Frenet coordinate library
│   └── ccma/               # Curvature Corrected Moving Average smoother
├── nodes/
│   ├── frenet_odom_republisher/   # Publishes car state in Frenet coordinates
│   ├── lap_analyser/              # Lap timing and boundary distance metrics
│   ├── map_editor/                # Post-process Cartographer maps before planning
│   ├── opponent_publisher/        # Simulated opponent for testing
│   ├── sector_tuner/              # Per-sector velocity scaling
│   ├── slam_tuner/                # Offline Cartographer parameter tuning
│   └── param_optimizer/           # Bayesian optimisation of race parameters
└── scripts/
    └── pit_starter/               # Race-day pit laptop setup script
```

---

## Audit Process

This document records a complete package-by-package comparison between the original ForzaETH ROS1 `f110_utils` repository and the ROS2 port in `race_stack/utilities`. Each component was audited for:

1. Functional equivalence between ROS1 and ROS2 implementations
2. Bugs introduced or left unresolved during porting
3. Components that were obsolete, absorbed, or required fresh implementation
4. Actions required before physical race deployment

**Legend:**
- ✅ Fully ported — confirmed working
- ⚠️ Ported with known issues / requires verification
- 🔁 Replaced/absorbed — functionality covered differently in ROS2
- 🆕 New — no ROS1 equivalent, written from scratch for ROS2
- ❌ Not ported — deliberate decision, documented below

---

## Libraries

---

### `f110_msgs` ✅

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/libs/f110_msgs/` | `utilities/libraries/f110_msgs/` |
| Implementation | catkin, CMakeLists, message_generation | ament_cmake, identical `.msg` files |

**What it is:** Custom ROS2 message definitions that form the data contract between all stack modules.

**Why it matters:** Every node in the stack communicates using these types. Without them, nothing connects. A `Wpnt` (waypoint) carries position, heading, speed, curvature, and track-width information for a single point on the raceline. The planning stack publishes arrays of these; the controller consumes them.

**Messages:** `Wpnt`, `WpntArray`, `OTWpntArray`, `Obstacle`, `ObstacleArray`, `LapData`, `PidData`, `OpponentTrajectory`, `OpponentTrajectories`, `OppWpnt`, `ProjOppPoint`, `ProjOppTraj`, `GapData`, `CarState`, `CarStateStamped`

**Status:** Fully ported before this audit. All message types confirmed present and in use across the stack.

---

### `frenet_conversion` ✅

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/libs/frenet_conversion/` | `utilities/libraries/frenet_conversion/` |
| Implementation | C++ library (`frenet_conversion.cc`). Python wrapper was a TODO. Exposed 4 ROS services. | Pure Python. `FrenetConverter` class using `CubicSpline`. Imported directly — no service overhead. |

**What it is:** Coordinate transformation between Cartesian `(x, y)` and Frenet `(s, d)` frames, where `s` is arc-length along the raceline and `d` is lateral deviation from it.

**Why it matters:** The entire planning and perception stack operates in Frenet coordinates. Obstacles, overtaking zones, the car's position, and the controller error signal are all expressed as `(s, d)`. Without this library nothing in the stack can locate itself relative to the raceline.

**API (ROS2):**
```python
from frenet_conversion.frenet_converter import FrenetConverter

converter = FrenetConverter(waypoints_x, waypoints_y, waypoints_psi)
s, d = converter.get_frenet([x], [y])        # Cartesian → Frenet
x, y = converter.get_cartesian(s, d)         # Frenet → Cartesian
vs, vd = converter.get_frenet_velocities(vx, vy, theta)
```

**Used by:** `frenet_odom_republisher`, `carstate_node`, `spline_planner`, `detect.py` (perception), `param_optimizer`

**Status:** Fully ported before this audit. The ROS2 Python rewrite is functionally superior — cleaner API, no service serialization overhead, no C++ build dependency.

> **Note:** The ROS1 `frenet_conversion_server` (a standalone ROS service node) is intentionally absent in ROS2. All callers import `FrenetConverter` directly. Audit your codebase for any residual calls to `/convert_glob2frenetarr_service` — these will silently do nothing.

---

### `ccma` 🆕

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/libs/ccma/` (git submodule: `ForzaETH/ccma`) | `utilities/libraries/ccma/` (vendored plain source) |
| Implementation | Git submodule | Python source files copied directly. No `.git` folder. Installed via `colcon build`. |

**What it is:** Curvature Corrected Moving Average — a path smoothing algorithm for 2D closed curves. Developed by UniBwTAS (Steinecker & Wuensche, IEEE IV 2023).

**Why it matters:** The raw centerline extracted from a skeletonized occupancy grid map has pixel-level noise and artifacts. The IQP raceline optimizer requires a smooth, curvature-continuous input. The standard Savitzky-Golay filter overshoots at hairpin-to-straight transitions — exactly the geometry of F1Tenth competition tracks — because it uses uniform parametrization. CCMA uses centripetal arc-length parametrization which prevents inward bending at high-curvature zones.

**Validated result:** On the porto sim map, CCMA produced visibly cleaner raceline geometry through hairpin corners compared to SavGol. The orange trajectory (global raceline) showed better wall clearance in CCMA mode.

**Why vendored instead of pip:** `ccma` is not on PyPI. The only installation method is `pip install git+https://github.com/UniBwTAS/ccma.git`, which requires runtime internet access and leaves a `.git` artifact. Vendoring as plain source avoids both problems and makes the dependency fully reproducible.

**Installation:** Automatic via `colcon build --packages-select ccma`. No pip install needed.

**Usage in the stack:** Called inside `smooth_centerline()` in `global_planner/global_planner_utils.py`. Controlled by the `centerline_smoother` parameter in `global_planner_params.yaml`.

```yaml
# stack_master/config/global_planner/global_planner_params.yaml
centerline_smoother: 'ccma'   # 'ccma' (default) or 'savgol'
show_plots: False              # Set True during first mapping of a new track
                               # Saves maps/<name>/smoother_comparison.png
```

**Correct API:**
```python
from ccma.ccma import CCMA
ccma = CCMA(w_ma=7, w_cc=3)
smoothed = ccma.filter(pts, mode='wrapping')  # mode='wrapping' for closed race tracks
```

> **Important:** `is_closed=True` does NOT exist in this library. Use `mode='wrapping'`.

---

## Nodes

---

### `frenet_odom_republisher` ✅

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/nodes/frenet_odom_republisher/` | `utilities/nodes/frenet_odom_republisher/` |
| Implementation | C++ node | Python node |

**What it is:** Subscribes to `/car_state/odom` (Cartesian odometry), converts position and velocity to Frenet coordinates, republishes as `/car_state/frenet/odom` and `/car_state/frenet/pose`.

**Frenet encoding convention** (identical ROS1 → ROS2):

| Field | Meaning |
|---|---|
| `pose.pose.position.x` | `s` — arc-length along raceline [m] |
| `pose.pose.position.y` | `d` — lateral deviation from raceline [m] |
| `twist.twist.linear.x` | `vs` — longitudinal velocity [m/s] |
| `twist.twist.linear.y` | `vd` — lateral velocity [m/s] |

**Why it matters:** The state machine, controller, perception, and `lap_analyser` all need the car's position in Frenet coordinates continuously. This node centralises that conversion so no other node duplicates the work.

**Status:** Fully ported before this audit. Confirmed working.

---

### `lap_analyser` ⚠️ (bug fixed during audit)

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/nodes/lap_analyser/` | `utilities/nodes/lap_analyser/` |
| Implementation | Python node | Python node |

**What it is:** Monitors `/car_state/frenet/odom`, detects when the car crosses the start/finish line (`s` wraps from track_length → 0), and publishes:

- `/lap_data` (`f110_msgs/LapData`) — lap time, average lateral error, max lateral error. Published once per completed lap.
- `/min_car_distance_to_boundary` (`std_msgs/Float32`) — minimum distance from car to track wall recorded during that lap. Published once per completed lap.

**Why it matters:** Objective performance measurement. Lap time tells you if you're getting faster. Boundary distance tells you how close to a crash you are. Both are consumed by `param_optimizer` to compute the multi-objective cost function.

**Bug found and fixed during audit:**

In the ROS2 port, `publish_min_distance()` was commented out in `frenet_odom_cb()`, meaning `/min_car_distance_to_boundary` was never published despite the publisher existing. Additionally, the boundary distance was never being appended to `self.car_distance_to_boundary` during the lap.

```python
# ROS1 — correct:
self.publish_lap_info()
self.publish_min_distance()   # ← present

# ROS2 port — buggy:
self.publish_lap_info()
# self.publish_min_distance()  # ← commented out
```

**Fix applied:** Uncommented `publish_min_distance()`, added guard for empty list, and added boundary distance computation inside `frenet_odom_cb()` using `np.searchsorted` on the waypoint arc-length array.

**Status:** Bug fixed. Confirmed working — `/min_car_distance_to_boundary` publishing ~0.11m on porto sim map.

---

### `map_editor` ✅

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/nodes/map_editor/` | `utilities/nodes/map_editor/` |
| Implementation | ROS1 launch, Gazebo models | ROS2 launch XML, RViz2 config |

**What it is:** Post-processing tool for maps generated by Cartographer. Allows editing the `.png` occupancy grid (e.g. in GIMP) to remove phantom obstacles before running the global planner.

**Why it matters:** At competition venues, the track is crowded — other cars, spectators, equipment create ghost obstacles in the Cartographer map. Running the IQP raceline optimizer on a corrupted map produces a bad raceline. `map_editor` lets you clean the map before optimisation.

**Workflow:**
1. Run mapping launch (car drives a lap, Cartographer builds map)
2. Open `maps/<name>/<name>.png` in GIMP, paint out phantom obstacles
3. Run `map_editor_launch.xml` with `map_editor_mapping:=False` to compute the raceline on the cleaned map

**Status:** Fully ported before this audit. RViz2 config confirmed present with `SetInitialPose` tool.

---

### `obstacle_publisher` → `opponent_publisher` ✅

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/nodes/obstacle_publisher/` | `utilities/nodes/opponent_publisher/` |
| Implementation | Python + `dynamic_reconfigure` | Python + ROS2 parameter callbacks |

**What it is:** Publishes a simulated opponent as `ObstacleArray` on `/perception/obstacles`. The opponent moves along a configurable trajectory at a configurable speed profile.

**Parameters:**

| Parameter | Options | Description |
|---|---|---|
| `trajectory` | `min_curv`, `shortest_path`, `centerline` | Which raceline the opponent follows |
| `start_s` | `float` [m] | Starting position along track |
| `speed_type` | `const`, `raceline`, `sine`, `step` | Speed profile |
| `speed_scaler` | `float` [0-1] | Multiplier on raceline speed |
| `type` | `virtual`, `lidar` | Virtual = direct ObstacleArray injection. Lidar = paints into occupancy grid for perception to detect. |

**New in ROS2:** Includes `collision_detector` node (not in ROS1) that publishes `/opponent_collision` (Bool) and `/opponent_dist` (Float32) to detect ego-opponent contact.

**Usage:**
```bash
# Opponent at 50% raceline speed on min-curvature line
ros2 launch opponent_publisher opponent_publisher_launch.xml \
  speed_scaler:=0.5 trajectory:=min_curv type:=virtual

# Opponent at constant 3 m/s
ros2 launch opponent_publisher opponent_publisher_launch.xml \
  speed_type:=const speed:=3.0
```

**Status:** Fully ported before this audit. Renamed from `obstacle_publisher` to `opponent_publisher` — correct naming.

---

### `overtaking_sector_tuner` + `sector_tuner` → `sector_tuner` ✅

| | ROS1 | ROS2 |
|---|---|---|
| Location | Two separate packages | Single `utilities/nodes/sector_tuner/` package |
| Implementation | Python + `dynamic_reconfigure` | Python + ROS2 parameter events |

**What it is:** Manages per-sector velocity scaling on the global raceline. The track is divided into sectors (defined interactively during mapping), each with its own speed scaler. The `sector_tuner` applies smooth interpolation between sector boundaries and publishes `global_waypoints_scaled`.

**Four executables:**
- `sector_slicer` — GUI to define sector boundaries during mapping
- `ot_sector_slicer` — GUI to define overtaking sectors
- `sector_tuner` — runtime node, publishes `/global_waypoints_scaled`
- `ot_interpolator` — runtime node, publishes overtaking trajectory

**Parameters accessible at runtime:**
```bash
ros2 param set /sector_tuner Sector0_scaling 0.75
ros2 param get /sector_tuner Sector2_scaling
```

**n_sectors is track-specific.** Check `speed_scaling.yaml` in your map directory for the number of sectors defined during mapping:
```bash
cat stack_master/maps/<your_map>/speed_scaling.yaml | grep n_sectors
```

**Status:** Fully ported before this audit (consolidated from two ROS1 packages into one).

---

### `slam_tuner` ✅

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/nodes/slam_tuner/` | `utilities/nodes/slam_tuner/` |
| Implementation | ROS1 bagfile replay, `reconstruction_error.py` | ROS2 bag replay, same Python logic |

**What it is:** Offline Cartographer parameter tuning tool. Records a rosbag during a mapping session, then replays it with different Cartographer configurations to measure localisation accuracy without driving the car again.

**Why it matters:** Cartographer has ~15 tunable parameters (scan matcher weights, odometry trust, occupancy grid probabilities). These affect localisation accuracy which directly determines raceline tracking precision. `slam_tuner` lets you tune these offline before competition.

**Workflow:**
```bash
# Record bag while driving (on car)
ros2 bag record /scan /odom /sensors/imu/raw

# Tune offline (on development machine)
ros2 launch slam_tuner tuner_launch.xml \
  bag:=<bag_name> map_name:=<map> racecar_version:=NUC6 \
  mapping_bool:=True rviz:=True rate:=1.0
```

**Key Cartographer parameters to tune** (in `slam_tuner/config/f110_2d_loc.lua`):

| Parameter | Effect |
|---|---|
| `ceres_scan_matcher.rotation_weight` | Trust in odometry heading |
| `ceres_scan_matcher.translation_weight` | Trust in odometry translation |
| `hit_probability` | Trust in LiDAR returns |
| `miss_probability` | Trust in free space (reduce if black tubes/cones present) |

**Status:** Fully ported before this audit.

---

### `param_optimizer` 🆕 (replaces `bayesopt4ros` + ROS1 `param_optimizer`)

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/nodes/param_optimizer/` + `bayesopt4ros/` | `utilities/nodes/param_optimizer/` |
| Implementation | ROS1 actionlib + C++ BoTorch + `dynamic_reconfigure` | Pure Python + scikit-optimize GP + ROS2 SetParameters service |

**What it is:** Bayesian optimisation of race stack parameters using a Gaussian Process surrogate model. Automatically finds the best sector speed scalers (or MAP controller gains) for a specific track by driving laps and learning from the results.

**Why it matters:** Manual parameter search is slow and suboptimal. The GP-based search finds better parameters in fewer laps by building a probabilistic model of the parameter-to-performance mapping and targeting the most promising unexplored regions.

**Cost function (multi-objective):**
```
cost = w_time  × (lap_time / lap_time_ref)
     + w_crash × crash_penalty
     + w_lat   × (avg_lat_err / lat_err_ref)

crash_penalty = max(0, boundary_margin - min_dist) / boundary_margin
              = 0.0  when car stayed > boundary_margin from all walls
              = 1.0  when car touched a wall
```

**Modes:**

| Mode | Optimises | Targets |
|---|---|---|
| `sector_scalers` | Per-sector velocity scale factors [0.5, 1.0] | `/sector_tuner/set_parameters` |
| `map_params` | MAP controller gains | `/controller_manager/set_parameters` |

**Python dependency:** `pip install scikit-optimize` (added to `.devcontainer/.install_utils/python_req.txt`)

**Usage (competition workflow):**
```bash
# Safety-first: qualifying configuration
ros2 launch param_optimizer param_optimizer_launch.xml \
  mode:=sector_scalers \
  n_calls:=25 \
  n_random_starts:=8 \
  n_sectors:=<check_speed_scaling.yaml> \
  w_time:=1.0 \
  w_crash:=5.0 \
  w_lat:=0.3 \
  lap_time_ref:=<actual_expected_lap_time_on_this_track> \
  boundary_margin:=0.20 \
  results_path:=/tmp/<track_name>_bayesopt.json
```

**Important — sim vs hardware:**

In the F1Tenth gym simulator, sector scalers do not modulate lap time (the gym drives at a fixed speed regardless). Running `param_optimizer` in sim validates the infrastructure but does not produce meaningful scaler values. **Run this on hardware at the competition venue for genuine results.**

**After optimisation:** Take the best iteration from the JSON results file and bake it into `speed_scaling.yaml`:
```bash
cat /tmp/<track>_bayesopt.json | python3 -c "
import json,sys
data=json.load(sys.stdin)
best=min(data,key=lambda x:x['cost'])
print(f'Best iteration: {best["iteration"]}  Cost: {best["cost"]:.4f}')
for k,v in best['params'].items(): print(f'  {k}: {v:.4f}')
"
```

Then rebuild stack_master:
```bash
colcon build --packages-select stack_master --symlink-install
source install/setup.bash
```

**Status:** Written from scratch for ROS2. Validated in sim (infrastructure confirmed). Requires hardware run for meaningful parameter values.

---

## Scripts

---

### `pit_starter` 🆕 (ROS2 rewrite of ROS1 bash script)

| | ROS1 | ROS2 |
|---|---|---|
| Location | `f110_utils/scripts/pit_starter/pit_starter.sh` | `utilities/scripts/pit_starter/pit_starter.sh` |
| Implementation | Set `ROS_MASTER_URI`, `ROS_HOSTNAME`, `ROS_IP`. Launch `rosrun rviz`. | Set `ROS_DOMAIN_ID`. Source workspace. Verify node discovery. Launch `rviz2`. |

**What it is:** One-command pit lane setup for the pit laptop at a competition.

**Why it matters:** On race day under time pressure you need one command to connect your pit laptop to the car correctly. Wrong `ROS_DOMAIN_ID` = you see nothing from the car. Wrong RViz config = flying blind during a race.

**Usage:**
```bash
chmod +x utilities/scripts/pit_starter/pit_starter.sh

# Domain only (verify connectivity)
./utilities/scripts/pit_starter/pit_starter.sh 0 

# Domain + RViz2 with race config
./utilities/scripts/pit_starter/pit_starter.sh 0 rviz

# Domain + RViz2 + rqt
./utilities/scripts/pit_starter/pit_starter.sh 0 rvizrqt
```

**Finding the car's ROS_DOMAIN_ID:**
```bash
# On the car terminal
echo $ROS_DOMAIN_ID   # blank = 0 (ROS2 default)
```

**Network troubleshooting:**
```bash
ping <car_ip>                    # verify network connectivity first
sudo ufw allow 7400/udp          # allow DDS multicast (firewall)
```

**Key monitoring commands from pit:**
```bash
ros2 topic echo /lap_data                    # lap time + lateral error
ros2 topic hz /scan                          # LiDAR: should be ~40 Hz
ros2 topic hz /car_state/odom               # state estimation: should be ~80 Hz
ros2 topic echo /state_machine              # current state: GBTRACK/OT/FTG
ros2 topic echo /min_car_distance_to_boundary  # wall clearance
ros2 topic echo /optimizer/current_objective   # live cost during param_optimizer
```

---

## Components Not Ported — Decisions

| ROS1 Component | Decision | Reason |
|---|---|---|
| `frenet_conversion_server` | 🔁 Absorbed | ROS1 used a C++ service server for Frenet conversion. In ROS2, `FrenetConverter` is imported directly by nodes — no service overhead, no network round-trip. Audit for stale service call references. |
| `gb_traj_publisher` | 🔁 Replaced | Functionality moved to `global_planner/global_trajectory_publisher.py` which is a strict superset — also publishes shortest path, centerline, track bounds, estimated lap time. |
| `set_pose` | 🔁 Covered natively | ROS1 needed a node to trigger Cartographer relocalization. In ROS2, Cartographer natively accepts `/initialpose` (`PoseWithCovarianceStamped`). Use the **2D Pose Estimate** button in RViz2. |
| `tf_transformer` | 🔁 Obsolete | Converted `/tf` to odometry with angular velocity computation. In ROS2, `carstate_node.py` does this directly via `tf_buffer.lookup_transform()` at 80 Hz. Nobody subscribes to `/tf_odom`. |
| `transform_broadcaster` | 🔁 Covered | Broadcasted `odom→base_link` TF from odometry. In ROS2 this is done by `robot_localization` EKF when `publish_tf: true`. **Verify `ekf.yaml` has `publish_tf: true`** — if false the TF chain breaks and `carstate_node` fails at startup. |
| `car_to_car_sync` | ❌ Deferred | Streams car state between two cars over ZeroTier for multi-agent debugging. In competition, opponent detection uses LiDAR perception, not state sharing. Implement with `ros2 run domain_bridge` if needed. |
| `bayesopt4ros` | 🔁 Replaced | The full ROS1 system (C++ BoTorch + actionlib + catkin services) would take weeks to port faithfully with no benefit. Replaced by `param_optimizer` using `scikit-optimize` — same GP algorithm, cleaner ROS2 integration, simpler dependencies. |
| `random_obstacle_publisher` | ❌ Legacy | Marked as "TODO: legacy?" in the original ROS1 README. `opponent_publisher` covers all active obstacle simulation requirements. |
| `forza_helpers` | ⚠️ Verify | `id_to_rgb_color()` utility for opponent visualization marker colors. Check if opponents render with distinct colors in RViz2. If all one color, inline these two functions into `opponent_publisher/obstacle_publisher.py`. |
| `cpu_monitor` | ❌ Replaced | CPU/GPU monitoring via ROS topics. On Jetson, use `jtop` from the `jetson-stats` package instead — better integration with Jetson power management, GPU frequency, and thermal throttling. Install: `pip install jetson-stats`. |
| `pit_starter` (ROS1) | 🆕 Rewritten | ROS1 version set `ROS_MASTER_URI`. ROS2 uses `ROS_DOMAIN_ID` and DDS discovery — entirely different networking model. Rewritten from scratch. |

---

## Cross-Cutting Change — Dual Centerline Smoother

This change is in `planner/global_planner/` but is documented here because it directly depends on the `ccma` library in `utilities/libraries/`.

**What changed:** `smooth_centerline()` in `global_planner/global_planner_utils.py` now supports two smoothers selectable at runtime via `centerline_smoother` in `global_planner_params.yaml`. When `show_plots: True`, both smoothers run and a side-by-side comparison PNG is saved to the map directory.

**Why:** SavGol (the original smoother) overshoots at hairpin-to-straight transitions on competition tracks. CCMA handles this correctly. The dual-smoother design allows validation of CCMA output before committing to it on a new track.

**Files changed:**
- `stack_master/config/global_planner/global_planner_params.yaml` — added `centerline_smoother: 'ccma'`
- `planner/global_planner/global_planner/global_planner_node.py` — reads and passes `centerline_smoother` parameter
- `planner/global_planner/global_planner/global_planner_logic.py` — accepts and stores `centerline_smoother`, passes to `smooth_centerline()`
- `planner/global_planner/global_planner/global_planner_utils.py` — `smooth_centerline()` refactored to support both modes

**Switching smoothers** (no rebuild required — runtime parameter):
```yaml
# global_planner_params.yaml
centerline_smoother: 'ccma'    # recommended for competition tracks
# centerline_smoother: 'savgol' # fallback
```

---

## Hardware Race Day Checklist

### Before Leaving for the Venue

- [ ] `colcon build` passes cleanly — zero errors across all packages
- [ ] `pip install scikit-optimize` confirmed in Docker container
- [ ] `python3 -c "from ccma.ccma import CCMA; print('OK')"` passes in container
- [ ] `ekf.yaml` has `publish_tf: true` — if false the TF chain breaks on hardware
- [ ] Map for the competition venue already generated — `global_waypoints.json` exists in `maps/<track>/`
- [ ] `speed_scaling.yaml` and `ot_sectors.yaml` present in the map directory
- [ ] `pit_starter.sh` is executable: `chmod +x utilities/scripts/pit_starter/pit_starter.sh`
- [ ] ROS_DOMAIN_ID agreed upon between car and pit laptop (default: 0)

### Verify TF chain before autonomous driving:
```bash
ros2 run tf2_tools view_frames
# Must show: map → odom → base_link
```

### At the Venue — Startup Sequence

**On the car:**
```bash
./.docker_utils/main_attach_dock.sh
cd ~/ws && source install/setup.bash
ros2 launch stack_master base_system_launch.xml \
  racecar_version:=<NUCX> sim:=False map_name:=<track_name>
```

**On the pit laptop:**
```bash
source /opt/ros/humble/setup.bash
./race_stack/utilities/scripts/pit_starter/pit_starter.sh 0 rviz
```

**Checklist before enabling autonomous driving:**
- [ ] `/scan` visible in RViz2 and updating — LiDAR working
- [ ] `/car_state/pose` arrow visible on map — Cartographer localised
- [ ] If pose arrow is in wrong location: click **2D Pose Estimate** in RViz2, click and drag to set correct position and heading. Cartographer relocates within 1-2 seconds. Do this before enabling autonomous driving.
- [ ] `ros2 topic hz /scan` → ~40 Hz
- [ ] `ros2 topic hz /car_state/odom` → ~80 Hz
- [ ] `ros2 topic echo /lap_data` → receiving after first lap

### Pre-Race Parameter Tuning (if time permits)

```bash
# On the car, with base_system and time_trials running:
ros2 launch param_optimizer param_optimizer_launch.xml \
  mode:=sector_scalers \
  n_calls:=25 \
  n_random_starts:=8 \
  n_sectors:=<from_speed_scaling.yaml> \
  w_time:=1.0 \
  w_crash:=5.0 \
  w_lat:=0.3 \
  lap_time_ref:=<measured_lap_time_from_lap_data> \
  boundary_margin:=0.20 \
  results_path:=/tmp/<track>_bayesopt.json
```

After completion, take best params from JSON and update `speed_scaling.yaml`.

### Weight guidance for `param_optimizer`:

| Competition phase | `w_time` | `w_crash` | `w_lat` | `boundary_margin` |
|---|---|---|---|---|
| Qualifying (safety first) | 1.0 | 5.0 | 0.3 | 0.20m |
| Head-to-head (balanced) | 1.5 | 3.0 | 0.5 | 0.15m |
| Push for fastest lap | 2.0 | 1.0 | 0.5 | 0.12m |

Set `lap_time_ref` to your actual measured lap time on that track — it is a normalisation reference, not a goal. Setting it too low inflates the cost and skews relative weights.

---

## Quick Reference — Topic Map

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/car_state/frenet/odom` | `nav_msgs/Odometry` | `frenet_odom_republisher` | `lap_analyser`, `spline_planner`, `state_machine` |
| `/lap_data` | `f110_msgs/LapData` | `lap_analyser` | `param_optimizer` |
| `/min_car_distance_to_boundary` | `std_msgs/Float32` | `lap_analyser` | `param_optimizer` |
| `/global_waypoints` | `f110_msgs/WpntArray` | `global_trajectory_publisher` | `sector_tuner`, `frenet_odom_republisher`, `controller` |
| `/global_waypoints_scaled` | `f110_msgs/WpntArray` | `sector_tuner` | `controller`, `state_machine` |
| `/perception/obstacles` | `f110_msgs/ObstacleArray` | `detect.py` or `opponent_publisher` | `state_machine`, `spline_planner` |
| `/optimizer/current_objective` | `std_msgs/Float32` | `param_optimizer` | monitoring only |

---

## Docker Setup

All Python dependencies not available as apt packages are managed in:

```
.devcontainer/.install_utils/python_req.txt
```

The following were added during the ROS2 port:
- `scikit-optimize` — required by `param_optimizer`

`ccma` is installed via `colcon build` (not pip) because it is not on PyPI.

To rebuild the Docker image with updated dependencies:
```bash
docker compose build nuc
```

To install in the currently running container without rebuilding:
```bash
pip install scikit-optimize
```

---

*Last updated: April 2026 — ICRA 2026 competition preparation*
