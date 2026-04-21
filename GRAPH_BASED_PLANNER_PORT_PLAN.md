# Graph-Based Local Trajectory Planner — ROS 1 → ROS 2 Port Plan

> Faithful port. No optimisation. No refactors. Match ROS 1 behaviour 1:1. Only ROS-binding layer changes.

---

## 0. Glossary / absolute paths

| Alias          | Path                                                                                           |
| -------------- | ---------------------------------------------------------------------------------------------- |
| `ROS1_ROOT`    | `/home/abhinav/Data_Drive/ros1_race_stack`                                                     |
| `ROS2_ROOT`    | `/home/abhinav/Data_Drive/forzaeth`                                                            |
| `ROS1_GBP`     | `ROS1_ROOT/planner/graph_based_planner`                                                        |
| `ROS2_LP`      | `ROS2_ROOT/race_stack/planner/local_planners`                                                  |
| `ROS2_GBP`     | `ROS2_LP/graph_based_planner` (to be created)                                                  |
| `ROS2_SM`      | `ROS2_ROOT/race_stack/state_machine/state_machine`                                             |
| `ROS2_STACK`   | `ROS2_ROOT/race_stack/stack_master`                                                            |
| `FRENET_REF`   | `ROS2_LP/frenet_planner` (template/reference — mimic its layout)                               |
| `UPSTREAM`     | `https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner`                                   |

---

## 1. What the ROS 1 planner does (one-paragraph mental model)

`GraphPlanner` wraps TUM's `graph_ltpl` python lib. On startup it loads a pre-generated `stored_graph.pckl` (offline graph built from the raceline CSV), then in a 30 Hz loop:

1. Reads `/global_waypoints` (once), `/car_state/odom`, `/car_state/odom_frenet`, `/perception/obstacles`, `/dyn_sector_server/parameter_updates` (speed scaling).
2. Builds a `traj_ltpl_cl_<map>.csv` if missing.
3. Calls `ltpl_obj.calc_paths(prev_action_id, object_list)` then `ltpl_obj.calc_vel_profile(...)`.
4. Picks an action ∈ {`straight`, `left`, `right`, `follow`} via cost, flattens the chosen (s,x,y,psi,kappa,vx,ax) path and publishes `std_msgs/Float32MultiArray` on `/planner/graph_based_wpnts` (with action label in `layout.dim[0].label`).
5. Publishes `/lattice_viz` (MarkerArray) + `/action_marker` (Marker).

Downstream consumer = **state machine**. In ROS 1, `state_machine_node.py` subscribes to `/planner/graph_based_wpnts`, reshapes the Float32MultiArray, and in `Overtaking()` returns `get_graph_based_wpts()` as local `WpntArray`.

`path_publisher.py` in ROS 1 is a **parallel** node listening to `/planner_path` and publishing `/local_waypoints`. **It is NOT used in the head-to-head flow** (state machine owns `/local_waypoints`). Only used inside the standalone `planner.launch`. **→ We port it for parity, but it's optional.**

---

## 2. What already exists in ROS 2

1. `head_to_head_launch.xml` already branches on `planner == 'graph_based'` and `include`s `$(find-pkg-share graph_based_planner)/launch/graph_based_planner_launch.xml`. **We must honour that filename.**
2. State machine (`ROS2_SM/`) already has spliner / predictive_spliner / frenet branches. **Graph-based branch is MISSING** — must be added.
3. `f110_msgs`, `frenet_conversion`, perception `/perception/obstacles` topic, `/global_waypoints`, `/car_state/odom`, `/car_state/odom_frenet` — **all exist** (verified).
4. `ax_max_machines.csv` is at `ROS2_STACK/config/global_planner/veh_dyn_info/ax_max_machines.csv` — planner will read from there (we parameterise the path).
5. `sector_tuner` exists (ROS 2 equivalent of `dyn_sector_server`). It publishes a `global_limit` param — we will **read via a parameter-event listener** (not a topic subscription to `parameter_updates`).
6. `frenet_planner` ROS 2 package = the reference template for our package layout (`ament_python`, `setup.py`, `config/*.yaml`, `launch/*.launch.py` + `*_launch.xml`).

---

## 3. What does NOT exist / must be created

1. ROS 2 package `graph_based_planner` under `ROS2_LP/`.
2. Vendored upstream library **`graph_ltpl`** as a subpackage inside the ROS 2 package (pure-python, no ROS). The ROS 1 code already has a working copy — **do NOT re-clone**, just copy the subtree.
3. Speed-scaling bridge: use `rclpy.parameter` events from node `/sector_tuner` instead of `/dyn_sector_server/parameter_updates`.
4. State machine: add `graph_based` branch (params + subscriber + state fns + transitions).
5. Config YAML for the node's tunables (the ROS 1 code hardcodes `v_max=15`, `trailing_distance=1`, `examined_points=80`). Keep defaults identical.

---

## 4. Topic/param mapping table (ROS 1 → ROS 2)

| ROS 1                                             | ROS 2                                              | Notes                                                         |
| ------------------------------------------------- | -------------------------------------------------- | ------------------------------------------------------------- |
| `/global_waypoints` (WpntArray)                   | `/global_waypoints` (WpntArray)                    | same                                                          |
| `/car_state/odom` (Odometry)                      | `/car_state/odom` (Odometry)                       | same                                                          |
| `/car_state/odom_frenet` (Odometry)               | `/car_state/odom_frenet` (Odometry)                | same                                                          |
| `/perception/obstacles` (ObstacleArray)           | `/perception/obstacles` (ObstacleArray)            | same                                                          |
| `/dyn_sector_server/parameter_updates` (Config)   | param event on node `/sector_tuner`, key `global_limit` | use `rclpy` parameter callback/async param client        |
| `/planner/graph_based_wpnts` (Float32MultiArray)  | `/planner/graph_based_wpnts` (Float32MultiArray)   | keep identical payload for state machine                      |
| `/lattice_viz` (MarkerArray)                      | `/lattice_viz` (MarkerArray)                       | same                                                          |
| `/action_marker` (Marker)                         | `/action_marker` (Marker)                          | same                                                          |
| `/planner/graph_based/latency` (Float32)          | `/planner/graph_based/latency` (Float32)           | only when `measure:=True`                                     |
| rosparam `/racecar_version`                       | node param `racecar_version`                       | pass via launch                                               |
| rosparam `/GraphPlanner/map_name`                 | node param `map_name`                              | pass via launch                                               |
| rosparam `/GraphPlanner/display_graph`            | node param `display_graph`                         |                                                               |
| rosparam `/GraphPlanner/follow_mode`              | node param `follow_mode`                           |                                                               |
| rosparam `/GraphPlanner/testing`                  | drop (always True on car)                          | sim-auto-detect path not used in head_to_head                 |
| rosparam `/measure`                               | node param `measure`                               |                                                               |
| rosparam `/profile`                               | node param `profile`                               |                                                               |
| rospkg `graph_based_planner`                      | `ament_index_python.get_package_share_directory`   |                                                               |

---

## 5. Package layout to create

```
race_stack/planner/local_planners/graph_based_planner/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── graph_based_planner
├── config/
│   └── graph_based_planner_params.yaml
├── launch/
│   ├── graph_based_planner_launch.xml        # wrapper — matches head_to_head include
│   └── graph_based_planner.launch.py         # actual launch
├── inputs/                                    # runtime data dir (populated at build via data_files)
│   ├── stored_graph.pckl                      # copied from ROS 1
│   ├── traj_ltpl_cl/                          # will be filled on first run per map
│   └── veh_dyn_info/
│       └── ax_max_machines.csv                # copied from ROS 1
├── params/                                    # graph_ltpl *.ini (NOT ros params)
│   ├── driving_task.ini
│   ├── ltpl_config_offline.ini
│   └── ltpl_config_online.ini
├── logs/                                      # runtime logs dir (empty, shipped as dir)
│   └── graph_ltpl/
│       └── Graph_Objects/
│           └── .gitkeep
└── graph_based_planner/                       # ament_python module
    ├── __init__.py
    ├── graph_planner_node.py                  # port of ROS1 Planner.py
    ├── path_publisher_node.py                 # port of ROS1 path_publisher.py (optional)
    └── graph_ltpl/                            # vendored pure-python lib (verbatim copy)
        ├── __init__.py
        ├── Graph_LTPL.py
        ├── data_objects/
        ├── helper_funcs/
        ├── imp_global_traj/
        ├── offline_graph/
        ├── online_graph/
        ├── testing_tools/
        └── visualization/
```

---

## 6. Step-by-step port

### Step 1 — Create package skeleton

```bash
mkdir -p /home/abhinav/Data_Drive/forzaeth/race_stack/planner/local_planners/graph_based_planner/{config,launch,resource,graph_based_planner,inputs/traj_ltpl_cl,inputs/veh_dyn_info,params,logs/graph_ltpl/Graph_Objects}
cd /home/abhinav/Data_Drive/forzaeth/race_stack/planner/local_planners/graph_based_planner
touch resource/graph_based_planner
touch graph_based_planner/__init__.py
touch logs/graph_ltpl/Graph_Objects/.gitkeep
```

### Step 2 — Copy vendored library verbatim (pure python, no ROS deps)

```bash
# from ROS1 to ROS2 (skip docs, tests, examples, logs, LICENSE, setup.py of upstream)
cp -r /home/abhinav/Data_Drive/ros1_race_stack/planner/graph_based_planner/src/GraphBasedPlanner/graph_ltpl \
      /home/abhinav/Data_Drive/forzaeth/race_stack/planner/local_planners/graph_based_planner/graph_based_planner/graph_ltpl
```

> The import path will change from `GraphBasedPlanner.graph_ltpl.*` (ROS 1) → `graph_based_planner.graph_ltpl.*` (ROS 2). **No edits needed inside `graph_ltpl/`** — it only does relative intra-package imports. Only the **node file** needs its imports updated (Step 5).

### Step 3 — Copy static assets

```bash
GBP_SRC=/home/abhinav/Data_Drive/ros1_race_stack/planner/graph_based_planner/src/GraphBasedPlanner
GBP_DST=/home/abhinav/Data_Drive/forzaeth/race_stack/planner/local_planners/graph_based_planner

# Graph params (.ini configs read by graph_ltpl)
cp $GBP_SRC/params/driving_task.ini          $GBP_DST/params/
cp $GBP_SRC/params/ltpl_config_offline.ini   $GBP_DST/params/
cp $GBP_SRC/params/ltpl_config_online.ini    $GBP_DST/params/

# Pre-generated graph pickle + vehicle dyn csv
cp $GBP_SRC/inputs/stored_graph.pckl                          $GBP_DST/inputs/
cp $GBP_SRC/inputs/veh_dyn_info/ax_max_machines.csv           $GBP_DST/inputs/veh_dyn_info/

# Map-specific trajectory CSVs (if any exist from ROS 1 use)
cp -r $GBP_SRC/inputs/traj_ltpl_cl/. $GBP_DST/inputs/traj_ltpl_cl/ 2>/dev/null || true
```

### Step 4 — Write package manifest files

**File: `race_stack/planner/local_planners/graph_based_planner/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>graph_based_planner</name>
  <version>0.0.1</version>
  <description>Graph-based local trajectory planner — faithful ROS 2 port of ROS 1 graph_based_planner (wraps TUMFTM graph_ltpl)</description>
  <maintainer email="abhinav@forzaeth.com">Abhinav</maintainer>
  <license>Apache-2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>f110_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-scipy</exec_depend>
  <exec_depend>python3-matplotlib</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**File: `race_stack/planner/local_planners/graph_based_planner/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/graph_based_planner
[install]
install_scripts=$base/lib/graph_based_planner
```

**File: `race_stack/planner/local_planners/graph_based_planner/setup.py`**

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'graph_based_planner'

# Recursively collect all data files inside inputs/, params/, logs/
def collect(base_dir, install_root):
    pairs = []
    for root, _dirs, files in os.walk(base_dir):
        if not files:
            continue
        rel = os.path.relpath(root, base_dir)
        inst = os.path.join(install_root, rel) if rel != '.' else install_root
        pairs.append((inst, [os.path.join(root, f) for f in files]))
    return pairs

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
]
data_files += collect('params',  os.path.join('share', package_name, 'params'))
data_files += collect('inputs',  os.path.join('share', package_name, 'inputs'))
data_files += collect('logs',    os.path.join('share', package_name, 'logs'))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    # graph_ltpl is pure python — ship it with the package
    include_package_data=True,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav',
    maintainer_email='abhinav@forzaeth.com',
    description='Graph-based local trajectory planner — ROS 2 port',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_planner_node   = graph_based_planner.graph_planner_node:main',
            'path_publisher_node  = graph_based_planner.path_publisher_node:main',
        ],
    },
)
```

**File: `race_stack/planner/local_planners/graph_based_planner/resource/graph_based_planner`**

Empty file (already created via `touch` in Step 1).

### Step 5 — Port the main node

> **Strategy:** copy ROS 1 `Planner.py` logic verbatim, swap rospy/rospkg/tf imports for rclpy equivalents, keep every function, threshold, and branch identical. Keep identical topic names & payload layout so state machine port is trivial.

**File: `race_stack/planner/local_planners/graph_based_planner/graph_based_planner/graph_planner_node.py`**

```python
#!/usr/bin/env python3
"""Graph-Based Local Trajectory Planner — ROS 2 port (faithful, no behavioural changes).

Port of ROS 1 ``Planner.py``. Wraps TUMFTM graph_ltpl. Publishes a flattened
Float32MultiArray with action label so the state machine can consume it
exactly like the ROS 1 version.
"""
import os
import sys
import time
import uuid
import hashlib
import cProfile
import pstats
from math import sin, cos

os.environ['OPENBLAS_NUM_THREADS'] = "1"

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float32
from f110_msgs.msg import WpntArray, ObstacleArray

# vendored, pure-python
from graph_based_planner.graph_ltpl import Graph_LTPL
from graph_based_planner.graph_ltpl.imp_global_traj.src import import_globtraj_csv  # noqa: F401


def _yaw_from_quat(x, y, z, w):
    # Replaces tf.transformations.euler_from_quaternion (yaw only)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    import math
    return math.atan2(siny_cosp, cosy_cosp)


class GraphPlanner(Node):
    def __init__(self):
        super().__init__('GraphPlanner')

        # --- Declare ROS params ---
        self.declare_parameter('map_name', '')
        self.declare_parameter('display_graph', True)
        self.declare_parameter('follow_mode', 0)
        self.declare_parameter('racecar_version', 'NUC2')
        self.declare_parameter('measure', False)
        self.declare_parameter('profile', False)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('v_max', 15.0)
        self.declare_parameter('trailing_distance', 1.0)
        self.declare_parameter('examined_points', 80)
        self.declare_parameter('waypoint_distance', 0.1)
        # optional override for veh dyn csv location (defaults to stack_master global_planner copy)
        self.declare_parameter(
            'ax_max_machines_path',
            '',  # if empty we fall back to package-internal copy
        )

        self.display_graph     = self.get_parameter('display_graph').value
        self.follow_mode       = self.get_parameter('follow_mode').value
        self.map_name          = self.get_parameter('map_name').value
        self.racecar_version   = self.get_parameter('racecar_version').value
        self.measuring         = self.get_parameter('measure').value
        self.profile           = self.get_parameter('profile').value
        self.rate_hz           = self.get_parameter('rate_hz').value
        self.v_max             = self.get_parameter('v_max').value
        self.trailing_distance = self.get_parameter('trailing_distance').value
        self.examined_points   = self.get_parameter('examined_points').value
        self.waypoint_distance = self.get_parameter('waypoint_distance').value

        # --- State ---
        self.waypoints = None
        self.waypoints_original = None
        self.current_position = None
        self.current_position_frenet = None
        self.current_heading = None
        self.current_speed = None
        self.obstacle_arr = np.array([])
        self.speed_scaling = 1.0          # set by sector_tuner listener (stub — see Step 7)
        self.max_s = None
        self.wp_flag = False
        self.close_to_s0 = False
        self.num_waypoints = 0

        # Paths
        pkg_share = get_package_share_directory('graph_based_planner')
        self.toppath = pkg_share  # inputs/, params/, logs/ live here
        self.path_dict = {
            'globtraj_input_path':      os.path.join(self.toppath, 'inputs', 'traj_ltpl_cl', f'traj_ltpl_cl_{self.map_name}.csv'),
            'graph_store_path':         os.path.join(self.toppath, 'inputs', 'stored_graph.pckl'),
            'ltpl_offline_param_path':  os.path.join(self.toppath, 'params', 'ltpl_config_offline.ini'),
            'ltpl_online_param_path':   os.path.join(self.toppath, 'params', 'ltpl_config_online.ini'),
        }

        ax_path_param = self.get_parameter('ax_max_machines_path').value
        if ax_path_param:
            ax_max_path = ax_path_param
        else:
            ax_max_path = os.path.join(self.toppath, 'inputs', 'veh_dyn_info', 'ax_max_machines.csv')
        self.ax_max = np.loadtxt(ax_max_path, delimiter=',', skiprows=1)

        # Profiling
        self.profiler = cProfile.Profile()
        self.profile_filename        = os.path.join(self.toppath, 'logs', 'graph_based_profile_log.prof')
        self.profile_report_filename = os.path.join(self.toppath, 'logs', 'graph_based_profile_report.txt')

        # Subscribers
        self.create_subscription(WpntArray,      '/global_waypoints',       self.waypoints_cb,  10)
        self.create_subscription(Odometry,       '/car_state/odom',         self.odom_cb,       10)
        self.create_subscription(Odometry,       '/car_state/odom_frenet',  self.odom_fre_cb,   10)
        self.create_subscription(ObstacleArray,  '/perception/obstacles',   self.obstacles_cb,  10)

        # Publishers
        self.lattice_visualizer = self.create_publisher(MarkerArray, '/lattice_viz',                 10)
        self.action_mrk         = self.create_publisher(Marker,      '/action_marker',               10)
        self.path_pub           = self.create_publisher(Float32MultiArray, '/planner/graph_based_wpnts', 10)
        if self.measuring:
            self.latency_pub    = self.create_publisher(Float32, '/planner/graph_based/latency', 10)

        # Speed scaling listener (replaces /dyn_sector_server/parameter_updates).
        # See Step 7 — we attach a rclpy AsyncParametersClient on /sector_tuner and poll via timer.
        self._setup_speed_scaling_listener()

        self.ltpl_obj = None
        self._initialised = False

        # Kick off the main loop timer AFTER we have all data (gated via has_all_data()).
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self._prev_action = 'straight'
        self._sel_action = 'straight'
        self.visualize_action('straight')

        if self.profile:
            self.profiler.enable()

    # ----------------------- CALLBACKS -------------------------
    def obstacles_cb(self, data: ObstacleArray):
        if len(data.obstacles) == 0:
            self.obstacle_arr = np.array([])
            return
        self.obstacle_arr = np.array([
            [o.id, o.s_start, o.s_end, o.d_left, o.d_right, o.is_actually_a_gap,
             o.s_center, o.d_center, o.size, o.vs, o.vd, o.is_static] for o in data.obstacles
        ])

    def odom_cb(self, data: Odometry):
        p = data.pose.pose.position
        q = data.pose.pose.orientation
        self.current_position = np.array([p.x, p.y])
        self.current_speed    = data.twist.twist.linear.x
        self.current_heading  = _yaw_from_quat(q.x, q.y, q.z, q.w)

    def odom_fre_cb(self, data: Odometry):
        p = data.pose.pose.position
        self.current_position_frenet = np.array([p.x, p.y])

    def waypoints_cb(self, data: WpntArray):
        if self.wp_flag:
            return
        self.waypoints_original = np.array([
            [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left,
             w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2] for w in data.wpnts
        ])
        rows = []
        for w in data.wpnts:
            base = self.frenettoglob(w.s_m, 0.0)
            tip  = self.frenettoglob(w.s_m, -1.0)
            norm_vec = self.unit_vector(np.array([tip[0] - base[0], tip[1] - base[1]]))
            rows.append([w.x_m, w.y_m, w.d_right, w.d_left,
                         norm_vec[0], norm_vec[1], 0.0, w.s_m,
                         w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2])
        self.waypoints = np.asarray(rows)
        self.num_waypoints = len(self.waypoints)
        self.wp_flag = True

    # ----------------------- HELPERS (verbatim port) ------------
    # NOTE: unit_vector, angle_between, find_global_index, frenettoglob, globtofrenet,
    #       get_closest_obs, get_obj_list, fromfrenet_to_cartesian, visualize_action,
    #       visualize_lattice, traj_cost, choose_action, send_path_topic, create_map_csv
    #       — COPY THESE VERBATIM from ROS1 Planner.py (lines ~247-722). Only changes:
    #           * replace rospy.Time.now()  →  self.get_clock().now().to_msg()
    #           * replace rospy.loginfo(...)/logwarn(...) → self.get_logger().info(...)/warn(...)
    #           * replace rospy.Publisher.publish → self.<pub>.publish
    #       Do not alter any math, thresholds, or indexing.

    # (See file copy instructions in Section 7 below.)

    # ----------------------- MAIN TICK --------------------------
    def _has_all_data(self):
        return (self.waypoints is not None
                and self.current_position is not None
                and self.current_position_frenet is not None)

    def _ensure_initialised(self):
        if self._initialised:
            return
        if not self._has_all_data():
            return
        self.max_s = float(np.amax(self.waypoints_original[:, 0]))
        if not self.create_map_csv():
            self.get_logger().info('[Graph Based Planner] Found existing trajectory file for this map!')
        else:
            self.get_logger().info('[Graph Based Planner] Created new trajectory file for this map!')

        self.ltpl_obj = Graph_LTPL.Graph_LTPL(path_dict=self.path_dict,
                                              visual_mode=False,
                                              log_to_file=False)
        self.ltpl_obj.graph_init(veh_param_mass=3.5)

        if self.display_graph:
            graph_container = self.ltpl_obj._Graph_LTPL__graph_base  # name-mangled attr, same as ROS1
            self.visualize_lattice(graph_container)

        self.ltpl_obj.set_startpos(pos_est=self.current_position,
                                   heading_est=self.current_heading + np.pi / 2)
        self._initialised = True
        self.get_logger().info('[Graph Based Planner] Initialised — entering main loop.')

    def _tick(self):
        self._ensure_initialised()
        if not self._initialised:
            return

        t0 = time.perf_counter()
        self.close_to_s0 = (self.current_position_frenet[0] < 0.5
                            or self.current_position_frenet[0] > self.max_s - 0.5)
        self._prev_action = self._sel_action

        obj_list = self.get_obj_list()
        self.ltpl_obj.calc_paths(prev_action_id=self._sel_action, object_list=obj_list)
        traj_set = self.ltpl_obj.calc_vel_profile(
            pos_est=self.current_position,
            vel_est=self.current_speed,
            vel_max=self.v_max,
            ax_max_machines=self.ax_max,
            safety_d=self.trailing_distance,
        )[0]
        self._sel_action = self.choose_action(traj_set)

        if self.measuring:
            self.latency_pub.publish(Float32(data=float(time.perf_counter() - t0)))

        if self._sel_action != self._prev_action:
            self.visualize_action(self._sel_action)

        self.send_path_topic(traj_set, self._sel_action)

    # ----------------------- SPEED SCALING (sector_tuner) -------
    def _setup_speed_scaling_listener(self):
        # Lazy: poll /sector_tuner's `global_limit` parameter @ 2 Hz.
        # Matches ROS1 behaviour of reading the first double from param updates.
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import GetParameters
        self._param_cli = self.create_client(GetParameters, '/sector_tuner/get_parameters')
        self.create_timer(0.5, self._poll_speed_scaling)

    def _poll_speed_scaling(self):
        from rcl_interfaces.srv import GetParameters
        if not self._param_cli.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['global_limit']
        fut = self._param_cli.call_async(req)
        fut.add_done_callback(self._speed_scaling_done)

    def _speed_scaling_done(self, fut):
        try:
            res = fut.result()
            if res and res.values:
                v = res.values[0]
                # ParameterType: double=3
                if v.type == 3:
                    self.speed_scaling = float(v.double_value)
        except Exception:
            pass  # keep previous value

    # ----------------------- SHUTDOWN ---------------------------
    def on_shutdown(self):
        if self.profile:
            self.profiler.disable()
            self.profiler.dump_stats(self.profile_filename)
            with open(self.profile_report_filename, 'w') as fh:
                pstats.Stats(self.profiler, stream=fh).sort_stats('cumtime').print_stats()


def main(args=None):
    rclpy.init(args=args)
    node = GraphPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> **IMPORTANT (file completion):** The helpers listed under “HELPERS (verbatim port)” must be pasted from ROS 1. See Section 7 for the exact copy block — **do not re-implement**, paste verbatim and do only the listed substitutions.

### Step 6 — Port `path_publisher.py` (optional parity node)

**File: `race_stack/planner/local_planners/graph_based_planner/graph_based_planner/path_publisher_node.py`**

```python
#!/usr/bin/env python3
"""Parity port of ROS 1 path_publisher.py.

NOTE: Not used in head_to_head (state machine owns /local_waypoints).
Ship it for feature parity with ROS 1 standalone planner.launch.
Do NOT launch it alongside the state machine — they fight for /local_waypoints.
"""
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from f110_msgs.msg import WpntArray, Wpnt


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.declare_parameter('path_length', 80)
        self.declare_parameter('loop_rate_hz', 80.0)

        self.path_length  = self.get_parameter('path_length').value
        self.loop_rate_hz = self.get_parameter('loop_rate_hz').value

        self.selected_path = None
        self.sel_action = None
        self.frenet_pose = None
        self.waypoints_original = None
        self.max_s = None
        self.close_to_s0 = False
        self.speed_scaling = 1.0

        self.create_subscription(WpntArray,          '/global_waypoints',       self.waypoint_cb,      10)
        self.create_subscription(Float32MultiArray,  '/planner_path',           self.planner_path_cb,  10)
        self.create_subscription(Odometry,           '/car_state/odom_frenet',  self.odom_fre_cb,      10)

        self.cmd_pub  = self.create_publisher(WpntArray,   '/local_waypoints',         10)
        self.wpnt_viz = self.create_publisher(MarkerArray, '/local_waypoints/markers', 10)

        self.create_timer(1.0 / self.loop_rate_hz, self._tick)

    def waypoint_cb(self, data: WpntArray):
        self.waypoints_original = np.array([
            [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left,
             w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2] for w in data.wpnts
        ])
        self.max_s = float(np.amax(self.waypoints_original[:, 0]))

    def odom_fre_cb(self, data: Odometry):
        p = data.pose.pose.position
        self.frenet_pose = np.array([p.x, p.y])

    def planner_path_cb(self, data: Float32MultiArray):
        arr = np.asarray(data.data)
        self.selected_path = arr.reshape(data.layout.dim[0].size, data.layout.dim[1].size)
        self.sel_action = data.layout.dim[0].label

    def _find_nearest_s(self, station, arr):
        return int(np.abs(arr - station).argmin())

    def _tick(self):
        if (self.selected_path is None or self.waypoints_original is None
                or self.frenet_pose is None or self.max_s is None):
            return

        self.close_to_s0 = (self.frenet_pose[0] < 0.5
                            or self.frenet_pose[0] > self.max_s - 0.5)

        wpt_arr = WpntArray()
        mrk_arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        wpt_arr.header.stamp = now
        wpt_arr.header.frame_id = 'map'

        path = self.selected_path
        for i, coord in enumerate(path[:self.path_length, :]):
            w = Wpnt()
            w.s_m = (coord[0] + self.frenet_pose[0]) % self.max_s
            w.x_m = coord[1]; w.y_m = coord[2]
            w.d_m = 0.0
            w.psi_rad    = coord[3]
            w.kappa_radpm= coord[4]
            w.vx_mps     = coord[5] * self.speed_scaling
            w.ax_mps2    = coord[6]
            w.id = i
            idx = self._find_nearest_s(w.s_m, self.waypoints_original[:, 0])
            w.d_left  = self.waypoints_original[idx, 5] - w.d_m
            w.d_right = self.waypoints_original[idx, 4] + w.d_m
            wpt_arr.wpnts.append(w)

            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = now
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.a = 1.0; m.color.g = 1.0
            m.id = i
            m.pose.position.x = w.x_m; m.pose.position.y = w.y_m; m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            mrk_arr.markers.append(m)

        self.cmd_pub.publish(wpt_arr)
        self.wpnt_viz.publish(mrk_arr)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 7 — Copy-verbatim helper block

Open `ROS1_GBP/src/Planner.py` in ROS 1 (already read in plan prep). Copy the full bodies of these methods — **no edits to logic**, only the 3 substitutions in the IMPORTANT note of Step 5:

| Method                       | ROS 1 lines |
| ---------------------------- | ----------- |
| `set_map_name`               | 196–207 — **replace with** `self.map_name = self.get_parameter('map_name').value`. Drop entirely if already set in `__init__`. |
| `create_map_csv`             | 209–245     |
| `visualize_lattice`          | 247–356     |
| `get_closest_obs`            | 358–393     |
| `find_global_index`          | 395–408     |
| `unit_vector`                | 410–423     |
| `angle_between`              | 425–442     |
| `get_obj_list`               | 444–487     |
| `fromfrenet_to_cartesian`    | 489–514     |
| `visualize_action`           | 516–563     |
| `send_path_topic`            | 565–591     |
| `traj_cost`                  | 593–622     |
| `globtofrenet`               | 624–647     |
| `frenettoglob`               | 649–671     |
| `choose_action`              | 673–720     |

Substitutions inside each:

- `rospy.Time.now()`                    → `self.get_clock().now().to_msg()`
- `rospy.loginfo(...)`                  → `self.get_logger().info(...)`
- `rospy.logwarn(...)`                  → `self.get_logger().warn(...)`
- `self.lattice_visualizer.publish(x)`  → keep as-is (we already created matching publisher)
- `self.action_mrk.publish(x)`          → keep
- `self.path_pub.publish(x)`            → keep

Do **not** touch the Marker field assignments, colour thresholds, `type = 2` / `type = 4` magic numbers, or array indexing — those are intentional in ROS 1.

### Step 8 — Config YAML (defaults match ROS 1 hardcoded values)

**File: `race_stack/planner/local_planners/graph_based_planner/config/graph_based_planner_params.yaml`**

```yaml
GraphPlanner:
  ros__parameters:
    map_name: ""
    display_graph: true
    follow_mode: 0            # 0 brakes often, 2 aggressive follow
    racecar_version: "NUC2"
    measure: false
    profile: false
    rate_hz: 30.0
    v_max: 15.0
    trailing_distance: 1.0
    examined_points: 80
    waypoint_distance: 0.1
    ax_max_machines_path: ""  # empty → use package-internal inputs/veh_dyn_info/ax_max_machines.csv

path_publisher:
  ros__parameters:
    path_length: 80
    loop_rate_hz: 80.0
```

### Step 9 — Launch files

**File: `race_stack/planner/local_planners/graph_based_planner/launch/graph_based_planner.launch.py`**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('graph_based_planner')
    default_cfg = os.path.join(pkg_share, 'config', 'graph_based_planner_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('graph_based_config', default_value=default_cfg),
        DeclareLaunchArgument('map_name',        default_value=''),
        DeclareLaunchArgument('display_graph',   default_value='True'),
        DeclareLaunchArgument('follow_mode',     default_value='0'),
        DeclareLaunchArgument('racecar_version', default_value='NUC2'),
        DeclareLaunchArgument('measure',         default_value='False'),
        DeclareLaunchArgument('profile',         default_value='False'),

        Node(
            package='graph_based_planner',
            executable='graph_planner_node',
            name='GraphPlanner',
            output='screen',
            parameters=[
                LaunchConfiguration('graph_based_config'),
                {
                    'map_name':        LaunchConfiguration('map_name'),
                    'display_graph':   LaunchConfiguration('display_graph'),
                    'follow_mode':     LaunchConfiguration('follow_mode'),
                    'racecar_version': LaunchConfiguration('racecar_version'),
                    'measure':         LaunchConfiguration('measure'),
                    'profile':         LaunchConfiguration('profile'),
                },
            ],
        ),
    ])
```

**File: `race_stack/planner/local_planners/graph_based_planner/launch/graph_based_planner_launch.xml`**

```xml
<launch>
    <arg name="map_name"        default=""/>
    <arg name="display_graph"   default="True"/>
    <arg name="follow_mode"     default="0"/>
    <arg name="racecar_version" default="NUC2"/>
    <arg name="measure"         default="False"/>

    <include file="$(find-pkg-share graph_based_planner)/launch/graph_based_planner.launch.py">
        <arg name="map_name"        value="$(var map_name)"/>
        <arg name="display_graph"   value="$(var display_graph)"/>
        <arg name="follow_mode"     value="$(var follow_mode)"/>
        <arg name="racecar_version" value="$(var racecar_version)"/>
        <arg name="measure"         value="$(var measure)"/>
    </include>
</launch>
```

### Step 10 — Wire into head-to-head launch

The branch already exists at `ROS2_STACK/launch/head_to_head_launch.xml:32-35`. It needs `map_name` to be forwarded. **Edit** that file:

**Replace**

```xml
    <group if="$(eval '\'$(var planner)\' == \'graph_based\'')">
        <!-- Assuming graph_based_planner package exists -->
        <include file="$(find-pkg-share graph_based_planner)/launch/graph_based_planner_launch.xml" />
    </group>
```

**with**

```xml
    <group if="$(eval '\'$(var planner)\' == \'graph_based\'')">
        <include file="$(find-pkg-share graph_based_planner)/launch/graph_based_planner_launch.xml">
            <arg name="map_name"        value="$(var map_name)"/>
            <arg name="display_graph"   value="True"/>
            <arg name="follow_mode"     value="0"/>
            <arg name="racecar_version" value="$(var racecar_version)"/>
            <arg name="measure"         value="$(var measure)"/>
        </include>
    </group>
```

> If `head_to_head_launch.xml` currently does not declare `map_name` as a `<arg>`, add `<arg name="map_name" default="hangar_1905_v0"/>` near the top alongside `racecar_version`. Also verify `racecar_version` is declared; if not, add `<arg name="racecar_version" default="NUC2"/>`.

### Step 11 — State machine integration

All edits are in `ROS2_SM/`.

#### 11.1 — `state_machine.py`

**(a) Subscriber + state variables** — inside `__init__`, in the `head_to_head` block (just below the frenet subscription, around line 106):

```python
            # Graph-based planner waypoints (flattened Float32MultiArray on /planner/graph_based_wpnts)
            self.graph_based_wpts = None         # np.ndarray, shape (N, 7): s, x, y, psi, kappa, vx, ax
            self.graph_based_action = None       # str: 'straight'/'left'/'right'/'follow'
            self.gb_wpnts_arr = None             # cached np view of glb_wpnts for d_left/d_right lookup
            if self.params.overtake_mode == "graph_based":
                from std_msgs.msg import Float32MultiArray
                self.create_subscription(Float32MultiArray, '/planner/graph_based_wpnts',
                                         self.graph_based_cb, 10)
```

**(b) Callback** — add next to `frenet_wpnts_cb`:

```python
    def graph_based_cb(self, data):
        import numpy as np
        arr = np.asarray(data.data)
        if len(arr) == 0 or len(data.layout.dim) < 2:
            self.graph_based_wpts = None
            return
        self.graph_based_wpts = arr.reshape(data.layout.dim[0].size, data.layout.dim[1].size)
        self.graph_based_action = data.layout.dim[0].label
```

**(c) Cache `gb_wpnts_arr`** — inside the existing global_waypoints callback (find the method that stores `self.glb_wpnts`), append after it's stored:

```python
        if self.params.overtake_mode == "graph_based":
            import numpy as np
            self.gb_wpnts_arr = np.array([
                [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left,
                 w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2] for w in self.glb_wpnts
            ])
```

**(d) New property `_check_availability_graph_wpts`** — place beside `_check_availability_splini_wpts`:

```python
    @property
    def _check_availability_graph_wpts(self) -> bool:
        if self.graph_based_wpts is None:
            return False
        return len(self.graph_based_wpts) > 0
```

**(e) Helper `get_graph_based_wpts`** — faithful port of ROS 1 lines 791-816. Place next to `get_splini_wpts`:

```python
    def get_graph_based_wpts(self):
        import numpy as np
        from f110_msgs.msg import WpntArray, Wpnt
        arr = WpntArray()
        if self.graph_based_wpts is None or self.gb_wpnts_arr is None:
            return arr
        for i, coord in enumerate(self.graph_based_wpts[:self.params.n_loc_wpnts, :]):
            w = Wpnt()
            w.s_m = (coord[0] + self.cur_s) % self.track_length
            w.x_m = coord[1]; w.y_m = coord[2]
            w.d_m = 0.0
            w.psi_rad     = coord[3]
            w.kappa_radpm = coord[4]
            w.vx_mps      = coord[5]
            w.ax_mps2     = coord[6]
            w.id = i
            idx = int(np.abs(self.gb_wpnts_arr[:, 0] - w.s_m).argmin())
            w.d_left  = self.gb_wpnts_arr[idx, 5] - w.d_m
            w.d_right = self.gb_wpnts_arr[idx, 4] + w.d_m
            arr.wpnts.append(w)
        return arr
```

**(f) `_check_ofree` extension** — add a graph-based branch before the final `else`:

```python
        elif self.params.overtake_mode == "graph_based":
            # Graph-based owns the action label; trust it. If the planner publishes a
            # valid overtake path at all, consider o_free True.
            return self.graph_based_wpts is not None and len(self.graph_based_wpts) > 0
```

#### 11.2 — `states.py` — extend `Overtaking`

```python
def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
    if state_machine.params.overtake_mode == "frenet":
        # ... unchanged frenet branch ...
        ...
    elif state_machine.params.overtake_mode == "graph_based":
        graph_wpts = state_machine.get_graph_based_wpts().wpnts
        if graph_wpts:
            return list(graph_wpts)
        # fallback to global
        s = int(state_machine.cur_s / state_machine.waypoints_dist + 0.5)
        return [state_machine.glb_wpnts[(s + i) % state_machine.num_glb_wpnts]
                for i in range(state_machine.params.n_loc_wpnts)]
    else:
        # ... unchanged spliner / predictive_spliner branch ...
        ...
```

> The other states (`GlobalTracking`, `Trailing`, `FTGOnly`) need no changes — they pull from `glb_wpnts` regardless of planner.

#### 11.3 — `transitions.py` — add GB transition functions

Append below the FRENET block, then wire them into `head_to_head_transition`.

```python
###########################
# GRAPH-BASED TRANSITIONS #
###########################
def GBGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    return StateType.GB_TRACK if state_machine._check_gbfree else StateType.TRAILING


def GBTrailingTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    gb_free   = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector
    graph_ok  = state_machine._check_availability_graph_wpts
    if not gb_free and not ot_sector:
        return StateType.TRAILING
    elif gb_free and state_machine._check_close_to_raceline:
        return StateType.GB_TRACK
    elif not gb_free and ot_sector and graph_ok:
        return StateType.OVERTAKE
    return StateType.TRAILING


def GBOvertakingTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    ot_sector      = state_machine._check_ot_sector
    graph_ok       = state_machine._check_availability_graph_wpts
    enemy_in_front = state_machine._check_enemy_in_front
    if ot_sector and graph_ok and enemy_in_front:
        return StateType.OVERTAKE
    return StateType.GB_TRACK


def GBFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    return StateType.GB_TRACK if state_machine._check_close_to_raceline else StateType.FTGONLY
```

And wire in `head_to_head_transition`:

```python
def head_to_head_transition(state_machine: StateMachine)->str:
    mode = state_machine.params.overtake_mode
    if mode == "predictive_spliner":
        gb_fn, trail_fn, ot_fn, ftg_fn = PSGlobalTrackingTransition, PSTrailingTransition, PSOvertakingTransition, PSFTGOnlyTransition
    elif mode == "frenet":
        gb_fn, trail_fn, ot_fn, ftg_fn = FrenetGlobalTrackingTransition, FrenetTrailingTransition, FrenetOvertakingTransition, FrenetFTGOnlyTransition
    elif mode == "graph_based":
        gb_fn, trail_fn, ot_fn, ftg_fn = GBGlobalTrackingTransition, GBTrailingTransition, GBOvertakingTransition, GBFTGOnlyTransition
    else:
        gb_fn, trail_fn, ot_fn, ftg_fn = SpliniGlobalTrackingTransition, SpliniTrailingTransition, SpliniOvertakingTransition, SpliniFTGOnlyTransition
    # ... rest unchanged ...
```

> Reminder: the ROS 2 properties are `@property` (no parens). The ROS 1 versions use method calls with parens. When porting the `GB*Transition` functions above, do NOT add parens — matches the existing ROS 2 Spliner/PS/Frenet convention.

#### 11.4 — state machine launch also accepts `graph_based`

Edit `ROS2_SM/launch/state_machine_launch.xml` description string to include it (optional, for clarity):

```xml
<arg name="overtake_mode" default="spliner" description="overtake mode: spliner / predictive_spliner / frenet / graph_based" />
```

### Step 12 — Python runtime deps

The vendored `graph_ltpl` requires these extra packages beyond what's already installed:

```bash
pip install --user python-igraph trajectory_planning_helpers velocity_optimization
```

> Do NOT pin versions — the ROS 1 `requirements.txt` pinned numpy 1.18.1 & scipy 1.3.3 which conflict with ROS 2 Humble Python. Use whatever is already on the system; if you hit ABI errors in `graph_ltpl`, fall back to the `requirements.txt` versions *for the graph_ltpl subtree only* (not system-wide).

### Step 13 — Build

```bash
cd /home/abhinav/Data_Drive/forzaeth
colcon build --symlink-install --packages-select graph_based_planner state_machine stack_master
source install/setup.bash
```

### Step 14 — Smoke test

Terminal 1 — base system:

```bash
ros2 launch stack_master base_system_launch.xml racecar_version:=NUC2 map_name:=hangar_1905_v0
```

Terminal 2 — head-to-head with graph planner:

```bash
ros2 launch stack_master head_to_head_launch.xml \
    racecar_version:=NUC2 \
    planner:=graph_based \
    map_name:=hangar_1905_v0
```

Acceptance checks:

```bash
ros2 topic hz   /planner/graph_based_wpnts          # expect ~30 Hz
ros2 topic echo /action_marker --once
ros2 topic echo /lattice_viz   --once               # sanity check graph markers
ros2 topic hz   /local_waypoints                    # expect > 20 Hz (state machine publishing)
```

Check logs for `[Graph Based Planner] Initialised — entering main loop.`

---

## 7. Copy block for the verbatim helpers (drop inside `graph_planner_node.py`)

> **Agent task:** open `/home/abhinav/Data_Drive/ros1_race_stack/planner/graph_based_planner/src/Planner.py`, copy **lines 209 to 720 inclusive** (methods from `create_map_csv` through `choose_action`). Paste into `graph_planner_node.py` **inside the `GraphPlanner` class**, replacing the "HELPERS (verbatim port)" comment block. Then apply these **search-and-replace** passes across the pasted region only:

| Find                                               | Replace                                              |
| -------------------------------------------------- | ---------------------------------------------------- |
| `rospy.Time.now()`                                 | `self.get_clock().now().to_msg()`                    |
| `rospy.loginfo(`                                   | `self.get_logger().info(`                            |
| `rospy.logwarn(`                                   | `self.get_logger().warn(`                            |
| `rospy.logerr(`                                    | `self.get_logger().error(`                           |
| `rospy.logdebug(`                                  | `self.get_logger().debug(`                           |

Do **not** modify any other line. Do **not** touch:

- Marker field assignments (`.type = 2`, `.type = 4`, scales, colours, thresholds).
- Numpy math, array indexing, cost weightings.
- The name-mangled attribute `_Graph_LTPL__graph_base`.

---

## 8. Files to delete / ignore from ROS 1 tree

Do NOT copy these from ROS 1 — they are either docs, test data, upstream scaffolding, or redundant:

- `ROS1_GBP/src/GraphBasedPlanner/docs/`
- `ROS1_GBP/src/GraphBasedPlanner/setup.py`
- `ROS1_GBP/src/GraphBasedPlanner/setup.cfg`
- `ROS1_GBP/src/GraphBasedPlanner/LICENSE`
- `ROS1_GBP/src/GraphBasedPlanner/README.md`
- `ROS1_GBP/src/GraphBasedPlanner/main_min_example.py`
- `ROS1_GBP/src/GraphBasedPlanner/main_std_example.py`
- `ROS1_GBP/src/GraphBasedPlanner/__init__.py` (replaced by our ROS 2 package `__init__.py`)
- `ROS1_GBP/src/GraphBasedPlanner/logs/`
- `ROS1_GBP/CMakeLists.txt`, `ROS1_GBP/package.xml`, `ROS1_GBP/launch/planner.launch`

---

## 9. Known divergences from ROS 1 (documented so we don't chase them)

1. **Speed-scaling source.** ROS 1 = `/dyn_sector_server/parameter_updates`. ROS 2 = async param get on `/sector_tuner`. Semantic identical (first double = global scaling), timing is 2 Hz polling instead of event-driven. Acceptable for faithful port; upgrade later.
2. **`testing` param dropped.** ROS 1 used it to auto-read map name from `/f1tenth_simulator/image`. ROS 2 always passes `map_name` explicitly via launch. No regression on car or sim if launch args are provided.
3. **`trailing_distance`, `v_max`, `examined_points` now ROS params** (were hardcoded). Defaults are the ROS 1 hardcoded values — identical behaviour unless user overrides.
4. **Profiler flush.** ROS 1 used `rospy.on_shutdown`. ROS 2 uses the `finally` block in `main()` + `on_shutdown` method. Same outcome.
5. **`path_publisher` is shipped but not launched by head_to_head.** State machine owns `/local_waypoints`. Identical to ROS 1 head-to-head flow.

---

## 10. Sequence summary (for the agentic IDE)

1. Run Step 1 mkdir block.
2. Run Step 2 (copy `graph_ltpl`).
3. Run Step 3 (copy static assets).
4. Write Step 4 files (`package.xml`, `setup.cfg`, `setup.py`).
5. Write Step 5 `graph_planner_node.py` skeleton.
6. Execute Step 7 (paste verbatim helpers + apply 5 sed substitutions).
7. Write Step 6 `path_publisher_node.py`.
8. Write Step 8 YAML.
9. Write Step 9 launch files.
10. Edit Step 10 `head_to_head_launch.xml`.
11. Edit Step 11 state machine / states / transitions.
12. Run Step 12 pip installs.
13. Run Step 13 `colcon build`.
14. Run Step 14 smoke test.

Stop and report after each numbered step. Do not proceed on build errors — surface them.
