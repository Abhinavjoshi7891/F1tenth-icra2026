"""
Data structures for the Frenet Optimal Trajectory planner.
Direct port of ROS1 py_cpp_struct.h — no ROS dependencies.
"""
from dataclasses import dataclass

MAX_PATH_LENGTH = 100


@dataclass
class FrenetInitialConditions:
    """Current state of the vehicle in Frenet coordinates."""
    s0: float = 0.0            # current track position [m]
    c_speed: float = 0.0       # current longitudinal speed [m/s]
    c_s_dd: float = 0.0        # current longitudinal acceleration [m/s^2]
    c_d: float = 0.0           # current lateral position [m]
    c_d_d: float = 0.0         # current lateral speed [m/s]
    c_d_dd: float = 0.0        # current lateral acceleration [m/s^2]
    target_speed: float = 0.0  # desired speed at end of trajectory [m/s]


@dataclass
class FrenetHyperparameters:
    """Tuning parameters for the FOT planner."""
    max_speed: float = 10.0
    min_speed: float = 0.5
    max_accel: float = 8.0
    max_curvature: float = 1.0
    max_road_width_l: float = 1.5
    max_road_width_r: float = 1.5
    d_road_w: float = 0.05
    dt: float = 0.05
    maxt: float = 1.5
    mint: float = 1.0
    d_sample_t: float = 0.5
    init_d_t_s: float = 0.25
    d_t_s_ot: float = 2.0
    d_t_s: float = 0.25
    n_s_sample: float = 12.0
    obstacle_clearance: float = 0.35
    track_clearance: float = 0.3
    k_lat_dev: float = 0.6
    k_lat_v: float = 0.2
    k_lat_a: float = 0.0
    k_lat_j: float = 0.1
    k_lon_dev: float = 0.8
    k_track_bounds: float = 0.05
    k_obstacles: float = 0.2
    k_temporal: float = 1.0
    k_lat: float = 0.6
    k_lon: float = 1.0
    k_time: float = 10.0
    global_path_length: float = 0.0
    num_threads: int = 0       # ignored in Python port (GIL), kept for config compat
    ic_avoid_obstacles: bool = False
    ego_width: float = 0.3     # ego car width buffer [m] for collision detection
