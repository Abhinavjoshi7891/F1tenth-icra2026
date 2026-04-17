"""
Frenet Optimal Trajectory solver.
Direct port of ROS1 FrenetOptimalTrajectory.cpp.
No ROS dependencies. Single-threaded (Python GIL makes threading moot).
"""
import copy
import math
import logging
from typing import Optional, List

from frenet_planner.cubic_spline import CubicSpline2D
from frenet_planner.frenet_path import FrenetPath
from frenet_planner.polynomials import QuarticPolynomial, QuinticPolynomial
from frenet_planner.data_structs import FrenetHyperparameters, FrenetInitialConditions

logger = logging.getLogger(__name__)


class FrenetOptimalTrajectory:
    """
    Generates candidate Frenet trajectories and selects the optimal one.
    Threading is disabled — Python GIL makes multi-threading counter-productive
    for CPU-bound numpy work. This is the primary Path-B isolation point:
    to accelerate, replace this class with a C++/pybind11 equivalent.
    """

    def __init__(self, debug_mode: bool = False):
        self.debug_mode = debug_mode
        self.fot_hp: Optional[FrenetHyperparameters] = None
        self.fot_ic = FrenetInitialConditions()
        self.csp: Optional[CubicSpline2D] = None
        self.obstacle_array: list = []

    def set_hyperparameters(self, fot_hp: FrenetHyperparameters):
        self.fot_hp = fot_hp

    def set_global_trajectory(self, wx, wy, d_right, d_left, v,
                              is_closed_contour: bool):
        """Build the reference CubicSpline2D from global waypoints."""
        if len(wx) < 2 or len(wx) != len(wy):
            raise RuntimeError("[FOT] Not enough waypoints for construction")
        self.csp = CubicSpline2D(wx, wy, d_right, d_left, v, is_closed_contour)

    def update_obstacles(self, obstacles) -> bool:
        """
        Process raw obstacles: expand by clearance, handle gaps.
        obstacles: list of obstacle-like objects with fields:
            s_start, s_end, d_left, d_right, is_actually_a_gap, id
        Returns True if there are obstacles to consider.
        """
        self.obstacle_array = []
        hp = self.fot_hp

        for ob in obstacles:
            ob = copy.copy(ob)

            if ob.is_actually_a_gap:
                ob.is_actually_a_gap = False
                # left obstacle of gap
                gap_ob = copy.copy(ob)
                gap_ob.id = ob.id + 100
                gap_ob.d_left = self.csp.calc_d_left(ob.s_start) + hp.track_clearance
                gap_ob.d_right = ob.d_left - hp.obstacle_clearance
                if gap_ob.d_right < gap_ob.d_left:
                    self.obstacle_array.append(gap_ob)

                # right obstacle of gap
                ob.d_left = ob.d_right + hp.obstacle_clearance
                ob.d_right = -self.csp.calc_d_right(ob.s_start) - hp.track_clearance
                if ob.d_right < ob.d_left:
                    self.obstacle_array.append(ob)
                continue

            # normal obstacle — check if within track
            if not (ob.d_left < -self.csp.calc_d_right(ob.s_start) or
                    ob.d_right > self.csp.calc_d_left(ob.s_start) or
                    ob.s_start == ob.s_end):
                ob.s_start -= hp.obstacle_clearance
                ob.s_end += hp.obstacle_clearance
                ob.d_left += hp.obstacle_clearance
                ob.d_right -= hp.obstacle_clearance
                self.obstacle_array.append(ob)

        return len(self.obstacle_array) > 0

    def update_current_position(self, fot_ic: FrenetInitialConditions):
        """Constrain initial conditions to be within track and outside obstacles."""
        hp = self.fot_hp
        csp = self.csp

        # clamp within track bounds (add 0.05m margin to avoid infinite cost)
        fot_ic.c_d = min(fot_ic.c_d, csp.calc_d_left(fot_ic.s0) - 0.05)
        fot_ic.c_d = max(fot_ic.c_d, -csp.calc_d_right(fot_ic.s0) + 0.05)

        # push IC outside obstacles if configured
        if hp.ic_avoid_obstacles:
            for ob in self.obstacle_array:
                consider = False
                if ob.s_end < ob.s_start:  # wrapping
                    if fot_ic.s0 < ob.s_end or fot_ic.s0 > ob.s_start:
                        consider = True
                elif fot_ic.s0 > ob.s_start and fot_ic.s0 < ob.s_end:
                    consider = True
                if consider:
                    d_center = 0.5 * (ob.d_left + ob.d_right)
                    if fot_ic.c_d > d_center:
                        fot_ic.c_d = max(fot_ic.c_d, ob.d_left + 0.05)
                    else:
                        fot_ic.c_d = min(fot_ic.c_d, ob.d_right - 0.05)

        self.fot_ic = fot_ic

    def update_optimal_trajectory(self, prev_best_path: FrenetPath) -> Optional[FrenetPath]:
        """
        Run the FOT solver. Returns best FrenetPath, or None if no valid path found.
        prev_best_path is used for temporal consistency cost.
        """
        hp = self.fot_hp
        ic = self.fot_ic
        csp = self.csp

        frenet_paths: List[FrenetPath] = []

        total_di_iter = int((hp.max_road_width_l + hp.max_road_width_r) / hp.d_road_w) + 1

        for di_idx in range(total_di_iter):
            di = -hp.max_road_width_l + di_idx * hp.d_road_w
            if di > hp.max_road_width_r:
                break

            ti = hp.mint
            while ti <= hp.maxt:
                lat_dev = 0.0
                lat_vel = 0.0
                lat_acc = 0.0
                lat_jerk = 0.0

                # lateral quintic polynomial
                lat_qp = QuinticPolynomial(
                    ic.c_d, ic.c_d_d, ic.c_d_dd, di, 0.0, 0.0, ti)

                # sample lateral trajectory
                fp_t = []
                fp_d = []
                fp_d_d = []
                fp_d_dd = []
                fp_d_ddd = []

                t_sample = 0.0
                while t_sample <= ti + 1e-9: # tiny epsilon
                    fp_t.append(t_sample)
                    d_val = lat_qp.calc_point(t_sample)
                    fp_d.append(d_val)
                    d1 = lat_qp.calc_first_derivative(t_sample)
                    fp_d_d.append(d1)
                    d2 = lat_qp.calc_second_derivative(t_sample)
                    fp_d_dd.append(d2)
                    d3 = lat_qp.calc_third_derivative(t_sample)
                    fp_d_ddd.append(d3)

                    lat_dev += abs(d_val)
                    lat_vel += abs(d1)
                    lat_acc += abs(d2)
                    lat_jerk += abs(d3)
                    t_sample += hp.dt

                # velocity keeping — sample valid target speeds
                max_speed = min(hp.max_speed,
                                ic.c_speed - hp.min_speed + ti * hp.max_accel)
                tv = max(hp.min_speed,
                         ic.target_speed - hp.d_t_s * hp.n_s_sample)
                tv = min(max_speed, tv)
                tv_max = ic.target_speed

                while tv <= min(tv_max, max_speed):
                    lon_dev = 0.0
                    lon_acc = 0.0
                    lon_jerk = 0.0

                    tfp = FrenetPath(hp)
                    tfp.t = list(fp_t)
                    tfp.d = list(fp_d)
                    tfp.d_d = list(fp_d_d)
                    tfp.d_dd = list(fp_d_dd)
                    tfp.d_ddd = list(fp_d_ddd)

                    lon_qp = QuarticPolynomial(
                        ic.s0, ic.c_speed, ic.c_s_dd, tv, 0.0, ti)

                    for tp in tfp.t:
                        s_val = lon_qp.calc_point(tp)
                        tfp.s.append(s_val)
                        sd = lon_qp.calc_first_derivative(tp)
                        tfp.s_d.append(sd)
                        sdd = lon_qp.calc_second_derivative(tp)
                        tfp.s_dd.append(sdd)
                        sddd = lon_qp.calc_third_derivative(tp)
                        tfp.s_ddd.append(sddd)

                        spline_v = csp.calc_v(s_val)
                        if not math.isnan(spline_v):
                            lon_dev += abs(sd - spline_v)
                        lon_acc += abs(sdd)
                        lon_jerk += abs(sddd)

                    # validate
                    valid = (len(tfp.s) >= 2 and
                             tfp.is_within_constraints() and
                             tfp.is_within_track(csp) and
                             not tfp.is_collision(self.obstacle_array) and
                             tfp.to_global_path(csp))

                    if not valid:
                        tv += hp.d_t_s
                        continue

                    # temporal consistency
                    if prev_best_path is not None and len(prev_best_path.s) > 0:
                        tfp.c_consistency = tfp.calc_difference(prev_best_path)

                    # costs
                    path_len_scale = 1.0 / max(len(tfp.t), 1)

                    tfp.c_lateral_deviation = lat_dev
                    tfp.c_lateral_velocity = lat_vel
                    tfp.c_lateral_acceleration = lat_acc
                    tfp.c_lateral_jerk = lat_jerk
                    tfp.c_lateral = path_len_scale * (
                        hp.k_lat_dev * lat_dev +
                        hp.k_lat_v * lat_vel +
                        hp.k_lat_a * lat_acc +
                        hp.k_lat_j * lat_jerk)

                    tfp.c_longitudinal = path_len_scale * (
                        hp.k_lon_dev * lon_dev)

                    tfp.cf = (hp.k_lat * tfp.c_lateral +
                              hp.k_lon * tfp.c_longitudinal +
                              hp.k_obstacles * tfp.c_inv_dist_to_obstacles +
                              hp.k_track_bounds * tfp.c_inv_dist_to_track_bound +
                              hp.k_temporal * tfp.c_consistency +
                              hp.k_time * (hp.maxt - ti))

                    frenet_paths.append(tfp)
                    tv += hp.d_t_s

                ti += hp.d_sample_t

        # select best
        if not frenet_paths:
            return None

        best = min(frenet_paths, key=lambda fp: fp.cf)
        best.calc_d_spline()
        return best
