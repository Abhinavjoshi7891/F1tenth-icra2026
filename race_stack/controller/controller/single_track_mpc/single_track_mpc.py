# race_stack/controller/controller/single_track_mpc/single_track_mpc.py
#
# ROS2 port of controller/mpc/src/single_track_mpc/single_track_mpc.py (ROS1 ForzaETH)
#
# Changes from ROS1:
#   - Removed: rospy, f110_msgs imports
#   - rospy.wait_for_message() replaced: waypoints passed as arrays to mpc_initialize_solver()
#   - pbl_config types replaced by types.SimpleNamespace (populated from YAML)
#   - rospy.loginfo/warn replaced by injected logger callables
#   - Return signature: 6-tuple (speed, acceleration, jerk, steer, states, status)
#
# State: [s, n, theta, v_x, v_y, delta, yaw_rate, accel] in Frenet frame.
# Dynamic single-track bicycle + Pacejka tires. ERK integrator via acados.

import os
import types
from typing import Tuple
import numpy as np
from scipy.integrate import solve_ivp
from frenet_conversion.frenet_converter import FrenetConverter
from .utils.splinify import SplineTrack
from .acados_settings import acados_settings


class Single_track_MPC_Controller:
    """
    Dynamic single-track MPC controller for F1Tenth racing.

    Invariants:
        - mpc_initialize_solver() called at __init__ time with pose_frenet
        - main_loop() called at stmpc_config.MPC_freq Hz
        - using_gokart = False for F1Tenth (measured_steer from buffer, measured_acc from prev_acc)
        - On solver failure: returns (0, 0, 0, last_steer, last_states, status!=0)
    """

    def __init__(
        self,
        pose_frenet,           # [s, d, alpha] — initial Frenet position at construction time
        stmpc_config,          # types.SimpleNamespace — all STMPCConfig fields
        car_config,            # types.SimpleNamespace — lr, lf, m, Iz, h_cg
        tire_config,           # types.SimpleNamespace — Pacejka tire params
        trailing_config,       # types.SimpleNamespace — trailing PID fields
        controller_frequency: float,  # Hz of the outer control loop (40 Hz)
        using_gokart: bool = False,
        logger_info=print,
        logger_warn=print,
    ) -> None:
        self.stmpc_config        = stmpc_config
        self.car_config          = car_config
        self.tire_config         = tire_config
        self.trailing_config     = trailing_config
        self.controller_frequency = controller_frequency
        self.using_gokart        = using_gokart
        self.logger_info         = logger_info
        self.logger_warn         = logger_warn

        self._mpc_init_params()
        self.mpc_initialize_solver(pose_frenet)

    def _mpc_init_params(self) -> None:
        self.t_MPC   = 1.0 / self.stmpc_config.MPC_freq
        self.t_delay = self.stmpc_config.t_delay + self.t_MPC

        self.steering_angle_buf = np.zeros(2)

        self.mpc_sd   = np.zeros((self.stmpc_config.N + 1, 2))
        self.u0       = np.zeros(2)
        self.fre_s    = 0.0
        self.comp_time = 0.0

        # Initial vehicle state
        self.speed          = 0.0
        self.steering_angle = 0.0
        self.acceleration   = 0.0
        self.jerk_out       = 0.0
        self.prev_acc       = 0.0
        self.states         = None

        # Dynamic state from sensors (F1Tenth: estimated from buffer/previous)
        self.vel_y        = 0.0
        self.yaw_rate     = 0.0
        self.measured_acc = 0.0
        self.measured_steer = 0.0

        # Lap counter
        self.previous_frenet_s = 0.0
        self.nr_laps           = 0

        self.overtake_d = self.stmpc_config.overtake_d
        self.loop_rate  = self.stmpc_config.MPC_freq

        # Trailing PID state
        self.gap              = None
        self.gap_should       = None
        self.gap_error        = None
        self.gap_actual       = None
        self.v_diff           = None
        self.i_gap            = 0.0
        self.trailing_command = 0.0
        self.flag1            = False

    def mpc_initialize_solver(self, pose_frenet: np.ndarray) -> None:
        """
        Build SplineTrack and compile acados OCP.
        Called once at __init__ time after waypoints are available.
        Replaces rospy.wait_for_message() from ROS1.

        First call: ~30-60s (C code generation + compilation, larger model than KMPC).
        Subsequent calls with same JSON: ~1s.
        """
        self.logger_info("[STMPC] Initializing solver — first run compiles acados C code (~60s)...")

        self.nr_laps           = 0
        self.previous_frenet_s = 0.0

    def mpc_initialize_solver_with_waypoints(
        self,
        waypoints_x:   np.ndarray,
        waypoints_y:   np.ndarray,
        waypoints_psi: np.ndarray,
        kapparef:      list,
        s_ref:         np.ndarray,
        d_left:        np.ndarray,
        d_right:       np.ndarray,
        pose_frenet:   np.ndarray,
    ) -> None:
        """
        Second-phase initialisation called by controller_manager after waypoints arrive.
        Separated from __init__ because waypoints are not available at construction time.
        """
        self.fren_conv = FrenetConverter(waypoints_x, waypoints_y, waypoints_psi)

        n = len(waypoints_x) - 1
        waypoints_xy = np.stack([waypoints_x[:n], waypoints_y[:n]], axis=1)
        boundaries   = np.zeros((n, 2))
        coords_path  = np.array([boundaries, waypoints_xy, boundaries])
        self.spline  = SplineTrack(coords_direct=coords_path)

        self.s_ref   = s_ref
        self.kappa   = kapparef

        # acados generates C code relative to cwd — must be a writable directory
        os.chdir('/tmp')
        self.constraint, self.model, self.acados_solver, self.model_params = \
            acados_settings(
                s_ref, kapparef,
                d_left, d_right,
                self.stmpc_config, self.car_config, self.tire_config)

        self.apply_warm_start(pose_frenet=pose_frenet)
        self.prev_acc = 0.0

        self.logger_info("[STMPC] Solver ready.")

    def main_loop(
        self,
        state:                  str,
        position_in_map:        np.ndarray,   # shape (1,3): [x, y, theta]
        waypoint_array_in_map:  np.ndarray,   # shape (N,8): local waypoints
        speed_now:              float,
        opponent,                             # None or [s, d, vs, is_static, is_visible]
        position_in_map_frenet: np.ndarray,   # [s, d, vs, vd]
        single_track_state:     np.ndarray,   # [vel_y, yaw_rate, measured_acc, 0]
        track_length:           float,
        compute_time:           float,
    ) -> Tuple[float, float, float, float, list, int]:
        """
        One SQP-RTI solve step.

        Returns:
            (speed, acceleration, jerk, steering_angle, states, status)
            status == 0: success. status != 0: solver failed.
        """
        # ── Update shared state ───────────────────────────────────────────────
        self.state                  = state
        self.position_in_map        = position_in_map
        self.waypoint_array_in_map  = waypoint_array_in_map
        self.speed_now              = speed_now
        self.opponent               = opponent
        self.position_in_map_frenet = position_in_map_frenet
        self.track_length           = track_length
        self.comp_time              = compute_time

        # ── Unpack single_track_state ─────────────────────────────────────────
        self.vel_y    = single_track_state[0]
        self.yaw_rate = single_track_state[1]

        # F1Tenth: using_gokart=False, so measured values from buffer/previous
        if self.using_gokart:
            self.measured_acc   = single_track_state[2]
            self.measured_steer = single_track_state[3]
        else:
            self.measured_steer = self.steering_angle_buf[-1]
            self.measured_acc   = self.prev_acc

        # ── Frenet state: theta_center (heading error wrt raceline) ───────────
        track_length  = self.spline.track_length - 0.1
        center_car_s  = (self.position_in_map_frenet[0]
                         + self.car_config.lr
                         * np.cos(self.position_in_map_frenet[2]))
        deriv_center  = self.spline.get_derivative(center_car_s)
        alpha_center  = (self.position_in_map[0, 2]
                         - np.arctan2(deriv_center[1], deriv_center[0]))

        alpha_center = alpha_center % (2.0 * np.pi)
        if alpha_center > np.pi:
            alpha_center -= 2.0 * np.pi

        # ── Lap counter (uses previous_frenet_s comparison, not floor-division) ─
        if self.position_in_map_frenet[0] < self.previous_frenet_s - 1:
            self.nr_laps += 1
        self.previous_frenet_s = self.position_in_map_frenet[0]

        current_pos_s = self.position_in_map_frenet[0] + self.nr_laps * self.spline.track_length
        self.fre_s    = current_pos_s
        self.fre_d    = self.position_in_map_frenet[1]
        self.fre_alpha = alpha_center

        # ── Initial state — 8-element ─────────────────────────────────────────
        # [s, n, theta, v_x, v_y, delta, yaw_rate, accel]
        x0 = np.array([
            self.fre_s,
            self.fre_d,
            self.fre_alpha,
            speed_now,
            self.vel_y,
            self.measured_steer,
            self.yaw_rate,
            self.measured_acc,
        ], dtype=np.float64)

        # Time-delay compensation disabled (matches ROS1 where t_delay = 0.00)
        propagated_x = x0

        # Fix initial state in OCP
        self.acados_solver.set(0, "lbx", propagated_x)
        self.acados_solver.set(0, "ubx", propagated_x)

        # ── Trailing speed target — computed once before stage loop ───────────
        self.flag1 = False
        trailing = (self.state == "TRAILING" and self.opponent is not None)
        if trailing:
            trailing_v = self.trailing_controller(self.waypoint_array_in_map[0, 2])
            self.flag1 = True
            self.overtake_d = 0.0   # FIX: reset lateral offset when trailing

        # ── Per-stage online parameters and constraint update ─────────────────
        for i in range(self.stmpc_config.N + 1):
            if trailing:
                target_v_speed = trailing_v
            else:
                pred_s = self.mpc_sd[i, 0] % self.spline.track_length
                idx = int(np.abs(self.waypoint_array_in_map[:, 4] - pred_s).argmin())
                target_v_speed = self.waypoint_array_in_map[idx, 2]
                self.overtake_d = 0.0
                if self.state == "OVERTAKE" and self.opponent is not None:
                    self.overtake_d = float(self.waypoint_array_in_map[idx, 3])

            online_parameters = np.array([
                target_v_speed,
                self.stmpc_config.qadv,
                self.stmpc_config.qv,
                self.stmpc_config.qn,
                self.stmpc_config.qalpha,
                self.stmpc_config.qjerk,
                self.stmpc_config.qddelta,
                self.stmpc_config.alat_max,
                self.stmpc_config.a_min,
                self.stmpc_config.a_max,
                self.stmpc_config.track_safety_margin,
                self.overtake_d,
            ], dtype=np.float64)
            self.acados_solver.set(i, "p", online_parameters)

            if i < self.stmpc_config.N:
                # Input box constraints per stage
                # NOTE: jerk bounds are wide (-50, 50) to avoid over-constraining
                self.acados_solver.set(i, "lbu",
                    np.array([-50.0, self.stmpc_config.ddelta_min]))
                self.acados_solver.set(i, "ubu",
                    np.array([50.0,  self.stmpc_config.ddelta_max]))
                # State box constraints at next stage
                # [v_x, delta, accel] — indices [3, 5, 7]
                self.acados_solver.set(i + 1, "lbx",
                    np.array([self.stmpc_config.v_min,
                               self.stmpc_config.delta_min,
                               self.stmpc_config.a_min]))
                self.acados_solver.set(i + 1, "ubx",
                    np.array([self.stmpc_config.v_max,
                               self.stmpc_config.delta_max,
                               self.stmpc_config.a_max]))

        # ── Solve OCP ─────────────────────────────────────────────────────────
        status = self.acados_solver.solve()
        if status != 0:
            self.logger_warn(
                f"[STMPC] Solver failed status={status}. Warm-starting next cycle.")
            self.apply_warm_start(
                pose_frenet=np.array([self.fre_s, self.fre_d, self.fre_alpha]))
            return 0.0, 0.0, 0.0, self.measured_steer, self.states, status

        # ── Extract solution ──────────────────────────────────────────────────
        # Delay index: steps_delay + 1 (from config, not hardcoded like KMPC)
        delayed_index = self.stmpc_config.steps_delay + 1

        self.u0             = self.acados_solver.get(0, "u")
        self.prev_acc       = self.acados_solver.get(0, "x")[7]   # accel state at stage 0
        self.pred_x         = self.acados_solver.get(delayed_index, "x")
        self.steering_angle = float(self.pred_x[5])   # delta state
        self.speed          = float(self.pred_x[3])   # v_x state

        # Euler integration for acceleration (different frequencies controller/MPC)
        self.acceleration = self.prev_acc + self.u0[0] / self.controller_frequency
        self.prev_acc     = self.acceleration
        self.jerk_out     = float(self.u0[0])

        # Steering ring buffer
        self.steering_angle_buf[1:] = self.steering_angle_buf[:-1]
        self.steering_angle_buf[0]  = float(self.pred_x[5])

        # Store predicted (s, n) trajectory for per-stage waypoint lookup
        self.mpc_sd = np.array([
            self.acados_solver.get(j, "x")[:2]
            for j in range(self.stmpc_config.N + 1)
        ])

        # Flatten full state trajectory for /mpc_controller/states topic
        states = []
        for node in range(self.stmpc_config.N + 1):
            states.extend(self.acados_solver.get(node, "x").tolist())
        self.states = states

        return self.speed, self.acceleration, self.jerk_out, self.steering_angle, self.states, status

    def apply_warm_start(self, pose_frenet: np.ndarray) -> None:
        warm_start = self.get_warm_start(
            pose_frenet=pose_frenet, const_acc=1.0, const_steer_vel=0.0)
        for i in range(self.stmpc_config.N + 1):
            self.acados_solver.set(i, "x", warm_start[i][:self.model.n_x])
            if i < self.stmpc_config.N:
                self.acados_solver.set(i, "u", warm_start[i][self.model.n_x:])

    def get_warm_start(
        self,
        pose_frenet:     np.ndarray,
        const_acc:       float,
        const_steer_vel: float,
    ) -> np.ndarray:
        """
        Forward-Euler warm start. 10-wide array: 8 states + 2 controls.
        Initial state: [s, d, alpha, 1.0m/s, 0, 0, 0, const_acc, 0, const_steer_vel]
        """
        warm_start    = np.zeros((self.stmpc_config.N + 1, 10))
        warm_start[0] = np.array([
            pose_frenet[0], pose_frenet[1], pose_frenet[2],
            1.0,           # v_x — small non-zero to avoid singularity in tire model
            0.0,           # v_y
            0.0,           # delta
            0.0,           # yaw_rate
            const_acc,     # accel
            0.0,           # jerk (control)
            const_steer_vel  # derDelta (control)
        ])
        for i in range(1, self.stmpc_config.N + 1):
            xdot          = self._dynamics_of_car(0, warm_start[i - 1])
            warm_start[i] = (warm_start[i - 1]
                             + np.array(xdot) / self.stmpc_config.MPC_freq)
        return warm_start

    def _dynamics_of_car(self, t: float, x0: np.ndarray) -> list:
        """
        Continuous dynamic single-track dynamics in Frenet frame.
        Used by forward Euler (warm start).

        x0 layout (10-element, controls appended):
            [s, n, theta, v_x, v_y, delta, yaw_rate, accel, derDelta, jerk]
        """
        s        = x0[0]
        n        = x0[1]
        theta    = x0[2]
        v_x      = x0[3]
        v_y      = x0[4]
        delta    = x0[5]
        yaw_rate = x0[6]
        accel    = x0[7]
        derDelta = x0[8]
        jerk     = x0[9]

        xdot = self.model.f_expl_func(
            s, n, theta, v_x, v_y, delta, yaw_rate, accel,
            jerk, derDelta, self.model_params.p)

        return [
            float(xdot[0]), float(xdot[1]), float(xdot[2]),
            float(xdot[3]), float(xdot[4]), float(xdot[5]),
            float(xdot[6]), float(xdot[7]),
            derDelta, jerk,  # pass-through: controls don't integrate
        ]

    def trailing_controller(self, global_speed: float) -> float:
        self.gap        = (self.opponent[0] - self.position_in_map_frenet[0]) % self.track_length
        self.gap_actual = self.gap

        if self.trailing_config.trailing_mode:
            self.gap_should = self.trailing_config.trailing_gap
        else:
            self.gap_should = self.position_in_map_frenet[2] * self.trailing_config.trailing_gap

        self.gap_error = self.gap_should - self.gap_actual
        self.v_diff    = self.position_in_map_frenet[2] - self.opponent[2]
        self.i_gap     = np.clip(
            self.i_gap + self.gap_error / self.loop_rate, -10.0, 10.0)

        p_val = self.gap_error * self.trailing_config.trailing_p_gain
        d_val = self.v_diff   * self.trailing_config.trailing_d_gain
        i_val = self.i_gap    * self.trailing_config.trailing_i_gain

        self.trailing_command = np.clip(
            self.opponent[2] - p_val - i_val - d_val, 0.0, global_speed)

        if not self.opponent[4] and self.gap_actual > self.gap_should:
            self.trailing_command = max(
                self.trailing_config.blind_trailing_speed, self.trailing_command)

        self.logger_info(
            f"TRAIL: opp_v={self.opponent[2]:.2f} gap_act={self.gap_actual:.2f} "
            f"gap_err={self.gap_error:.2f} v_diff={self.v_diff:.2f} "
            f"p={p_val:.2f} i={i_val:.2f} d={d_val:.2f} "
            f"cmd={self.trailing_command:.2f}"
        )
        
        return self.trailing_command
