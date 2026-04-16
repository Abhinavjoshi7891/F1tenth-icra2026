# race_stack/controller/controller/kinematic_mpc/kinematic_mpc.py
#
# ROS2 port of controller/mpc/src/kinematic_mpc/kinematic_mpc.py (ROS1 ForzaETH)
#
# Changes from ROS1:
#   - Removed: rospy, f110_msgs imports
#   - rospy.wait_for_message() → waypoints passed as arrays to mpc_initialize_solver()
#   - pbl_config types → types.SimpleNamespace (populated from YAML in controller_manager)
#   - rospy.loginfo/warn → injected logger callables
#   - Return signature: added status code as 6th element (speed, acc, jerk, steer, states, status)
#
# Algorithm: identical to ROS1.
# State: [s, n, alpha, v, delta] in Frenet frame.
# Kinematic bicycle model, discrete RK4, SQP-RTI via PARTIAL_CONDENSING_HPIPM.

import types
from typing import Tuple
import numpy as np
from scipy.integrate import solve_ivp
from frenet_conversion.frenet_converter import FrenetConverter
from .utils.splinify import SplineTrack
from .acados_settings import acados_settings


class Kinematic_MPC_Controller:
    """
    Kinematic MPC controller for F1Tenth racing.

    Invariants:
        - mpc_initialize_solver() must be called before main_loop()
        - main_loop() is called at kmpc_config.MPC_freq Hz
        - On solver failure: returns (0, 0, 0, last_steer, last_states, status!=0)
          Controller_manager must check status and zero the drive command.
    """

    def __init__(
        self,
        kmpc_config,        # types.SimpleNamespace — all KMPCConfig fields
        car_config,         # types.SimpleNamespace — lr, lf
        trailing_config,    # types.SimpleNamespace — trailing PID fields
        logger_info=print,
        logger_warn=print,
    ) -> None:
        self.kmpc_config     = kmpc_config
        self.car_config      = car_config
        self.trailing_config = trailing_config
        self.logger_info     = logger_info
        self.logger_warn     = logger_warn
        self._mpc_init_params()
        # mpc_initialize_solver() called externally after waypoints are available

    def _mpc_init_params(self) -> None:
        """Initialise all state variables. Zero-cost to call."""
        self.t_MPC           = 1.0 / self.kmpc_config.MPC_freq
        self.bound_inflation = self.kmpc_config.track_safety_margin
        self.t_delay         = self.kmpc_config.t_delay + self.t_MPC

        # Steering angle ring buffer [newest, oldest]
        self.steering_angle_buf = np.zeros(2)

        # Predicted (s, n) per horizon step — used for waypoint lookup
        self.mpc_sd         = np.zeros((self.kmpc_config.N + 1, 2))
        self.u0             = np.zeros(2)
        self.fre_s          = 0.0
        self.conp_time      = 0.0
        self.speed          = 0.0
        self.steering_angle = 0.0
        self.states         = None

        # Multi-lap arc-length unwrapping
        self.nr_laps           = 0
        self.previous_frenet_s = 0.0

        # Overtaking lateral offset target (0 = on raceline)
        self.overtake_d = self.kmpc_config.overtake_d

        # Trailing PID state — exposed for PID data publishing
        self.loop_rate        = self.kmpc_config.MPC_freq
        self.gap              = None
        self.gap_should       = None
        self.gap_error        = None
        self.gap_actual       = None
        self.v_diff           = None
        self.i_gap            = 0.0
        self.trailing_command = 0.0
        self.flag1            = False   # True when trailing PID fires this cycle

    def mpc_initialize_solver(
        self,
        waypoints_x:   np.ndarray,
        waypoints_y:   np.ndarray,
        waypoints_psi: np.ndarray,
        kapparef:      list,
        vx_ref:        list,
        s_ref:         np.ndarray,
        d_left:        np.ndarray,
        d_right:       np.ndarray,
    ) -> None:
        """
        Build SplineTrack and compile acados OCP.
        Called once from controller_manager after /global_waypoints arrives.
        Replaces rospy.wait_for_message() from ROS1.

        First call: ~20-30s (C code generation + compilation).
        Subsequent calls: ~0.5s (loads compiled shared library).
        """
        self.logger_info("[KMPC] Initializing solver — first run compiles acados C code (~30s)...")

        # FrenetConverter: Cartesian↔Frenet for trailing marker visualization
        self.fren_conv = FrenetConverter(waypoints_x, waypoints_y, waypoints_psi)

        # SplineTrack: arc-length parameterised path for alpha_center computation.
        # coords_direct format expected by SplineTrack: shape (3, N-1, 2)
        # [inner_boundary, center_line, outer_boundary]
        # We don't have separate boundaries here, so we pass zeros for boundary
        # rows — SplineTrack only uses coords[1] (center line) for refline.
        n = len(waypoints_x) - 1   # exclude last point (duplicate of first)
        waypoints_xy = np.stack([waypoints_x[:n], waypoints_y[:n]], axis=1)
        boundaries   = np.zeros((n, 2))
        coords_path  = np.array([boundaries, waypoints_xy, boundaries])
        self.spline  = SplineTrack(coords_direct=coords_path)

        self.s_ref           = s_ref
        self.kappa           = kapparef
        self.nr_laps         = 0
        self.previous_frenet_s = 0.0

        # Build and compile OCP solver
        self.constraint, self.model, self.acados_solver, self.model_params = \
            acados_settings(
                s_ref, kapparef, vx_ref,
                d_left, d_right,
                self.kmpc_config, self.car_config)

        # Apply initial warm start so solver has a feasible trajectory
        self.apply_warm_start(pose_frenet=np.array([0.0, 0.0, 0.0]))

        self.logger_info("[KMPC] Solver ready.")

    # ─────────────────────────────────────────────────────────────────────────
    # MAIN LOOP
    # ─────────────────────────────────────────────────────────────────────────

    def main_loop(
        self,
        state:                  str,
        position_in_map:        np.ndarray,   # shape (1,3): [x, y, theta]
        waypoint_array_in_map:  np.ndarray,   # shape (N,8): local waypoints
        speed_now:              float,
        opponent,                             # None or [s, d, vs, is_static, is_visible]
        position_in_map_frenet: np.ndarray,   # [s, d, vs, vd]
        acc_now:                np.ndarray,   # rolling acc buffer (unused by KMPC)
        track_length:           float,
        compute_time:           float,
    ) -> Tuple[float, float, float, float, list, int]:
        """
        One SQP-RTI solve step.

        Returns:
            (speed, acceleration=0, jerk=0, steering_angle, states, status)
            status == 0: success. status != 0: solver failed, outputs are safe fallback.
        """
        # ── Update shared state ───────────────────────────────────────────────
        self.state                  = state
        self.position_in_map        = position_in_map
        self.waypoint_array_in_map  = waypoint_array_in_map
        self.speed_now              = speed_now
        self.opponent               = opponent
        self.position_in_map_frenet = position_in_map_frenet
        self.acc_now                = acc_now
        self.track_length           = track_length
        self.conp_time              = compute_time

        # ── Frenet state: alpha_center (heading error wrt raceline) ───────────
        # Use SplineTrack tangent at the car's arc-length position.
        # position_in_map_frenet[0] = s (arc-length along raceline)
        # car_config.lr = distance from CoM to rear axle
        # The +lr*cos(...) term shifts the reference point to the rear axle
        # where the kinematic model's point mass is located.
        # position_in_map_frenet[2] = vs (longitudinal speed) — used as a
        # proxy for alpha in the center_car_s shift (same as ROS1 KMPC source).
        center_car_s = (self.position_in_map_frenet[0]
                        + self.car_config.lr
                        * np.cos(self.position_in_map_frenet[2]))

        deriv_center = self.spline.get_derivative(center_car_s)
        alpha_center = (self.position_in_map[0, 2]
                        - np.arctan2(deriv_center[1], deriv_center[0]))

        # Wrap alpha to [-pi, pi]
        alpha_center = alpha_center % (2.0 * np.pi)
        if alpha_center > np.pi:
            alpha_center -= 2.0 * np.pi

        # ── Lap counter — arc-length unwrapping ───────────────────────────────
        # ROS1 logic exactly:
        # On first crossing of start line (s < 0.2) after having advanced,
        # increment lap counter using floor-division check.
        spline_tl = self.spline.track_length - 0.1   # numerical safety margin
        if (self.position_in_map_frenet[0] < 0.2
                and self.fre_s // spline_tl != self.nr_laps):
            self.nr_laps = int(self.fre_s // spline_tl)

        self.fre_s   = self.position_in_map_frenet[0] + self.nr_laps * self.spline.track_length
        self.fre_d   = self.position_in_map_frenet[1]
        self.fre_alpha = alpha_center

        # ── Initial state ─────────────────────────────────────────────────────
        # State vector: [s, n, alpha, v, delta]
        # Use last predicted steering angle (steering_angle_buf[-1]) as initial delta.
        # speed_now from odometry as initial v.
        x0 = np.array([
            self.fre_s,
            self.fre_d,
            self.fre_alpha,
            self.speed,              # last solved speed (smoother than speed_now)
            self.steering_angle_buf[-1],
        ], dtype=np.float64)

        # Time-delay compensation — disabled (matches ROS1 where propagated_x = x0)
        propagated_x = x0

        # Fix initial state in OCP (equality constraint)
        self.acados_solver.set(0, "lbx", propagated_x)
        self.acados_solver.set(0, "ubx", propagated_x)

        # ── Trailing speed target — computed once, applied across all stages ──
        self.flag1 = False
        if self.state == "TRAILING" and self.opponent is not None:
            trailing_v = self.trailing_controller(self.waypoint_array_in_map[0, 2])
            self.flag1 = True
            self.overtake_d = 0.0   # FIX: reset lateral offset when trailing

        # ── Per-stage online parameters and constraint update ─────────────────
        for i in range(self.kmpc_config.N + 1):
            if self.state == "TRAILING" and self.opponent is not None:
                target_v_speed = trailing_v
            else:
                # Find waypoint closest to predicted s at this horizon step
                pred_s = self.mpc_sd[i, 0] % self.spline.track_length
                idx = int(np.abs(self.waypoint_array_in_map[:, 4] - pred_s).argmin())
                target_v_speed = self.waypoint_array_in_map[idx, 2]
                self.overtake_d = 0.0

                if self.state == "OVERTAKE" and self.opponent is not None:
                    self.overtake_d = float(self.waypoint_array_in_map[idx, 3])

            online_parameters = np.array([
                target_v_speed,
                self.kmpc_config.qadv,
                self.kmpc_config.qv,
                self.kmpc_config.qn,
                self.kmpc_config.qalpha,
                self.kmpc_config.qac,
                self.kmpc_config.qddelta,
                self.kmpc_config.alat_max,
                self.kmpc_config.track_safety_margin,
                self.overtake_d,
            ], dtype=np.float64)
            self.acados_solver.set(i, "p", online_parameters)

            # Input and state box constraints per stage
            # (also set in acados_settings, but reset here allows online changes)
            if i < self.kmpc_config.N:
                self.acados_solver.set(i, "lbu",
                    np.array([self.kmpc_config.a_min,   self.kmpc_config.ddelta_min]))
                self.acados_solver.set(i, "ubu",
                    np.array([self.kmpc_config.a_max,   self.kmpc_config.ddelta_max]))
                # State bounds at stage i+1 — only for non-terminal stages.
                # Stage 0 is fixed by lbx/ubx equality above.
                # Terminal stage N has no idxbx_e defined (dim=0), so skip it.
                if i + 1 < self.kmpc_config.N:
                    self.acados_solver.set(i + 1, "lbx",
                        np.array([self.kmpc_config.v_min,   self.kmpc_config.delta_min]))
                    self.acados_solver.set(i + 1, "ubx",
                        np.array([self.kmpc_config.v_max,   self.kmpc_config.delta_max]))

        # ── Solve OCP ─────────────────────────────────────────────────────────
        status = self.acados_solver.solve()
        if status != 0:
            self.logger_warn(
                f"[KMPC] Solver failed status={status}. "
                f"Warm-starting next cycle.")
            self.apply_warm_start(
                pose_frenet=np.array([self.fre_s, self.fre_d, self.fre_alpha]))
            # Return safe fallback: zero speed, last steering
            return 0.0, 0.0, 0.0, self.steering_angle, self.states, status

        # ── Extract solution ──────────────────────────────────────────────────
        # Take prediction at time_delay_step steps ahead to account for
        # ~3 * (1/MPC_freq) seconds of actuation latency.
        # ROS1 value: 3 (hardcoded, commented as TODO).
        time_delay_step = 3
        self.u0           = self.acados_solver.get(0, "u")
        self.pred_x       = self.acados_solver.get(time_delay_step, "x")
        self.steering_angle = float(self.pred_x[4])   # delta state
        self.speed          = float(self.pred_x[3])   # v state

        # Update steering ring buffer (newest at index 0)
        self.steering_angle_buf[1:] = self.steering_angle_buf[:-1]
        self.steering_angle_buf[0]  = float(self.pred_x[-1])

        # Store predicted (s, n) trajectory for per-stage waypoint lookup
        self.mpc_sd = np.array([
            self.acados_solver.get(j, "x")[:2]
            for j in range(self.kmpc_config.N + 1)
        ])

        # Flatten full state trajectory for /mpc_controller/states topic
        states = []
        for node in range(self.kmpc_config.N + 1):
            states.extend(self.acados_solver.get(node, "x").tolist())
        self.states = states

        return self.speed, 0.0, 0.0, self.steering_angle, self.states, status

    # ─────────────────────────────────────────────────────────────────────────
    # UTILITIES
    # ─────────────────────────────────────────────────────────────────────────

    def apply_warm_start(self, pose_frenet: np.ndarray) -> None:
        """
        Apply a constant-velocity warm start across all horizon stages.
        Called at initialization and after solver failure.
        """
        warm_start = self.get_warm_start(
            pose_frenet=pose_frenet, const_v=1.0, const_steer_vel=0.0)
        for i in range(self.kmpc_config.N + 1):
            self.acados_solver.set(i, "x", warm_start[i][:self.model.n_x])
            if i < self.kmpc_config.N:
                self.acados_solver.set(i, "u", warm_start[i][self.model.n_x:])

    def get_warm_start(
        self,
        pose_frenet:    np.ndarray,
        const_v:        float,
        const_steer_vel: float,
    ) -> np.ndarray:
        """
        Forward-Euler integration for N+1 steps to produce a feasible
        initial trajectory. Crude but sufficient for warm start.

        Internal state layout (7-element, controls appended):
            [s, n, alpha, v, delta, der_v, derDelta]
        """
        warm_start    = np.zeros((self.kmpc_config.N + 1, 7))
        warm_start[0] = np.array([
            pose_frenet[0], pose_frenet[1], pose_frenet[2],
            const_v, const_steer_vel, 0.0, 0.0])
        for i in range(1, self.kmpc_config.N + 1):
            der_state    = self._dynamics_of_car(0, warm_start[i - 1])
            warm_start[i] = (warm_start[i - 1]
                             + np.array(der_state) / self.kmpc_config.MPC_freq)
        return warm_start

    def propagate_time_delay(
        self,
        states: np.ndarray,
        inputs: np.ndarray,
    ) -> np.ndarray:
        """
        RK45 forward integration for time-delay compensation.
        Currently disabled in main_loop (t_delay=0 in sim).
        Enable for hardware by uncommenting in main_loop when
        actuation latency is measurable and consistent.
        """
        x0 = np.concatenate((states, inputs), axis=0)
        solution = solve_ivp(
            self._dynamics_of_car,
            t_span=[0.0, self.t_delay],
            y0=x0,
            method='RK45',
            atol=1e-8,
            rtol=1e-8)
        sol = [x[-1] for x in solution.y]

        # Clamp to physical limits after propagation
        if abs(sol[4]) > self.model.delta_max:
            sol[4] = np.sign(sol[4]) * self.model.delta_max
        if abs(sol[3]) > self.constraint.v_max:
            sol[3] = np.sign(sol[3]) * self.constraint.v_max
        if abs(sol[3]) < self.constraint.v_min:
            sol[3] = self.constraint.v_min

        # Strip appended control inputs before returning state
        return np.array(sol)[:-2]

    def _dynamics_of_car(self, t: float, x0: np.ndarray) -> list:
        """
        Continuous kinematic bicycle dynamics in Frenet frame.
        Used by solve_ivp (time-delay) and forward Euler (warm start).

        x0: [s, n, alpha, v, delta, der_v, derDelta]
        """
        s, n, alpha, v, delta = x0[0], x0[1], x0[2], x0[3], x0[4]
        derv, derDelta         = x0[5], x0[6]

        xdot = self.model.f_expl_func(
            s, n, alpha, v, delta, derv, derDelta, self.model_params.p)

        return [
            float(xdot[0]), float(xdot[1]), float(xdot[2]),
            float(xdot[3]), float(xdot[4]),
            derv, derDelta,    # pass-through: controls don't integrate
        ]

    def trailing_controller(self, global_speed: float) -> float:
        """
        PID gap controller for TRAILING state.
        Sets self.flag1=True so controller_manager publishes /trailing/gap_data.

        gap = distance to opponent in Frenet s [m]
        gap_error = gap_should - gap_actual (positive = too close → slow down)
        """
        self.gap        = (self.opponent[0] - self.position_in_map_frenet[0]) % self.track_length
        self.gap_actual = self.gap

        # Fixed-distance mode vs time-headway mode
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

        # Blind trailing: opponent not LiDAR-visible, use minimum speed floor
        if not self.opponent[4] and self.gap_actual > self.gap_should:
            self.trailing_command = max(
                self.trailing_config.blind_trailing_speed, self.trailing_command)

        return self.trailing_command
