# race_stack/controller/controller/single_track_mpc/bicycle_model.py
#
# BSD 2-Clause License — original author: Daniel Kloeser / acados team
# ROS2 port: removed pbl_config dependency, config objects are
# types.SimpleNamespace populated from YAML in controller_manager.py
#
# State vector: [s, n, theta, v_x, v_y, delta, yaw_rate, accel]  — 8 states
# Control vector: [jerk, derDelta]                                 — 2 controls
# Integrator: ERK (continuous time, acados handles integration)
# No discrete RK4 — acados uses ERK with 4 stages, 3 steps

import types
import casadi as cs
import numpy as np
from casadi import MX, Function, cos, interpolant, fmod, sin, vertcat


def bicycle_model(s0: list, kapparef: list, d_left: list, d_right: list,
                  stmpc_config, car_config, tire_config):
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Spatialbicycle_model"

    length = len(s0)
    pathlength = s0[-1]

    # periodic extension for bspline interpolant
    s0 = np.append(s0, [s0[length - 1] + s0[1:length]])
    s0 = np.append([s0[:length - 1] - s0[length - 1]], s0)

    kapparef = np.append(kapparef, kapparef[1:length])
    kapparef = np.append([kapparef[:length - 1] - kapparef[length - 1]], kapparef)

    d_left = np.append(d_left, d_left[1:length])
    d_left = np.append([d_left[:length - 1] - d_left[length - 1]], d_left)

    d_right = np.append(d_right, d_right[1:length])
    d_right = np.append([d_right[:length - 1] - d_right[length - 1]], d_right)

    # CasADi bspline interpolants
    kapparef_s  = interpolant("kapparef_s",  "bspline", [s0], kapparef)
    left_bound_s  = interpolant("left_bound_s",  "bspline", [s0], d_left)
    right_bound_s = interpolant("right_bound_s", "bspline", [s0], d_right)

    # ── State variables ───────────────────────────────────────────────────────
    # 8-state dynamic single-track in Frenet frame
    s       = MX.sym("s")
    n       = MX.sym("n")
    theta   = MX.sym("theta")
    v_x     = MX.sym("v_x")
    v_y     = MX.sym("v_y")
    delta   = MX.sym("delta")
    yaw_rate = MX.sym("yaw_rate")
    accel   = MX.sym("accel")
    x = vertcat(s, n, theta, v_x, v_y, delta, yaw_rate, accel)
    n_x = x.size()[0]   # 8

    # ── Control inputs ────────────────────────────────────────────────────────
    jerk     = MX.sym("jerk")      # longitudinal jerk [m/s^3]
    derDelta = MX.sym("derDelta")  # steering rate [rad/s]
    u = vertcat(jerk, derDelta)
    n_u = u.size()[0]   # 2

    # algebraic variables (empty)
    z = vertcat([])

    # ── Vehicle parameters ────────────────────────────────────────────────────
    freq = stmpc_config.MPC_freq

    lr   = car_config.lr
    lf   = car_config.lf
    m    = car_config.m
    Iz   = car_config.Iz
    h_cg = car_config.h_cg

    # ── Pacejka tire parameters ───────────────────────────────────────────────
    friction_coeff = tire_config.friction_coeff
    Bf = tire_config.Bf
    Cf = tire_config.Cf
    Df = tire_config.Df
    Ef = tire_config.Ef
    Br = tire_config.Br
    Cr = tire_config.Cr
    Dr = tire_config.Dr
    Er = tire_config.Er
    g  = 9.81

    # ── Online parameters (updated every control cycle) ───────────────────────
    V_target      = MX.sym('V_target')
    weight_adv    = MX.sym('weight_adv')
    weight_qv     = MX.sym('weight_qv')
    weight_qn     = MX.sym('weight_qn')
    weight_qalpha = MX.sym('weight_qalpha')
    weight_qjerk  = MX.sym('weight_qjerk')
    weight_ddelta = MX.sym('weight_ddelta')
    a_lat_max     = MX.sym('a_lat_max')
    a_long_min    = MX.sym('a_long_min')
    a_long_max    = MX.sym('a_long_max')
    safety_margin = MX.sym('safety_margin')
    overtake_d    = MX.sym('overtake_d')

    p = vertcat(
        V_target, weight_adv, weight_qv, weight_qn,
        weight_qalpha, weight_qjerk, weight_ddelta,
        a_lat_max, a_long_min, a_long_max, safety_margin, overtake_d)

    # ── Tire slip angles (Pacejka) ────────────────────────────────────────────
    s_mod = fmod(s, pathlength)
    v_0   = 0.5   # small velocity offset prevents division by zero at standstill

    alpha_f = cs.atan2((-v_y - lf * yaw_rate), v_x + v_0) + delta
    alpha_r = cs.atan2((-v_y + lr * yaw_rate), v_x + v_0)

    # ── Normal load (with optional load transfer) ──────────────────────────────
    if stmpc_config.load_transfer:
        F_zf = m * ((-accel * h_cg) + (g * lr)) / (lf + lr)
        F_zr = m * ((accel * h_cg) + (g * lf)) / (lf + lr)
    else:
        F_zf = m * g * lr / (lf + lr)
        F_zr = m * g * lf / (lf + lr)

    # ── Lateral tire forces (Pacejka magic formula) ───────────────────────────
    F_yf = (friction_coeff * Df * F_zf *
            sin(Cf * cs.atan(Bf * alpha_f - Ef * (Bf * alpha_f - cs.atan(Bf * alpha_f)))))
    F_yr = (friction_coeff * Dr * F_zr *
            sin(Cr * cs.atan(Br * alpha_r - Er * (Br * alpha_r - cs.atan(Br * alpha_r)))))

    # ── Motor/braking forces ──────────────────────────────────────────────────
    F_motor        = accel * m
    motor_split_front = 0.5
    motor_split_rear  = 1 - motor_split_front
    F_xf = motor_split_front * F_motor
    F_xr = motor_split_rear  * F_motor

    # ── Continuous dynamics in Frenet frame ───────────────────────────────────
    s_dot   = ((v_x * cos(theta)) - (v_y * sin(theta))) / (1 - kapparef_s(s_mod) * n)
    n_dot   = v_x * sin(theta) + v_y * cos(theta)
    theta_dot = (yaw_rate
                 - kapparef_s(s_mod)
                 * ((v_x * cos(theta) - v_y * sin(theta)) / (1 - kapparef_s(s_mod) * n)))
    v_x_dot = (1 / m) * (F_xr + F_xf * cos(delta) - F_yf * sin(delta) + m * v_y * yaw_rate)
    v_y_dot = (1 / m) * (F_yr + F_xf * sin(delta) + F_yf * cos(delta) - m * v_x * yaw_rate)
    yaw_rate_dot = (1 / Iz) * ((-F_yr * lr) + (F_yf * lf) * cos(delta))
    delta_dot    = derDelta
    accel_dot    = jerk

    f_expl = vertcat(
        s_dot, n_dot, theta_dot,
        v_x_dot, v_y_dot, delta_dot,
        yaw_rate_dot, accel_dot)

    # CasADi function for dynamics evaluation (used in warm start / propagation)
    f_expl_func = Function(
        'f_expl_func',
        [s, n, theta, v_x, v_y, delta, yaw_rate, accel, jerk, derDelta, p],
        [f_expl])

    # ── Initial conditions ────────────────────────────────────────────────────
    model.x0 = np.zeros(n_x)

    # ── Cost function ─────────────────────────────────────────────────────────
    terminal_multiplier = 10

    model.cost_expr_ext_cost_0 = (
        weight_qn    * (n - overtake_d)**2
        + weight_qalpha * theta**2
        + weight_qv     * (v_x - V_target)**2
        + weight_qjerk  * jerk**2
        + weight_ddelta * derDelta**2
    )
    model.cost_expr_ext_cost = (
        weight_qn    * (n - overtake_d)**2
        + weight_qalpha * theta**2
        + weight_qv     * (v_x - V_target)**2
        + weight_qjerk  * jerk**2
        + weight_ddelta * derDelta**2
    )
    model.cost_expr_ext_cost_e = terminal_multiplier * (
        weight_qn    * (n - overtake_d)**2
        + weight_qalpha * theta**2
        + weight_qv     * (v_x - V_target)**2
    )

    # Optional cost terms controlled by YAML flags
    if stmpc_config.vy_minimization:
        model.cost_expr_ext_cost_0 += 100 * v_y**2
        model.cost_expr_ext_cost   += 100 * v_y**2
        model.cost_expr_ext_cost_e += terminal_multiplier * (100 * v_y**2)

    if stmpc_config.adv_maximization:
        model.cost_expr_ext_cost   -= weight_adv * s_dot / freq
        model.cost_expr_ext_cost_e -= terminal_multiplier * weight_adv * s_dot / freq

    # ── Nonlinear constraints ─────────────────────────────────────────────────
    n_right_bound = n + right_bound_s(s_mod) - safety_margin   # must be >= 0
    n_left_bound  = n - left_bound_s(s_mod)  + safety_margin   # must be <= 0

    # Lateral acceleration: either use v_y_dot or kinematic approximation
    if stmpc_config.correct_v_y_dot:
        a_lat = v_y_dot
    else:
        a_lat = v_x * v_x * cs.tan(delta) / (lr + lf)

    # Combined constraint (friction ellipse / diamond / none)
    if stmpc_config.combined_constraints == "ellipse":
        combined_constraints = (a_lat / a_lat_max)**2 + (v_x_dot / a_long_max)**2
    elif stmpc_config.combined_constraints == "diamond":
        combined_constraints = cs.fabs(a_lat / a_lat_max) + cs.fabs(v_x_dot / a_long_max)
    else:
        combined_constraints = cs.MX(0)   # zero — effectively no combined constraint

    constraint.pathlength = pathlength

    # Stage constraint (n_right, n_left, combined) — 3 nonlinear constraints
    constraint.expr = vertcat(
        n_right_bound,
        n_left_bound,
        combined_constraints)

    # Terminal constraint (n_right, n_left only) — 2 nonlinear constraints
    constraint.expr_e = vertcat(
        n_right_bound,
        n_left_bound)

    # ── State and input bounds (read by acados_settings) ─────────────────────
    constraint.delta_min  = stmpc_config.delta_min
    constraint.delta_max  = stmpc_config.delta_max
    constraint.v_x_min    = stmpc_config.v_min
    constraint.v_x_max    = stmpc_config.v_max
    constraint.a_min      = stmpc_config.a_min
    constraint.a_max      = stmpc_config.a_max
    constraint.ddelta_min = stmpc_config.ddelta_min
    constraint.ddelta_max = stmpc_config.ddelta_max
    constraint.jerk_min   = stmpc_config.jerk_min
    constraint.jerk_max   = stmpc_config.jerk_max

    # ── Assemble model struct ─────────────────────────────────────────────────
    params = types.SimpleNamespace()
    model.f_expl_expr  = f_expl
    model.x      = x
    model.n_x    = n_x
    model.u      = u
    model.n_u    = n_u
    model.z      = z
    model.p      = p
    model.name   = model_name
    model.params = params
    model.kappa          = kapparef_s
    model.f_expl_func    = f_expl_func
    model.left_bound_s   = left_bound_s
    model.right_bound_s  = right_bound_s
    model.track_max      = stmpc_config.track_max_width

    return model, constraint, params
