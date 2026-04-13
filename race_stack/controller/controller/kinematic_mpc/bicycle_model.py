# race_stack/controller/controller/kinematic_mpc/bicycle_model.py
#
# BSD 2-Clause License — original author: Daniel Kloeser / acados team
# ROS2 port: removed pbl_config dependency, config objects are now
# types.SimpleNamespace populated from YAML in controller_manager.py
#

import types
import casadi as cs
import numpy as np
from casadi import MX, Function, cos, interpolant, fmod, sin, vertcat


def bicycle_model(
        s0: list,
        kapparef: list,
        vx_ref: list,
        d_left: list,
        d_right: list,
        kmpc_config,     # types.SimpleNamespace — same attributes as ROS1 KMPCConfig
        car_config):     # types.SimpleNamespace — same attributes as ROS1 CarConfig
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Spatialbicycle_model"

    length = len(s0)
    pathlength = s0[-1]

    # copy loop to beginning and end (periodic extension for bspline interpolant)
    s0 = np.append(s0, [s0[length - 1] + s0[1:length]])
    s0 = np.append([s0[:length - 1] - s0[length - 1]], s0)

    kapparef = np.append(kapparef, kapparef[1:length])
    kapparef = np.append(kapparef[:length - 1], kapparef)

    vx_ref = np.append(vx_ref, vx_ref[1:length])
    vx_ref = np.append(vx_ref[:length - 1], vx_ref)

    d_left = np.append(d_left, d_left[1:length])
    d_left = np.append(d_left[:length - 1], d_left)

    d_right = np.append(d_right, d_right[1:length])
    d_right = np.append(d_right[:length - 1], d_right)

    # CasADi bspline interpolants over the periodic arc-length grid
    kapparef_s  = interpolant("kapparef_s",  "bspline", [s0], kapparef)
    vx_ref_s    = interpolant("vx_ref_s",    "bspline", [s0], vx_ref)
    left_bound_s  = interpolant("left_bound_s",  "bspline", [s0], d_left)
    right_bound_s = interpolant("right_bound_s", "bspline", [s0], d_right)

    # ── State variables ───────────────────────────────────────────────────────
    # Frenet frame: s=arc-length, n=lateral deviation, alpha=heading error,
    # v=speed, delta=steering angle
    s     = MX.sym("s")
    n     = MX.sym("n")
    alpha = MX.sym("alpha")
    v     = MX.sym("v")
    delta = MX.sym("delta")
    x = vertcat(s, n, alpha, v, delta)
    n_x = x.size()[0]   # 5

    # ── Control inputs ────────────────────────────────────────────────────────
    der_v    = MX.sym("der_v")     # longitudinal acceleration [m/s²]
    derDelta = MX.sym("derDelta")  # steering rate [rad/s]
    u = vertcat(der_v, derDelta)
    n_u = u.size()[0]   # 2

    # ── State derivatives (xdot) for implicit form ────────────────────────────
    sdot     = MX.sym("sdot")
    ndot     = MX.sym("ndot")
    alphadot = MX.sym("alphadot")
    vdot     = MX.sym("vdot")
    deltadot = MX.sym("deltadot")
    xdot = vertcat(sdot, ndot, alphadot, vdot, deltadot)

    # algebraic variables (empty)
    z = vertcat([])

    # ── Vehicle and cost parameters ───────────────────────────────────────────
    lr   = car_config.lr
    lf   = car_config.lf
    freq = kmpc_config.MPC_freq

    # Online parameters (updated every control cycle)
    V_target     = MX.sym('V_target')
    weight_adv   = MX.sym('weight_adv')
    weight_qn    = MX.sym('weight_qn')
    weight_qv    = MX.sym('weight_qv')
    weight_qalpha= MX.sym('weight_qalpha')
    weight_qac   = MX.sym('weight_qac')
    weight_ddelta= MX.sym('weight_ddelta')
    a_lat_max    = MX.sym('a_lat_max')
    bound_inflation = MX.sym('boundary_inflation')
    overtake_d   = MX.sym('overtake_d')

    p = vertcat(
        V_target, weight_adv, weight_qv, weight_qn,
        weight_qalpha, weight_qac, weight_ddelta,
        a_lat_max, bound_inflation, overtake_d)

    # ── Kinematic bicycle model in Frenet frame ───────────────────────────────
    s_mod   = fmod(s, pathlength)
    sdota   = (v * cos(alpha)) / (1 - kapparef_s(s_mod) * n)
    ndot_   = v * sin(alpha)
    alphadot_ = v / (lr + lf) * cs.tan(delta) - kapparef_s(s_mod) * sdota

    f_expl = vertcat(sdota, ndot_, alphadot_, der_v, derDelta)

    # CasADi function for time-delay forward propagation in kinematic_mpc.py
    f_expl_func = Function(
        'f_expl_func',
        [s, n, alpha, v, delta, der_v, derDelta, p],
        [f_expl])

    # ── Discrete RK4 dynamics (used as disc_dyn_expr in acados) ──────────────
    control_period = 1.0 / freq
    x_runge   = vertcat(s, n, alpha)
    f_c_runge = vertcat(sdota, ndot_, alphadot_)
    f_c_runge_func = Function(
        'f_c_runge_func',
        [s, n, alpha, v, delta, p], [f_c_runge])

    dis_v_k     = v + der_v
    dis_delta_k = delta + derDelta

    k1 = control_period * f_c_runge_func(s, n, alpha, dis_v_k, dis_delta_k, p)
    x_next = x_runge + 0.5 * k1
    k2 = control_period * f_c_runge_func(x_next[0], x_next[1], x_next[2], dis_v_k, dis_delta_k, p)
    x_next = x_runge + 0.5 * k2
    k3 = control_period * f_c_runge_func(x_next[0], x_next[1], x_next[2], dis_v_k, dis_delta_k, p)
    x_next = x_runge + k3
    k4 = control_period * f_c_runge_func(x_next[0], x_next[1], x_next[2], dis_v_k, dis_delta_k, p)
    x_next_runge = x_runge + (k1 + 2*k2 + 2*k3 + k4) / 6.0

    f_disc = vertcat(x_next_runge, dis_v_k, dis_delta_k)

    # ── Initial condition ─────────────────────────────────────────────────────
    model.x0 = np.array([-2.0, 0.0, 0.0, 0.0, 0.0])

    # ── External cost function ────────────────────────────────────────────────
    # Progress maximisation + lateral error + heading error + speed tracking
    # + acceleration regularisation + steering rate regularisation
    model.cost_expr_ext_cost = (
        - weight_adv * sdota / freq
        + weight_qn * (n - overtake_d)**2
        + weight_qalpha * alpha**2
        + weight_qv * (v - V_target)**2
        + weight_qac * der_v**2
        + weight_ddelta * derDelta**2
    )
    model.cost_expr_ext_cost_e = (
        - weight_adv * sdota / freq
        + weight_qn * (n - overtake_d)**2
        + weight_qalpha * alpha**2
        + weight_qv * (v - V_target)**2
    )

    # ── Nonlinear constraints ─────────────────────────────────────────────────
    a_lat = v * v * cs.tan(delta) / (lr + lf)   # lateral acceleration [m/s²]

    n_right_bound = n + right_bound_s(s_mod) - bound_inflation   # must be ≥ 0
    n_left_bound  = n - left_bound_s(s_mod)  + bound_inflation   # must be ≤ 0

    constraint.alat = cs.Function("a_lat", [x, u], [a_lat])
    constraint.pathlength = pathlength
    constraint.expr = vertcat(
        a_lat - (-a_lat_max),   # a_lat ≥ -a_lat_max
        a_lat - a_lat_max,      # a_lat ≤  a_lat_max
        n_right_bound,
        n_left_bound)
    constraint.expr_e = constraint.expr

    # ── Model bounds (read by acados_settings) ────────────────────────────────
    model.n_min        = 0.0
    model.n_max        = kmpc_config.track_max_width
    model.delta_min    = kmpc_config.delta_min
    model.delta_max    = kmpc_config.delta_max
    model.ddelta_min   = kmpc_config.ddelta_min
    model.ddelta_max   = kmpc_config.ddelta_max
    model.dv_min       = kmpc_config.a_min
    model.dv_max       = kmpc_config.a_max
    constraint.alat_max = kmpc_config.alat_max
    constraint.v_min   = kmpc_config.v_min
    constraint.v_max   = kmpc_config.v_max

    # ── Assemble model struct ─────────────────────────────────────────────────
    params = types.SimpleNamespace()
    model.f_impl_expr  = xdot - f_expl
    model.f_expl_expr  = f_expl
    model.dyn_disc_fun = f_disc
    model.x      = x
    model.n_x    = n_x
    model.xdot   = xdot
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

    return model, constraint, params
