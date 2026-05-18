# race_stack/controller/controller/single_track_mpc/acados_settings.py
#
# BSD 2-Clause License — original author: Daniel Kloeser / acados team
# ROS2 port: removed pbl_config import, relative import for bicycle_model

import numpy as np
from typing import Tuple
from .bicycle_model import bicycle_model
from .utils.indicies import StateIndex


def acados_settings(s0, kapparef, d_left, d_right,
                    stmpc_config, car_config, tire_config) -> Tuple:
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
    ocp = AcadosOcp()

    model, constraint, params = bicycle_model(
        s0, kapparef, d_left, d_right, stmpc_config, car_config, tire_config)

    # ── acados model — ERK integrator (continuous dynamics, no disc_dyn_expr) ─
    model_ac = AcadosModel()
    model_ac.f_expl_expr  = model.f_expl_expr
    model_ac.con_h_expr_0 = constraint.expr
    model_ac.con_h_expr   = constraint.expr
    model_ac.con_h_expr_e = constraint.expr_e
    model_ac.x    = model.x
    model_ac.u    = model.u
    model_ac.z    = model.z
    model_ac.p    = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    # ── Online parameters ─────────────────────────────────────────────────────
    p = get_parameters(stmpc_config)
    params.p = p
    ocp.parameter_values = p

    # ── Dimensions ───────────────────────────────────────────────────────────
    nx    = model.n_x        # 8
    nu    = model.n_u        # 2
    nsh   = constraint.expr.shape[0]    # 3: n_right, n_left, combined
    nsh_e = constraint.expr_e.shape[0]  # 2: n_right, n_left (terminal only)

    # State box indices: v_x (3), delta (5), accel (7)
    state_constraint_index = np.array([
        StateIndex.VELOCITY_V_X.value,
        StateIndex.STEERING_ANGLE_DELTA.value,
        StateIndex.ACCEL.value,
    ])
    state_constraint_min = np.array([constraint.v_x_min, constraint.delta_min, constraint.a_min])
    state_constraint_max = np.array([constraint.v_x_max, constraint.delta_max, constraint.a_max])

    ocp.dims.N    = stmpc_config.N
    ocp.dims.nx   = nx
    ocp.dims.nu   = nu
    ocp.dims.ny   = 0
    ocp.dims.ny_e = 0

    # ── Cost — external, with separate stage-0 cost ───────────────────────────
    ocp.cost.cost_type   = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost_0 = model.cost_expr_ext_cost_0
    ocp.model.cost_expr_ext_cost   = model.cost_expr_ext_cost
    ocp.model.cost_expr_ext_cost_e = model.cost_expr_ext_cost_e

    # ── Slack costs for nonlinear and box constraints ──────────────────────────
    # Dimensions: ns = nsh + ns_box
    ns_box   = state_constraint_index.shape[0] # 3
    ns_box_e = state_constraint_index.shape[0] # 3
    ns    = nsh + ns_box      # total slacks per node (typically 6)
    ns_e  = nsh_e + ns_box_e  # total slacks per terminal node (typically 5)
    ns_0  = nsh               # total slacks at node 0 (only nonlinear slacks, typically 3)

    ocp.cost.Zl_0 = stmpc_config.Zl * np.ones((ns_0,))
    ocp.cost.Zu_0 = stmpc_config.Zu * np.ones((ns_0,))
    ocp.cost.zl_0 = stmpc_config.zl * np.ones((ns_0,))
    ocp.cost.zu_0 = stmpc_config.zu * np.ones((ns_0,))

    ocp.cost.Zl   = stmpc_config.Zl * np.ones((ns,))
    ocp.cost.Zu   = stmpc_config.Zu * np.ones((ns,))
    ocp.cost.zl   = stmpc_config.zl * np.ones((ns,))
    ocp.cost.zu   = stmpc_config.zu * np.ones((ns,))

    ocp.cost.Zl_e = stmpc_config.Zl * np.ones((ns_e,))
    ocp.cost.Zu_e = stmpc_config.Zu * np.ones((ns_e,))
    ocp.cost.zl_e = stmpc_config.zl * np.ones((ns_e,))
    ocp.cost.zu_e = stmpc_config.zu * np.ones((ns_e,))

    # ── Input box constraints: [jerk, derDelta] ───────────────────────────────
    ocp.constraints.lbu_0   = np.array([constraint.jerk_min, constraint.ddelta_min])
    ocp.constraints.ubu_0   = np.array([constraint.jerk_max, constraint.ddelta_max])
    ocp.constraints.idxbu_0 = np.array([0, 1])

    ocp.constraints.lbu    = np.array([constraint.jerk_min, constraint.ddelta_min])
    ocp.constraints.ubu    = np.array([constraint.jerk_max, constraint.ddelta_max])
    ocp.constraints.idxbu  = np.array([0, 1])

    ocp.constraints.lbu_e   = np.array([constraint.jerk_min, constraint.ddelta_min])
    ocp.constraints.ubu_e   = np.array([constraint.jerk_max, constraint.ddelta_max])
    ocp.constraints.idxbu_e = np.array([0, 1])

    # ── State box constraints: [v_x, delta, accel] at stages 1..N ────────────
    # v_x = index 3, delta = index 5, accel = index 7

    ocp.constraints.idxbx_0 = np.arange(nx)
    ocp.constraints.lbx_0   = np.zeros(nx)
    ocp.constraints.ubx_0   = np.zeros(nx)

    ocp.constraints.lbx   = state_constraint_min
    ocp.constraints.ubx   = state_constraint_max
    ocp.constraints.idxbx = state_constraint_index
    ocp.constraints.idxsbx = np.arange(state_constraint_index.shape[0])

    ocp.constraints.lbx_e   = state_constraint_min
    ocp.constraints.ubx_e   = state_constraint_max
    ocp.constraints.idxbx_e = state_constraint_index
    ocp.constraints.idxsbx_e = np.arange(state_constraint_index.shape[0])

    # ── Nonlinear constraint bounds ────────────────────────────────────────────
    # constraint.expr = [n_right_bound, n_left_bound, combined_constraints]
    # n_right_bound = n + d_right - margin >= 0   → lh=0, uh=INF
    # n_left_bound  = n - d_left  + margin <= 0   → lh=-INF, uh=0
    # combined_constraints (ellipse/diamond value) <= 1 → lh=0, uh=1
    INF = 1e6

    Nonlinear_constraint_min = np.array([0.0,  -INF, 0.0])
    Nonlinear_constraint_max = np.array([INF,   0.0, 1.0])
    Nonlinear_constraint_index = np.arange(nsh)

    # Terminal constraint bounds (only n_right, n_left — no combined)
    Nonlinear_constraint_min_e = np.array([0.0,  -INF])
    Nonlinear_constraint_max_e = np.array([INF,   0.0])
    Nonlinear_constraint_index_e = np.arange(nsh_e)

    # stage 0
    ocp.constraints.lh_0    = Nonlinear_constraint_min
    ocp.constraints.uh_0    = Nonlinear_constraint_max
    ocp.constraints.idxsh_0 = Nonlinear_constraint_index

    # stages 1..N-1
    ocp.constraints.lh    = Nonlinear_constraint_min
    ocp.constraints.uh    = Nonlinear_constraint_max
    ocp.constraints.idxsh = Nonlinear_constraint_index

    # terminal stage N
    ocp.constraints.lh_e    = Nonlinear_constraint_min_e
    ocp.constraints.uh_e    = Nonlinear_constraint_max_e
    ocp.constraints.idxsh_e = Nonlinear_constraint_index_e

    # ── Initial state (fixed by lbx/ubx in main_loop) ────────────────────────
    ocp.constraints.x0 = model.x0

    # ── QP solver — ERK integrator (continuous dynamics) ─────────────────────
    ocp.solver_options.tf                  = stmpc_config.N / stmpc_config.MPC_freq
    ocp.solver_options.qp_solver           = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type     = "SQP_RTI"
    ocp.solver_options.hessian_approx      = "GAUSS_NEWTON"
    # NOTE: do NOT set hessian_approx to "EXACT" — it causes solver failure
    ocp.solver_options.integrator_type     = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps  = 3
    ocp.solver_options.tol                 = 1e-2
    ocp.solver_options.print_level         = 0

    # IMPORTANT: different JSON filename from KMPC to avoid collision
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_stmpc.json")

    return constraint, model, acados_solver, params


def get_parameters(stmpc_config) -> np.ndarray:
    """
    Online parameter vector p — must match symbolic p in bicycle_model.py exactly:
    [V_target, qadv, qv, qn, qalpha, qjerk, qddelta,
     alat_max, a_min, a_max, track_safety_margin, overtake_d]
    """
    return np.array([
        2.0,
        stmpc_config.qadv,
        stmpc_config.qv,
        stmpc_config.qn,
        stmpc_config.qalpha,
        stmpc_config.qjerk,
        stmpc_config.qddelta,
        stmpc_config.alat_max,
        stmpc_config.a_min,
        stmpc_config.a_max,
        stmpc_config.track_safety_margin,
        stmpc_config.overtake_d,
    ], dtype=np.float64)
