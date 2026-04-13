# race_stack/controller/controller/kinematic_mpc/acados_settings.py
#
# BSD 2-Clause License — original author: Daniel Kloeser / acados team
# ROS2 port: removed pbl_config import, relative import for bicycle_model
#
# Constraint structure (4 nonlinear constraints, matching bicycle_model.py):
#   constraint.expr = [a_lat - (-alat_max),   # alat >= -alat_max → lh=0
#                      a_lat - alat_max,        # alat <= alat_max  → uh=0
#                      n_right_bound,           # n + d_right - margin >= 0 → lh=0
#                      n_left_bound]            # n - d_left + margin <= 0  → uh=0

import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from .bicycle_model import bicycle_model


def acados_settings(s0, kapparef, vx_ref, d_left, d_right, kmpc_config, car_config):
    ocp = AcadosOcp()

    # Build model — returns (model, constraint, params)
    model, constraint, params = bicycle_model(
        s0, kapparef, vx_ref, d_left, d_right, kmpc_config, car_config)

    # ── acados model ──────────────────────────────────────────────────────────
    model_ac = AcadosModel()
    model_ac.f_impl_expr   = model.f_impl_expr
    model_ac.f_expl_expr   = model.f_expl_expr
    model_ac.disc_dyn_expr = model.dyn_disc_fun   # custom discrete RK4
    model_ac.con_h_expr_0  = constraint.expr
    model_ac.con_h_expr    = constraint.expr
    model_ac.con_h_expr_e  = constraint.expr_e    # same as stage (set in bicycle_model)
    model_ac.x     = model.x
    model_ac.xdot  = model.xdot
    model_ac.u     = model.u
    model_ac.z     = model.z
    model_ac.p     = model.p
    model_ac.name  = model.name
    ocp.model = model_ac

    # ── Online parameters ─────────────────────────────────────────────────────
    p = get_parameters(kmpc_config)
    params.p = p
    ocp.parameter_values = p

    # ── Dimensions ───────────────────────────────────────────────────────────
    nx  = model.x.size()[0]    # 5: [s, n, alpha, v, delta]
    nu  = model.u.size()[0]    # 2: [der_v, derDelta]
    nsh = constraint.expr.shape[0]   # 4: alat_lo, alat_hi, n_right, n_left
    ns  = nsh

    ocp.dims.N    = kmpc_config.N
    ocp.dims.ny   = 0
    ocp.dims.ny_e = 0

    # ── Cost — external (defined symbolically in bicycle_model) ───────────────
    ocp.cost.cost_type   = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost   = model.cost_expr_ext_cost
    ocp.model.cost_expr_ext_cost_e = model.cost_expr_ext_cost_e

    # Slack weight matrices — all 4 constraints are soft to prevent QP failure.
    # Lateral acceleration constraints (0,1) get high penalty; track bounds (2,3) lower.
    Nonlinear_soft_min_value = np.array([0.0, 0.0, 0.0, 0.0])
    Nonlinear_soft_max_value = np.array([0.1, 0.1, 0.1, 0.1])

    # Slack cost (L1 and L2 on constraint violation)
    ocp.cost.Zl   = 100.0 * np.ones(ns)
    ocp.cost.Zu   = 100.0 * np.ones(ns)
    ocp.cost.zl   = 10.0  * np.ones(ns)
    ocp.cost.zu   = 10.0  * np.ones(ns)
    ocp.cost.Zl_e = 100.0 * np.ones(ns)
    ocp.cost.Zu_e = 100.0 * np.ones(ns)
    ocp.cost.zl_e = 10.0  * np.ones(ns)
    ocp.cost.zu_e = 10.0  * np.ones(ns)

    # ── Nonlinear constraint bounds ────────────────────────────────────────────
    # Constraint vector: [a_lat-(-alat_max), a_lat-alat_max, n_right, n_left]
    #
    # expr[0] = a_lat + alat_max >= 0  → lh[0]=0,    uh[0]=+INF
    # expr[1] = a_lat - alat_max <= 0  → lh[1]=-INF, uh[1]=0
    # expr[2] = n_right_bound    >= 0  → lh[2]=0,    uh[2]=+INF
    # expr[3] = n_left_bound     <= 0  → lh[3]=-INF, uh[3]=0
    #
    # IMPORTANT: uh[0]/uh[2] must NOT be n_max (3.0)!
    # At rest: expr[0] = 0 + 7.85 = 7.85 > 3.0 → immediate infeasibility.
    INF = 1e6

    Nonlinear_constraint_min = np.array([0.0,  -INF, 0.0,  -INF])
    Nonlinear_constraint_max = np.array([INF,   0.0, INF,   0.0 ])
    Nonlinear_constraint_index = np.arange(nsh)

    # Stage 0
    ocp.constraints.lh_0   = Nonlinear_constraint_min
    ocp.constraints.uh_0   = Nonlinear_constraint_max
    ocp.constraints.lsh_0  = Nonlinear_soft_min_value
    ocp.constraints.ush_0  = Nonlinear_soft_max_value
    ocp.constraints.idxsh_0 = Nonlinear_constraint_index

    # Stages 1..N-1
    ocp.constraints.lh     = Nonlinear_constraint_min
    ocp.constraints.uh     = Nonlinear_constraint_max
    ocp.constraints.lsh    = Nonlinear_soft_min_value
    ocp.constraints.ush    = Nonlinear_soft_max_value
    ocp.constraints.idxsh  = Nonlinear_constraint_index

    # Terminal stage N
    ocp.constraints.lh_e   = Nonlinear_constraint_min
    ocp.constraints.uh_e   = Nonlinear_constraint_max
    ocp.constraints.lsh_e  = Nonlinear_soft_min_value
    ocp.constraints.ush_e  = Nonlinear_soft_max_value
    ocp.constraints.idxsh_e = Nonlinear_constraint_index

    # ── State box constraints (v, delta at stages 1..N) ───────────────────────
    # acados requires idxbx to index into the state vector
    # v is index 3, delta is index 4
    ocp.constraints.lbx   = np.array([kmpc_config.v_min, kmpc_config.delta_min])
    ocp.constraints.ubx   = np.array([kmpc_config.v_max, kmpc_config.delta_max])
    ocp.constraints.idxbx = np.array([3, 4])

    # ── Initial state constraint (fixed by lbx_0 = ubx_0 in main_loop) ────────
    ocp.constraints.x0 = model.x0

    # ── Input box constraints (der_v, derDelta) ───────────────────────────────
    ocp.constraints.lbu   = np.array([kmpc_config.a_min,     kmpc_config.ddelta_min])
    ocp.constraints.ubu   = np.array([kmpc_config.a_max,     kmpc_config.ddelta_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # ── QP solver ─────────────────────────────────────────────────────────────
    ocp.solver_options.tf               = kmpc_config.N / kmpc_config.MPC_freq
    ocp.solver_options.qp_solver        = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type  = "SQP_RTI"
    ocp.solver_options.hessian_approx   = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type  = "DISCRETE"
    ocp.solver_options.tol              = 1e-2
    ocp.solver_options.nlp_solver_tol_comp = 1e-1
    ocp.solver_options.print_level      = 0

    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_kmpc.json")

    return constraint, model, acados_solver, params


def get_parameters(kmpc_config) -> np.ndarray:
    """
    Online parameter vector p — must match symbolic p in bicycle_model.py exactly:
    [V_target, qadv, qv, qn, qalpha, qac, qddelta, alat_max,
     track_safety_margin, overtake_d]
    """
    return np.array([
        2.0,
        kmpc_config.qadv,
        kmpc_config.qv,
        kmpc_config.qn,
        kmpc_config.qalpha,
        kmpc_config.qac,
        kmpc_config.qddelta,
        kmpc_config.alat_max,
        kmpc_config.track_safety_margin,
        kmpc_config.overtake_d,
    ], dtype=np.float64)
