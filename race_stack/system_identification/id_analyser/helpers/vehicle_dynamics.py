import math


def vehicle_dynamics(x, uInit, p):
    """
    Single-track vehicle dynamics (Pacejka tire model).
    p: DotDict loaded from a model .txt file.
       Required fields: mu, C_Pf, C_Pr, l_f, l_r, h_cg, m, I_z
       C_Pf / C_Pr are lists [B, C, D, E].
    """
    g = 9.81

    mu = p.mu
    B_f, C_f, D_f, E_f = p.C_Pf
    B_r, C_r, D_r, E_r = p.C_Pr

    lf = p.l_f
    lr = p.l_r
    h  = p.h_cg
    m  = p.m
    I  = p.I_z

    u = uInit

    alpha_f = -math.atan((x[4] + x[5] * lf) / x[3]) + u[0]
    alpha_r = -math.atan((x[4] - x[5] * lr) / x[3])

    F_zf = m * (-u[1] * h + g * lr) / (lr + lf)
    F_zr = m * (u[1] * h + g * lf) / (lr + lf)

    F_yf = mu * F_zf * D_f * math.sin(
        C_f * math.atan(B_f * alpha_f - E_f * (B_f * alpha_f - math.atan(B_f * alpha_f)))
    )
    F_yr = mu * F_zr * D_r * math.sin(
        C_r * math.atan(B_r * alpha_r - E_r * (B_r * alpha_r - math.atan(B_r * alpha_r)))
    )

    return [
        x[3] * math.cos(x[2]) - x[4] * math.sin(x[2]),
        x[3] * math.sin(x[2]) + x[4] * math.cos(x[2]),
        x[5],
        u[1],
        1 / m * (F_yr + F_yf) - x[3] * x[5],
        1 / I * (-lr * F_yr + lf * F_yf),
    ]
