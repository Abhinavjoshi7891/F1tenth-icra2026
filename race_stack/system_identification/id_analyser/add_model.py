#!/usr/bin/env python3
"""
Create a new vehicle model file. Edit the parameters below and run:
    python3 add_model.py
"""
from helpers.save_model import save

## ── Edit these ──────────────────────────────────────────────────────────────
model_name = 'MYCAR'
tire_model = 'pacejka'   # 'linear' or 'pacejka'

m     = 3.54             # mass [kg]
l_wb  = 0.307            # wheelbase [m]
l_f   = 0.162            # CoM → front axle [m]
l_r   = l_wb - l_f       # CoM → rear axle [m]
h_cg  = 0.014            # CoG height [m]
I_z   = 0.05797          # yaw inertia [kg·m²]

mu    = 1.0              # friction coefficient
C_d   = -1.1             # servo → steering angle gain
C_0d  = 0.48             # servo → steering angle offset
C_v   = 3500.0           # velocity → eRPM gain
C_0v  = 220.0            # eRPM offset
C_acc = 3.2              # accel cmd → current gain
C_dec = 3.2              # decel cmd → current gain
C_R   = 0.1              # velocity drag → current gain
a_min = -3.0             # min acceleration [m/s²]
a_max = 3.0              # max acceleration [m/s²]

# Pacejka coefficients [B, C, D, E] — front / rear
C_Pf = [0.1, 0.1, 0.1, 0.1]
C_Pr = [0.1, 0.1, 0.1, 0.1]

# Linear tire coefficients (used when tire_model == 'linear')
C_Sf = 4.718             # front cornering stiffness [N/rad]
C_Sr = 5.456             # rear cornering stiffness [N/rad]

overwrite_existing = False  # set True only if intentionally overwriting
## ────────────────────────────────────────────────────────────────────────────


def add_model():
    model_dict = {
        'model_name': model_name,
        'tire_model': tire_model,
        'm': m,
        'l_wb': l_wb,
        'l_f': l_f,
        'l_r': l_r,
        'h_cg': h_cg,
        'I_z': I_z,
        'mu': mu,
        'C_d': C_d,
        'C_0d': C_0d,
        'C_v': C_v,
        'C_0v': C_0v,
        'C_acc': C_acc,
        'C_dec': C_dec,
        'C_R': C_R,
        'a_min': a_min,
        'a_max': a_max,
    }
    if tire_model == 'linear':
        model_dict['C_Sf'] = C_Sf
        model_dict['C_Sr'] = C_Sr
    if tire_model == 'pacejka':
        model_dict['C_Pf'] = C_Pf
        model_dict['C_Pr'] = C_Pr

    save(model_dict, overwrite_existing, verbose=True)


if __name__ == '__main__':
    add_model()
