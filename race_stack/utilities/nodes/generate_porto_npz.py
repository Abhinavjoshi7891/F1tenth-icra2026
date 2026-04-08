#!/usr/bin/env python3
"""
generate_porto_npz.py — Offline one-time script.

Reads the ForzaETH IQP raceline from global_waypoints.json,
recomputes true geometric curvature from psi_rad heading differences
(ignoring the clamped kappa_radpm ±5.0 values in the JSON),
runs the forward-backward velocity profile solver calibrated for
the AutoDRIVE RoboRacer sim physics, and saves a porto_raceline.npz
compatible with local_map_pp_cartographer.py.

Output npz keys (same as track_map.npz from local_map_pp_v2):
  points          — (N,2) float64  x,y in Cartographer map frame
  curvatures      — (N,)  float64  geometric curvature [rad/m]
  arc_length      — (N,)  float64  cumulative arc-length s [m]
  velocity_profile — (N,) float64  target speed [m/s]

Usage:
  python3 generate_porto_npz.py \
      --json /path/to/global_waypoints.json \
      --out  /path/to/porto_raceline.npz

Then mount porto_raceline.npz into the devkit docker and point
local_map_pp_cartographer.py at it via NPZ_PATH.
"""

import argparse
import json
import math
import numpy as np


# ── AutoDRIVE RoboRacer sim physics ──────────────────────────────────────────
# From official AutoDRIVE technical specification
VEHICLE_MASS        = 3.906   # kg  (total mass)
# Lateral tire extremum: slip=0.01, normalised_force=1.00
# → peak lateral friction coeff ≈ 1.00
# A_lat_max = mu * g  (conservative: use 0.9 of peak to give safety margin)
MU_LAT              = 0.90
GRAVITY             = 9.81
A_LAT_MAX           = MU_LAT * GRAVITY     # ≈ 8.83 m/s²  (cornering limit)
A_LONG_ACCEL_MAX    = 5.0                  # m/s²  (conservative accel)
A_LONG_BRAKE_MAX    = 5.0                  # m/s²  (conservative braking)
MIN_CORNER_SPEED    = 1.0                  # m/s   (never go slower)
MAX_SPEED           = 8.0                  # m/s   (sim platform cap)

# ── Curvature smoothing ───────────────────────────────────────────────────────
SMOOTH_KERNEL       = 15   # moving-average window for curvature smoothing


def load_iqp_waypoints(json_path: str):
    """Load IQP raceline from global_waypoints.json."""
    with open(json_path, 'r') as f:
        data = json.load(f)
    wpnts = data['global_traj_wpnts_iqp']['wpnts']
    print(f'[JSON] Loaded {len(wpnts)} IQP waypoints')
    return wpnts


def compute_geometry(wpnts):
    """
    Extract (x,y,s) and recompute curvature from psi_rad heading
    using finite differences.  The kappa_radpm field in the JSON is
    clamped to ±5.0 (the QP bound) and is NOT used here.
    """
    n   = len(wpnts)
    pts = np.array([[w['x_m'], w['y_m']] for w in wpnts], dtype=np.float64)
    s   = np.array([w['s_m']             for w in wpnts], dtype=np.float64)
    psi = np.array([w['psi_rad']         for w in wpnts], dtype=np.float64)

    # Arc-length element lengths from s array (more accurate than coord diffs)
    el = np.diff(s)                              # shape (N-1,)

    # Curvature = d(psi)/ds — handle angle wrapping
    dpsi = np.diff(psi)
    dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi  # wrap to [-π, π]

    with np.errstate(divide='ignore', invalid='ignore'):
        kappa = np.where(el > 1e-6, dpsi / el, 0.0)   # shape (N-1,)

    # Smooth with moving average
    kernel = np.ones(SMOOTH_KERNEL) / SMOOTH_KERNEL
    kappa_smooth = np.convolve(np.abs(kappa), kernel, mode='same')

    # Pad to full length N
    kappa_full = np.zeros(n, dtype=np.float64)
    kappa_full[1:] = kappa_smooth
    kappa_full[0]  = kappa_full[1]

    print(f'[GEOM] Track length     : {s[-1]:.2f} m')
    print(f'[GEOM] Curvature range  : {kappa_full.min():.4f} … {kappa_full.max():.4f} rad/m')
    print(f'[GEOM] Max corner speed : {math.sqrt(A_LAT_MAX / max(kappa_full.max(), 1e-6)):.2f} m/s')
    return pts, s, kappa_full, el


def compute_velocity_profile(curvatures, el_lengths, s):
    """
    Forward-backward velocity profile solver.
    Identical algorithm to local_map_pp_v2._compute_velocity_profile()
    but uses AutoDRIVE sim physics constants.

    Step 1: lateral acceleration limit at each point
    Step 2: forward pass  — acceleration limited
    Step 3: backward pass — braking limited
    Two passes each to handle closed-track wrap-around.
    """
    n   = len(curvatures)
    vel = np.full(n, MAX_SPEED, dtype=np.float64)

    # Step 1: cornering speed limit
    for i in range(n):
        if curvatures[i] > 1e-4:
            v_corner = math.sqrt(A_LAT_MAX / curvatures[i])
            vel[i]   = min(vel[i], v_corner)
    vel = np.clip(vel, MIN_CORNER_SPEED, MAX_SPEED)

    # Step 2: forward pass (2 laps for wrap-around)
    for _ in range(2):
        for i in range(n - 1):
            ds          = el_lengths[i]
            v_next_max  = math.sqrt(max(vel[i]**2 + 2.0 * A_LONG_ACCEL_MAX * ds, 0.0))
            vel[i + 1]  = min(vel[i + 1], v_next_max)
        # wrap last → first
        ds_wrap = el_lengths[-1] if len(el_lengths) == n - 1 else 0.15
        vel[0] = min(vel[0], math.sqrt(max(vel[-1]**2 + 2.0 * A_LONG_ACCEL_MAX * ds_wrap, 0.0)))

    # Step 3: backward pass (2 laps for wrap-around)
    for _ in range(2):
        for i in range(n - 1, 0, -1):
            ds          = el_lengths[i - 1]
            v_prev_max  = math.sqrt(max(vel[i]**2 + 2.0 * A_LONG_BRAKE_MAX * ds, 0.0))
            vel[i - 1]  = min(vel[i - 1], v_prev_max)
        ds_wrap = el_lengths[-1] if len(el_lengths) == n - 1 else 0.15
        vel[-1] = min(vel[-1], math.sqrt(max(vel[0]**2 + 2.0 * A_LONG_BRAKE_MAX * ds_wrap, 0.0)))

    vel = np.clip(vel, MIN_CORNER_SPEED, MAX_SPEED)

    print(f'[VPROF] Speed range : {vel.min():.2f} … {vel.max():.2f} m/s')
    print(f'[VPROF] Mean speed  : {vel.mean():.2f} m/s')
    # Estimate lap time
    lap_time = float(np.sum(el_lengths / vel[:-1]))
    print(f'[VPROF] Est lap time: {lap_time:.2f} s')
    return vel


def main():
    parser = argparse.ArgumentParser(description='Generate porto_raceline.npz for local_map_pp_cartographer')
    parser.add_argument('--json', required=True,
                        help='Path to global_waypoints.json')
    parser.add_argument('--out',  required=True,
                        help='Output path for porto_raceline.npz')
    args = parser.parse_args()

    # 1. Load
    wpnts = load_iqp_waypoints(args.json)

    # 2. Geometry
    pts, s, curvatures, el_lengths = compute_geometry(wpnts)

    # 3. Velocity profile
    velocity_profile = compute_velocity_profile(curvatures, el_lengths, s)

    # 4. Save — same four keys as track_map.npz from local_map_pp_v2
    np.savez(
        args.out,
        points          = pts,
        curvatures      = curvatures,
        arc_length      = s,
        velocity_profile = velocity_profile,
    )
    print(f'\n[DONE] Saved {len(pts)} waypoints → {args.out}')
    print(f'       Keys: points{pts.shape}  curvatures{curvatures.shape}  '
          f'arc_length{s.shape}  velocity_profile{velocity_profile.shape}')


if __name__ == '__main__':
    main()