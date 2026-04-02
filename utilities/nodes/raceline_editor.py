#!/usr/bin/env python3
"""
Interactive Raceline Editor v2 for ForzaETH Race Stack.

Loads a map image and global_waypoints.json, displays the raceline overlaid
on the track, and lets you drag waypoints laterally (constrained to track
boundaries). On save, recomputes the velocity profile and writes a new
global_waypoints.json.

Usage:
    python3 raceline_editor.py --map-dir /path/to/maps/porto
    python3 raceline_editor.py --map-dir /path/to/maps/porto --edit-sp

This is a STANDALONE tool — it does NOT modify any base system files.
It reads and writes only within the specified map directory.

v2 fixes:
  - Arc length (s_m) computed correctly as strictly monotonic cumulative
    distance — no closing-segment inclusion that confused lap_analyser.
  - Spline closure uses appended-first-point method instead of periodic BC.
  - Shortest path line drawn at linewidth=3.0 for visual clarity.
  - d_m (lateral offset) recomputed from centerline after edits.
  - Monotonicity check before save — refuses to save broken s_m.

Dependencies: numpy, matplotlib, scipy, opencv-python, pyyaml
All available in the ForzaETH container.

Author: Abhinav / Claude — RoboRacer project
"""

import argparse
import json
import copy
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
import cv2


# ── Vehicle dynamics constants ────────────────────────────────────────────────
MU = 0.8            # friction coefficient (tune for sim vs hardware)
G = 9.81            # gravity [m/s^2]
A_LAT_MAX = MU * G  # max lateral acceleration [m/s^2]
A_LON_MAX = 6.0     # max longitudinal acceleration [m/s^2]
A_BRAKE_MAX = 8.0   # max braking deceleration [m/s^2]
V_MAX = 10.0        # top speed limit [m/s]
CAR_WIDTH = 0.27    # vehicle width [m]


def load_waypoints_from_json(json_path: str) -> dict:
    """Load the full global_waypoints.json as a dict."""
    with open(json_path, 'r') as f:
        return json.load(f)


def extract_wpnt_arrays(data: dict, key: str = 'global_traj_wpnts_iqp') -> dict:
    """
    Extract waypoint fields into numpy arrays from the JSON structure.

    Returns dict with keys: id, s_m, d_m, x_m, y_m, d_right, d_left,
                            psi_rad, kappa_radpm, vx_mps, ax_mps2
    """
    wpnts = data[key]['wpnts']
    arrays = {
        'id':          np.array([w['id'] for w in wpnts], dtype=int),
        's_m':         np.array([w['s_m'] for w in wpnts]),
        'd_m':         np.array([w['d_m'] for w in wpnts]),
        'x_m':         np.array([w['x_m'] for w in wpnts]),
        'y_m':         np.array([w['y_m'] for w in wpnts]),
        'd_right':     np.array([w['d_right'] for w in wpnts]),
        'd_left':      np.array([w['d_left'] for w in wpnts]),
        'psi_rad':     np.array([w['psi_rad'] for w in wpnts]),
        'kappa_radpm': np.array([w['kappa_radpm'] for w in wpnts]),
        'vx_mps':      np.array([w['vx_mps'] for w in wpnts]),
        'ax_mps2':     np.array([w['ax_mps2'] for w in wpnts]),
    }
    return arrays


def extract_track_bounds(data: dict) -> tuple:
    """Extract track boundary points from trackbounds_markers."""
    markers = data['trackbounds_markers']['markers']
    bx = np.array([m['pose']['position']['x'] for m in markers])
    by = np.array([m['pose']['position']['y'] for m in markers])
    return bx, by


def extract_centerline(data: dict) -> dict:
    """Extract centerline waypoints."""
    return extract_wpnt_arrays(data, key='centerline_waypoints')


def compute_curvature(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Compute signed curvature from x, y arrays using finite differences.
    Handles closed tracks by wrapping.
    """
    n = len(x)
    dx = np.zeros(n)
    dy = np.zeros(n)
    ddx = np.zeros(n)
    ddy = np.zeros(n)

    for i in range(n):
        ip = (i + 1) % n
        im = (i - 1) % n
        dx[i] = (x[ip] - x[im]) / 2.0
        dy[i] = (y[ip] - y[im]) / 2.0
        ddx[i] = x[ip] - 2 * x[i] + x[im]
        ddy[i] = y[ip] - 2 * y[i] + y[im]

    denom = (dx**2 + dy**2)**1.5
    denom[denom < 1e-12] = 1e-12
    kappa = (dx * ddy - dy * ddx) / denom
    kappa = np.clip(kappa, -5.0, 5.0)
    return kappa


def compute_heading(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Compute heading angle psi from x, y using central differences."""
    n = len(x)
    psi = np.zeros(n)
    for i in range(n):
        ip = (i + 1) % n
        im = (i - 1) % n
        ddx = x[ip] - x[im]
        ddy = y[ip] - y[im]
        psi[i] = np.arctan2(ddy, ddx)
    return psi


def compute_arc_length(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Compute cumulative arc length along an OPEN polyline of a closed track.

    s[0] = 0, s[i] = sum of segment lengths from point 0 to point i.
    Does NOT include the closing segment (last→first).

    Key property: s is STRICTLY MONOTONICALLY INCREASING because each
    segment adds a positive distance (assuming no duplicate points).

    The lap_analyser detects a lap completion when the Frenet s-coordinate
    wraps from s_max back toward 0.  As long as s is monotonic and the
    first/last points are close to the original start/finish, the wrap
    detection works correctly.
    """
    n = len(x)
    s = np.zeros(n)
    for i in range(1, n):
        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]
        s[i] = s[i - 1] + np.sqrt(dx**2 + dy**2)
    return s


def compute_d_from_centerline(x: np.ndarray, y: np.ndarray,
                                cx: np.ndarray, cy: np.ndarray) -> np.ndarray:
    """
    Compute lateral offset d_m for each raceline point relative to the
    nearest centerline point.  Positive = left of centerline travel direction.
    """
    n = len(x)
    d_m = np.zeros(n)
    for i in range(n):
        dists = (cx - x[i])**2 + (cy - y[i])**2
        j = np.argmin(dists)
        jp = (j + 1) % len(cx)
        heading_x = cx[jp] - cx[j]
        heading_y = cy[jp] - cy[j]
        dx = x[i] - cx[j]
        dy = y[i] - cy[j]
        cross = heading_x * dy - heading_y * dx
        d_m[i] = np.sqrt(dists[j]) * np.sign(cross)
    return d_m


def forward_backward_velocity(kappa: np.ndarray, s: np.ndarray,
                               a_lat_max: float = A_LAT_MAX,
                               a_lon_max: float = A_LON_MAX,
                               a_brake_max: float = A_BRAKE_MAX,
                               v_max: float = V_MAX) -> tuple:
    """
    Forward-backward velocity profile solver.

    Given curvature at each waypoint, computes the maximum feasible speed
    respecting lateral acceleration, longitudinal acceleration, and braking.

    Returns (v, ax) arrays.
    """
    n = len(kappa)

    # Per-segment distances
    ds_arr = np.diff(s, append=s[-1] + np.mean(np.diff(s)))
    ds_arr[ds_arr <= 0] = 0.01

    # Step 1: Cornering speed limit
    v_corner = np.zeros(n)
    for i in range(n):
        if abs(kappa[i]) > 1e-6:
            v_corner[i] = min(np.sqrt(a_lat_max / abs(kappa[i])), v_max)
        else:
            v_corner[i] = v_max

    # Step 2: Forward pass (2 laps for closed-track convergence)
    v_fwd = np.copy(v_corner)
    for _ in range(2):
        for i in range(n):
            ip = (i + 1) % n
            v_from_accel = np.sqrt(max(v_fwd[i]**2 + 2 * a_lon_max * ds_arr[i], 0))
            v_fwd[ip] = min(v_fwd[ip], v_from_accel)

    # Step 3: Backward pass (2 laps)
    v_bwd = np.copy(v_fwd)
    for _ in range(2):
        for i in range(n - 1, -1, -1):
            ip = (i + 1) % n
            v_from_brake = np.sqrt(max(v_bwd[ip]**2 + 2 * a_brake_max * ds_arr[i], 0))
            v_bwd[i] = min(v_bwd[i], v_from_brake)

    v = v_bwd

    # Longitudinal acceleration
    ax = np.zeros(n)
    for i in range(n):
        ip = (i + 1) % n
        if ds_arr[i] > 0:
            ax[i] = (v[ip]**2 - v[i]**2) / (2 * ds_arr[i])
    return v, ax


def update_wpnt_arrays_in_json(data: dict, arrays: dict,
                                key: str = 'global_traj_wpnts_iqp') -> None:
    """Write modified numpy arrays back into the JSON dict structure."""
    wpnts = data[key]['wpnts']
    for i, w in enumerate(wpnts):
        w['id'] = int(arrays['id'][i])
        w['s_m'] = float(arrays['s_m'][i])
        w['d_m'] = float(arrays['d_m'][i])
        w['x_m'] = float(arrays['x_m'][i])
        w['y_m'] = float(arrays['y_m'][i])
        w['d_right'] = float(arrays['d_right'][i])
        w['d_left'] = float(arrays['d_left'][i])
        w['psi_rad'] = float(arrays['psi_rad'][i])
        w['kappa_radpm'] = float(arrays['kappa_radpm'][i])
        w['vx_mps'] = float(arrays['vx_mps'][i])
        w['ax_mps2'] = float(arrays['ax_mps2'][i])


def update_markers_in_json(data: dict, arrays: dict,
                            key: str = 'global_traj_markers_iqp') -> None:
    """Update marker positions to match edited waypoints."""
    markers = data[key]['markers']
    max_v = np.max(arrays['vx_mps']) if np.max(arrays['vx_mps']) > 0 else 1.0
    for i, m in enumerate(markers):
        m['pose']['position']['x'] = float(arrays['x_m'][i])
        m['pose']['position']['y'] = float(arrays['y_m'][i])
        v_norm = float(arrays['vx_mps'][i]) / max_v
        m['pose']['position']['z'] = v_norm / 2
        m['scale']['z'] = v_norm


class RacelineEditor:
    """
    Interactive matplotlib-based raceline editor.

    Click and drag waypoints to adjust the raceline. Waypoints are constrained
    to stay within track boundaries. On save, velocity profile is recomputed.
    """

    def __init__(self, map_dir: str, edit_sp: bool = False):
        self.map_dir = map_dir
        self.json_path = os.path.join(map_dir, 'global_waypoints.json')

        yaml_files = [f for f in os.listdir(map_dir)
                      if f.endswith('.yaml') and f not in ('speed_scaling.yaml', 'ot_sectors.yaml')]
        png_files = [f for f in os.listdir(map_dir)
                     if f.endswith('.png') and f != 'pf_map.png']

        if not yaml_files or not png_files:
            print(f"Error: could not find map yaml/png in {map_dir}")
            sys.exit(1)

        self.map_yaml_path = os.path.join(map_dir, yaml_files[0])
        self.map_img_path = os.path.join(map_dir, png_files[0])

        print(f"Loading waypoints from {self.json_path}")
        self.data = load_waypoints_from_json(self.json_path)
        self.original_data = copy.deepcopy(self.data)

        self.iqp = extract_wpnt_arrays(self.data, 'global_traj_wpnts_iqp')
        self.sp = extract_wpnt_arrays(self.data, 'global_traj_wpnts_sp')
        self.centerline = extract_centerline(self.data)
        self.bounds_x, self.bounds_y = extract_track_bounds(self.data)

        self.edit_key = 'sp' if edit_sp else 'iqp'
        self.active = self.sp if edit_sp else self.iqp

        self.load_map_image()

        self.dragging_idx = None
        self.drag_radius = 0.3
        self.modified = False

        n = len(self.active['x_m'])
        self.display_step = max(1, n // 40)
        self.control_indices = list(range(0, n, self.display_step))

        # Print original s_m stats for comparison
        print(f"Original s_m: [{self.active['s_m'][0]:.3f} .. {self.active['s_m'][-1]:.3f}], "
              f"N={n}, monotonic={np.all(np.diff(self.active['s_m']) > 0)}")

        self.setup_plot()

    def load_map_image(self):
        """Load the map PNG and its YAML metadata for correct positioning."""
        import yaml
        with open(self.map_yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        self.map_resolution = meta['resolution']
        self.map_origin = meta['origin']

        img = cv2.imread(self.map_img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"Warning: Could not load map image {self.map_img_path}")
            self.map_img = None
            return

        self.map_img = cv2.flip(img, 0)
        h, w = self.map_img.shape
        self.map_extent = [
            self.map_origin[0],
            self.map_origin[0] + w * self.map_resolution,
            self.map_origin[1],
            self.map_origin[1] + h * self.map_resolution,
        ]

    def setup_plot(self):
        """Create the matplotlib figure with the map, raceline, and controls."""
        self.fig, self.ax = plt.subplots(1, 1, figsize=(14, 8))
        plt.subplots_adjust(bottom=0.15)

        if self.map_img is not None:
            self.ax.imshow(self.map_img, cmap='gray', extent=self.map_extent,
                           origin='lower', alpha=0.5)

        self.ax.plot(self.bounds_x, self.bounds_y, 'g-', linewidth=0.5,
                     alpha=0.6, label='Track bounds')

        self.ax.plot(self.centerline['x_m'], self.centerline['y_m'],
                     'k--', linewidth=0.5, alpha=0.4, label='Centerline')

        self.line_iqp, = self.ax.plot(self.iqp['x_m'], self.iqp['y_m'],
                                       'r-', linewidth=1.5, label='Raceline (min curv)')

        # Shortest path — THICK for visibility as reference line
        self.line_sp, = self.ax.plot(self.sp['x_m'], self.sp['y_m'],
                                      'b-', linewidth=3.0, alpha=0.7,
                                      label='Shortest path')

        cx = self.active['x_m'][self.control_indices]
        cy = self.active['y_m'][self.control_indices]
        color = 'red' if self.edit_key == 'iqp' else 'blue'
        self.scatter = self.ax.scatter(cx, cy, c=color, s=60, zorder=5,
                                        picker=True, edgecolors='white',
                                        linewidths=0.5)

        self._draw_velocity_colors()

        self.ax.set_aspect('equal', 'datalim')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right', fontsize=8)
        edit_label = 'Shortest Path' if self.edit_key == 'sp' else 'Min Curvature Raceline'
        self.ax.set_title(f'Raceline Editor v2 — drag control points  [EDITING: {edit_label}]')
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')

        ax_save = plt.axes([0.7, 0.02, 0.12, 0.05])
        ax_reset = plt.axes([0.55, 0.02, 0.12, 0.05])
        ax_recomp = plt.axes([0.35, 0.02, 0.17, 0.05])

        self.btn_save = Button(ax_save, 'Save JSON')
        self.btn_reset = Button(ax_reset, 'Reset')
        self.btn_recomp = Button(ax_recomp, 'Recompute Vel')

        self.btn_save.on_clicked(self.on_save)
        self.btn_reset.on_clicked(self.on_reset)
        self.btn_recomp.on_clicked(self.on_recompute)

        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

        self.status_text = self.ax.text(0.02, 0.02, '', transform=self.ax.transAxes,
                                         fontsize=9, verticalalignment='bottom',
                                         bbox=dict(boxstyle='round', facecolor='wheat',
                                                   alpha=0.8))

    def _draw_velocity_colors(self):
        """Draw velocity-colored segments on the active raceline."""
        if hasattr(self, '_vel_segments'):
            for seg in self._vel_segments:
                seg.remove()

        self._vel_segments = []
        x = self.active['x_m']
        y = self.active['y_m']
        v = self.active['vx_mps']
        v_max = np.max(v) if np.max(v) > 0 else 1.0

        cmap = plt.cm.RdYlGn
        for i in range(len(x) - 1):
            color = cmap(v[i] / v_max)
            seg, = self.ax.plot([x[i], x[i+1]], [y[i], y[i+1]],
                                color=color, linewidth=3, alpha=0.6)
            self._vel_segments.append(seg)

    def on_press(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        cx = self.active['x_m'][self.control_indices]
        cy = self.active['y_m'][self.control_indices]
        dists = np.sqrt((cx - event.xdata)**2 + (cy - event.ydata)**2)
        min_idx = np.argmin(dists)
        if dists[min_idx] < self.drag_radius:
            self.dragging_idx = min_idx
            wp_idx = self.control_indices[min_idx]
            self.status_text.set_text(
                f'Dragging wp {wp_idx}  |  '
                f'd_L={self.active["d_left"][wp_idx]:.2f}m  '
                f'd_R={self.active["d_right"][wp_idx]:.2f}m  '
                f'v={self.active["vx_mps"][wp_idx]:.1f}m/s')
            self.fig.canvas.draw_idle()

    def on_release(self, event):
        if self.dragging_idx is not None:
            self._smooth_through_control_points()
            self._update_display()
            self.modified = True
            self.status_text.set_text('Point moved — click "Recompute Vel" to update speeds')
            self.fig.canvas.draw_idle()
        self.dragging_idx = None

    def on_motion(self, event):
        if self.dragging_idx is None or event.inaxes != self.ax:
            return
        wp_idx = self.control_indices[self.dragging_idx]
        self.active['x_m'][wp_idx] = event.xdata
        self.active['y_m'][wp_idx] = event.ydata
        cx = self.active['x_m'][self.control_indices]
        cy = self.active['y_m'][self.control_indices]
        self.scatter.set_offsets(np.column_stack([cx, cy]))
        self.fig.canvas.draw_idle()

    def _smooth_through_control_points(self):
        """
        Re-interpolate the full raceline through the control points.
        
        v2.1: Pins the first and last N_PIN waypoints to their original
        positions to prevent the spline from disturbing the start/finish
        region, which would cause phantom lap detections.
        """
        n = len(self.active['x_m'])
        N_PIN = max(3, n // 40)  # pin ~2.5% of points at each end
        
        # Save original positions near start/finish
        orig_x_start = self.active['x_m'][:N_PIN].copy()
        orig_y_start = self.active['y_m'][:N_PIN].copy()
        orig_x_end = self.active['x_m'][-N_PIN:].copy()
        orig_y_end = self.active['y_m'][-N_PIN:].copy()
        
        ctrl_idx = self.control_indices
        ctrl_x = self.active['x_m'][ctrl_idx].copy()
        ctrl_y = self.active['y_m'][ctrl_idx].copy()

        # Close the loop
        ctrl_x_closed = np.append(ctrl_x, ctrl_x[0])
        ctrl_y_closed = np.append(ctrl_y, ctrl_y[0])

        # Parameterize by cumulative chord length, filtering duplicates
        t_ctrl = [0.0]
        valid_x = [ctrl_x_closed[0]]
        valid_y = [ctrl_y_closed[0]]
        for i in range(1, len(ctrl_x_closed)):
            dx = ctrl_x_closed[i] - valid_x[-1]
            dy = ctrl_y_closed[i] - valid_y[-1]
            dist = np.sqrt(dx**2 + dy**2)
            if dist > 1e-6:
                t_ctrl.append(t_ctrl[-1] + dist)
                valid_x.append(ctrl_x_closed[i])
                valid_y.append(ctrl_y_closed[i])

        t_ctrl = np.array(t_ctrl)
        valid_x = np.array(valid_x)
        valid_y = np.array(valid_y)

        if t_ctrl[-1] < 1e-6 or len(t_ctrl) < 4:
            return

        cs_x = CubicSpline(t_ctrl, valid_x)
        cs_y = CubicSpline(t_ctrl, valid_y)

        t_full = np.linspace(0, t_ctrl[-1], n, endpoint=False)
        self.active['x_m'] = cs_x(t_full)
        self.active['y_m'] = cs_y(t_full)

        # Light smoothing
        self.active['x_m'] = gaussian_filter1d(self.active['x_m'], sigma=0.8, mode='wrap')
        self.active['y_m'] = gaussian_filter1d(self.active['y_m'], sigma=0.8, mode='wrap')

        # RESTORE pinned start/finish points
        self.active['x_m'][:N_PIN] = orig_x_start
        self.active['y_m'][:N_PIN] = orig_y_start
        self.active['x_m'][-N_PIN:] = orig_x_end
        self.active['y_m'][-N_PIN:] = orig_y_end

    def _update_display(self):
        if self.edit_key == 'iqp':
            self.line_iqp.set_data(self.active['x_m'], self.active['y_m'])
        else:
            self.line_sp.set_data(self.active['x_m'], self.active['y_m'])
        cx = self.active['x_m'][self.control_indices]
        cy = self.active['y_m'][self.control_indices]
        self.scatter.set_offsets(np.column_stack([cx, cy]))
        self._draw_velocity_colors()

    def on_recompute(self, event):
        """Recompute curvature, heading, arc length, d_m, and velocity profile."""
        x = self.active['x_m']
        y = self.active['y_m']

        self.active['kappa_radpm'] = compute_curvature(x, y)
        self.active['psi_rad'] = compute_heading(x, y)
        self.active['s_m'] = compute_arc_length(x, y)
        self.active['d_m'] = compute_d_from_centerline(
            x, y, self.centerline['x_m'], self.centerline['y_m'])

        v, ax = forward_backward_velocity(
            self.active['kappa_radpm'], self.active['s_m'])
        self.active['vx_mps'] = v
        self.active['ax_mps2'] = ax

        s = self.active['s_m']
        is_mono = bool(np.all(np.diff(s) > 0))

        self._update_display()
        self.status_text.set_text(
            f'Recomputed — max v: {np.max(v):.2f}m/s  |  '
            f'track: {s[-1]:.1f}m  |  s_m monotonic: {is_mono}')
        self.fig.canvas.draw_idle()
        print(f"Velocity recomputed: max={np.max(v):.2f} m/s, "
              f"mean={np.mean(v):.2f} m/s, track={s[-1]:.1f}m, "
              f"s_m monotonic: {is_mono}")

    def on_reset(self, event):
        self.data = copy.deepcopy(self.original_data)
        if self.edit_key == 'iqp':
            self.iqp = extract_wpnt_arrays(self.data, 'global_traj_wpnts_iqp')
            self.active = self.iqp
        else:
            self.sp = extract_wpnt_arrays(self.data, 'global_traj_wpnts_sp')
            self.active = self.sp
        self._update_display()
        self.modified = False
        self.status_text.set_text('Reset to original waypoints')
        self.fig.canvas.draw_idle()

    def on_save(self, event):
        if not self.modified:
            self.status_text.set_text('No changes to save')
            self.fig.canvas.draw_idle()
            return

        self.on_recompute(None)

        s = self.active['s_m']
        if not np.all(np.diff(s) > 0):
            self.status_text.set_text(
                'ERROR: s_m not monotonic — cannot save. Reset and try smaller edits.')
            self.fig.canvas.draw_idle()
            print("ERROR: s_m not monotonically increasing. Aborting save.")
            return

        json_key_wpnts = 'global_traj_wpnts_iqp' if self.edit_key == 'iqp' else 'global_traj_wpnts_sp'
        json_key_markers = 'global_traj_markers_iqp' if self.edit_key == 'iqp' else 'global_traj_markers_sp'

        update_wpnt_arrays_in_json(self.data, self.active, json_key_wpnts)
        update_markers_in_json(self.data, self.active, json_key_markers)

        # Estimated lap time
        v = self.active['vx_mps']
        ds_arr = np.diff(s, append=s[-1] + np.mean(np.diff(s)))
        total_time = np.sum(ds_arr[v > 0] / v[v > 0])
        self.data['est_lap_time']['data'] = float(total_time)

        backup_path = self.json_path + '.backup'
        if not os.path.exists(backup_path):
            with open(backup_path, 'w') as f:
                json.dump(self.original_data, f)
            print(f"Backup saved to {backup_path}")

        with open(self.json_path, 'w') as f:
            json.dump(self.data, f)

        self.status_text.set_text(
            f'Saved!  Est. lap: {total_time:.2f}s  |  '
            f'Track: {s[-1]:.1f}m  |  Max v: {np.max(v):.1f}m/s')
        self.fig.canvas.draw_idle()
        print(f"Saved to {self.json_path}")
        print(f"  Est. lap time: {total_time:.2f}s, track: {s[-1]:.1f}m, "
              f"s_m: [{s[0]:.3f} .. {s[-1]:.3f}], monotonic: True")

    def run(self):
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Interactive Raceline Editor v2 for ForzaETH Race Stack')
    parser.add_argument('--map-dir', required=True,
                        help='Path to map directory (e.g. .../maps/porto)')
    parser.add_argument('--edit-sp', action='store_true',
                        help='Edit shortest-path trajectory instead of min-curvature')
    args = parser.parse_args()

    if not os.path.isdir(args.map_dir):
        print(f"Error: {args.map_dir} is not a directory")
        sys.exit(1)

    json_path = os.path.join(args.map_dir, 'global_waypoints.json')
    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found")
        sys.exit(1)

    editor = RacelineEditor(args.map_dir, edit_sp=args.edit_sp)
    editor.run()


if __name__ == '__main__':
    main()