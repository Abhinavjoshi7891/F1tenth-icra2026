#!/usr/bin/env python3
"""
ROS2 port of controller/map/src/plotter.py (ROS1 ForzaETH).

What it does:
    Writes one CSV row per control cycle to ~/plot_datas/plot_data_{timestamp}.csv.
    Zero rendering overhead — this runs in the control loop on every cycle.
    Load the CSV offline in Python/PlotJuggler/Excel for analysis.

Columns (semicolon-delimited, identical to ROS1):
    goal_speed        : raw raceline speed at lookahead position [m/s]
                        — before lateral error / trailing adjustments
    car_speed         : measured speed from /car_state/odom [m/s]
    calculated_speed  : commanded speed after all adjustments [m/s]
    curvature         : mean abs curvature of waypoints ahead [rad/m]
    trackbounds_avg   : normalised trackbound = min(d_left,d_right)/(d_left+d_right) [-]
                        0.5 = on raceline, <0.5 = closer to one wall
    lateral_error     : |d| Frenet lateral deviation from raceline [m]
    position_on_path  : index of nearest waypoint on the raceline [-]
    lookahead_distance: L1 distance used for steering computation [m]
    steering_angle    : final commanded steering angle [rad]

ROS1 differences fixed:
    - bare except replaced with explicit OSError + get_logger warning
    - os.makedirs() creates ~/plot_datas/ if missing instead of silently failing
    - flush() called every cycle (unchanged from ROS1)
    - no rospy dependency
"""

import csv
import os
import time


class Plotter:
    """
    CSV data logger for controller diagnostics.

    Invariant: self._ok is True only when the file and writer are valid.
    All public methods are no-ops when self._ok is False — the control
    loop must never crash because of a logging failure.
    """

    # Flush every N rows to amortise syscall cost without risking
    # data loss on a crash. ROS1 flushed every cycle; we keep that
    # behaviour (N=1) to match exactly. Change to e.g. 10 for hardware
    # if you measure the flush as a control-loop bottleneck.
    FLUSH_EVERY = 1

    HEADERS = [
        'goal_speed',
        'car_speed',
        'calculated_speed',
        'curvature',
        'trackbounds_avg',
        'lateral_error',
        'position_on_path',
        'lookahead_distance',
        'steering_angle',
    ]

    def __init__(self, logger_warn=print):
        """
        Args:
            logger_warn: callable(str) — injected from controller_manager
                         so we can call self.get_logger().warn() without
                         making this class a ROS node.
        """
        self._ok = False
        self._row_count = 0
        self._logger_warn = logger_warn

        plot_dir = os.path.join(os.environ["HOME"], 'plot_datas')
        filepath = os.path.join(plot_dir, f'plot_data_{int(time.time())}.csv')

        try:
            os.makedirs(plot_dir, exist_ok=True)   # ROS1 required manual mkdir — fixed
            self._file = open(filepath, 'w', newline='')
            self._writer = csv.writer(self._file, delimiter=';')
            self._writer.writerow(self.HEADERS)
            self._ok = True
        except OSError as e:
            self._logger_warn(
                f'[Plotter] Failed to open {filepath}: {e}. '
                f'Data logging disabled.'
            )

    def __del__(self):
        self.close()

    def close(self):
        """Flush and close the file. Safe to call multiple times."""
        if self._ok:
            try:
                self._file.flush()
                self._file.close()
            except OSError:
                pass
            self._ok = False

    def add_values(
        self,
        goal_speed: float,
        car_speed: float,
        calculated_speed: float,
        curvature: float,
        trackbounds_avg: float,
        lateral_error: float,
        position_on_path: float,
        lookahead_distance: float,
        steering_angle: float,
    ) -> None:
        """
        Write one row. Signature identical to ROS1 add_values().
        No-op if file failed to open.
        """
        if not self._ok:
            return
        self._writer.writerow([
            goal_speed,
            car_speed,
            calculated_speed,
            curvature,
            trackbounds_avg,
            lateral_error,
            position_on_path,
            lookahead_distance,
            steering_angle,
        ])
        self._row_count += 1
        if self._row_count % self.FLUSH_EVERY == 0:
            self._file.flush()

    def flush(self):
        """Explicit flush. Call at end of session or from signal handler."""
        if self._ok:
            self._file.flush()
