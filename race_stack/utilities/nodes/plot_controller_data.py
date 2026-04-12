#!/usr/bin/env python3
"""
controller_plotter.py — offline diagnostic plotter for controller performance.

Usage:
    python3 plot_controller_data.py ~/plot_datas/plot_data_XXXXX.csv
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import os

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_controller_data.py <path_to_csv>")
        return

    filepath = sys.argv[1]
    if not os.path.exists(filepath):
        # try prepending home if it's just the filename
        filepath = os.path.expanduser(filepath)
    
    if not os.path.exists(filepath):
        print(f"Error: File {filepath} not found.")
        return

    df = pd.read_csv(filepath, sep=';')

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    # Speed: goal vs calculated vs actual
    axes[0].plot(df['goal_speed'],       label='goal (raceline)',  lw=1)
    axes[0].plot(df['calculated_speed'], label='calculated',       lw=1)
    axes[0].plot(df['car_speed'],        label='actual',           lw=1, ls='--')
    axes[0].set_ylabel('Speed [m/s]')
    axes[0].legend(fontsize=8)
    axes[0].grid(True)

    # Lateral error
    axes[1].plot(df['lateral_error'], color='red', lw=1)
    axes[1].axhline(0.05, color='orange', ls='--', lw=0.8, label='5cm threshold')
    axes[1].set_ylabel('Lateral Error [m]')
    axes[1].legend(fontsize=8)
    axes[1].grid(True)

    # L1 distance vs curvature
    axes[2].plot(df['lookahead_distance'], label='L1 distance', lw=1)
    axes[2].plot(df['curvature'],          label='curvature',   lw=1, alpha=0.7)
    axes[2].set_ylabel('L1 / Curvature')
    axes[2].legend(fontsize=8)
    axes[2].grid(True)

    # Steering angle
    axes[3].plot(df['steering_angle'], color='purple', lw=1)
    axes[3].axhline( 0.4189, color='r', ls='--', lw=0.8, label='±24° limit')
    axes[3].axhline(-0.4189, color='r', ls='--', lw=0.8)
    axes[3].set_ylabel('Steering [rad]')
    axes[3].set_xlabel('Control cycle')
    axes[3].legend(fontsize=8)
    axes[3].grid(True)

    plt.suptitle(f'Controller Diagnostic — {df.shape[0]} cycles\nFile: {os.path.basename(filepath)}', fontsize=13)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
