#!/usr/bin/env python3
"""
Motor RPM model identification (identifies C_v, C_0v).
Model: erpm = C_v * v_x + C_0v
Edit model_name_ and bag_dir_ below, then run:
    python3 analyse_motor_rpm.py
"""
import os
import fire
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import lstsq
from scipy.ndimage import median_filter
from scipy.signal import sosfiltfilt, butter

from helpers.bagloader import load_bags
from helpers.load_model import get_dotdict
from helpers.save_model import save

model_name_ = 'DEXTER_linear'
model, tires = model_name_.split('_')
local_bag_dir = './data/' + model + '/motor'

script_dir = os.path.dirname(os.path.realpath(__file__))
bag_dir_ = os.path.join(script_dir, local_bag_dir)

_C_0V_FIXED = 220  # fixed eRPM offset (deadband)


class RPMAnalyser:
    def __init__(self, model_name=model_name_, bag_dir=bag_dir_):
        print(f'Loading model {model_name}...')
        self.model = get_dotdict(model_name)
        self.model.C_0v = _C_0V_FIXED

        self.cmd_topic = '/commands/motor/speed'
        field_dict = {
            '/car_state/odom': ['twist.twist.linear.x'],
            self.cmd_topic: ['data'],
            '/sensors/imu/raw': ['linear_acceleration.y'],
        }
        self.raw_data = load_bags(bag_dir, field_dict)

        self._transform_data()
        self._solve_fixed_offset()
        for i in range(1, 4):
            self._discard_outliers(i)
            self._solve_fixed_offset()

        save(self.model)
        print(f'Final C_v:  {self.model.C_v}')
        print(f'Final C_0v: {self.model.C_0v}')
        self._transform_data()
        self._plot_solution()

    def _transform_data(self):
        sos = butter(2, 0.05, output='sos')
        a_long_raw = -self.raw_data['linear_acceleration.y'].to_numpy()
        a_long = sosfiltfilt(sos, a_long_raw)
        filtered = self.raw_data[abs(a_long) < 2.0]
        if len(filtered) == 0:
            print('  [WARNING] Acceleration filter dropped all rows. Bypassing filter.')
            filtered = self.raw_data

        self.v_x = median_filter(filtered['twist.twist.linear.x'].to_numpy(), size=5)
        self.erpm = filtered[self.cmd_topic + '.data'].to_numpy()

    def _discard_outliers(self, iteration):
        threshold = 4000 * np.exp(-iteration)
        print(f'Outlier threshold: {threshold:.1f}')
        error = abs(self.model.C_v * self.v_x + self.model.C_0v - self.erpm)
        keep = error <= threshold
        self.v_x = self.v_x[keep]
        self.erpm = self.erpm[keep]

    def _solve_fixed_offset(self):
        A = np.reshape(self.v_x, (-1, 1))
        y = np.reshape(self.erpm, (-1, 1))
        sol = lstsq(A, y, rcond=None)
        print('Least-squares solution:')
        print(sol)
        self.model.C_v = float(sol[0][0])

        self._space = np.linspace(self.v_x.min(), self.v_x.max(), 50)
        self._fit = self.model.C_v * self._space + self.model.C_0v

    def _plot_solution(self):
        plt.figure(figsize=(8, 6))
        plt.scatter(self.v_x, self.erpm, color='green', alpha=0.15, label='Measurements')
        plt.plot(self._space, self._fit, 'b', label=f'Model: C_v={self.model.C_v:.1f}, C_0v={self.model.C_0v}')
        plt.xlabel(r'$v_x$ [m/s]')
        plt.ylabel('ERPM')
        plt.grid()
        plt.legend(loc='best')
        plt.tight_layout()
        plt.show()


def main(model_name: str = model_name_, bag_dir: str = bag_dir_):
    RPMAnalyser(model_name, bag_dir)


if __name__ == '__main__':
    fire.Fire(main)
