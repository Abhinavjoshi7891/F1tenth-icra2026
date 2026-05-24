#!/usr/bin/env python3
"""
Deceleration model identification (identifies C_dec).
Requires C_R already identified (run analyse_accel.py first).
Edit model_name_ and bag_dir_ below, then run:
    python3 analyse_decel.py
"""
import os
import fire
import numpy as np
import pandas as pd
from numpy.linalg import lstsq
import matplotlib.pyplot as plt

from helpers.bagloader import load_bags
from helpers.load_model import get_dotdict
from helpers.save_model import save

model_name_ = 'MYCAR_linear'
model, tires = model_name_.split('_')
local_bag_dir = './data/' + model + '/decel'

script_dir = os.path.dirname(os.path.realpath(__file__))
bag_dir_ = os.path.join(script_dir, local_bag_dir)


class DecelerationAnalyser:
    def __init__(self, model_name=model_name_, bag_dir=bag_dir_):
        print(f'Loading model {model_name}...')
        self.model = get_dotdict(model_name)

        self.cmd_topic = '/vesc/commands/motor/brake'
        field_dict = {
            '/vesc/sensors/imu/raw': ['linear_acceleration.y'],
            '/vesc/odom': ['twist.twist.linear.x'],
            self.cmd_topic: ['data'],
        }

        self.data = load_bags(bag_dir, field_dict)
        self.analyse_decel()
        self.refine_solution()
        self.plot_solution()
        save(self.model)

    def _solve_ls(self):
        self.current = -self.data[self.cmd_topic + '.data'].to_numpy()
        self.velocity = self.data['twist.twist.linear.x'].to_numpy()
        self.accel = -self.data['linear_acceleration.y'].to_numpy()

        v = np.reshape(self.velocity, (-1, 1))
        a = np.reshape(self.accel, (-1, 1))
        y = np.reshape(self.current, (-1, 1)) - self.model.C_R * v
        sol = lstsq(a, y, rcond=None)
        print('Least-squares solution:')
        print(sol)
        return sol

    def analyse_decel(self):
        sol = self._solve_ls()
        self.model.C_dec = float(sol[0][0])

    def refine_solution(self):
        error = (self.model.C_dec * self.accel + self.model.C_R * self.velocity) - self.current
        self.data = self.data[error <= np.std(error)]
        sol = self._solve_ls()
        self.model.C_dec = float(sol[0][0])

    def plot_solution(self):
        fit = (self.current - self.model.C_R * self.velocity) / self.model.C_dec
        n = len(self.accel)
        t = np.linspace(0, 1, n)

        fig, ax = plt.subplots(1, figsize=(7, 5))
        ax.plot(t, self.accel, 'b', label='est. x accel')
        ax.plot(t, self.current * 0.05, 'r-', alpha=0.5, label='cmd brake / 100')
        ax.plot(t, self.velocity, 'r:', alpha=0.5, label='est. velocity')
        ax.plot(t, fit, 'g', label='predicted accel')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(r'Acceleration [m/s²]')
        ax.grid()
        ax.legend(loc='best')
        plt.tight_layout()
        plt.show()


def main(model_name: str = model_name_, bag_dir: str = bag_dir_):
    DecelerationAnalyser(model_name, bag_dir)


if __name__ == '__main__':
    fire.Fire(main)
