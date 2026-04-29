#!/usr/bin/env python3
"""
Experiment 5 analysis: fit servo-position → steering-angle mapping.
Edit model_name_ and bag_dir_ below, then run:
    python3 analyse_steering.py
Saves updated C_d and C_0d into the model .txt file.
"""
import os
import fire
from scipy.ndimage import median_filter
import numpy as np
from numpy.linalg import lstsq
import matplotlib.pyplot as plt

from helpers.bagloader import load_bags
from helpers.load_model import get_dotdict
from helpers.save_model import save

g_ = 9.81

model_name_ = 'MYCAR_pacejka'  # edit: {model_name}_{tire_model}
model, tires = model_name_.split('_')
local_bag_dir = './data/' + model + '/steering_angle'

script_dir = os.path.dirname(os.path.realpath(__file__))
bag_dir_ = os.path.join(script_dir, local_bag_dir)


class SteeringAnalyser:
    def __init__(self, model_name=model_name_, bag_dir=bag_dir_):
        print(f'Loading model {model_name}...')
        self.model = get_dotdict(model_name)

        self.cmd_topic = '/vesc/commands/servo/position'
        field_dict = {
            '/car_state/odom': [
                'twist.twist.linear.x',
                'twist.twist.linear.y',
                'twist.twist.angular.z',
            ],
            self.cmd_topic: ['data'],
        }

        self.data = load_bags(bag_dir, field_dict)
        self.transform_data()

        for i in range(9):
            self.solve_steering()
            self.discard_outliers(i)

        save(self.model)
        print(f'Final C_d:  {self.model.C_d}')
        print(f'Final C_0d: {self.model.C_0d}')
        self.transform_data()
        self.plot_solution()

    def transform_data(self):
        self.omega = median_filter(self.data['twist.twist.angular.z'].to_numpy(), size=5)
        self.v_x = median_filter(self.data['twist.twist.linear.x'].to_numpy(), size=5)
        self.servo_position = self.data[self.cmd_topic + '.data'].to_numpy()

    def solve_steering(self):
        self.atan = np.arctan2((self.model.l_r + self.model.l_f) * self.omega, self.v_x)
        atan_vector = np.reshape(self.atan, (-1, 1))
        A_i = np.concatenate((atan_vector, np.ones_like(atan_vector)), axis=1)
        y_i = np.reshape(self.servo_position, (-1, 1))
        sol = lstsq(A_i, y_i, rcond=None)
        print('Least-squares solution:')
        print(sol)
        self.model.C_d = float(sol[0][0])
        self.model.C_0d = float(sol[0][1])
        self.fit = (self.servo_position - self.model.C_0d) / self.model.C_d

    def discard_outliers(self, i):
        threshold = 4 * 2 ** (-i)
        print(f'Outlier threshold: {threshold}')
        error = abs(self.fit - self.atan)
        keep = np.where(error <= threshold)
        self.v_x = self.v_x[keep]
        self.omega = self.omega[keep]
        self.servo_position = self.servo_position[keep]

    def plot_solution(self):
        self.atan = np.arctan2((self.model.l_r + self.model.l_f) * self.omega, self.v_x)
        self.fit = (self.servo_position - self.model.C_0d) / self.model.C_d

        fig, axs = plt.subplots(1, figsize=(8, 5))
        plt.xlabel('Normalized Time')

        cmd_axs = axs.twinx()
        p3, = cmd_axs.plot(
            np.linspace(0, 1, len(self.servo_position)),
            self.servo_position, 'r', label='cmd servo position',
        )
        p1, = axs.plot(
            np.linspace(0, 1, len(self.atan)),
            self.atan, 'g', alpha=0.7, label='est. measured steering angle',
        )
        p2, = axs.plot(
            np.linspace(0, 1, len(self.fit)),
            self.fit, 'b', label='predicted steering angle',
        )
        axs.set_ylabel('Steering Angle [rad]')
        axs.set_ylim(-1.0, 1.0)
        axs.grid()
        cmd_axs.set_ylabel('Servo Position [0-1]')
        cmd_axs.yaxis.label.set_color('red')
        cmd_axs.spines['right'].set_edgecolor('red')
        cmd_axs.tick_params(axis='y', colors='red')
        cmd_axs.set_ylim(0.0, 1.0)
        axs.legend([p1, p2, p3], [l.get_label() for l in [p1, p2, p3]])
        plt.xlim(0.0, 1.0)
        plt.tight_layout()
        plt.show()


def main(model_name: str = model_name_, bag_dir: str = bag_dir_):
    SteeringAnalyser(model_name, bag_dir)


if __name__ == '__main__':
    fire.Fire(main)
