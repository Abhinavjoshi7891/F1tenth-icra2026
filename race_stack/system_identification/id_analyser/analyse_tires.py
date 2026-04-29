#!/usr/bin/env python3
"""
Tire dynamics identification (Pacejka or linear model).
Edit model_name_ and bag_path_ below, then run:
    python3 analyse_tires.py
"""
import os
import fire
import numpy as np
import pandas as pd
from numpy.linalg import lstsq
from scipy.optimize import least_squares
from scipy.ndimage import median_filter
from scipy.signal import sosfiltfilt, butter
import matplotlib.pyplot as plt

from helpers.bagloader import load_bags
from helpers.load_model import get_dotdict
from helpers.save_model import save

g_ = 9.81

model_name_ = 'MYCAR_pacejka'
model, tires = model_name_.split('_')
bag_path_ = './data/' + model + '/tire_dynamics'


class TireAnalyser:
    def __init__(self, model_name=model_name_, bag_path=bag_path_):
        print(f'Loading model {model_name}...')
        self.model = get_dotdict(model_name)

        self.cmd_topic = '/vesc/commands/servo/position'
        field_dict = {
            '/ekf/odometry/filtered': ['twist.twist.angular.z'],
            '/car_state/odom': ['twist.twist.linear.x', 'twist.twist.linear.y'],
            self.cmd_topic: ['data'],
            '/vesc/sensors/imu/raw': ['linear_acceleration.x', 'linear_acceleration.y'],
        }

        self.issim = (model == 'SIM')
        if self.issim:
            field_dict = {
                '/car_state/odom': [
                    'twist.twist.linear.x',
                    'twist.twist.linear.y',
                    'twist.twist.angular.z',
                ],
                '/nav': ['drive.speed', 'drive.acceleration', 'drive.steering_angle'],
                '/imu': ['linear_acceleration.x', 'linear_acceleration.y'],
            }

        self.data = load_bags(bag_path, field_dict)
        self.data = self.data[self.data['twist.twist.linear.x'] > 1.0]

        self.analyse_tires()
        save(self.model)

    def analyse_tires(self):
        self.transform_data()
        self.discard_alpha_outliers()

        for i in range(4):
            if i > 0:
                self.discard_outliers(i)
            if self.model.tire_model == 'linear':
                self.solve_linear_problem()
            elif self.model.tire_model == 'pacejka':
                self.solve_pacejka_problem()

        self.transform_data()
        self.plot_solution()

    def transform_data(self):
        if self.issim:
            self.a_long = self.data['linear_acceleration.x'].to_numpy()
            self.a_lat = self.data['linear_acceleration.y'].to_numpy()
            self.omega = self.data['twist.twist.angular.z'].to_numpy()
        else:
            sos = butter(2, 0.05, output='sos')
            self.a_long = sosfiltfilt(sos, -self.data['linear_acceleration.y'].to_numpy())
            self.a_lat = sosfiltfilt(sos, self.data['linear_acceleration.x'].to_numpy())
            self.omega = self.data['twist.twist.angular.z'].to_numpy()

        self.v_x = median_filter(self.data['twist.twist.linear.x'].to_numpy(), size=5)
        self.v_y = median_filter(self.data['twist.twist.linear.y'].to_numpy(), size=5)
        self.delta = (self.data[self.cmd_topic + '.data'].to_numpy() - self.model.C_0d) / self.model.C_d

        self.F_zf = self.model.m * (g_ * self.model.l_r - self.a_long * self.model.h_cg) / self.model.l_wb
        self.F_zr = self.model.m * (g_ * self.model.l_f + self.a_long * self.model.h_cg) / self.model.l_wb
        self.meanFz_f = np.mean(self.F_zf)
        self.meanFz_r = np.mean(self.F_zr)

        self.alpha_f = -np.arctan((self.v_y + self.omega * self.model.l_f) / self.v_x) + self.delta
        self.alpha_r = -np.arctan((self.v_y - self.omega * self.model.l_r) / self.v_x)

        self.F_yf = self.model.m * self.model.l_r * self.a_lat / ((self.model.l_r + self.model.l_f) * np.cos(self.delta))
        self.F_yr = self.model.m * self.model.l_f * self.a_lat / (self.model.l_r + self.model.l_f)

    def discard_alpha_outliers(self):
        keep = np.where((abs(self.alpha_f) <= 2) & (abs(self.alpha_r) <= 2))
        for attr in ['v_x', 'v_y', 'omega', 'delta', 'F_zf', 'F_zr', 'alpha_f', 'alpha_r', 'F_yf', 'F_yr']:
            setattr(self, attr, getattr(self, attr)[keep])

    def discard_outliers(self, i):
        threshold = 20 * 2 ** (-i)
        print(f'Outlier threshold: {threshold}')
        keep = np.where((self.error_f <= threshold) & (self.error_r <= threshold))
        for attr in ['v_x', 'v_y', 'omega', 'delta', 'F_zf', 'F_zr', 'alpha_f', 'alpha_r', 'F_yf', 'F_yr']:
            setattr(self, attr, getattr(self, attr)[keep])

    def solve_linear_problem(self):
        A_f = np.reshape(self.model.mu * self.F_zf * self.alpha_f, (-1, 1))
        A_r = np.reshape(self.model.mu * self.F_zr * self.alpha_r, (-1, 1))
        sol_f = lstsq(A_f, np.reshape(self.F_yf, (-1, 1)), rcond=None)
        sol_r = lstsq(A_r, np.reshape(self.F_yr, (-1, 1)), rcond=None)
        self.model.C_Sf = float(sol_f[0][0])
        self.model.C_Sr = float(sol_r[0][0])

        self.space_f = np.linspace(-0.65, 0.65, 50)
        self.fit_f = self.model.mu * self.meanFz_f * self.model.C_Sf * self.space_f
        self.space_r = np.linspace(-0.65, 0.65, 50)
        self.fit_r = self.model.mu * self.meanFz_r * self.model.C_Sr * self.space_r

        self.error_f = abs(self.model.mu * self.F_zf * self.model.C_Sf * self.alpha_f - self.F_yf)
        self.error_r = abs(self.model.mu * self.F_zr * self.model.C_Sr * self.alpha_r - self.F_yr)

    def _pacejka(self, params, alpha, F_z):
        B, C, D, E = params
        return self.model.mu * F_z * D * np.sin(C * np.arctan(B * alpha - E * (B * alpha - np.arctan(B * alpha))))

    def _pacejka_error(self, params, alpha, F_z, F_y):
        return (self._pacejka(params, alpha, F_z) - F_y) ** 2

    def solve_pacejka_problem(self):
        p0 = [1, 1, 1, 1]
        sol_f = least_squares(self._pacejka_error, p0,
                              args=(self.alpha_f, self.F_zf, self.F_yf),
                              bounds=([0]*4, [25, 4.0, 2.0, 1.1]))
        sol_r = least_squares(self._pacejka_error, p0,
                              args=(self.alpha_r, self.F_zr, self.F_yr),
                              bounds=([0]*4, [20, 1.5, 2.0, 1.1]))
        self.model.C_Pf = sol_f.x.tolist()
        self.model.C_Pr = sol_r.x.tolist()

        self.space_f = np.linspace(-0.65, 0.65, 50)
        self.fit_f = self._pacejka(self.model.C_Pf, self.space_f, self.meanFz_f)
        self.space_r = np.linspace(-0.65, 0.65, 50)
        self.fit_r = self._pacejka(self.model.C_Pr, self.space_r, self.meanFz_r)

        self.error_f = abs(self._pacejka(self.model.C_Pf, self.alpha_f, self.F_zf) - self.F_yf)
        self.error_r = abs(self._pacejka(self.model.C_Pr, self.alpha_r, self.F_zr) - self.F_yr)

    def plot_solution(self):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
        ax1.scatter(self.alpha_f, self.F_yf, color='green', alpha=0.15, label='Est. Lateral Force')
        ax2.scatter(self.alpha_r, self.F_yr, color='green', alpha=0.15, label='Est. Lateral Force')
        ax1.plot(self.space_f, self.fit_f, 'b', label=f'Model Fz={self.meanFz_f:.2f}')
        ax2.plot(self.space_r, self.fit_r, 'b', label=f'Model Fz={self.meanFz_r:.2f}')
        ax1.set_title('Front tires')
        ax2.set_title('Rear tires')
        for ax in [ax1, ax2]:
            ax.set_xlabel(r'$\alpha$ [rad]')
            ax.set_ylabel(r'$F_y$ [N]')
            ax.grid()
            ax.legend(loc='best')
            ax.set_xlim([-0.65, 0.65])
            ax.set_ylim([-20, 20])
        plt.tight_layout()
        plt.show()


def main(model_name: str = model_name_, bag_path: str = bag_path_):
    TireAnalyser(model_name, bag_path)


if __name__ == '__main__':
    fire.Fire(main)
