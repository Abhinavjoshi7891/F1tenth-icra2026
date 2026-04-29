#!/usr/bin/env python3
"""
Generate steering lookup table CSV from a fitted model .txt file.
Usage:
    python3 generate_lookup_table.py --model_name MYCAR_pacejka
The output CSV is saved to steering_lookup/cfg/{model_name}_lookup_table.csv
and is immediately usable by the MAP controller via the steering_lookup package.
"""
import os
from datetime import datetime
from scipy.integrate import odeint
import fire
import numpy as np
import matplotlib.pyplot as plt

from helpers.load_model import get_dotdict
from helpers.vehicle_dynamics import vehicle_dynamics

SIMULATION_DURATION = 2.0
SIMULATION_DT = 0.01
PLOT_LOOKUP = True

START_STEER = 0.0
STEER_FINE_END = 0.1
FINE_STEP_SIZE = 0.0033
END_STEER = 0.4
COARSE_STEP_SIZE = 0.01
START_VEL = 0.5
END_VEL = 7.0
VEL_STEP_SIZE = 0.1

_script_dir = os.path.dirname(os.path.realpath(__file__))
# steering_lookup/cfg lives two levels up from id_analyser/
_lu_cfg_dir = os.path.join(_script_dir, '..', 'steering_lookup', 'cfg')


class Simulator:
    def __init__(self, model):
        self.model = model

    def _func_ST(self, x, t, u):
        return vehicle_dynamics(x, u, self.model)

    def run_simulation(self, initial_state, u, duration=SIMULATION_DURATION, dt=SIMULATION_DT):
        t = np.arange(0, duration, dt)
        return odeint(self._func_ST, initial_state, t, args=(u,))


class LookupGenerator:
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.model = get_dotdict(model_name)
        if self.model.tire_model != 'pacejka':
            raise ValueError(f'generate_lookup_table requires a pacejka model, got: {self.model.tire_model}')
        self.sim = Simulator(self.model)
        self.lookup_table = None

    def run_generator(self):
        self.generate_lookup()
        self.find_upper_limits()
        if PLOT_LOOKUP:
            self.plot_lookup()
        self.save_lookup()

    def generate_lookup(self):
        fine_steers = np.linspace(
            START_STEER, STEER_FINE_END,
            int((STEER_FINE_END - START_STEER) / FINE_STEP_SIZE),
            endpoint=False,
        )
        coarse_steers = np.linspace(
            STEER_FINE_END, END_STEER,
            int((END_STEER - STEER_FINE_END) / COARSE_STEP_SIZE),
        )
        steers = np.concatenate((fine_steers, coarse_steers))
        vels = np.linspace(START_VEL, END_VEL, int((END_VEL - START_VEL) / VEL_STEP_SIZE))

        n_steer = len(steers)
        n_vel = len(vels)
        self.lookup_table = np.empty([n_steer + 1, n_vel + 1])
        self.lookup_table[0, 1:] = vels
        self.lookup_table[1:, 0] = steers

        for si, steer in enumerate(steers):
            for vi, vel in enumerate(vels):
                sol = self.sim.run_simulation([0, 0, 0, vel, 0, 0], [steer, 0])
                if np.allclose(sol[-11:-1, 5], sol[-15:-5, 5], rtol=1e-3):
                    self.lookup_table[si + 1, vi + 1] = sol[-1, 5] * vel
                else:
                    self.lookup_table[si + 1, vi + 1:] = None
                    break

    def find_upper_limits(self):
        for vi in range(1, self.lookup_table.shape[1]):
            a_lats = self.lookup_table[1:, vi]
            a_lats = a_lats[~np.isnan(a_lats)]
            max_idx = int(np.argmax(a_lats))
            if max_idx != 0:
                d = np.diff(a_lats[0:max_idx])
                local_max = np.argwhere(d < 0)
                if local_max.size:
                    max_idx = int(local_max[0])
            self.lookup_table[max_idx + 1:, vi] = None

    def plot_lookup(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel(r'$v_x$ [m/s]')
        ax.set_ylabel(r'$\delta$ [rad]')
        ax.yaxis.labelpad = 10
        ax.set_zlabel(r'$a_c$ [m/s$^2$]')
        ax.set_xlim(START_VEL, END_VEL)
        ax.set_ylim(START_STEER, END_STEER)
        ax.set_zlim(0, 10)

        X, Y = np.meshgrid(self.lookup_table[0, 1:], self.lookup_table[1:, 0])
        Z = self.lookup_table[1:, 1:]
        ax.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha=0.6, zorder=0)
        ax.view_init(15, -160)
        plt.tight_layout()
        plt.show()

    def save_lookup(self):
        os.makedirs(_lu_cfg_dir, exist_ok=True)
        filename = f'{self.model_name}_lookup_table.csv'
        file_path = os.path.join(_lu_cfg_dir, filename)
        np.savetxt(file_path, self.lookup_table, delimiter=',')
        print(f'Saved lookup table: {file_path}')
        print('Rebuild the steering_lookup package so MAP can find the new CSV:')
        print('  colcon build --packages-select steering_lookup')


def main(model_name: str):
    LookupGenerator(model_name).run_generator()


if __name__ == '__main__':
    fire.Fire(main)
