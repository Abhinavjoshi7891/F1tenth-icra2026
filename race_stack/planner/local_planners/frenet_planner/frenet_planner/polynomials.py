"""
Quartic and Quintic polynomial trajectory primitives.
Direct port of ROS1 QuarticPolynomial.cpp and QuinticPolynomial.cpp.
No ROS dependencies. Uses numpy for matrix solve instead of Eigen.
"""
import numpy as np


class QuarticPolynomial:
    """
    4th-order polynomial for longitudinal (velocity-keeping) trajectories.
    Boundary conditions: position, velocity, acceleration at start;
                         velocity, acceleration at end.
    """

    def __init__(self, xs: float, vxs: float, axs: float,
                 vxe: float, axe: float, t: float):
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([
            [3.0 * t**2, 4.0 * t**3],
            [6.0 * t,    12.0 * t**2]
        ])
        B = np.array([
            vxe - self.a1 - 2.0 * self.a2 * t,
            axe - 2.0 * self.a2
        ])
        x = np.linalg.solve(A, B)
        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t: float) -> float:
        return (self.a0 + self.a1 * t + self.a2 * t**2
                + self.a3 * t**3 + self.a4 * t**4)

    def calc_first_derivative(self, t: float) -> float:
        return (self.a1 + 2.0 * self.a2 * t + 3.0 * self.a3 * t**2
                + 4.0 * self.a4 * t**3)

    def calc_second_derivative(self, t: float) -> float:
        return 2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t**2

    def calc_third_derivative(self, t: float) -> float:
        return 6.0 * self.a3 + 24.0 * self.a4 * t


class QuinticPolynomial:
    """
    5th-order polynomial for lateral trajectories.
    Boundary conditions: position, velocity, acceleration at both start and end.
    """

    def __init__(self, xs: float, vxs: float, axs: float,
                 xe: float, vxe: float, axe: float, t: float):
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([
            [t**3,       t**4,        t**5],
            [3.0 * t**2, 4.0 * t**3,  5.0 * t**4],
            [6.0 * t,    12.0 * t**2, 20.0 * t**3]
        ])
        B = np.array([
            xe - self.a0 - self.a1 * t - self.a2 * t**2,
            vxe - self.a1 - 2.0 * self.a2 * t,
            axe - 2.0 * self.a2
        ])
        x = np.linalg.solve(A, B)
        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t: float) -> float:
        return (self.a0 + self.a1 * t + self.a2 * t**2
                + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5)

    def calc_first_derivative(self, t: float) -> float:
        return (self.a1 + 2.0 * self.a2 * t + 3.0 * self.a3 * t**2
                + 4.0 * self.a4 * t**3 + 5.0 * self.a5 * t**4)

    def calc_second_derivative(self, t: float) -> float:
        return (2.0 * self.a2 + 6.0 * self.a3 * t
                + 12.0 * self.a4 * t**2 + 20.0 * self.a5 * t**3)

    def calc_third_derivative(self, t: float) -> float:
        return 6.0 * self.a3 + 24.0 * self.a4 * t + 60.0 * self.a5 * t**2
