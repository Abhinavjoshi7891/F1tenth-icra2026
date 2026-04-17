"""
Cubic spline classes for trajectory representation.
Direct port of ROS1 CubicSpline1D.cpp and CubicSpline2D.cpp.
No ROS dependencies.
"""
import math
import numpy as np
from bisect import bisect_right


class CubicSpline1D:
    """
    1D cubic spline with optional closed-contour wrapping.
    Port of ROS1 CubicSpline1D — uses same natural spline construction.
    """

    def __init__(self, v1=None, v2=None, closed_contour=False):
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        self.x = []
        self.y = []
        self.nx = 0
        self.is_closed_contour = closed_contour

        if v1 is not None and v2 is not None:
            self._build(list(v1), list(v2))

    def _build(self, v1, v2):
        self.nx = len(v1)
        self.x = list(v1)
        self.y = list(v2)
        self.a = list(v2)

        # element-wise differences
        deltas = [self.x[i + 1] - self.x[i] for i in range(self.nx - 1)]

        # build matrix A and vector B
        A = np.zeros((self.nx, self.nx))
        B_vec = np.zeros(self.nx)

        # matrix_a
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != self.nx - 2:
                A[i + 1, i + 1] = 2.0 * (deltas[i] + deltas[i + 1])
            A[i + 1, i] = deltas[i]
            A[i, i + 1] = deltas[i]
        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0

        # vector_b
        for i in range(self.nx - 2):
            B_vec[i + 1] = (3.0 * (self.a[i + 2] - self.a[i + 1]) / deltas[i + 1]
                            - 3.0 * (self.a[i + 1] - self.a[i]) / deltas[i])

        # solve
        c_vec = np.linalg.solve(A, B_vec)
        self.c = list(c_vec)

        # compute b, d
        self.b = []
        self.d = []
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * deltas[i]))
            self.b.append((self.a[i + 1] - self.a[i]) / deltas[i]
                          - deltas[i] * (self.c[i + 1] + 2.0 * self.c[i]) / 3.0)

    def _handle_wrapping(self, t):
        return t - self.x[-1]

    def _search_index(self, t):
        return bisect_right(self.x, t)

    def calc_der0(self, t):
        """Evaluate spline at t (0th derivative)."""
        if not self.x:
            return float('nan')
        if t < self.x[0]:
            return float('nan')
        if t >= self.x[-1]:
            if not self.is_closed_contour:
                return float('nan')
            else:
                t = self._handle_wrapping(t)

        i = self._search_index(t) - 1
        dx = t - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * dx**2 + self.d[i] * dx**3

    def calc_der1(self, t):
        """1st derivative."""
        if not self.x:
            return float('nan')
        if t < self.x[0]:
            return float('nan')
        if t >= self.x[-1]:
            if not self.is_closed_contour:
                return float('nan')
            else:
                t = self._handle_wrapping(t)

        i = self._search_index(t) - 1
        dx = t - self.x[i]
        return self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx**2

    def calc_der2(self, t):
        """2nd derivative."""
        if not self.x:
            return float('nan')
        if t < self.x[0]:
            return float('nan')
        if t >= self.x[-1]:
            if not self.is_closed_contour:
                return float('nan')
            else:
                t = self._handle_wrapping(t)

        i = self._search_index(t) - 1
        dx = t - self.x[i]
        return 2.0 * self.c[i] + 6.0 * self.d[i] * dx


def _norm(x, y):
    return math.sqrt(x * x + y * y)


def _are_collinear(x1, y1, x2, y2, x3, y3):
    area = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
    return abs(area) <= 1e-5


class CubicSpline2D:
    """
    2D parametric cubic spline over (x, y, d_right, d_left, v).
    Port of ROS1 CubicSpline2D with collinear-point removal.
    """

    def __init__(self, x=None, y=None, d_right=None, d_left=None,
                 v=None, is_closed_contour=False):
        self.s = []
        self.sx = CubicSpline1D()
        self.sy = CubicSpline1D()
        self.sd_right = CubicSpline1D()
        self.sd_left = CubicSpline1D()
        self.sv = CubicSpline1D()

        if x is not None:
            self._build(list(x), list(y), list(d_right), list(d_left),
                        list(v), is_closed_contour)

    def _build(self, x, y, d_right, d_left, v, is_closed_contour):
        filtered = self._remove_collinear_points(x, y, d_right, d_left, v)
        fx, fy, fr, fl, fv = filtered

        # calc s
        self.s = [0.0]
        for i in range(len(fx) - 1):
            self.s.append(self.s[-1] + _norm(fx[i + 1] - fx[i], fy[i + 1] - fy[i]))
        # remove duplicate s values
        seen = set()
        unique_indices = []
        for i, sv in enumerate(self.s):
            if sv not in seen:
                seen.add(sv)
                unique_indices.append(i)
        self.s = [self.s[i] for i in unique_indices]
        fx = [fx[i] for i in unique_indices]
        fy = [fy[i] for i in unique_indices]
        fr = [fr[i] for i in unique_indices]
        fl = [fl[i] for i in unique_indices]
        fv = [fv[i] for i in unique_indices]

        self.sx = CubicSpline1D(self.s, fx, is_closed_contour)
        self.sy = CubicSpline1D(self.s, fy, is_closed_contour)
        self.sd_right = CubicSpline1D(self.s, fr, is_closed_contour)
        self.sd_left = CubicSpline1D(self.s, fl, is_closed_contour)
        self.sv = CubicSpline1D(self.s, fv, is_closed_contour)

    def _remove_collinear_points(self, x, y, d_right, d_left, v):
        n = len(x)
        if n < 3:
            return x, y, d_right, d_left, v

        fx = [x[0], x[1]]
        fy = [y[0], y[1]]
        fr = [d_right[0], d_right[1]]
        fl = [d_left[0], d_left[1]]
        fv = [v[0], v[1]]

        for i in range(2, n - 1):
            if _are_collinear(x[i - 2], y[i - 2], x[i - 1], y[i - 1], x[i], y[i]):
                continue
            fx.append(x[i])
            fy.append(y[i])
            fr.append(d_right[i])
            fl.append(d_left[i])
            fv.append(v[i])

        # always add last point
        fx.append(x[-1])
        fy.append(y[-1])
        fr.append(d_right[-1])
        fl.append(d_left[-1])
        fv.append(v[-1])

        return fx, fy, fr, fl, fv

    def calc_x(self, t):
        return self.sx.calc_der0(t)

    def calc_y(self, t):
        return self.sy.calc_der0(t)

    def calc_d_right(self, t):
        return self.sd_right.calc_der0(t)

    def calc_d_left(self, t):
        return self.sd_left.calc_der0(t)

    def calc_v(self, t):
        return self.sv.calc_der0(t)

    def calc_curvature(self, t):
        dx = self.sx.calc_der1(t)
        ddx = self.sx.calc_der2(t)
        dy = self.sy.calc_der1(t)
        ddy = self.sy.calc_der2(t)
        denom = (dx**2 + dy**2)**1.5
        if abs(denom) < 1e-12:
            return 0.0
        return (ddy * dx - ddx * dy) / denom

    def calc_yaw(self, t):
        dx = self.sx.calc_der1(t)
        dy = self.sy.calc_der1(t)
        return math.atan2(dy, dx)

    def find_s(self, x, y, s0):
        """Find closest s to (x, y). Brute-force scan — same as ROS 1."""
        s_closest = s0
        closest = float('inf')
        si = self.s[0] if self.s else 0.0
        s_end = self.s[-1] if self.s else 0.0

        while si < s_end:
            px = self.calc_x(si)
            py = self.calc_y(si)
            dist = _norm(x - px, y - py)
            if dist < closest:
                closest = dist
                s_closest = si
            si += 0.1

        return s_closest
