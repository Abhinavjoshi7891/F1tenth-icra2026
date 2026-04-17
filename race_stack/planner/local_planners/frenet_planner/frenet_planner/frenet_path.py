"""
Single Frenet trajectory candidate — stores path data, validates constraints,
checks collisions, computes costs.
Direct port of ROS1 FrenetPath.cpp. No ROS dependencies.
"""
import math
import copy
from frenet_planner.cubic_spline import CubicSpline1D, CubicSpline2D

PI = math.pi


class FrenetPath:
    """One candidate trajectory in Frenet + Cartesian space."""

    def __init__(self, fot_hp=None):
        # Frenet attributes
        self.t = []           # time steps
        self.d = []           # lateral offset
        self.d_d = []         # lateral speed
        self.d_dd = []        # lateral acceleration
        self.d_ddd = []       # lateral jerk
        self.s = []           # s position along spline
        self.s_d = []         # s speed
        self.s_dd = []        # s acceleration
        self.s_ddd = []       # s jerk
        self.d_spline = None  # CubicSpline1D for d(s)
        self.d_left = []
        self.d_right = []

        # Euclidean attributes
        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []          # arc-length between consecutive points
        self.c = []           # curvature

        # Debug
        self.ix = []
        self.iy = []
        self.iyaw = []

        # Cost attributes — lateral
        self.c_lateral_deviation = 0.0
        self.c_lateral_velocity = 0.0
        self.c_lateral_acceleration = 0.0
        self.c_lateral_jerk = 0.0
        self.c_lateral = 0.0

        # Cost attributes — longitudinal
        self.c_longitudinal_acceleration = 0.0
        self.c_longitudinal_jerk = 0.0
        self.c_time_taken = 0.0
        self.c_end_speed_deviation = 0.0
        self.c_speed_deviation = 0.0
        self.c_longitudinal = 0.0

        # Obstacle / track costs
        self.c_inv_dist_to_obstacles = 0.0
        self.c_inv_dist_to_track_bound = 0.0

        # Temporal consistency
        self.c_consistency = 0.0

        # Final cost
        self.cf = 0.0

        # Hyperparameters reference
        self.fot_hp = fot_hp

    def set_hyperparameters(self, fot_hp):
        self.fot_hp = fot_hp

    def calc_d_spline(self):
        """Build a CubicSpline1D of d(s) for temporal consistency checks."""
        if len(self.s) >= 2:
            self.d_spline = CubicSpline1D(self.s, self.d, closed_contour=False)

    def calc_difference(self, other):
        """
        Compute temporal consistency cost vs previous best path.
        'other' must already have d_spline built.
        """
        diff = 0.0
        if other.d_spline is None or not other.s:
            return diff
        for i in range(len(self.s)):
            if self.s[i] > other.s[0] and self.s[i] < other.s[-1]:
                other_d = other.d_spline.calc_der0(self.s[i])
                if not math.isnan(other_d):
                    diff += (self.d[i] - other_d) ** 2
        return diff

    def to_global_path(self, csp: CubicSpline2D) -> bool:
        """Convert Frenet path to global (x, y) coordinates using the reference spline."""
        for i in range(len(self.s)):
            ix_ = csp.calc_x(self.s[i])
            iy_ = csp.calc_y(self.s[i])
            if math.isnan(ix_) or math.isnan(iy_):
                break
            iyaw_ = csp.calc_yaw(self.s[i])
            self.ix.append(ix_)
            self.iy.append(iy_)
            self.iyaw.append(iyaw_)
            di = self.d[i]
            fx = ix_ + di * math.cos(iyaw_ + PI / 2.0)
            fy = iy_ + di * math.sin(iyaw_ + PI / 2.0)
            self.x.append(fx)
            self.y.append(fy)

        if len(self.x) <= 1:
            return False

        # calc yaw and ds
        for i in range(len(self.x) - 1):
            dx = self.x[i + 1] - self.x[i]
            dy = self.y[i + 1] - self.y[i]
            self.yaw.append(math.atan2(dy, dx))
            self.ds.append(math.sqrt(dx * dx + dy * dy))
        if len(self.yaw) > 0:
            self.yaw.append(self.yaw[-1])
            self.ds.append(self.ds[-1])
        else:
            return False

        # calc curvature
        for i in range(len(self.yaw) - 1):
            dyaw = self.yaw[i + 1] - self.yaw[i]
            if dyaw > PI / 2.0:
                dyaw -= 2.0 * PI
            elif dyaw < -PI / 2.0:
                dyaw += 2.0 * PI
            curvature = dyaw / self.ds[i] if self.ds[i] > 1e-9 else 0.0
            if self.ds[i] < 0.1 and self.fot_hp is not None:
                curvature = max(-self.fot_hp.max_curvature,
                                min(self.fot_hp.max_curvature, curvature))
            self.c.append(curvature)

        # max curvature check
        if self.fot_hp is not None:
            if any(abs(cv) > self.fot_hp.max_curvature for cv in self.c):
                return False

        return True

    def is_within_constraints(self) -> bool:
        """Check speed and acceleration constraints."""
        if self.fot_hp is None:
            return True
        if any(abs(v) > self.fot_hp.max_speed for v in self.s_d):
            return False
        if any(abs(a) > self.fot_hp.max_accel for a in self.s_dd):
            return False
        return True

    def is_within_track(self, csp: CubicSpline2D) -> bool:
        """Check if path stays within track bounds. Also computes track-bound cost."""
        min_d = float('inf')
        for i in range(len(self.s)):
            d_r = self.d[i] + csp.calc_d_right(self.s[i])
            d_l = csp.calc_d_left(self.s[i]) - self.d[i]

            if d_r <= 1e-3 or d_l <= 1e-3: # tiny epsilon to avoid being right on boundary
                return False
            else:
                self.d_right.append(d_r)
                self.d_left.append(d_l)
                min_d = min(min_d, d_l, d_r)

        if self.fot_hp is not None and min_d > 0:
            self.c_inv_dist_to_track_bound = (self.fot_hp.max_road_width_r / min_d) - 1.0
        return True

    def is_collision(self, obstacles) -> bool:
        """
        Check for collision with obstacles in Frenet frame.
        obstacles: list of objects with s_start, s_end, d_left, d_right, is_actually_a_gap.
        Returns True if collision detected.
        """
        if not obstacles:
            return False

        self.c_inv_dist_to_obstacles = 0.0
        hp = self.fot_hp

        # filter relevant obstacles
        relevant = []
        for ob in obstacles:
            ob = copy.copy(ob)  # shallow copy to avoid mutating input
            # handle wrapping
            if self.s[0] > hp.global_path_length / 2.0:
                if ob.s_start < hp.global_path_length / 2.0:
                    ob.s_start += hp.global_path_length
                if ob.s_end < hp.global_path_length / 2.0:
                    ob.s_end += hp.global_path_length
            else:
                if ob.s_start > ob.s_end:
                    ob.s_start -= hp.global_path_length

            k_obstacle_lookahead = 5.0
            if (ob.s_start >= self.s[0] and
                    ob.s_start <= self.s[-1] + k_obstacle_lookahead):
                relevant.append(ob)
            elif ob.s_start < self.s[0] and ob.s_end > self.s[0]:
                relevant.append(ob)

        for ob in relevant:
            first_point = True
            min_d_ob = float('inf')

            for i in range(1, len(self.s)):
                if self.s[i] >= ob.s_start:
                    # leading edge
                    if first_point:
                        ds_seg = self.s[i] - self.s[i - 1]
                        if abs(ds_seg) > 1e-12:
                            d_check = ((self.d[i] - self.d[i - 1]) / ds_seg
                                       * (ob.s_start - self.s[i - 1]) + self.d[i - 1])
                        else:
                            d_check = self.d[i]

                        ego_hw = hp.ego_width / 2.0
                        if (ob.d_right - ego_hw) <= d_check <= (ob.d_left + ego_hw):
                            return True
                        elif d_check >= (ob.d_left + ego_hw):
                            min_d_ob = min(min_d_ob, d_check - (ob.d_left + ego_hw))
                        elif d_check <= (ob.d_right - ego_hw):
                            min_d_ob = min(min_d_ob, (ob.d_right - ego_hw) - d_check)
                        first_point = False

                    # trailing edge
                    if self.s[i] > ob.s_end:
                        ds_seg = self.s[i] - self.s[i - 1]
                        if abs(ds_seg) > 1e-12:
                            d_check = ((self.d[i] - self.d[i - 1]) / ds_seg
                                       * (ob.s_end - self.s[i - 1]) + self.d[i - 1])
                        else:
                            d_check = self.d[i]

                        ego_hw = hp.ego_width / 2.0
                        if (ob.d_right - ego_hw) <= d_check <= (ob.d_left + ego_hw):
                            return True
                        elif d_check >= (ob.d_left + ego_hw):
                            min_d_ob = min(min_d_ob, d_check - (ob.d_left + ego_hw))
                        elif d_check <= (ob.d_right - ego_hw):
                            min_d_ob = min(min_d_ob, (ob.d_right - ego_hw) - d_check)
                        break

                    # normal point
                    d_check = self.d[i]
                    ego_hw = hp.ego_width / 2.0
                    if (ob.d_right - ego_hw) <= d_check <= (ob.d_left + ego_hw):
                        return True
                    elif d_check >= (ob.d_left + ego_hw):
                        min_d_ob = min(min_d_ob, d_check - (ob.d_left + ego_hw))
                    elif d_check <= (ob.d_right - ego_hw):
                        min_d_ob = min(min_d_ob, (ob.d_right - ego_hw) - d_check)

            # obstacle starts after path
            if first_point:
                ego_hw = hp.ego_width / 2.0
                dd_sq = 0.0
                if self.d[-1] >= (ob.d_left + ego_hw):
                    dd_sq = (self.d[-1] - (ob.d_left + ego_hw)) ** 2
                elif self.d[-1] <= (ob.d_right - ego_hw):
                    dd_sq = ((ob.d_right - ego_hw) - self.d[-1]) ** 2
                ds_sq = (ob.s_start - self.s[-1]) ** 2
                min_d_ob = math.sqrt(dd_sq + ds_sq)

            if min_d_ob < float('inf') and min_d_ob > 1e-12:
                self.c_inv_dist_to_obstacles += 1.0 / min_d_ob

        return False
