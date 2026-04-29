#!/usr/bin/env python3
"""
Frenet-space Informed RRT* planner for static obstacle avoidance.

Activates only when a static obstacle is detected inside the lookahead window.
Publishes OTWpntArray to /planner/avoidance/otwpnts — the same topic as the
spliner — so the existing state machine handles OVERTAKE transitions unchanged.
Publishes an empty message when no static obstacle is present; the state machine
ignores empty messages, allowing the spliner to serve dynamic opponent avoidance
concurrentally on the same topic.

Algorithm: Informed RRT* in (s, d) Frenet coordinates.
  - Free space  : d_right(s) < d < d_left(s) from global waypoints (walls)
                  AND outside all static obstacle bounding boxes (with margin)
  - Start       : (cur_s, cur_d)
  - Goal        : (s_obs_end + goal_offset, 0)  — back on raceline past the obstacle
  - Sampling    : uniform until first solution found, then ellipse focused between
                  start and goal (Informed RRT*)
  - Nearest/near: numpy vectorized linear scan — no KDTree build overhead
  - Output      : B-spline-smoothed path resampled at 0.1m, converted to Cartesian
                  via FrenetConverter, published as OTWpntArray
"""
import time
import numpy as np
from scipy.interpolate import splprep, splev

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from f110_msgs.msg import Wpnt, WpntArray, ObstacleArray, OTWpntArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
from frenet_conversion.frenet_converter import FrenetConverter


class _Node:
    """Lightweight RRT tree node in Frenet (s, d) space."""
    __slots__ = ('s', 'd', 'parent', 'cost')

    def __init__(self, s: float, d: float, parent=None, cost: float = 0.0):
        self.s = s
        self.d = d
        self.parent = parent
        self.cost = cost


class StaticRRTPlanner(Node):

    def __init__(self):
        super().__init__('static_rrt_node')

        # ── car state ──────────────────────────────────────────────────────────
        self.cur_s: float = 0.0
        self.cur_d: float = 0.0
        self.cur_vs: float = 0.0

        # ── map data ───────────────────────────────────────────────────────────
        self.obs = ObstacleArray()
        self.gb_s = None        # np.ndarray  s values of scaled global waypoints
        self.gb_dleft = None    # np.ndarray  d_left per waypoint
        self.gb_dright = None   # np.ndarray  d_right per waypoint
        self.gb_v = None        # np.ndarray  reference velocity per waypoint
        self.gb_max_s: float = 1.0
        self.gb_max_idx: int = 1
        self.gb_vmax: float = 1.0
        self.converter: FrenetConverter = None

        self._gb_received = False
        self._state_received = False

        # hysteresis anchor: set once so state machine sees a "long-ago" switch
        self._hyst_anchor: TimeMsg = None

        # debug diagnostics populated by _plan() on failure
        self._last_plan_iters: int   = 0
        self._last_plan_ms:    float = 0.0
        self._last_plan_nodes: int   = 0

        # path-persistence cache: last successfully planned path, kept around so a
        # one-off RRT failure doesn't blank the avoidance topic and trigger FTG.
        self._cached_s: np.ndarray = None
        self._cached_d: np.ndarray = None
        self._cached_obs_s: float  = None  # s_center of obstacle the cache was built for
        self._cached_stamp: float  = 0.0   # time.perf_counter when cache was filled

        # ── parameters ─────────────────────────────────────────────────────────
        self.declare_parameter('lookahead',             15.0)   # [m] larger than spliner for early reaction
        self.declare_parameter('obs_traj_tresh',         1.0)   # [m] max |d_center| to consider obstacle
        self.declare_parameter('max_iter',             400)     # RRT* iterations per planning cycle
        self.declare_parameter('time_limit',            0.040)  # [s] hard wall-clock cap per cycle
        self.declare_parameter('step_size',              0.5)   # [m] Frenet steer distance
        self.declare_parameter('goal_thresh',            0.8)   # [m] goal acceptance radius
        self.declare_parameter('rrt_star_radius',        2.5)   # [m] rewire neighbourhood
        self.declare_parameter('goal_offset',           10.0)   # [m] past obstacle s_end
        self.declare_parameter('avoidance_speed_factor', 0.65)  # fraction of reference speed
        self.declare_parameter('spline_bound_mindist',   0.15)  # [m] wall safety margin
        self.declare_parameter('car_width',              0.28)  # [m]
        self.declare_parameter('obs_margin',             0.15)  # [m] extra clearance from obstacle edge
        # B: clearance-weighted cost. Path cost = length * (1 + k * mean(1/clearance)).
        # k=0 reproduces pure shortest-path RRT*. k>0 makes wider arcs cheaper than tight squeezes.
        self.declare_parameter('clearance_weight',       0.5)
        # C: sampling bias toward the side with more room past the obstacle.
        # bias_prob fraction of pre-solution samples constrained to preferred half-corridor.
        self.declare_parameter('bias_prob',              0.7)
        # Smoothing factor passed to splprep. 0.0 = exact interpolation through RRT nodes.
        # Higher = smoother but flattens detour amplitude.
        self.declare_parameter('smoothing_factor',       0.0)
        self.declare_parameter('measure',               False)
        self._read_params()

        # ── publishers ─────────────────────────────────────────────────────────
        qos = QoSProfile(depth=10)
        self.evasion_pub  = self.create_publisher(OTWpntArray, '/planner/avoidance/otwpnts',      qos)
        self.mrks_pub     = self.create_publisher(MarkerArray, '/planner/avoidance/markers_rrt',  qos)
        if self.measure:
            self.latency_pub = self.create_publisher(Float32, '/planner/rrt/latency', qos)

        # ── subscribers ────────────────────────────────────────────────────────
        self.create_subscription(ObstacleArray, '/perception/obstacles',      self._obs_cb,    qos)
        self.create_subscription(Odometry,      '/car_state/frenet/odom',     self._state_cb,  qos)
        self.create_subscription(WpntArray,     '/global_waypoints_scaled',   self._gb_cb,     qos)
        self.create_subscription(WpntArray,     '/global_waypoints',          self._gb_raw_cb, qos)

        # wait for map + state before starting the planner timer
        self.get_logger().info('Static RRT planner waiting for global waypoints and car state...')
        while rclpy.ok() and not (self._gb_received and self._state_received):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Static RRT planner ready.')

        # Timer matches time_limit so each cycle has the full budget without queue
        # starvation. State machine consumes wpnts at ~5 Hz so 10 Hz publish is plenty.
        self.create_timer(max(0.1, self.time_limit + 0.02), self._plan_loop)

    # ── param helper ───────────────────────────────────────────────────────────

    def _read_params(self):
        self.lookahead             = self.get_parameter('lookahead').value
        self.obs_traj_tresh        = self.get_parameter('obs_traj_tresh').value
        self.max_iter              = int(self.get_parameter('max_iter').value)
        self.time_limit            = float(self.get_parameter('time_limit').value)
        self.step_size             = self.get_parameter('step_size').value
        self.goal_thresh           = self.get_parameter('goal_thresh').value
        self.rrt_star_radius       = self.get_parameter('rrt_star_radius').value
        self.goal_offset           = self.get_parameter('goal_offset').value
        self.avoidance_speed_factor = self.get_parameter('avoidance_speed_factor').value
        self.spline_bound_mindist  = self.get_parameter('spline_bound_mindist').value
        self.car_width             = self.get_parameter('car_width').value
        self.obs_margin            = self.get_parameter('obs_margin').value
        self.clearance_weight      = float(self.get_parameter('clearance_weight').value)
        self.bias_prob             = float(self.get_parameter('bias_prob').value)
        self.smoothing_factor      = float(self.get_parameter('smoothing_factor').value)
        self.measure               = self.get_parameter('measure').value

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _obs_cb(self, msg: ObstacleArray):
        self.obs = msg

    def _state_cb(self, msg: Odometry):
        self.cur_s  = msg.pose.pose.position.x
        self.cur_d  = msg.pose.pose.position.y
        self.cur_vs = msg.twist.twist.linear.x
        self._state_received = True

    def _gb_raw_cb(self, msg: WpntArray):
        if self.converter is None and len(msg.wpnts) > 1:
            xs   = np.array([w.x_m    for w in msg.wpnts])
            ys   = np.array([w.y_m    for w in msg.wpnts])
            psis = np.array([w.psi_rad for w in msg.wpnts])
            self.converter = FrenetConverter(xs, ys, psis)
            self.get_logger().info('FrenetConverter initialised.')

    def _gb_cb(self, msg: WpntArray):
        if len(msg.wpnts) < 2:
            return
        self.gb_s      = np.array([w.s_m    for w in msg.wpnts])
        self.gb_dleft  = np.array([w.d_left  for w in msg.wpnts])
        self.gb_dright = np.array([w.d_right for w in msg.wpnts])
        self.gb_v      = np.array([w.vx_mps  for w in msg.wpnts])
        self.gb_max_s   = float(msg.wpnts[-1].s_m)
        self.gb_max_idx = len(msg.wpnts)
        self.gb_vmax    = float(np.max(self.gb_v))
        self._gb_received = True

    # ── map query helpers ──────────────────────────────────────────────────────

    def _wpnt_idx(self, s: float) -> int:
        """Nearest global-waypoint index for arc-length s (handles wrap)."""
        s_mod = s % self.gb_max_s
        idx = int(np.searchsorted(self.gb_s, s_mod))
        return min(idx, self.gb_max_idx - 1)

    def _wall_bounds(self, s: float):
        """Return (d_lo, d_hi) drivable lateral bounds at s, including car-width margin."""
        i = self._wpnt_idx(s)
        half_car = self.car_width / 2.0
        d_lo = -self.gb_dright[i] + self.spline_bound_mindist + half_car
        d_hi =  self.gb_dleft[i]  - self.spline_bound_mindist - half_car
        return d_lo, d_hi

    def _ref_velocity(self, s: float) -> float:
        return float(self.gb_v[self._wpnt_idx(s)])

    # ── collision checks ───────────────────────────────────────────────────────

    def _point_free(self, s: float, d: float, obs_list: list) -> bool:
        """True iff (s, d) is inside track bounds and outside all obstacle boxes."""
        d_lo, d_hi = self._wall_bounds(s)
        if not (d_lo < d < d_hi):
            return False
        s_mod = s % self.gb_max_s
        margin = self.car_width / 2.0 + self.obs_margin
        for o in obs_list:
            if o.s_end >= o.s_start:
                in_s = o.s_start <= s_mod <= o.s_end
            else:
                in_s = s_mod >= o.s_start or s_mod <= o.s_end
            if in_s and (o.d_right - margin) <= d <= (o.d_left + margin):
                return False
        return True

    def _edge_free(self, n1: _Node, n2: _Node, obs_list: list, n_checks: int = 5) -> bool:
        """Collision-only check along the straight Frenet segment from n1 to n2."""
        for t in np.linspace(0.0, 1.0, n_checks + 1)[1:]:
            if not self._point_free(n1.s + t * (n2.s - n1.s),
                                    n1.d + t * (n2.d - n1.d),
                                    obs_list):
                return False
        return True

    def _clearance(self, s: float, d: float, obs_list: list) -> float:
        """Lateral clearance to the nearest obstacle/wall at (s,d). Always positive
        for points returned by `_point_free=True`. Used as the inverse weight in
        the clearance-penalised edge cost (Fix B)."""
        d_lo, d_hi = self._wall_bounds(s)
        clr = min(d - d_lo, d_hi - d)
        s_mod = s % self.gb_max_s
        margin = self.car_width / 2.0 + self.obs_margin
        for o in obs_list:
            if o.s_end >= o.s_start:
                in_s = o.s_start <= s_mod <= o.s_end
            else:
                in_s = s_mod >= o.s_start or s_mod <= o.s_end
            if not in_s:
                continue
            if d > o.d_left + margin:
                clr = min(clr, d - (o.d_left + margin))
            elif d < o.d_right - margin:
                clr = min(clr, (o.d_right - margin) - d)
            else:
                return 0.0
        return max(clr, 0.0)

    def _edge_eval(self, n1: _Node, n2: _Node, obs_list: list, n_checks: int = 5):
        """Combined collision check + clearance-weighted cost (Fix B).

        Returns (free, weighted_cost). Pure path-length cost when
        clearance_weight=0; otherwise length * (1 + k * mean(1/clearance))
        — wider corridors become strictly cheaper than tight squeezes."""
        length = float(np.hypot(n2.s - n1.s, n2.d - n1.d))
        if self.clearance_weight <= 0.0:
            return self._edge_free(n1, n2, obs_list, n_checks), length
        inv_clr_sum = 0.0
        n_used = 0
        for t in np.linspace(0.0, 1.0, n_checks + 1)[1:]:
            s = n1.s + t * (n2.s - n1.s)
            d = n1.d + t * (n2.d - n1.d)
            if not self._point_free(s, d, obs_list):
                return False, 0.0
            clr = self._clearance(s, d, obs_list)
            inv_clr_sum += 1.0 / max(clr, 0.05)
            n_used += 1
        avg_inv_clr = inv_clr_sum / n_used
        return True, length * (1.0 + self.clearance_weight * avg_inv_clr)

    # ── Informed RRT* ──────────────────────────────────────────────────────────

    def _preferred_side(self, obs):
        """Pick which side of the obstacle has more usable corridor (Fix C).

        Returns 'left', 'right', or None (both sides blocked). The signed-Frenet
        accounting matches `_point_free`: total margin per side is one car width
        plus obs_margin plus spline_bound_mindist.
        """
        i = self._wpnt_idx(obs.s_center)
        margin_total = self.car_width + self.obs_margin + self.spline_bound_mindist
        left_avail  = self.gb_dleft[i]  - obs.d_left  - margin_total
        right_avail = obs.d_right + self.gb_dright[i] - margin_total
        if max(left_avail, right_avail) <= 0.0:
            return None
        return 'left' if left_avail >= right_avail else 'right'

    def _sample(self, s0: float, d0: float, sg: float,
                c_best: float, c_min: float,
                best_pos: np.ndarray,
                preferred_side) -> np.ndarray:
        # Pre-solution: uniform over track corridor [s0, sg+buf] × [d_lo(s), d_hi(s)].
        # Fix C: with probability bias_prob, restrict d to the half-corridor on the
        # side with more room past the obstacle. Falls back to full corridor if the
        # half is empty (e.g. inside a tight chicane).
        if c_best == float('inf'):
            s_samp = np.random.uniform(s0, sg + 2.0)
            d_lo, d_hi = self._wall_bounds(s_samp)
            if preferred_side is not None and np.random.random() < self.bias_prob:
                if preferred_side == 'left':
                    d_lo_b = max(d_lo, 0.0)
                    d_hi_b = d_hi
                else:
                    d_lo_b = d_lo
                    d_hi_b = min(d_hi, 0.0)
                if d_hi_b > d_lo_b:
                    d_lo, d_hi = d_lo_b, d_hi_b
            d_samp = np.random.uniform(d_lo, d_hi)
            return np.array([s_samp, d_samp])

        # Post-solution: ellipse focused on (start, best_leaf). Half-plane goal
        # has no fixed point, so the second focus rides the best leaf found so far.
        cx, cy = (s0 + best_pos[0]) / 2.0, (d0 + best_pos[1]) / 2.0
        a = c_best / 2.0
        b = max(np.sqrt(max(c_best**2 - c_min**2, 0.0)) / 2.0, 1e-4)
        angle = np.arctan2(best_pos[1] - d0, best_pos[0] - s0)

        r     = np.sqrt(np.random.uniform(0.0, 1.0))
        theta = np.random.uniform(0.0, 2.0 * np.pi)
        ca, sa = np.cos(angle), np.sin(angle)
        x_loc = r * np.cos(theta) * a
        y_loc = r * np.sin(theta) * b
        return np.array([ca * x_loc - sa * y_loc + cx,
                         sa * x_loc + ca * y_loc + cy])

    def _plan(self, s0: float, d0: float, sg: float, obs_list: list, preferred_side=None):
        """Run Informed RRT* from (s0,d0) to half-plane {s >= sg}.

        Goal is a finish line, not a point — leaf may end at any safe d.
        Removes raceline bias: cost is path length only, no penalty for
        finishing far from d=0. Spliner/global planner takes over the merge.
        """
        start = _Node(s0, d0, cost=0.0)
        nodes = [start]
        pos = np.empty((self.max_iter + 2, 2), dtype=np.float64)
        pos[0] = (s0, d0)
        n = 1

        best_leaf = None
        best_pos  = np.array([sg, d0])
        c_best = float('inf')
        c_min  = abs(sg - s0)
        r2     = self.rrt_star_radius ** 2
        t_start = time.perf_counter()
        iters_done = 0

        for _ in range(self.max_iter):
            if time.perf_counter() - t_start > self.time_limit:
                break
            iters_done += 1

            x_rand = self._sample(s0, d0, sg, c_best, c_min, best_pos, preferred_side)
            diffs  = pos[:n] - x_rand
            dists2 = diffs[:, 0]**2 + diffs[:, 1]**2
            nn_idx = int(np.argmin(dists2))
            d_nn   = np.sqrt(dists2[nn_idx])
            if d_nn < 1e-6: continue
            
            x_near = nodes[nn_idx]
            step    = min(self.step_size, d_nn)
            new_pos = pos[nn_idx] - diffs[nn_idx] * (step / d_nn)
            x_new   = _Node(new_pos[0], new_pos[1])

            free, edge_cost = self._edge_eval(x_near, x_new, obs_list)
            if not free:
                continue

            nd  = pos[:n] - new_pos
            nd2 = nd[:, 0]**2 + nd[:, 1]**2
            near_idxs = np.where(nd2 <= r2)[0]

            best_parent = x_near
            best_cost   = x_near.cost + edge_cost
            for i in near_idxs:
                cand = nodes[i]
                free_c, c_edge = self._edge_eval(cand, x_new, obs_list)
                if not free_c:
                    continue
                c = cand.cost + c_edge
                if c < best_cost:
                    best_parent = cand
                    best_cost   = c

            x_new.parent = best_parent
            x_new.cost   = best_cost
            pos[n]       = new_pos
            nodes.append(x_new)
            n += 1

            for i in near_idxs:
                cand = nodes[i]
                if cand is best_parent: continue
                free_c, c_edge = self._edge_eval(x_new, cand, obs_list)
                if not free_c:
                    continue
                c = x_new.cost + c_edge
                if c < cand.cost:
                    cand.parent = x_new
                    cand.cost   = c

            # Half-plane acceptance: any leaf past the finish line s=sg counts.
            # Cost is path length from start; no penalty for finishing off-raceline.
            if new_pos[0] >= sg:
                if x_new.cost < c_best:
                    c_best    = x_new.cost
                    best_leaf = x_new
                    best_pos  = new_pos.copy()
                    c_min     = np.hypot(best_pos[0] - s0, best_pos[1] - d0)

        self._last_plan_iters = iters_done
        self._last_plan_ms    = (time.perf_counter() - t_start) * 1000.0
        self._last_plan_nodes = n

        if best_leaf is None:
            return None, nodes

        path, node = [], best_leaf
        while node is not None:
            path.append((node.s, node.d))
            node = node.parent
        path.reverse()
        return path, nodes

    # ── path smoothing ─────────────────────────────────────────────────────────

    def _smooth(self, path: list, obs_list: list):
        s_raw = np.array([p[0] for p in path])
        d_raw = np.array([p[1] for p in path])

        if len(path) >= 4:
            try:
                k = min(3, len(path) - 1)
                tck, _ = splprep([s_raw, d_raw], s=self.smoothing_factor, k=k)
                n_pts  = max(int((s_raw[-1] - s_raw[0]) / 0.05) + 1, 10)
                s_raw, d_raw = splev(np.linspace(0.0, 1.0, n_pts), tck)
            except Exception:
                pass

        s_out = np.arange(s_raw[0], s_raw[-1], 0.1)
        d_out = np.interp(s_out, s_raw, d_raw)
        for i, s in enumerate(s_out):
            d_lo, d_hi = self._wall_bounds(s)
            d_out[i] = float(np.clip(d_out[i], d_lo, d_hi))
            # Post-smooth obstacle re-check: splev can drift the curve back into an
            # obstacle's rejection zone even when raw RRT nodes avoided it. Push the
            # sample laterally to the nearest collision-free d.
            if not self._point_free(s, d_out[i], obs_list):
                d_out[i] = self._project_d_free(s, d_out[i], obs_list, d_lo, d_hi)

        return s_out % self.gb_max_s, d_out

    def _project_d_free(self, s: float, d: float, obs_list: list,
                        d_lo: float, d_hi: float, step: float = 0.05) -> float:
        """Return the d in (d_lo, d_hi) nearest to input that satisfies _point_free.
        Falls back to the input value if no free d exists within the wall-bounded
        corridor — caller can detect via a follow-up _point_free check."""
        if self._point_free(s, d, obs_list):
            return d
        max_k = int((d_hi - d_lo) / step) + 1
        for k in range(1, max_k + 1):
            for sign in (-1.0, +1.0):
                d_try = d + sign * k * step
                if d_lo < d_try < d_hi and self._point_free(s, d_try, obs_list):
                    return d_try
        return d

    # ── main planning loop ─────────────────────────────────────────────────────

    def _plan_loop(self):
        if self.converter is None or self.gb_s is None:
            return

        t0 = time.perf_counter()
        if self._hyst_anchor is None:
            self._hyst_anchor = self.get_clock().now().to_msg()

        static_obs = []
        for o in self.obs.obstacles:
            if not o.is_static: continue
            if abs(o.d_center) > self.obs_traj_tresh: continue
            dist = (o.s_center - self.cur_s) % self.gb_max_s
            if 0.0 < dist < self.lookahead:
                static_obs.append(o)

        if not static_obs:
            self.evasion_pub.publish(OTWpntArray())
            return

        closest = min(static_obs, key=lambda o: (o.s_center - self.cur_s) % self.gb_max_s)
        s_goal = (closest.s_end % self.gb_max_s) + self.goal_offset

        # Compute preferred_side BEFORE start projection so the projection places
        # the car on the same side as the avoidance corridor.
        preferred_side = self._preferred_side(closest)

        s0, d0 = self.cur_s, self.cur_d
        if not self._point_free(s0, d0, static_obs):
            d_lo, d_hi = self._wall_bounds(s0)
            projected = False
            for delta in np.arange(0.05, d_hi - d_lo, 0.05):
                # Try the preferred avoidance side first so the RRT tree starts
                # on the correct side instead of the nearest collision-free point.
                if preferred_side == 'left':
                    tries = [d0 + delta, d0 - delta]
                else:
                    tries = [d0 - delta, d0 + delta]
                for d_try in tries:
                    if d_lo < d_try < d_hi and self._point_free(s0, d_try, static_obs):
                        d0 = d_try
                        projected = True
                        break
                if projected:
                    break
            if not projected:
                self.get_logger().warn('RRT*: start inside obstacle and no free d found — skip', throttle_duration_sec=2.0)
                self.evasion_pub.publish(OTWpntArray())
                return
            self.get_logger().info(f'RRT*: start d projected {self.cur_d:.2f}→{d0:.2f}', throttle_duration_sec=1.0)
        path, tree = self._plan(s0, d0, s_goal, static_obs, preferred_side)
        if path is None:
            self.get_logger().warn(
                f'RRT*: no path — obs S={closest.s_center:.1f}m '
                f'actual: {self._last_plan_iters} iters / {self._last_plan_ms:.1f}ms '
                f'/ {self._last_plan_nodes} nodes (limit: {self.max_iter}/{self.time_limit*1000:.0f}ms)',
                throttle_duration_sec=2.0)
            # Path persistence: prefer republishing the previous good path over an
            # empty wpnts message. State machine treats empty as "no plan available"
            # and bleeds speed → FTG, even when the prior path is still valid.
            if self._republish_cached(closest):
                if self.measure:
                    self.latency_pub.publish(Float32(data=float(time.perf_counter() - t0)))
                return
            self.evasion_pub.publish(OTWpntArray())
            return

        s_arr, d_arr = self._smooth(path, static_obs)
        cart, v_arr, n_pts = self._publish_wpnts(s_arr, d_arr)
        # Cache for next-cycle republish on transient planner failure.
        self._cached_s     = s_arr.copy()
        self._cached_d     = d_arr.copy()
        self._cached_obs_s = closest.s_center
        self._cached_stamp = time.perf_counter()

        # Half-plane goal — visualise the actual leaf the planner committed to.
        goal_cart = self.converter.get_cartesian(np.array([s_arr[-1]]), np.array([d_arr[-1]]))
        disp_tree = tree[::2]
        tree_s, tree_d = [], []
        for node in disp_tree:
            if node.parent:
                tree_s.extend([node.parent.s, node.s])
                tree_d.extend([node.parent.d, node.d])
        tree_cart = ([], [])
        if tree_s: tree_cart = self.converter.get_cartesian(np.array(tree_s), np.array(tree_d))

        self._visualise(s_arr, d_arr, cart[0], cart[1], v_arr, tree_cart[0], tree_cart[1], float(goal_cart[0][0]), float(goal_cart[1][0]))
        elapsed = time.perf_counter() - t0
        self.get_logger().info(f'RRT*: {n_pts} pts | obs S={closest.s_center:.1f}m | {elapsed*1000:.1f}ms', throttle_duration_sec=1.0)
        if self.measure: self.latency_pub.publish(Float32(data=float(elapsed)))

    # ── publish helpers ────────────────────────────────────────────────────────

    def _publish_wpnts(self, s_arr: np.ndarray, d_arr: np.ndarray):
        """Build an OTWpntArray from s/d arrays and publish. Returns
        (cart, v_arr, n_pts) so the caller can reuse cart/v for visualisation."""
        cart  = self.converter.get_cartesian(s_arr, d_arr)
        v_arr = np.array([self._ref_velocity(s) * self.avoidance_speed_factor for s in s_arr])
        msg = OTWpntArray()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'static_rrt'
        msg.ot_side          = 'left' if float(np.mean(d_arr)) >= 0.0 else 'right'
        msg.ot_line          = msg.ot_side
        msg.side_switch      = False
        msg.last_switch_time = self._hyst_anchor
        msg.wpnts = [
            Wpnt(id=i, s_m=float(s), d_m=float(d), x_m=float(x), y_m=float(y), vx_mps=float(v))
            for i, (s, d, x, y, v) in enumerate(zip(s_arr, d_arr, cart[0], cart[1], v_arr))
        ]
        self.evasion_pub.publish(msg)
        return cart, v_arr, len(msg.wpnts)

    def _republish_cached(self, closest_obs) -> bool:
        """Re-publish the last good path trimmed to the segment ahead of the car.
        Returns True if a valid cache was used. Cache is only used when the same
        obstacle is still being addressed and the cache is fresh enough that the
        underlying scene cannot have meaningfully changed.

        Why this matters: a single empty publish on the avoidance topic kicks the
        state machine into its FTG ramp. RRT* is stochastic — one bad cycle out
        of ten is normal. Persisting a stale-but-valid path bridges those gaps.
        """
        if self._cached_s is None or self._cached_d is None:
            return False
        if self._cached_obs_s is None:
            return False
        # Same obstacle? s_center jumps to the next obstacle once the car passes,
        # which invalidates the previous detour geometry.
        if abs(self._cached_obs_s - closest_obs.s_center) > 0.5:
            return False
        # Freshness: 1.0s is roughly 10 plan cycles at the new 10 Hz timer.
        if (time.perf_counter() - self._cached_stamp) > 1.0:
            return False
        # Trim to ahead of car. Small lookback so the controller can interpolate.
        mask = self._cached_s >= (self.cur_s - 0.2)
        if int(np.sum(mask)) < 4:
            return False
        s_trim = self._cached_s[mask]
        d_trim = self._cached_d[mask]
        self._publish_wpnts(s_trim, d_trim)
        self.get_logger().info(
            f'RRT*: republished cached path ({len(s_trim)} pts) — bridging plan miss',
            throttle_duration_sec=1.0)
        return True

    def _visualise(self, s, d, x, y, v, tree_x, tree_y, gx, gy):
        mrks = MarkerArray()
        del_m = Marker()
        del_m.header.frame_id = 'map'
        del_m.action = Marker.DELETEALL
        mrks.markers.append(del_m)
        now = self.get_clock().now().to_msg()
        
        gm = Marker()
        gm.header.frame_id, gm.header.stamp = 'map', now
        gm.type, gm.id = Marker.SPHERE, 0
        gm.scale.x = gm.scale.y = gm.scale.z = 0.4
        gm.color.a, gm.color.r, gm.color.g, gm.color.b = 0.8, 0.0, 0.4, 1.0
        gm.pose.position.x, gm.pose.position.y, gm.pose.position.z = gx, gy, 0.2
        mrks.markers.append(gm)

        if len(tree_x) > 0:
            tm = Marker()
            tm.header.frame_id, tm.header.stamp = 'map', now
            tm.type, tm.id = Marker.LINE_LIST, 1000
            tm.scale.x, tm.color.a = 0.02, 0.3
            tm.color.r = tm.color.g = tm.color.b = 1.0
            from geometry_msgs.msg import Point
            for tx, ty in zip(tree_x, tree_y): tm.points.append(Point(x=float(tx), y=float(ty), z=0.01))
            mrks.markers.append(tm)

        for i in range(0, len(s), 3):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', now
            m.type, m.id = Marker.CYLINDER, i + 1001
            m.scale.x = m.scale.y = 0.12
            m.scale.z = float(v[i] / max(self.gb_vmax, 1.0))
            m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.5, 0.0
            m.pose.position.x, m.pose.position.y = float(x[i]), float(y[i])
            m.pose.position.z, m.pose.orientation.w = m.scale.z / 2.0, 1.0
            mrks.markers.append(m)
        self.mrks_pub.publish(mrks)

def main(args=None):
    rclpy.init(args=args)
    node = StaticRRTPlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
