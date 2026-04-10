#!/usr/bin/env python3
"""
param_optimizer_node.py — Bayesian optimisation of race stack parameters.

Replaces bayesopt4ros from the ROS1 ForzaETH stack.
Backend: scikit-optimize (GP + Expected Improvement acquisition function).

OBJECTIVE FUNCTION
------------------
cost = (w_time * lap_time_normalised)
     + (w_crash * crash_penalty)
     + (w_lat   * avg_lateral_error_normalised)

where:
  lap_time_normalised        = lap_time / lap_time_ref
  crash_penalty              = max(0, (boundary_safe_margin - min_dist_to_boundary))
                               / boundary_safe_margin
  avg_lateral_error_normalised = avg_lat_err / lat_err_ref

This multi-term cost matches the ROS1 bayesopt4ros cost function:
  cost = f(lap_time, min_distance_to_boundary, average_lateral_error)

COMPETITION RATIONALE
---------------------
- crash_penalty is non-zero only when the car comes closer than
  boundary_safe_margin [m] to the track wall. The penalty grows
  linearly as the car approaches the wall. This steers the optimizer
  away from parameters that are fast but dangerous.
- For qualifying: set w_crash HIGH (e.g. 5.0) to strongly penalise
  wall proximity. The optimizer will sacrifice lap time to stay safe.
- For fastest lap: reduce w_crash (e.g. 1.0) once you trust the setup.

WORKFLOW
--------
1. Node suggests parameters → applies them via ROS2 SetParameters service.
2. Car drives a lap (manual or autonomous).
3. lap_analyser publishes on /lap_data (once per lap) and
   /min_car_distance_to_boundary (continuously).
4. Node reads cost, updates GP, suggests next parameters.
5. Repeat for n_calls laps. First n_random_starts are random exploration.

MODES
-----
  sector_scalers : velocity scale factors per sector [0.5, 1.0]
                   → targets /sector_tuner/set_parameters
  map_params     : MAP controller gains
                   → targets /controller_manager/set_parameters

USAGE
-----
  ros2 launch param_optimizer param_optimizer_launch.xml \\
    mode:=sector_scalers n_calls:=20 n_random_starts:=5 n_sectors:=4

SAFETY
------
This is an offline TUNING TOOL. Never run during a race.
"""

import json
import time
import threading
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from f110_msgs.msg import LapData
from std_msgs.msg import Float32

try:
    from skopt import gp_minimize
    from skopt.space import Real
    _SKOPT_AVAILABLE = True
except ImportError:
    _SKOPT_AVAILABLE = False


# ── Search space definitions ──────────────────────────────────────────────────

SEARCH_SPACES = {
    'sector_scalers': {
        'target_node': '/sector_tuner',
    },
    'map_params': {
        'dims': [
            Real(0.10, 0.50, name='speed_lookahead'),
            Real(0.00, 1.00, name='lat_err_coeff'),
            Real(0.50, 2.00, name='acc_scaler_for_steer'),
            Real(0.50, 1.00, name='dec_scaler_for_steer'),
        ],
        'target_node': '/controller_manager',
    },
}

# ── Cost function reference values ────────────────────────────────────────────
# These normalise each term so the weights are interpretable.
# lap_time_ref    : expected lap time in seconds — adjust per track.
# lat_err_ref     : expected average lateral error in metres.
# boundary_margin : minimum acceptable distance to track wall in metres.
#                   Below this, crash_penalty is non-zero.
LAP_TIME_REF      = 30.0   # seconds  — tune per track
LAT_ERR_REF       = 0.10   # metres
BOUNDARY_MARGIN   = 0.15   # metres  — car width/2 ≈ 0.135m, so 0.15 gives ~1.5cm margin


class ParamOptimizer(Node):

    def __init__(self):
        super().__init__('param_optimizer')

        if not _SKOPT_AVAILABLE:
            self.get_logger().error(
                'scikit-optimize not installed. '
                'Run inside container: pip install scikit-optimize')
            raise RuntimeError('scikit-optimize not available')

        # ── Declare parameters ────────────────────────────────────────────────
        self.declare_parameter('mode',            'sector_scalers')
        self.declare_parameter('n_calls',         20)
        self.declare_parameter('n_random_starts', 5)
        self.declare_parameter('n_sectors',       4)
        self.declare_parameter('results_path',    '/tmp/bayesopt_results.json')

        # Cost weights — tune these per competition context
        # w_time  : weight on lap time (always keep > 0)
        # w_crash : weight on wall proximity penalty (set HIGH for qualifying)
        # w_lat   : weight on average lateral error (tracking quality)
        self.declare_parameter('w_time',  1.0)
        self.declare_parameter('w_crash', 3.0)   # HIGH: qualifying safety
        self.declare_parameter('w_lat',   0.5)

        # Reference values for normalisation
        self.declare_parameter('lap_time_ref',    LAP_TIME_REF)
        self.declare_parameter('lat_err_ref',     LAT_ERR_REF)
        self.declare_parameter('boundary_margin', BOUNDARY_MARGIN)

        self.mode            = self.get_parameter('mode').value
        self.n_calls         = self.get_parameter('n_calls').value
        self.n_random_starts = self.get_parameter('n_random_starts').value
        self.n_sectors       = self.get_parameter('n_sectors').value
        self.results_path    = self.get_parameter('results_path').value
        self.w_time          = self.get_parameter('w_time').value
        self.w_crash         = self.get_parameter('w_crash').value
        self.w_lat           = self.get_parameter('w_lat').value
        self.lap_time_ref    = self.get_parameter('lap_time_ref').value
        self.lat_err_ref     = self.get_parameter('lat_err_ref').value
        self.boundary_margin = self.get_parameter('boundary_margin').value

        if self.mode not in SEARCH_SPACES:
            raise ValueError(
                f"Unknown mode '{self.mode}'. "
                f"Valid: {list(SEARCH_SPACES.keys())}")

        # ── Build search space ────────────────────────────────────────────────
        if self.mode == 'sector_scalers':
            self.dimensions = [
                Real(0.5, 1.0, name=f'Sector{i}_scaling')
                for i in range(self.n_sectors)
            ]
        else:
            self.dimensions = SEARCH_SPACES[self.mode]['dims']

        self.target_node = SEARCH_SPACES[self.mode]['target_node']

        self.get_logger().info(
            f'\n[ParamOptimizer] Initialised\n'
            f'  mode             : {self.mode}\n'
            f'  dimensions       : {[d.name for d in self.dimensions]}\n'
            f'  n_calls          : {self.n_calls}\n'
            f'  n_random_starts  : {self.n_random_starts}\n'
            f'  w_time           : {self.w_time}\n'
            f'  w_crash          : {self.w_crash}  ← crash avoidance weight\n'
            f'  w_lat            : {self.w_lat}\n'
            f'  boundary_margin  : {self.boundary_margin}m\n'
            f'  lap_time_ref     : {self.lap_time_ref}s\n'
            f'  results_path     : {self.results_path}')

        # ── State ─────────────────────────────────────────────────────────────
        self._lap_data          = None      # most recent LapData message
        self._lap_event         = threading.Event()
        self._min_dist          = float('inf')   # minimum boundary distance this lap
        self._iteration         = 0
        self._results           = []
        self._cb_group          = ReentrantCallbackGroup()

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.get_logger().info(f"[ParamOptimizer] Subscribing to: /lap_data and /min_car_distance_to_boundary")
        self.create_subscription(
            LapData, '/lap_data',
            self._lap_cb, 10,
            callback_group=self._cb_group)

        self.create_subscription(
            Float32, '/min_car_distance_to_boundary',
            self._dist_cb, 10,
            callback_group=self._cb_group)

        self.objective_pub = self.create_publisher(
            Float32, '/optimizer/current_objective', 10)

        # ── Threading Setup ───────────────────────────────────────────────────
        # Use MultiThreadedExecutor so callbacks can run in parallel
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info('[ParamOptimizer] Ready. Starting optimisation...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _lap_cb(self, msg: LapData):
        """Called once per completed lap by lap_analyser."""
        self._lap_data = msg
        self._lap_event.set()

    def _dist_cb(self, msg: Float32):
        """Called continuously. Track the minimum boundary distance this lap."""
        # Log every ~10th message to avoid flooding while still confirming activity
        self.get_logger().debug(f"[ParamOptimizer] Received dist: {msg.data:.3f}m | Min so far: {self._min_dist:.3f}m")
        if msg.data < self._min_dist:
            self._min_dist = msg.data

    # ── Cost function ─────────────────────────────────────────────────────────

    def _compute_cost(self, lap_data: LapData, min_dist: float) -> float:
        """
        Compute the weighted multi-objective cost.

        cost = w_time  * (lap_time / lap_time_ref)
             + w_crash * crash_penalty
             + w_lat   * (avg_lat_err / lat_err_ref)

        crash_penalty = max(0, boundary_margin - min_dist) / boundary_margin
          = 0.0  when min_dist >= boundary_margin  (car stayed safe all lap)
          = 1.0  when min_dist = 0  (car touched the wall)
          Grows linearly between these extremes.

        All three terms are dimensionless and O(1) for nominal driving,
        so the weights are directly interpretable as relative importance.
        """
        # Term 1: lap time
        t_norm = lap_data.lap_time / self.lap_time_ref

        # Term 2: crash avoidance
        # Non-zero only when car came within boundary_margin of the wall
        crash_penalty = max(0.0, self.boundary_margin - min_dist) / self.boundary_margin

        # Term 3: lateral tracking error
        lat_norm = lap_data.average_lateral_error_to_global_waypoints / self.lat_err_ref

        cost = (self.w_time  * t_norm
              + self.w_crash * crash_penalty
              + self.w_lat   * lat_norm)

        self.get_logger().info(
            f'[ParamOptimizer] Cost breakdown:\n'
            f'  t_norm        = {t_norm:.4f}  × w={self.w_time}  → {self.w_time * t_norm:.4f}\n'
            f'  crash_penalty = {crash_penalty:.4f}  × w={self.w_crash} → {self.w_crash * crash_penalty:.4f}\n'
            f'  lat_norm      = {lat_norm:.4f}  × w={self.w_lat}  → {self.w_lat * lat_norm:.4f}\n'
            f'  TOTAL COST    = {cost:.4f}')

        return cost

    # ── Parameter application ─────────────────────────────────────────────────

    def _apply_params(self, param_values: list) -> bool:
        """Push suggested parameters to target node via SetParameters service."""
        client = self.create_client(
            SetParameters, f'{self.target_node}/set_parameters',
            callback_group=self._cb_group)

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'SetParameters service not available on {self.target_node}. '
                f'Is the node running?')
            return False

        params = []
        for dim, val in zip(self.dimensions, param_values):
            pv              = ParameterValue()
            pv.type         = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(val)
            p               = Parameter()
            p.name          = dim.name
            p.value         = pv
            params.append(p)

        req            = SetParameters.Request()
        req.parameters = params
        future         = client.call_async(req)
        
        # Wait for future without touching the executor (the background thread handles it)
        start_wait = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start_wait > 5.0:
                self.get_logger().error('SetParameters call timed out.')
                return False

        if future.result() is None:
            self.get_logger().error('SetParameters call timed out.')
            return False

        self.get_logger().info(
            f'[ParamOptimizer] Applied parameters:\n'
            + '\n'.join(f'  {d.name} = {v:.4f}'
                        for d, v in zip(self.dimensions, param_values)))
        return True

    # ── Lap wait ──────────────────────────────────────────────────────────────

    def _wait_for_lap(self, timeout_sec: float = 300.0) -> LapData:
        """
        Block until lap_analyser publishes a completed lap.
        The MultiThreadedExecutor runs in a background thread so
        callbacks process continuously — no spin needed here.
        """
        self._lap_event.clear()
        self._min_dist = float('inf')
        
        # Block until _lap_cb calls .set()
        if not self._lap_event.wait(timeout=timeout_sec):
            raise TimeoutError(
                f'No lap completed within {timeout_sec}s. '
                f'Is lap_analyser running and is the car moving?')

        # Brief sleep to catch any final distance messages from the background thread
        time.sleep(0.2)

        self.get_logger().info(
            f'[ParamOptimizer] Lap {self._lap_data.lap_count} complete:\n'
            f'  lap_time    : {self._lap_data.lap_time:.3f}s\n'
            f'  avg_lat_err : {self._lap_data.average_lateral_error_to_global_waypoints:.4f}m\n'
            f'  min_dist    : {self._min_dist:.3f}m  '
            f'(margin={self.boundary_margin}m, '
            f'{"SAFE ✓" if self._min_dist >= self.boundary_margin else "DANGER ✗"})')

        return self._lap_data

    # ── Objective (called by gp_minimize) ────────────────────────────────────

    def _objective(self, param_values: list) -> float:
        self._iteration += 1
        sep = '=' * 60
        self.get_logger().info(
            f'\n{sep}\n'
            f'[ParamOptimizer] ITERATION {self._iteration}/{self.n_calls}\n'
            f'{sep}')

        if not self._apply_params(param_values):
            self.get_logger().warn('Parameter application failed — returning penalty cost.')
            return 999.0

        self.get_logger().info(
            '[ParamOptimizer] Parameters applied. Drive a lap now...')

        try:
            lap_data = self._wait_for_lap(timeout_sec=300.0)
        except TimeoutError as e:
            self.get_logger().warn(str(e))
            return 999.0

        # Capture min_dist at lap end (it resets at start of next _wait_for_lap)
        min_dist_this_lap = self._min_dist
        cost = self._compute_cost(lap_data, min_dist_this_lap)

        # Persist result
        self._results.append({
            'iteration'   : self._iteration,
            'params'      : dict(zip([d.name for d in self.dimensions], param_values)),
            'lap_time'    : lap_data.lap_time,
            'min_dist_to_boundary' : min_dist_this_lap,
            'avg_lat_err' : lap_data.average_lateral_error_to_global_waypoints,
            'cost'        : cost,
        })
        self._save_results()

        msg      = Float32()
        msg.data = float(cost)
        self.objective_pub.publish(msg)

        return cost

    def _save_results(self):
        with open(self.results_path, 'w') as f:
            json.dump(self._results, f, indent=2)
        self.get_logger().info(f'[ParamOptimizer] Results → {self.results_path}')

    # ── Main optimisation loop ────────────────────────────────────────────────

    def run(self):
        self.get_logger().info(
            f'[ParamOptimizer] Starting GP Bayesian optimisation.\n'
            f'  Total laps     : {self.n_calls}\n'
            f'  Random explore : first {self.n_random_starts} laps\n'
            f'  GP exploit     : remaining {self.n_calls - self.n_random_starts} laps\n'
            f'  Cost weights   : time={self.w_time}, crash={self.w_crash}, lat={self.w_lat}\n'
            f'  Results file   : {self.results_path}')

        result = gp_minimize(
            func=self._objective,
            dimensions=self.dimensions,
            n_calls=self.n_calls,
            n_random_starts=self.n_random_starts,
            acq_func='EI',      # Expected Improvement
            noise=0.01,
            random_state=42,
        )

        best = dict(zip([d.name for d in self.dimensions], result.x))
        self.get_logger().info(
            f'\n{"=" * 60}\n'
            f'[ParamOptimizer] OPTIMISATION COMPLETE\n'
            f'  Best cost   : {result.fun:.4f}\n'
            f'  Best params : {best}\n'
            f'{"=" * 60}')

        # Apply best params permanently
        self._apply_params(result.x)
        self._save_results()


def main(args=None):
    rclpy.init(args=args)
    node = ParamOptimizer()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('[ParamOptimizer] Interrupted by user.')
    finally:
        node._executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()