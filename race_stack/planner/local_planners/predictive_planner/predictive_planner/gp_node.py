#!/usr/bin/env python3
import os
# Force scikit-learn to use single thread for low-level parallelism to avoid conflicts with multiprocessing
os.environ["OMP_NUM_THREADS"] = "1"

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from scipy.optimize import fmin_l_bfgs_b
from sklearn.gaussian_process.kernels import Matern, RBF, WhiteKernel, ConstantKernel
from sklearn.gaussian_process import GaussianProcessRegressor
import numpy as np
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, OpponentTrajectory, OppWpnt, OpponentTrajectories
from frenet_conversion.frenet_converter import FrenetConverter
from multiprocessing import Pool
from .utils import id_to_rgb_color
from std_msgs.msg import Float32, Bool
from ccma import CCMA
import time
import warnings
from sklearn.exceptions import ConvergenceWarning

# Per-bin history depth — enough to build lap-scale memory without unbounded growth.
_BIN_CAPACITY = 50
# When total observations exceed this fraction of the track's bins, compress to this depth.
_COMPRESSION_CAPACITY = 5
# Deviation threshold (m) for off-trajectory detection (matches ROS1: 0.3m).
_OFF_TRAJ_THRESHOLD = 0.3
# Consecutive-frame count before flagging opponent as off-trajectory (matches ROS1: 5).
_OFF_TRAJ_FRAMES = 5

class GPNode(Node):
    """
    ROS node for Gaussian Process based trajectory prediction.
    Uses multiprocessing to fit GPs in the background.
    """
    def __init__(self, pool: Pool):
        super().__init__('gp_node')
        self.pool = pool

        # Parameters
        self.declare_parameter('num_opponents', 2)
        self.declare_parameter('measure', False)
        self.num_opponents = self.get_parameter('num_opponents').value
        self.measuring = self.get_parameter('measure').value

        self.wpnt_spacing = 0.1
        self.bin_spacing = 0.1

        # Internal state
        self.current_s = 0.0
        self.tracked_obs = None
        self.global_waypoints = None
        self.converter = None
        self.max_s = 100.0
        self.max_v = 10.0
        self.boundaries = None

        # Tracking and GP Dictionaries
        self.detection_matching = {
            k: {"id": None, "dist": 1000.0, "cummulated_dist": 0.0, "last_s": None, "last_seen": self.get_clock().now()}
            for k in range(self.num_opponents)
        }
        self.binned_data = {k: {} for k in range(self.num_opponents)}
        self.predictions = {k: None for k in range(self.num_opponents)}
        self.gp_trajectory = {k: {"d": None, "vs": None} for k in range(self.num_opponents)}
        self.is_processing = [False] * self.num_opponents
        self.last_trigger_time = time.time()
        self.collected_messages = []

        # Off-trajectory detection state (per opponent)
        self.off_traj_counter = [0] * self.num_opponents
        self.opp_off_traj = [False] * self.num_opponents

        # Define GP Kernels (Static for worker reuse)
        self.kernel_d = ConstantKernel(0.5) * Matern(length_scale=1.0, nu=1.5) + WhiteKernel(noise_level=1)
        self.kernel_vs = ConstantKernel(0.5) * RBF(length_scale=1.0) + WhiteKernel(noise_level=1)
        
        self.gps_models = {
            "d": GaussianProcessRegressor(kernel=self.kernel_d, optimizer=self.optimizer_wrapper),
            "vs": GaussianProcessRegressor(kernel=self.kernel_vs, optimizer=self.optimizer_wrapper)
        }

        # Subscribers
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_cb, 10)
        self.create_subscription(MarkerArray, "/trackbounds/markers", self.bounds_cb, 10)
        self.create_subscription(ObstacleArray, "/perception/obstacles", self.tracker_cb, 10)
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.odom_frenet_cb, 10)

        # Publishers
        self.opp_trajectory_markers_pub = self.create_publisher(MarkerArray, "/perception/opp_trajectory_markers", 10)
        self.opp_trajectory_pub = self.create_publisher(OpponentTrajectory, '/opponent_trajectory', 10)
        self.opp_trajectories_pub = self.create_publisher(OpponentTrajectories, "/perception/opp_trajectories", 10)
        self.opp_off_traj_pub = self.create_publisher(Bool, '/gp_node/opp_off_traj', 10)
        if self.measuring:
            self.latency_pub = self.create_publisher(Float32, '/gp_node/latency', 10)

        # Main Loop (40Hz)
        self.create_timer(1.0/40.0, self.loop)
        self.get_logger().info("GP Node (Multiprocessing) Initialized")

    @staticmethod
    def optimizer_wrapper(obj_func, initial_theta, bounds):
        sol, val, _ = fmin_l_bfgs_b(obj_func, initial_theta, bounds=bounds)
        return sol, val

    def gb_cb(self, msg: WpntArray):
        if self.global_waypoints is None:
            self.global_waypoints = msg
            self.max_s = msg.wpnts[-1].s_m
            self.max_v = max([w.vx_mps for w in msg.wpnts])
            x = np.array([w.x_m for w in msg.wpnts])
            y = np.array([w.y_m for w in msg.wpnts])
            psi = np.array([w.psi_rad for w in msg.wpnts])
            self.get_logger().info("Initializing FrenetConverter...")
            self.converter = FrenetConverter(x, y, psi)
            self.get_logger().info(f"GP Converter Initialized. Track Length: {self.max_s:.2f}m")

    def bounds_cb(self, msg: MarkerArray):
        if self.boundaries is not None or self.converter is None:
            return
        # Separate boundaries by color (ROS 1 logic: Green is left)
        pts_r = np.array([[m.pose.position.x, m.pose.position.y] for m in msg.markers if m.color.g < 0.5])
        pts_l = np.array([[m.pose.position.x, m.pose.position.y] for m in msg.markers if m.color.g >= 0.5])
        
        f_r = self.converter.get_frenet(pts_r[:, 0], pts_r[:, 1])
        f_l = self.converter.get_frenet(pts_l[:, 0], pts_l[:, 1])
        
        # Sort by s
        f_r = f_r[:, f_r[0].argsort()]
        f_l = f_l[:, f_l[0].argsort()]
        
        new_s = np.linspace(0.0, self.max_s, num=int(self.max_s / self.wpnt_spacing))
        interp_r = np.interp(new_s, f_r[0], f_r[1])
        interp_l = np.interp(new_s, f_l[0], f_l[1])
        self.boundaries = np.stack((interp_r, interp_l), axis=-1)
        self.get_logger().info("GP Track Boundaries Binned")

    def tracker_cb(self, msg: ObstacleArray): self.tracked_obs = msg
    def odom_frenet_cb(self, msg: Odometry): self.current_s = msg.pose.pose.position.x

    def loop(self):
        if self.converter is None or self.tracked_obs is None:
            return
            
        # Watchdog: Reset lock if it hangs for more than 5 seconds
        if any(self.is_processing) and (time.time() - self.last_trigger_time > 5.0):
            self.get_logger().warn("[GP] Processing watchdog triggered! Resetting lock.")
            self.is_processing = [False] * self.num_opponents
        new_obs = [None] * self.num_opponents
        unmatched = []
        
        sorted_obs = sorted(self.tracked_obs.obstacles, key=lambda x: (x.s_center - self.current_s) % self.max_s)
        
        for obs in sorted_obs:
            active_ids = [self.detection_matching[k]["id"] for k in range(self.num_opponents)]
            if obs.id in active_ids:
                idx = active_ids.index(obs.id)
                self.detection_matching[idx]["dist"] = (obs.s_center - self.current_s) % self.max_s
                self.detection_matching[idx]["last_seen"] = self.get_clock().now()
                new_obs[idx] = obs
            else:
                unmatched.append(obs)
        
        if None in new_obs:
            for obs in unmatched:
                idx = np.argmin([(self.get_clock().now() - self.detection_matching[k]["last_seen"]).nanoseconds for k in range(self.num_opponents)])
                if new_obs[idx] is not None:
                    idx = new_obs.index(None)
                self.detection_matching[idx]["id"] = obs.id
                self.detection_matching[idx]["dist"] = (obs.s_center - self.current_s) % self.max_s
                self.detection_matching[idx]["last_s"] = obs.s_center
                self.detection_matching[idx]["last_seen"] = self.get_clock().now()
                new_obs[idx] = obs
                if None not in new_obs: break

        # Accumulate lap distance
        for i in range(self.num_opponents):
            if new_obs[i] is not None and self.detection_matching[i]["last_s"] is not None:
                delta = new_obs[i].s_center - self.detection_matching[i]["last_s"]
                if delta > 0: self.detection_matching[i]["cummulated_dist"] += delta
                elif delta < -0.5 * self.max_s: self.detection_matching[i]["cummulated_dist"] += self.max_s + delta
                self.detection_matching[i]["last_s"] = new_obs[i].s_center

        self.collected_messages.append(new_obs)

        # --- Priority 3: Off-trajectory anomaly detection ---
        # Compare each live detection to the last GP prediction at that s-position.
        # Flag opponents that deviate >0.3m for 5+ consecutive frames (mirrors ROS1).
        any_off_traj = False
        for i in range(self.num_opponents):
            pred = self.predictions[i]
            if pred is not None and new_obs[i] is not None:
                obs_s = new_obs[i].s_center
                obs_d = new_obs[i].d_center
                pred_idx = min(int(obs_s // self.wpnt_spacing), len(pred["d"][0]) - 1)
                predicted_d = float(pred["d"][0][pred_idx])
                if abs(obs_d - predicted_d) > _OFF_TRAJ_THRESHOLD:
                    self.off_traj_counter[i] += 1
                else:
                    self.off_traj_counter[i] = 0
                self.opp_off_traj[i] = self.off_traj_counter[i] > _OFF_TRAJ_FRAMES
            else:
                self.off_traj_counter[i] = 0
                self.opp_off_traj[i] = False
            if self.opp_off_traj[i]:
                any_off_traj = True
        self.opp_off_traj_pub.publish(Bool(data=any_off_traj))

        if not any(self.is_processing):
            self.trigger_processing()

    def trigger_processing(self):
        if not self.collected_messages: return
        start = time.perf_counter()
        self.last_trigger_time = time.time()
        self.is_processing = [True] * self.num_opponents
        msgs = list(self.collected_messages)
        self.collected_messages.clear()

        batch = {n: [m[n] for m in msgs if m[n] is not None] for n in range(self.num_opponents)}

        for opp_id in range(self.num_opponents):
            if not batch[opp_id]:
                self.is_processing[opp_id] = False
                continue
                
            self.get_logger().info(f"[GP] Triggering worker for Opponent {opp_id} with {len(batch[opp_id])} samples...")
            self.pool.apply_async(
                self.process_worker,
                args=(batch[opp_id], opp_id, self.binned_data[opp_id], self.max_s, self.boundaries),
                callback=self.on_worker_done,
                error_callback=self.on_worker_error
            )
        
        self.publish_all()
        if self.measuring:
            self.latency_pub.publish(Float32(data=float(time.perf_counter() - start)))

    def on_worker_error(self, err):
        self.get_logger().error(f"[GP] BACKGROUND WORKER CRASHED: {err}")
        self.is_processing = [False] * self.num_opponents # Full reset to clear the lock

    @staticmethod
    def process_worker(obs_list, opp_id, my_bins, max_s, boundaries=None):
        # Local model definitions for better pickling
        from sklearn.gaussian_process.kernels import Matern, RBF, WhiteKernel, ConstantKernel
        from sklearn.gaussian_process import GaussianProcessRegressor

        kernel_d = ConstantKernel(0.5) * Matern(length_scale=1.0, nu=1.5) + WhiteKernel(noise_level=1)
        kernel_vs = ConstantKernel(0.5) * RBF(length_scale=1.0) + WhiteKernel(noise_level=1)

        model_d = GaussianProcessRegressor(kernel=kernel_d)
        model_v = GaussianProcessRegressor(kernel=kernel_vs)

        bin_spacing = 0.1
        wpnt_spacing = 0.1
        total_bins = int(max_s / bin_spacing)
        # Compress all bins to _COMPRESSION_CAPACITY when total observations
        # exceed 1.5x the expected full-lap bin count (prevents unbounded growth).
        compression_trigger = int(1.5 * total_bins) * _BIN_CAPACITY

        for obs in obs_list:
            # --- Priority 1: Boundary pre-filter ---
            # Discard detections outside track walls before they reach the GP.
            if boundaries is not None:
                b_idx_bound = min(int(obs.s_center // bin_spacing), len(boundaries) - 1)
                d_min, d_max = boundaries[b_idx_bound, 0], boundaries[b_idx_bound, 1]
                if not (d_min <= obs.d_center <= d_max):
                    continue

            b_idx = int(obs.s_center // bin_spacing)
            if b_idx not in my_bins:
                my_bins[b_idx] = {"s": [], "d": [], "vs": []}
            my_bins[b_idx]["s"].append(obs.s_center)
            my_bins[b_idx]["d"].append(obs.d_center)
            my_bins[b_idx]["vs"].append(obs.vs)
            # --- Priority 2: Per-bin ring buffer (50 samples vs original 2) ---
            if len(my_bins[b_idx]["s"]) > _BIN_CAPACITY:
                for k in ["s", "d", "vs"]:
                    my_bins[b_idx][k].pop(0)

        # --- Priority 2: Global compression when memory exceeds threshold ---
        total_obs = sum(len(v["s"]) for v in my_bins.values())
        if total_obs > compression_trigger:
            for b in my_bins:
                for k in ["s", "d", "vs"]:
                    my_bins[b][k] = my_bins[b][k][-_COMPRESSION_CAPACITY:]

        # Fit and Predict
        idxs = sorted(my_bins.keys())
        THRESHOLD = int(0.1 * max_s / bin_spacing)
        full_lap = len(idxs) > int(0.8 * total_bins)

        if len(idxs) % 20 == 0:
            print(f"[GP-Worker-{opp_id}] Bins: {len(idxs)}/{THRESHOLD} threshold, full_lap={full_lap}")

        if len(idxs) > THRESHOLD:
            s_vals, d_vals, vs_vals = [], [], []
            for idx in idxs:
                s_vals.extend(my_bins[idx]["s"])
                d_vals.extend(my_bins[idx]["d"])
                vs_vals.extend(my_bins[idx]["vs"])

            X = np.array(s_vals).reshape(-1, 1)
            warnings.simplefilter("ignore", ConvergenceWarning)
            model_d.fit(X, np.array(d_vals).reshape(-1, 1))
            model_v.fit(X, np.array(vs_vals).reshape(-1, 1))

            s_future = np.linspace(0, max_s, num=int(max_s / wpnt_spacing)).reshape(-1, 1)
            d_p, d_s = model_d.predict(s_future, return_std=True)
            v_p, v_s = model_v.predict(s_future, return_std=True)

            return {
                "id": opp_id, "bins": my_bins, "full_lap": full_lap,
                "preds": {"s": s_future, "d": (d_p, d_s), "vs": (v_p, v_s)}
            }
        return {"id": opp_id, "bins": my_bins, "full_lap": full_lap, "preds": None}

    def on_worker_done(self, res):
        idx = res["id"]
        self.get_logger().info(f"[GP] Worker for Opponent {idx} finished.")
        self.binned_data[idx] = res["bins"]
        if res["preds"]:
            preds = res["preds"]
            # --- Priority 4: CCMA smoothing for full-lap d predictions ---
            # On a complete lap, apply curvature-corrected smoothing to the lateral
            # trajectory before publishing.  This prevents the car from chasing
            # GP artefacts that GP doesn't naturally suppress on closed curves.
            if res.get("full_lap", False) and self.converter is not None:
                try:
                    s_vals = preds["s"].flatten()
                    d_vals = preds["d"][0].flatten()
                    cart = self.converter.get_cartesian(s_vals, d_vals)  # (2, n)
                    xy = cart.T  # (n, 2)
                    ccma = CCMA(w_ma=5, w_cc=3)
                    smoothed_xy = ccma.filter(xy, mode="wrapping")  # (n, 2)
                    frenet = self.converter.get_frenet(
                        smoothed_xy[:, 0], smoothed_xy[:, 1]
                    )  # (2, n)
                    d_smoothed = frenet[1].reshape(-1, 1)
                    preds = dict(preds)
                    preds["d"] = (d_smoothed, preds["d"][1])
                except Exception as e:
                    self.get_logger().warn(f"[GP] CCMA smoothing failed: {e}")
            self.predictions[idx] = preds
        self.is_processing[idx] = False

    def publish_all(self):
        markers = MarkerArray()
        all_msg = OpponentTrajectories()
        all_msg.header.stamp = self.get_clock().now().to_msg()
        all_msg.header.frame_id = "map"
        
        last_id_viz = 0
        for i in range(self.num_opponents):
            pred = self.predictions[i]
            if not pred: continue
            
            traj_msg = OpponentTrajectory()
            traj_msg.header = all_msg.header
            traj_msg.lap_count = int(self.detection_matching[i]["cummulated_dist"] // self.max_s)
            
            s_fut = pred["s"]
            (d_p, d_s) = pred["d"]
            (v_p, v_s) = pred["vs"]
            r, g, b = id_to_rgb_color(i)
            
            for j in range(len(d_p)):
                if j % 5 != 0: continue # Subsample viz
                
                s_val = float(s_fut[j])
                d_val = float(d_p[j])
                
                # Limit to boundaries
                if self.boundaries is not None:
                    b_idx = min(int(s_val // self.wpnt_spacing), len(self.boundaries) - 1)
                    d_val = np.clip(d_val, self.boundaries[b_idx, 0], self.boundaries[b_idx, 1])

                cart = self.converter.get_cartesian(np.array([s_val]), np.array([d_val]))
                x, y = float(cart[0,0]), float(cart[1,0])
                
                wp = OppWpnt()
                wp.id = last_id_viz + j
                wp.s_m, wp.d_m, wp.x_m, wp.y_m = s_val, d_val, x, y
                wp.proj_vs_mps = float(v_p[j])
                wp.d_var, wp.vs_var = float(d_s[j]), float(v_s[j])
                traj_msg.oppwpnts.append(wp)
                
                m = Marker(); m.header.frame_id = "map"; m.header.stamp = all_msg.header.stamp
                m.id = last_id_viz + j; m.type = Marker.CYLINDER
                m.pose.position.x, m.pose.position.y = x, y
                m.pose.position.z = wp.proj_vs_mps / self.max_v / 2.0
                m.scale.x, m.scale.y, m.scale.z = 0.1, 0.1, float(wp.proj_vs_mps / self.max_v)
                m.color.a, m.color.r, m.color.g, m.color.b = 1.0, r, g, b
                m.pose.orientation.w = 1.0
                markers.markers.append(m)
                
            last_id_viz += len(d_p)
            all_msg.trajectories.append(traj_msg)
            
        if all_msg.trajectories:
            # Publish closest to main /opponent_trajectory for collision predictor compatibility
            dists = [self.detection_matching[k]["dist"] for k in range(len(all_msg.trajectories))]
            closest = np.argmin(dists)
            self.opp_trajectory_pub.publish(all_msg.trajectories[closest])
            self.opp_trajectories_pub.publish(all_msg)
            self.opp_trajectory_markers_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    with Pool(2) as pool:
        node = GPNode(pool)
        try: rclpy.spin(node)
        except KeyboardInterrupt: pass
        node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
