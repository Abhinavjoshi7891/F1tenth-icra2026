#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Bool, Float32, String
from nav_msgs.msg import Odometry
from f110_msgs.msg import ObstacleArray, WpntArray, Wpnt, Obstacle
from f110_msgs.msg import OpponentTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from frenet_conversion.frenet_converter import FrenetConverter
import time
import copy

class CollisionPredictor(Node):
    """
    Predict the region of collision between the ego car and the opponent.
    """
    def __init__(self):
        super().__init__('collision_predictor')

        # Parameters
        self.declare_parameter('n_time_steps', 200)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('save_distance_front', 0.6)
        self.declare_parameter('save_distance_back', 0.4)
        self.declare_parameter('max_v', 10.0)
        self.declare_parameter('min_v', 0.0)
        self.declare_parameter('max_a', 5.5)
        self.declare_parameter('min_a', 5.0)
        self.declare_parameter('max_expire_counter', 10)
        self.declare_parameter('speed_offset', 0.0)

        self.update_params()

        # Data storage
        self.opponent_pos = ObstacleArray()
        self.car_odom = Odometry()
        self.wpnts_opponent = []
        self.wpnts_updated = []
        self.state = "GB_TRACK"
        self.max_s_updated = 0.0
        self.max_s_opponent = 0.0
        self.opponent_lap_count = 0
        
        self.expire_counter = 0
        self.converter = None

        # Subscribers
        self.create_subscription(ObstacleArray, "/perception/obstacles", self.opponent_state_cb, 10)
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.odom_cb, 10)
        self.create_subscription(OpponentTrajectory, "/opponent_trajectory", self.opponent_trajectory_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints_updated", self.wpnts_updated_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_wpnts_cb, 10) # For converter init
        self.create_subscription(String, "/state_machine", self.state_cb, 10)

        # Publishers
        self.marker_pub_beginn = self.create_publisher(Marker, "/collision_predict/beginn", 10)
        self.marker_pub_end = self.create_publisher(Marker, "/collision_predict/end", 10)
        self.collision_obs_pub = self.create_publisher(ObstacleArray, "/collision_prediction/obstacles", 10)
        self.force_trailing_pub = self.create_publisher(Bool, "/collision_prediction/force_trailing", 10)

        # Timer (10Hz)
        self.create_timer(0.1, self.loop)
        
        # Metadata for viz
        self.marker_beginn = self.marker_init(a=0.5, r=0.63, g=0.13, b=0.94, mid=0)
        self.marker_end = self.marker_init(a=0.5, r=0.63, g=0.13, b=0.94, mid=1)

        self.get_logger().info("Collision Predictor Node Initialized")

    def update_params(self):
        self.time_steps = self.get_parameter('n_time_steps').value
        self.dt = self.get_parameter('dt').value
        self.save_distance_front = self.get_parameter('save_distance_front').value
        self.save_distance_back = self.get_parameter('save_distance_back').value
        self.max_v = self.get_parameter('max_v').value
        self.min_v = self.get_parameter('min_v').value
        self.max_a = self.get_parameter('max_a').value
        self.min_a = self.get_parameter('min_a').value
        self.max_expire_counter = self.get_parameter('max_expire_counter').value
        self.speed_offset = self.get_parameter('speed_offset').value

    def marker_init(self, a=1.0, r=1.0, g=0.0, b=0.0, mid=0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = mid
        marker.type = Marker.SPHERE
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = 0.4, 0.4, 0.4
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = a, r, g, b
        return marker

    def gb_wpnts_cb(self, msg: WpntArray):
        if self.converter is None and len(msg.wpnts) > 0:
            x = np.array([w.x_m for w in msg.wpnts])
            y = np.array([w.y_m for w in msg.wpnts])
            psi = np.array([w.psi_rad for w in msg.wpnts])
            self.get_logger().info("Initializing FrenetConverter...")
            self.converter = FrenetConverter(x, y, psi)
            self.get_logger().info("FrenetConverter initialized in CollisionPredictor")

    def opponent_state_cb(self, data: ObstacleArray):
        self.opponent_pos.header = data.header
        found_dynamic = False
        for obs in data.obstacles:
            if not obs.is_static:
                self.opponent_pos.obstacles = [obs]
                found_dynamic = True
                break
        if not found_dynamic:
            self.opponent_pos.obstacles = []

    def odom_cb(self, data: Odometry): self.car_odom = data
    def state_cb(self, data: String): self.state = data.data

    def opponent_trajectory_cb(self, data: OpponentTrajectory):
        self.wpnts_opponent = data.oppwpnts
        if len(self.wpnts_opponent) > 0:
            self.max_s_opponent = self.wpnts_opponent[-1].s_m
        self.opponent_lap_count = data.lap_count

    def wpnts_updated_cb(self, data: WpntArray):
        self.wpnts_updated = data.wpnts[:-1]
        if len(self.wpnts_updated) > 0:
            self.max_s_updated = self.wpnts_updated[-1].s_m

    def delete_all(self):
        empty_marker = Marker()
        empty_marker.header.frame_id = "map"
        empty_marker.header.stamp = self.get_clock().now().to_msg()
        empty_marker.action = Marker.DELETE
        
        empty_marker.id = 0
        self.marker_pub_beginn.publish(empty_marker)
        empty_marker.id = 1
        self.marker_pub_end.publish(empty_marker)

        empty_obs_arr = ObstacleArray()
        empty_obs_arr.header.frame_id = "map"
        empty_obs_arr.header.stamp = self.get_clock().now().to_msg()
        self.collision_obs_pub.publish(empty_obs_arr)

    def loop(self):
        if not self.wpnts_updated or not self.wpnts_opponent or not self.opponent_pos.obstacles:
            self.expire_counter += 1
            if self.expire_counter >= self.max_expire_counter:
                self.delete_all()
            return

        # Pre-calculation
        s_points_opp = np.array([wpnt.s_m for wpnt in self.wpnts_opponent])
        s_points_ego = np.array([wpnt.s_m for wpnt in self.wpnts_updated])

        ego_s = self.car_odom.pose.pose.position.x
        opp_s = self.opponent_pos.obstacles[0].s_center
        
        # Wrap around
        if ego_s > self.max_s_updated * 0.66 and opp_s < self.max_s_updated * 0.33:
            opp_s += self.max_s_updated
        
        # Trailing check
        curr_opp_d = self.opponent_pos.obstacles[0].d_center
        opp_idx = np.abs(s_points_opp - (opp_s % self.max_s_opponent)).argmin()
        opp_raceline_d = self.wpnts_opponent[opp_idx].d_m
        
        force_trailing = (abs(curr_opp_d - opp_raceline_d) > 0.25) or (self.opponent_lap_count < 1)
        self.force_trailing_pub.publish(Bool(data=force_trailing))

        curr_ego_v = self.car_odom.twist.twist.linear.x
        
        beginn, end = False, False
        beginn_s, end_s = 0.0, 0.0
        beginn_d, end_d = 0.0, 0.0

        sim_ego_s, sim_opp_s = ego_s, opp_s
        sim_ego_v = curr_ego_v

        for _ in range(self.time_steps):
            opp_idx = np.abs(s_points_opp - (sim_opp_s % self.max_s_opponent)).argmin()
            ego_idx = np.abs(s_points_ego - (sim_ego_s % self.max_s_updated)).argmin()
            
            opp_v = self.wpnts_opponent[opp_idx].proj_vs_mps
            ref_ego_v = self.wpnts_updated[ego_idx].vx_mps
            
            accel = self.min_a + (self.max_a - self.min_a) / max(self.max_v - self.min_v, 1e-6) * (self.max_v - sim_ego_v)

            if sim_ego_v < ref_ego_v:
                sim_ego_s += sim_ego_v * self.dt + 0.5 * accel * self.dt**2
                sim_ego_v += accel * self.dt
            else:
                sim_ego_s += ref_ego_v * self.dt
                
            sim_opp_s += opp_v * self.dt
            opp_d = self.wpnts_opponent[opp_idx].d_m
            
            # Collision detection
            dist = (sim_opp_s - sim_ego_s) % self.max_s_updated
            if not beginn and (dist < self.save_distance_front or abs(sim_opp_s - sim_ego_s) < self.save_distance_front):
                beginn_s, beginn_d = sim_ego_s, opp_d
                beginn = True
            elif beginn and not end and (sim_ego_s - sim_opp_s) > self.save_distance_back:
                end_s, end_d = sim_ego_s, opp_d
                end = True
                break

        if beginn:
            if not end:
                end_s, end_d = sim_opp_s, opp_d
            
            # ROC construction
            if end:
                d_right = min(beginn_d, end_d) - 0.25
                d_left = max(beginn_d, end_d) + 0.25
            else:
                d_right, d_left = -10.0, 10.0

            roc = Obstacle(id=0, d_left=float(d_left), d_right=float(d_right), 
                           d_center=float((d_right + d_left)/2.0),
                           s_start=float(beginn_s), s_end=float(end_s), 
                           s_center=float((beginn_s + end_s)/2.0),
                           vs=0.0, vd=0.0, is_static=False)

            roc_arr = ObstacleArray()
            roc_arr.header.stamp = self.get_clock().now().to_msg()
            roc_arr.header.frame_id = "map"
            roc_arr.obstacles = [roc]
            self.collision_obs_pub.publish(roc_arr)
            self.expire_counter = 0

            # Viz
            if self.converter:
                pos_b = self.converter.get_cartesian(np.array([beginn_s % self.max_s_updated]), np.array([beginn_d]))
                self.marker_beginn.pose.position.x, self.marker_beginn.pose.position.y = float(pos_b[0, 0]), float(pos_b[1, 0])
                self.marker_beginn.header.stamp = self.get_clock().now().to_msg()
                self.marker_pub_beginn.publish(self.marker_beginn)

                pos_e = self.converter.get_cartesian(np.array([end_s % self.max_s_updated]), np.array([end_d]))
                self.marker_end.pose.position.x, self.marker_end.pose.position.y = float(pos_e[0, 0]), float(pos_e[1, 0])
                self.marker_end.header.stamp = self.get_clock().now().to_msg()
                self.marker_pub_end.publish(self.marker_end)
        else:
            self.expire_counter += 1
            if self.expire_counter >= self.max_expire_counter:
                self.delete_all()

def main(args=None):
    rclpy.init(args=args)
    node = CollisionPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
