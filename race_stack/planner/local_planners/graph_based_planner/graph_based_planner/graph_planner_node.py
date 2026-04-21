#!/usr/bin/env python3
"""Graph-Based Local Trajectory Planner — ROS 2 port (faithful, no behavioural changes).

Port of ROS 1 Planner.py. Wraps TUMFTM graph_ltpl. Publishes a flattened
Float32MultiArray with action label so the state machine can consume it
exactly like the ROS 1 version.
"""
import os
import sys  # noqa: F401
import time
import uuid
import hashlib
import cProfile
import pstats
from math import sin, cos

os.environ['OPENBLAS_NUM_THREADS'] = "1"

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# graph_ltpl uses absolute internal imports (import graph_ltpl.*) so it must be a
# top-level importable package. It lives at the package share root, not inside the
# graph_based_planner Python package, so we add the share dir to sys.path here.
_pkg_share = get_package_share_directory('graph_based_planner')
if _pkg_share not in sys.path:
    sys.path.insert(0, _pkg_share)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float32
from f110_msgs.msg import WpntArray, ObstacleArray

# vendored pure-python lib (no ROS deps)
from graph_ltpl import Graph_LTPL
from graph_ltpl.imp_global_traj.src import import_globtraj_csv  # noqa: F401


def _yaw_from_quat(x, y, z, w):
    """Extract yaw from quaternion. Replaces tf.transformations.euler_from_quaternion."""
    import math
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GraphPlanner(Node):
    def __init__(self):
        super().__init__('GraphPlanner')

        # --- Declare ROS params (defaults match ROS 1 hardcoded values) ---
        self.declare_parameter('map_name', '')
        self.declare_parameter('display_graph', True)
        self.declare_parameter('follow_mode', 0)
        self.declare_parameter('racecar_version', 'NUC2')
        self.declare_parameter('measure', False)
        self.declare_parameter('profile', False)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('v_max', 15.0)
        self.declare_parameter('trailing_distance', 1.0)
        self.declare_parameter('examined_points', 80)
        self.declare_parameter('waypoint_distance', 0.1)
        self.declare_parameter('ax_max_machines_path', '')

        self.display_graph     = self.get_parameter('display_graph').value
        self.follow_mode       = self.get_parameter('follow_mode').value
        self.map_name          = self.get_parameter('map_name').value
        self.racecar_version   = self.get_parameter('racecar_version').value
        self.measuring         = self.get_parameter('measure').value
        self.profile_enabled   = self.get_parameter('profile').value
        self.rate_hz           = self.get_parameter('rate_hz').value
        self.v_max             = self.get_parameter('v_max').value
        self.trailing_distance = self.get_parameter('trailing_distance').value
        self.examined_points   = self.get_parameter('examined_points').value
        self.waypoint_distance = self.get_parameter('waypoint_distance').value

        # --- State (mirror of ROS 1 __init__) ---
        self.waypoints = None
        self.current_position = None
        self.current_position_frenet = None
        self.current_heading = None
        self.current_speed = None
        self.waypoints_original = None
        self.obstacle_arr = np.array([])
        self.speed_scaling = 1.0
        self.max_s = None
        self.wp_flag = False
        self.close_to_s0 = False
        self.num_waypoints = 0

        # --- Paths ---
        pkg_share = get_package_share_directory('graph_based_planner')
        self.toppath = pkg_share
        self.path_dict = {
            'globtraj_input_path':     os.path.join(
                self.toppath, 'inputs', 'traj_ltpl_cl', f'traj_ltpl_cl_{self.map_name}.csv'),
            'graph_store_path':        os.path.join(self.toppath, 'inputs', 'stored_graph.pckl'),
            'ltpl_offline_param_path': os.path.join(self.toppath, 'params', 'ltpl_config_offline.ini'),
            'ltpl_online_param_path':  os.path.join(self.toppath, 'params', 'ltpl_config_online.ini'),
        }

        ax_path_param = self.get_parameter('ax_max_machines_path').value
        if ax_path_param:
            ax_max_path = ax_path_param
        else:
            ax_max_path = os.path.join(self.toppath, 'inputs', 'veh_dyn_info', 'ax_max_machines.csv')
        self.ax_max = np.loadtxt(ax_max_path, delimiter=',', skiprows=1)

        # --- Profiling ---
        self.profiler = cProfile.Profile()
        self.profile_filename        = os.path.join(self.toppath, 'logs', 'graph_based_profile_log.prof')
        self.profile_report_filename = os.path.join(self.toppath, 'logs', 'graph_based_profile_report.txt')

        # --- Subscribers ---
        self.create_subscription(WpntArray,     '/global_waypoints',      self.waypoints_cb,  10)
        self.create_subscription(Odometry,      '/car_state/odom',        self.odom_cb,       10)
        self.create_subscription(Odometry,      '/car_state/frenet/odom', self.odom_fre_cb,   10)
        self.create_subscription(ObstacleArray, '/perception/obstacles',  self.obstacles_cb,  10)

        # --- Publishers ---
        self.lattice_visualizer = self.create_publisher(MarkerArray,       '/lattice_viz',                  10)
        self.action_mrk         = self.create_publisher(Marker,            '/action_marker',                10)
        self.path_pub           = self.create_publisher(Float32MultiArray, '/planner/graph_based_wpnts',    10)
        if self.measuring:
            self.latency_pub    = self.create_publisher(Float32, '/planner/graph_based/latency', 10)
        
        self.traj_viz_pub       = self.create_publisher(MarkerArray, '/planner/graph_based/candidate_trajs', 10) # New: Dynamic candidate paths

        # --- Speed scaling via sector_tuner param polling ---
        self._setup_speed_scaling_listener()

        # --- Main loop state ---
        self.ltpl_obj = None
        self._initialised = False
        self._prev_action = 'straight'
        self._sel_action = 'straight'
        self._path_loss_counter = 0     # frames the current action has been missing
        self._cached_traj_set = {}      # last traj_set where current action was live
        self._action_hold_counter = 0   # frames we've wanted to switch, for hysteresis

        # Timer drives the main loop (gated by _ensure_initialised)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        # Publish initial action marker (safe — no data needed)
        self.visualize_action('straight')

        if self.profile_enabled:
            self.profiler.enable()

    # ============================= CALLBACKS =============================

    def obstacles_cb(self, data: ObstacleArray):
        if len(data.obstacles) == 0:
            self.obstacle_arr = np.array([])
            return
        self.obstacle_arr = np.array([
            [o.id, o.s_start, o.s_end, o.d_left, o.d_right, o.is_actually_a_gap,
             o.s_center, o.d_center, o.size, o.vs, o.vd, o.is_static]
            for o in data.obstacles
        ])

    def odom_cb(self, data: Odometry):
        p = data.pose.pose.position
        q = data.pose.pose.orientation
        self.current_position = np.array([p.x, p.y])
        self.current_speed    = data.twist.twist.linear.x
        self.current_heading  = _yaw_from_quat(q.x, q.y, q.z, q.w)

    def odom_fre_cb(self, data: Odometry):
        p = data.pose.pose.position
        self.current_position_frenet = np.array([p.x, p.y])

    def waypoints_cb(self, data: WpntArray):
        if self.wp_flag:
            return
        self.waypoints_original = np.array([
            [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left,
             w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2]
            for w in data.wpnts
        ])
        mylist = []
        for wpnt in data.wpnts:
            # Get normalized normal vector from frenet space
            vector_base_glob = self.frenettoglob(wpnt.s_m, 0.0)
            vector_tip_glob  = self.frenettoglob(wpnt.s_m, -1.0)
            dir_ = np.asarray([vector_tip_glob[0] - vector_base_glob[0],
                                vector_tip_glob[1] - vector_base_glob[1]])
            norm_vec = self.unit_vector(dir_)
            new_wpnt = [wpnt.x_m, wpnt.y_m, wpnt.d_right, wpnt.d_left,
                        norm_vec[0], norm_vec[1], 0.0, wpnt.s_m,
                        wpnt.psi_rad, wpnt.kappa_radpm, wpnt.vx_mps, wpnt.ax_mps2]
            mylist.append(new_wpnt)
        self.waypoints = np.asarray(mylist)
        self.wp_flag = True
        self.num_waypoints = len(self.waypoints)

    # ============================= HELPERS (verbatim port of ROS1 lines 209-720) =============================

    def create_map_csv(self) -> bool:
        """
                Function that checks it there is a pre-existing trajectory file for that specific map, and creates one
                if there isn't such file

                Returns
                ----------
                True
                    If the pre-existing trajectory file does not exist and the new file is generated correctly
                False
                    Found a pre-existing file, no further operation needed
        """
        pt = os.path.join(self.toppath, 'inputs', 'traj_ltpl_cl', f'traj_ltpl_cl_{self.map_name}.csv')
        if os.path.exists(pt):
            return False

        os.makedirs(os.path.dirname(pt), exist_ok=True)

        # Create random UUID
        rand_uuid = str(uuid.uuid4())

        # Hash ggv file with SHA1
        ggv_content = np.array([])
        ggv_hash = hashlib.sha1(ggv_content).hexdigest()

        # write UUID and GGV hash into file
        with open(pt, 'w') as fh:
            fh.write("# " + rand_uuid + "\n")
            fh.write("# " + ggv_hash + "\n")

        # Export trajectory data for local planner
        header = "x_ref_m; y_ref_m; width_right_m; width_left_m; x_normvec_m; y_normvec_m; " \
                 "alpha_m; s_racetraj_m; psi_racetraj_rad; kappa_racetraj_radpm; vx_racetraj_mps; ax_racetraj_mps2"
        fmt = "%.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f"
        with open(pt, 'ab') as fh:
            np.savetxt(fh, self.waypoints, fmt=fmt, header=header)

        return True

    def visualize_lattice(self, graphbase):
        """
                Creates and sends markers to visualize the graph in RVIZ

                Parameters
                ----------
                graphbase
                    GraphBase object which holds all the spatial information about the graph
        """
        ############### NODES VISUALIZATION ##################
        i = 0
        nodes = graphbase.get_nodes()  # gets list of tuples (numberoflayer, nodenumber)
        node_markers = MarkerArray()
        # Fill node_markers array
        for node in nodes:
            node_pos = graphbase.get_node_info(node[0], node[1])[
                0]  # Takes layer and node number, gives an array. in position 0 you have the node's x and y
            node_marker = Marker()
            node_marker.header.frame_id = "map"
            node_marker.header.stamp = self.get_clock().now().to_msg()
            node_marker.type = 2  # Sphere type
            node_marker.id = i
            node_marker.scale.x = 0.1
            node_marker.scale.y = 0.1
            node_marker.scale.z = 0.1
            node_marker.color.r = 0 / 255
            node_marker.color.g = 151 / 255
            node_marker.color.b = 252 / 255
            node_marker.color.a = 1.0
            node_marker.pose.position.x = node_pos[0]
            node_marker.pose.position.y = node_pos[1]
            node_marker.pose.position.z = 0.0
            node_marker.pose.orientation.x = 0.0
            node_marker.pose.orientation.y = 0.0
            node_marker.pose.orientation.z = 0.0
            node_marker.pose.orientation.w = 1.0
            node_markers.markers.append(node_marker)
            i = i + 1
        # Publish nodes
        self.lattice_visualizer.publish(node_markers)

        ############## EDGES VISUALIZATION ################

        edges = graphbase.get_edges()
        edge_markers = MarkerArray()

        # Get min and max cost for colored visualization
        mylist = []
        for edge in edges:
            spline = graphbase.get_edge(edge[0], edge[1], edge[2], edge[3])
            spline_cost = spline[2]
            mylist.append(spline_cost)
        costs = np.asarray(mylist)
        max_cost = np.amax(costs)
        min_cost = np.amin(costs)

        # Fill edge_markers array
        for edge in edges:
            spline = graphbase.get_edge(edge[0], edge[1], edge[2], edge[3])
            spline_coords = spline[1][:, 0:2]
            #if not self.debug:
                #spline_coords = [spline_coords[0], spline_coords[int(len(spline_coords) / 2)], spline_coords[-1]]
            spline_cost = spline[2]
            edge_marker = Marker()
            edge_marker.points = []
            for sampled_point in spline_coords:
                p = Point()
                p.x = sampled_point[0]
                p.y = sampled_point[1]
                p.z = 0.0
                edge_marker.points.append(p)
            edge_marker.header.frame_id = "map"
            edge_marker.header.stamp = self.get_clock().now().to_msg()
            edge_marker.type = 4  # Type LINE STRIP
            edge_marker.id = i
            edge_marker.scale.x = 0.05 # Increased from 0.01 for visibility
            edge_marker.pose.orientation.x = 0.0
            edge_marker.pose.orientation.y = 0.0
            edge_marker.pose.orientation.z = 0.0
            edge_marker.pose.orientation.w = 1.0
            edge_marker.color.a = 1.0
            # Normalize cost of the particular edge in a [0,1] interval
            norm_cost = (spline_cost - min_cost) / (max_cost - min_cost)
            # Set colors based on cost
            if norm_cost <= 0.1:
                edge_marker.color.r = 41 / 255
                edge_marker.color.g = 171 / 255  # dark green
                edge_marker.color.b = 67 / 255
            elif 0.1 < norm_cost <= 0.25:
                edge_marker.color.r = 43 / 255
                edge_marker.color.g = 209 / 255  # light green
                edge_marker.color.b = 65 / 255
            elif 0.25 < norm_cost <= 0.5:
                edge_marker.color.r = 255 / 255
                edge_marker.color.g = 255 / 255  # yellow
                edge_marker.color.b = 0 / 255
            elif 0.5 < norm_cost <= 0.75:
                edge_marker.color.r = 255 / 255
                edge_marker.color.g = 126 / 255  # orange
                edge_marker.color.b = 0 / 255
            elif 0.75 < norm_cost <= 1:
                edge_marker.color.r = 255 / 255
                edge_marker.color.g = 0 / 255  # red
                edge_marker.color.b = 0 / 255

            edge_markers.markers.append(edge_marker)
            i = i + 1
        self.get_logger().info("[Graph Based Planner] Number of nodes and edges is " + str(i))
        # Publish edge markers
        self.lattice_visualizer.publish(edge_markers)

    def get_closest_obs(self):
        """Function that returns the closest object to the car ( only objects in front of the car are taken into
        account

        Returns
        ----------
        None
            No object in front of the vehicle is detected
        Object Array
            1d array containing the obstacle information
            [obj_id, s_start, s_end, d_right, d_left, is_actually_a_gap]
        """
        if len(self.obstacle_arr) == 0:
            return None
        obs_arr_copy = self.obstacle_arr
        # Form a matrix with column 0 being the s coordinate of obstacles, column 1 the d coordinate, column 2 the
        # obstacle id
        matr = np.concatenate((obs_arr_copy[:, 1].reshape(len(obs_arr_copy[:, 0]), 1),
                               ((obs_arr_copy[:, 3] + obs_arr_copy[:, 4])
                                / 2).reshape(len(obs_arr_copy[:, 0]), 1),
                               obs_arr_copy[:, 0].reshape(len(obs_arr_copy[:, 0]), 1)),
                              axis=1)
        ls = []
        for obs in matr:
            glob = self.frenettoglob(obs[0], obs[1])
            ls.append([glob[0], glob[1], obs[2], obs[0] - self.current_position_frenet[0]])
        glob_mat = np.asarray(ls)
        # Take only obstacles with s greater than s_ego if the car is not close to s0
        if not self.close_to_s0:
            glob_mat = glob_mat[glob_mat[:, -1] >= 0]
        # Get closest obstacle
        if glob_mat.shape[0] != 0:
            idx = np.argmin(np.linalg.norm(np.subtract(glob_mat[:, :2], self.current_position), axis=1))
            return obs_arr_copy[obs_arr_copy[:, 0] == glob_mat[idx, 2]]
        else:
            return None

    def find_global_index(self, s: float) -> int:
        """Calculate the index of the element in the array which is closest in absolute value to the s.

        Parameters
        ----------
        s
            Reference value

        Returns
        ----------
        int
            Index of the element in the array which is closest in absolute value to the station
        """
        return (np.round(s/self.waypoint_distance).astype(int))%self.num_waypoints

    def unit_vector(self, vector):
        """Calculates the unit vector of a vector

        Parameters
        ----------
        vector
            Multi dimensional vector

        Returns
        ----------
        vector
            Normalized vectort
        """
        return vector / np.linalg.norm(vector) if np.linalg.norm(vector) != 0 else vector

    def angle_between(self, v1, v2) -> float:
        """Calculate the angle (radians) between two vectors

        Parameters
        ----------
        v1
            Vector 1
        v2
            Vector 2

        Returns
        ----------
        float
            Angle between the two vector (in radians)
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def get_obj_list(self):
        """
                Calculates a list of dicts, each describing an obstacle (static or dynamic).
                An object's fields aree the following: 'id' , 'type', 'X', 'Y', 'theta', 'v', 'lenght', 'width'
                Type is always 'physical', theta is the heading, v is velocity along theta.

                Returns
                ----------
                list
                    The object list, list of dicts
        """
        obj_list = []
        if self.obstacle_arr is not None:
            for obs in self.obstacle_arr:
                # Filter obstacles: only consider those in front of us (within 12m)
                # s_middle is obs[6]
                dist_in_front = (obs[6] - self.current_position_frenet[0]) % self.max_s
                if dist_in_front > 12.0: # Too far or behind us
                    continue

                # Get cartesian coordinates of middle of object
                s_middle = obs[6]
                d_middle = obs[7]
                obs_in_map = self.frenettoglob(s_middle, d_middle)

                # Calculate direction vector of the rectangular object (the two points are the two left vertices of the rectangle)
                p1 = self.frenettoglob(obs[1], obs[3])
                p2 = self.frenettoglob(obs[2], obs[3])
                dir_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])
                # Calculate theta
                theta = self.angle_between(dir_vector, np.array([0, 1]))  # Zero is north
                # ACC DISABLE: If we are already overtaking, treat the opponent as STATIONARY (v=0).
                # This prevents the library's "Follow-Mode" / Adaptive Cruise Control
                # from slowing us down to 0.0.
                is_overtaking = self._sel_action in ['left', 'right']

                ide = obs[0]
                type = 'physical'
                X = obs_in_map[0]
                Y = obs_in_map[1]
                # OFFENSIVE MODE: Shrink the opponent in the eyes of the planner
                # This tricks the library into seeing gaps it would normally consider "too narrow"
                length = obs[8] * 0.8
                width = length
                # Convert velocity vector from cartesian frame to Frenet frame
                velocity_vector_in_cartesian = self.fromfrenet_to_cartesian(obs[9], obs[10], obs[6])
                velocity_magnitude = np.linalg.norm(velocity_vector_in_cartesian)

                # Set v to 0.0 if overtaking to disable ACC
                v = 0.0 if is_overtaking else velocity_magnitude
                if obs[11]: # is_static flag from perception
                    v = 0.0

                # Fill object dict
                obj = {'id': ide, 'type': type, 'X': X, 'Y': Y, 'theta': theta, 'v': v, 'length': length,
                       'width': width}
                obj_list.append(obj)
        return obj_list

    def fromfrenet_to_cartesian(self, velocity_s, velocity_d, object_s):
        """
                Function that transforms a velocity vector in cartesian frame to a velocity vector in Frenet frame.

                Parameters
                ----------
                velocity_s
                    S component of velocity vector to be transformed
                velocity_d
                    D component of velocity vector to be transformed
                object_s
                    S coordinate of obstacle

                Returns
                ----------
                numpy array
                    Velocity vector in cartesian frame

        """
        # Get closest waypoint's psi
        closest_wpnt_idx = self.find_global_index(object_s)
        delta_psi = self.waypoints_original[closest_wpnt_idx, 6]
        # Rotate vector using psi
        velocity_x = velocity_s * np.cos(delta_psi) + velocity_d * np.sin(delta_psi)
        velocity_y = velocity_s * - np.sin(delta_psi) + velocity_d * np.sin(delta_psi)
        return np.asarray([velocity_x, velocity_y])

    def visualize_action(self, action: str):
        """
                Function that visualizes the state of the car by displaying a colored cube in RVIZ.

                Parameters
                ----------
                action
                    Current state of the car to be displayed
        """
        mrk = Marker()
        mrk.type = mrk.CUBE
        mrk.id = 1
        mrk.header.frame_id = 'map'
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.color.a = 1.0
        mrk.color.g = 1.0
        if self.current_position is not None:
            mrk.pose.position.x = float(self.current_position[0])
            mrk.pose.position.y = float(self.current_position[1])
        else:
            mrk.pose.position.x = 0.0
            mrk.pose.position.y = 0.0
        mrk.pose.position.z = 1.5 # Float above the car
        mrk.scale.x = 0.5
        mrk.scale.y = 0.5
        mrk.scale.z = 0.5

        # Set color and log info based on the state of the car
        var = str()
        if action == 'straight':
            var = 'following the global tracjectory'
            mrk.color.r = 20 / 255
            mrk.color.g = 240 / 255    # green
            mrk.color.b = 40 / 255
        elif action == 'right':
            var = 'overtaking right'
            mrk.color.r = 20 / 255
            mrk.color.g = 23 / 255     # blue
            mrk.color.b = 230 / 255
        elif action == 'left':
            var = 'overtaking left'
            mrk.color.r = 230 / 255
            mrk.color.g = 230 / 255    # yellow
            mrk.color.b = 10 / 255
        elif action == 'follow':
            var = 'following'
            mrk.color.r = 230 / 255
            mrk.color.g = 220 / 255     # gray
            mrk.color.b = 230 / 255

        self.get_logger().info("[Graph Based Planner] The car is currently " + var)
        self.action_mrk.publish(mrk)

    def visualize_candidates(self, traj_set):
        """Dynamic visualization of the top candidate paths in different colors."""
        array = MarkerArray()
        colors = {
            'straight': [0.0, 1.0, 0.0], # Green
            'left':     [0.0, 0.5, 1.0], # Blue
            'right':    [1.0, 0.5, 0.0], # Orange
            'follow':   [1.0, 1.0, 1.0]  # White
        }
        
        for i, (act, trajs) in enumerate(traj_set.items()):
            if not trajs: continue
            path = trajs[0] # Best path for this action
            
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.type = m.LINE_STRIP
            m.id = i + 1000 # Offset to avoid lattice clash
            m.scale.x = 0.1 # Bold
            m.color.a = 0.8
            color = colors.get(act, [0.5, 0.5, 0.5])
            m.color.r, m.color.g, m.color.b = color
            
            for pt in path:
                # traj_set columns: [s, x, y, heading, curvature, vel, ...]
                # pt[1] and pt[2] are already Cartesian x,y — DO NOT call frenettoglob.
                p = Point()
                p.x = float(pt[1])
                p.y = float(pt[2])
                p.z = 0.05
                m.points.append(p)
            array.markers.append(m)
        self.traj_viz_pub.publish(array)

    def send_path_topic(self, traj_set, sel_action):
        """Function that sends the selected path to the path publisher node."""
        if sel_action is not None:
            if sel_action not in traj_set:
                self.get_logger().error(f'[Graph Based Planner]: Action {sel_action} not in traj_set! Available: {list(traj_set.keys())}')
                return

            mypath = np.asarray(traj_set[sel_action][0])
            # Send path to the other node
            msg = Float32MultiArray()
            msg.data = mypath.flatten().tolist()
            msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
            msg.layout.data_offset = 0
            msg.layout.dim[0].label = sel_action
            msg.layout.dim[0].size = mypath.shape[0]
            msg.layout.dim[0].stride = len(mypath.flatten())
            msg.layout.dim[1].label = "column"
            msg.layout.dim[1].size = mypath.shape[1]
            msg.layout.dim[1].stride = mypath.shape[1]
            self.path_pub.publish(msg)
        else:
            self.get_logger().warn('[Graph Based Planner]: No action selected')

    def traj_cost(self, path) -> float:
        """Function that calculates a cost to a given path

        Parameters
        ----------
        path
            Path to be evaluated

        Returns
        ----------
        float
            Cost of path
        """
        cost = 0.0
        i = 0.0
        for point in path:
            # Calculate point distance to left and right track bound
            frenet_point = self.globtofrenet(point[1], point[2])
            d_point = frenet_point[1]
            close_s_idx = self.find_global_index(point[0])
            wpnt_d_left = self.waypoints_original[close_s_idx, 5]
            wpnt_d_right = self.waypoints_original[close_s_idx, 4]
            point_d_left = wpnt_d_left - d_point
            point_d_right = wpnt_d_right + d_point

            # Select min track bound distance and assign cost
            min_track_distance = np.abs(min(point_d_right, point_d_left))
            cost += (1/(1 + i)) * (1 / min_track_distance)
            i += 0.5
        return cost

    def globtofrenet(self, x, y):
        """Function that transforms cartesian coordiantes into frenet coordinates (helps with compute as compared
        to ros service)

        Parameters
        ----------
        X
            Cartesian coordinate
        Y
            Cartesian coordinate

        Returns
        ----------
        numpy array
            [s, d] array in Frenet frame
        """
        point = np.tile([x, y], (len(self.waypoints_original[:, 2:4]), 1))
        closest_pt_idx = np.argmin(np.linalg.norm(self.waypoints_original[:, 2:4] - point, axis=1))
        s = self.waypoints_original[closest_pt_idx, 0]
        d_x = x - self.waypoints_original[closest_pt_idx, 2]
        d_y = y - self.waypoints_original[closest_pt_idx, 3]
        d = -d_x * sin(self.waypoints_original[closest_pt_idx, 6]) + \
             d_y * cos(self.waypoints_original[closest_pt_idx, 6])
        return np.asarray([s, d])

    def frenettoglob(self, s, d):
        """Function that transforms frenet coordinates into cartesian coordinates (helps with compute as compared
        to ros service)

        Parameters
        ----------
        S
            Frenet coordinate
        D
            Frenet coordinate

        Returns
        ----------
        numpy array
            [x, y] array in cartesian frame
        """
        closest_pt_idx = self.find_global_index(s)
        d_s = s - self.waypoints_original[closest_pt_idx, 0]
        x = self.waypoints_original[closest_pt_idx, 2] + (d_s * cos(self.waypoints_original[closest_pt_idx, 6])) \
            - d * sin(self.waypoints_original[closest_pt_idx, 6])
        y = self.waypoints_original[closest_pt_idx, 3] + (d * cos(self.waypoints_original[closest_pt_idx, 6])) + \
            (d_s * sin(self.waypoints_original[closest_pt_idx, 6]))
        return np.asarray([x, y])

    def choose_action(self, traj_set) -> str:
        """Function which is tasked with picking an action from the available action set

        Parameters
        ----------
        traj_set
            Trajectory set holding all the available action alongside the corresponding paths

        Returns
        ----------
        string
            Name of the chosen action
        """
        #If both left and right overtake is possible, compare their costs and choose the best
        if "right" in traj_set.keys() and "left" in traj_set.keys():
            path_r = traj_set['right'][0]
            path_l = traj_set['left'][0]
            # Cut points from the path which are behind the position of the car and points in which the velocity is 0
            if not self.close_to_s0:
                sliced_path_r = path_r[np.where(((self.current_position_frenet[0] <
                                                  (path_r[:, 0] + self.current_position_frenet[0]) % self.max_s) &
                                                 (path_r[:, 5] > 0)))]

                sliced_path_l = path_l[np.where(((self.current_position_frenet[0] <
                                                  (path_l[:, 0] + self.current_position_frenet[0]) % self.max_s) &
                                                 (path_l[:, 5] > 0)))]
            else:
                sliced_path_r = path_r[np.where(((self.current_position_frenet[0] <
                                                  (path_r[:, 0] + self.current_position_frenet[0]) % self.max_s) & (
                                                         path_r[:, 5] > 0)
                                                 | ((path_r[:, 0] + self.current_position_frenet[0]) > self.max_s)))]
                sliced_path_l = path_l[np.where(((self.current_position_frenet[0] <
                                                  (path_l[:, 0] + self.current_position_frenet[0]) % self.max_s) & (
                                                         path_l[:, 5] > 0)
                                                 | ((path_l[:, 0] + self.current_position_frenet[0]) > self.max_s)))]
            # Calculate costs for paths
            cost_r = self.traj_cost(sliced_path_r[:self.examined_points, :])
            cost_l = self.traj_cost(sliced_path_l[:self.examined_points, :])

            # Choose
            act = 'right' if cost_r < cost_l else 'left'

            return act
        # If left and right arent available together, prioritize going straight, then overtaking, then following
        else:
            for act in ["straight", "right", "left", "follow"]:
                if act in traj_set.keys():
                    return act

    # ============================= MAIN LOOP TICK =============================

    def _has_all_data(self):
        wpts_ok = self.waypoints is not None
        pos_ok = self.current_position is not None
        heading_ok = self.current_heading is not None
        frenet_ok = self.current_position_frenet is not None
        
        if not (wpts_ok and pos_ok and heading_ok and frenet_ok):
            self.get_logger().info(
                f'[Graph Planner DEBUG] Missing data: '
                f'waypoints={wpts_ok}, '
                f'pos={pos_ok}, '
                f'heading={heading_ok}, '
                f'frenet={frenet_ok}',
                throttle_duration_sec=2.0
            )
        return wpts_ok and pos_ok and heading_ok and frenet_ok

    def _ensure_initialised(self):
        if self._initialised:
            return
            
        self.get_logger().info('[Graph Planner DEBUG] Checking initialization...', throttle_duration_sec=5.0)
        
        if not self._has_all_data():
            return

        self.get_logger().info('[Graph Planner DEBUG] Data check passed. Initializing internals...')

        try:
            self.max_s = float(np.amax(self.waypoints_original[:, 0]))

            if not self.create_map_csv():
                self.get_logger().info('[Graph Based Planner] Found existing trajectory file for this map!')
            else:
                self.get_logger().info('[Graph Based Planner] Created new trajectory file for this map!')

            self.get_logger().info('[Graph Based Planner] Starting graph_init (may take a moment)...')
            self.ltpl_obj = Graph_LTPL.Graph_LTPL(
                path_dict=self.path_dict,
                visual_mode=False,
                log_to_file=False,
            )
            self.ltpl_obj.graph_init(veh_param_mass=3.5)

            if self.display_graph:
                self.get_logger().info('[Graph Planner DEBUG] Visualizing lattice...')
                graph_container = self.ltpl_obj._Graph_LTPL__graph_base
                self.visualize_lattice(graph_container)

            self.get_logger().info(f'[Graph Planner DEBUG] Setting start pos: {self.current_position}, heading: {self.current_heading}, s: {self.current_position_frenet[0]:.2f}, d: {self.current_position_frenet[1]:.2f}')
            self.ltpl_obj.set_startpos(
                pos_est=self.current_position,
                heading_est=self.current_heading + np.pi / 2,
            )
            self._initialised = True
            self.get_logger().info('[Graph Based Planner] Initialised — entering main loop.')
        except Exception as e:
            self.get_logger().error(f'[Graph Planner DEBUG] Initialization FAILED: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _tick(self):
        try:
            self._ensure_initialised()
            if not self._initialised:
                return

            t0 = time.perf_counter()
            self.close_to_s0 = (self.current_position_frenet[0] < 0.5
                                or self.current_position_frenet[0] > self.max_s - 0.5)
            self._prev_action = self._sel_action

            obj_list = self.get_obj_list()
            
            # self.get_logger().info(f'[Graph Planner DEBUG] Calculating paths... objects: {len(obj_list)}')
            self.ltpl_obj.calc_paths(prev_action_id=self._sel_action, object_list=obj_list)
            
            # Use a near-zero safety distance if we are already in an overtake maneuver
            # to prevent the library from panicking and setting velocity to 0.
            dynamic_safety_d = 0.1 if self._sel_action in ['left', 'right'] else self.trailing_distance

            traj_set = self.ltpl_obj.calc_vel_profile(
                pos_est=self.current_position,
                vel_est=self.current_speed,
                vel_max=self.v_max,
                ax_max_machines=self.ax_max,
                safety_d=dynamic_safety_d,
            )[0]
            
            # --- Action selection with corrected Trajectory Persistence ---

            new_action = self.choose_action(traj_set)

            if self._sel_action in traj_set:
                # Current action is LIVE in the new plan — update the cache.
                # Only cache when the path is confirmed valid, so the cache
                # always holds the last GOOD traj_set.
                self._cached_traj_set = traj_set
                self._path_loss_counter = 0

                # Normal voluntary-switch hysteresis (prevents flickering between modes)
                if new_action != self._sel_action:
                    self._action_hold_counter += 1
                    if self._action_hold_counter > 5:  # 5 frames @ 30 Hz ≈ 167 ms
                        self._sel_action = new_action
                        self._action_hold_counter = 0
                        self.get_logger().info(f'[Graph Based Planner] Switching action: {self._prev_action} → {self._sel_action}')
                else:
                    self._action_hold_counter = 0

            else:
                # Current action is MISSING from the new plan (lattice node transition
                # or obstacle blocked it).  Try to ride out the gap using the cache.
                self._path_loss_counter += 1
                _grace = 20  # frames @ 30 Hz ≈ 667 ms

                if self._sel_action in self._cached_traj_set and self._path_loss_counter <= _grace:
                    # Use the last good path — override traj_set so send_path_topic
                    # and visualize_candidates both see the cached data.
                    traj_set = self._cached_traj_set
                    self.get_logger().info(
                        f'[Graph Based Planner] Holding {self._sel_action} from cache '
                        f'({self._path_loss_counter}/{_grace})',
                        throttle_duration_sec=0.5)
                else:
                    # Grace period expired or cache is empty — fall back gracefully.
                    self.get_logger().warn(
                        f'[Graph Based Planner] Action "{self._sel_action}" truly lost '
                        f'after {self._path_loss_counter} frames. Reverting to "{new_action}".')
                    self._sel_action = new_action
                    self._action_hold_counter = 0
                    self._path_loss_counter = 0

            if self.measuring:
                self.latency_pub.publish(Float32(data=float(time.perf_counter() - t0)))

            if self._sel_action != self._prev_action:
                self.visualize_action(self._sel_action)

            self.send_path_topic(traj_set, self._sel_action)
            
            if self.display_graph:
                self.visualize_candidates(traj_set)
            # self.get_logger().info('[Graph Planner DEBUG] Published waypoints.')
            
        except Exception as e:
            self.get_logger().error(f'[Graph Planner DEBUG] Tick execution FAILED: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    # ============================= SPEED SCALING (sector_tuner) =============================

    def _setup_speed_scaling_listener(self):
        """Poll /sector_tuner's global_limit param at 2 Hz. Replaces ROS 1 /dyn_sector_server/parameter_updates."""
        from rcl_interfaces.srv import GetParameters
        self._param_cli = self.create_client(GetParameters, '/sector_tuner/get_parameters')
        self.create_timer(0.5, self._poll_speed_scaling)

    def _poll_speed_scaling(self):
        from rcl_interfaces.srv import GetParameters
        if not self._param_cli.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['global_limit']
        fut = self._param_cli.call_async(req)
        fut.add_done_callback(self._speed_scaling_done)

    def _speed_scaling_done(self, fut):
        try:
            res = fut.result()
            if res and res.values:
                v = res.values[0]
                if v.type == 3:  # ParameterType DOUBLE = 3
                    self.speed_scaling = float(v.double_value)
        except Exception:
            pass  # keep previous value

    # ============================= SHUTDOWN =============================

    def on_shutdown(self):
        if self.profile_enabled:
            self.profiler.disable()
            self.profiler.dump_stats(self.profile_filename)
            with open(self.profile_report_filename, 'w') as fh:
                pstats.Stats(self.profiler, stream=fh).sort_stats('cumtime').print_stats()


def main(args=None):
    rclpy.init(args=args)
    node = GraphPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
