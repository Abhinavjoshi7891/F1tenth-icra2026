import numpy as np
import math
import time as _time
import os
import yaml

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from PIL import Image


# ---------------------------------------------------------------------------
# Circular buffer + timer helpers (unchanged from ROS1 utils.py)
# ---------------------------------------------------------------------------

class CircularArray:
    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self):
        return np.mean(self.arr[:self.num_els])

    def median(self):
        return np.median(self.arr[:self.num_els])


class Timer:
    def __init__(self, smoothing):
        self.arr = CircularArray(smoothing)
        self.last_time = _time.time()

    def tick(self):
        t = _time.time()
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    def fps(self):
        return self.arr.mean()


# ---------------------------------------------------------------------------
# Quaternion helpers  (replaces tf.transformations imports)
# ---------------------------------------------------------------------------

def quaternion_from_euler(roll, pitch, yaw):
    """Return (x, y, z, w) quaternion from RPY angles."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


def angle_to_quaternion(angle):
    """Convert a yaw angle (radians) into a Quaternion message."""
    x, y, z, w = quaternion_from_euler(0, 0, angle)
    q = Quaternion()
    q.x = x
    q.y = y
    q.z = z
    q.w = w
    return q


def quaternion_to_angle(q) -> float:
    """Extract yaw from a Quaternion message."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])


# ---------------------------------------------------------------------------
# Header helper  (replaces rospy.Time.now() — caller must supply stamp)
# ---------------------------------------------------------------------------

def make_header(frame_id, stamp) -> Header:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


# ---------------------------------------------------------------------------
# ParticleUtils (unchanged logic, no ROS1 API)
# ---------------------------------------------------------------------------

class ParticleUtils:
    def __init__(self, laser_base_link_offset: np.ndarray):
        self.laser_base_link_offset = laser_base_link_offset

    def particle_to_pose(self, particle) -> Pose:
        pose = Pose()
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.orientation = angle_to_quaternion(particle[2])
        pose = self.transpose_laser_to_base_link_xy(pose)
        return pose

    def transpose_laser_to_base_link_xy(self, pose: Pose) -> Pose:
        yaw = quaternion_to_angle(pose.orientation)
        pose.position.x -= self.laser_base_link_offset[0] * math.cos(yaw)
        pose.position.y -= self.laser_base_link_offset[0] * math.sin(yaw)
        return pose

    def particles_to_poses(self, particles):
        return list(map(self.particle_to_pose, particles))


# ---------------------------------------------------------------------------
# Coordinate transforms (unchanged logic, no ROS1 API)
# ---------------------------------------------------------------------------

def map_to_world(poses, map_info):
    """Convert poses from map pixel space to world metres, in-place."""
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    c, s = np.cos(angle), np.sin(angle)
    temp = np.copy(poses[:, 0])
    poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
    poses[:, 1] = s * temp + c * poses[:, 1]
    poses[:, :2] *= float(scale)
    poses[:, 0] += map_info.origin.position.x
    poses[:, 1] += map_info.origin.position.y
    poses[:, 2] += angle


def world_to_map(poses, map_info):
    """Convert poses from world metres to map pixel space, in-place."""
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)
    poses[:, 0] -= map_info.origin.position.x
    poses[:, 1] -= map_info.origin.position.y
    poses[:, :2] *= (1.0 / float(scale))
    c, s = np.cos(angle), np.sin(angle)
    temp = np.copy(poses[:, 0])
    poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
    poses[:, 1] = s * temp + c * poses[:, 1]
    poses[:, 2] += angle


def normalize(z: float) -> float:
    if -np.pi <= z <= np.pi:
        return z
    return np.arctan2(np.sin(z), np.cos(z))


def angle_diff(a: float, b: float) -> float:
    a = normalize(a)
    b = normalize(b)
    d1 = a - b
    d2 = 2 * np.pi - abs(d1)
    if d1 > 0:
        d2 *= -1.0
    return d1 if abs(d1) < abs(d2) else d2


# ---------------------------------------------------------------------------
# Map loading  (replaces rospy.wait_for_service("static_map") + ServiceProxy)
# ---------------------------------------------------------------------------

def load_map_as_occupancy_grid(yaml_path: str) -> OccupancyGrid:
    """
    Load a map YAML + image file and return a nav_msgs/OccupancyGrid.
    Compatible with range_libc.PyOMap(msg) duck-typing.
    """
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    img_path = os.path.join(os.path.dirname(yaml_path), cfg['image'])
    img = np.array(Image.open(img_path).convert('L'), dtype=np.uint8)

    free_thresh = cfg.get('free_thresh', 0.196)
    occupied_thresh = cfg.get('occupied_thresh', 0.65)
    negate = cfg.get('negate', 0)

    if negate:
        img = 255 - img

    # Convert to OGM: 0=free, 100=occupied, -1=unknown
    occ = np.full(img.shape, -1, dtype=np.int8)
    occ[img >= int(occupied_thresh * 255)] = 100   # occupied (dark pixels)
    occ[img <= int(free_thresh * 255)]      = 0     # this handles < free_thresh (no-wait)

    # Actually OGM convention: 0=free(255), 100=occupied(0)
    # range_libc expects: 0=free, >0=occupied in the data
    # Re-derive: white(255) = free, black(0) = occupied
    occ2 = np.zeros(img.shape, dtype=np.int8)
    occ2[img < int((1.0 - free_thresh) * 255)] = 100   # darker than free_thresh -> occupied

    # PIL/numpy reads PNG with row 0 = TOP of image. ROS OccupancyGrid spec
    # requires row 0 = BOTTOM of map (origin at bottom-left). nav2_map_server
    # applies this flip internally; ROS1 fetched the grid via static_map so it
    # was already flipped. Direct PIL load must flip explicitly, otherwise
    # range_libc raycasts against a vertically mirrored world and the PF
    # never converges to the true pose.
    occ2 = np.flipud(occ2)

    msg = OccupancyGrid()
    msg.info = MapMetaData()
    msg.info.resolution = float(cfg['resolution'])
    msg.info.width = img.shape[1]
    msg.info.height = img.shape[0]
    msg.info.origin.position.x = float(cfg['origin'][0])
    msg.info.origin.position.y = float(cfg['origin'][1])
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.x = 0.0
    msg.info.origin.orientation.y = 0.0
    msg.info.origin.orientation.z = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = occ2.flatten().tolist()
    return msg
