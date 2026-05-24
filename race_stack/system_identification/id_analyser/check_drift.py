#!/usr/bin/env python3
"""Check heading drift during straight driving from a rosbag.
Usage: python3 check_drift.py [bag_path]
Default bag_path: /ros2_ws/rosbags/dexter_drift_test

Uses IMU gyro_z (actual rotation) NOT odometry yaw (computed from servo = circular).
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

BAG_PATH = sys.argv[1] if len(sys.argv) > 1 else '/ros2_ws/rosbags/dexter_drift_test'
print(f'Reading bag: {BAG_PATH}')

reader = SequentialReader()
reader.open(
    StorageOptions(uri=BAG_PATH, storage_id='sqlite3'),
    ConverterOptions('', '')
)

odom_t, speeds = [], []
imu_t, gyro_z = [], []

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == '/car_state/odom':
        msg = deserialize_message(data, Odometry)
        odom_t.append(t * 1e-9)
        speeds.append(msg.twist.twist.linear.x)
    elif topic == '/sensors/imu/raw':
        msg = deserialize_message(data, Imu)
        imu_t.append(t * 1e-9)
        gyro_z.append(msg.angular_velocity.z)

odom_t = np.array(odom_t) - odom_t[0]
speeds = np.array(speeds)
imu_t = np.array(imu_t) - imu_t[0]
gyro_z = np.array(gyro_z)

# Integrate gyro_z to get yaw (actual physical rotation)
dt = np.diff(imu_t, prepend=imu_t[0])
yaw_imu = np.degrees(np.cumsum(gyro_z * dt))

# Interpolate speed onto IMU timestamps for segmentation
speed_at_imu = np.interp(imu_t, odom_t, speeds)
moving = speed_at_imu > 0.2

# Segment into individual runs
segments = []
in_run = False
start = 0
for i in range(len(moving)):
    if moving[i] and not in_run:
        start = i
        in_run = True
    elif not moving[i] and in_run:
        segments.append((start, i))
        in_run = False
if in_run:
    segments.append((start, len(moving)))

print(f'Found {len(segments)} runs')
for i, (s, e) in enumerate(segments):
    drift = yaw_imu[e-1] - yaw_imu[s]
    duration = imu_t[e-1] - imu_t[s]
    if duration > 0.5:
        print(f'  Run {i+1}: duration={duration:.1f}s  yaw drift={drift:.2f}°  ({yaw_imu[s]:.1f}° → {yaw_imu[e-1]:.1f}°)')

real_runs = [(s, e) for s, e in segments if imu_t[e-1] - imu_t[s] > 0.5]

plt.figure(figsize=(14, 4))
plt.subplot(1, 2, 1)
for i, (s, e) in enumerate(real_runs):
    plt.plot(imu_t[s:e] - imu_t[s], yaw_imu[s:e] - yaw_imu[s], label=f'Run {i+1}')
plt.xlabel('time within run (s)')
plt.ylabel('yaw change (deg)')
plt.title('IMU yaw drift per run (0=straight)')
plt.legend()
plt.grid(True)
plt.subplot(1, 2, 2)
plt.plot(odom_t, speeds)
plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.title('Speed over time')
plt.grid(True)
plt.tight_layout()
plt.show()
