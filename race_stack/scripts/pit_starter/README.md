# Pit Starter (ROS2)

One-command pit lane setup for the F1Tenth race stack.

## Usage

```bash
source /opt/ros/humble/setup.bash   # if not in .bashrc

./pit_starter.sh <ROS_DOMAIN_ID> [rviz|rqt|rvizrqt]

# Examples:
./pit_starter.sh 0              # set domain only
./pit_starter.sh 0 rviz         # domain + RViz2 with race config
./pit_starter.sh 0 rvizrqt      # domain + RViz2 + rqt
```

## Finding the car's ROS_DOMAIN_ID

On the car terminal:
```bash
echo $ROS_DOMAIN_ID
# If blank: the car is using domain 0 (ROS2 default)
```

## Key monitoring commands from pit

```bash
ros2 topic echo /lap_data           # lap time + lateral error
ros2 topic hz /scan                 # should be ~40 Hz
ros2 topic hz /car_state/odom       # should be ~80 Hz
ros2 topic echo /state_machine      # GBTRACK / OT / FTG
ros2 topic echo /optimizer/current_objective  # live lap times during param_optimizer
```

## Network troubleshooting

```bash
# Ping car first
ping <car_ip>

# Allow DDS multicast through firewall (pit laptop)
sudo ufw allow 7400/udp

# If multicast doesn't work (e.g. bridged network), use FastDDS discovery server
# Set on both car and pit: export RMW_FASTRTPS_USE_BUILTIN_DISCOVERY_CLIENT=1
```