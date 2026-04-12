# eth_roboracer - ICRA 2026 Sim Racing League

This is a customized version of the **ForzaETH Race Stack**, specifically adapted for the **AutoDRIVE RoboRacer ICRA 2026 Sim Racing League**. It integrates the high-performance racing algorithms from ETH Zurich with the AutoDRIVE simulator environment.

## 🚀 Key RoboRacer Additions

*   **AutoDRIVE Bridge:** A custom `autodrive_adapter` package that translates AutoDRIVE simulator topics (LiDAR, Odometry, IPS) to the standard ROS 2 topics expected by the ForzaETH stack.
*   **Porto Track Support:** Full integration for the Porto competition circuit, including:
    *   Pre-processed track maps (`pf_map.yaml`, `porto.yaml`).
    *   LiDAR scan cleaning and denoising scripts (`clean_map.py`, `clean_advanced.py`).
    *   Optimizedraceline waypoints for minimum curvature racing.
*   **Advanced Gap Follower (v2):** Improved obstacle avoidance with disparity extension, forward bias targeting, and proximity-based speed scaling.
*   **Custom Launch Infrastructure:** Dedicated launch scripts (`launch_localmap_v5.sh`, `launch_ips_mapper.sh`) tailored for the RoboRacer Docker environment.

---

## 🏁 End-to-End Run Instructions (v1.0)

### 🏎️ Racing Setup (5 Terminals)

**Terminal 1 — AutoDRIVE Simulator (Host)**
```bash
cd /home/abhinav/Data_Drive/forzaeth/autodrive_sim/autodrive_simulator_practice_linux/autodrive_simulator
./AutoDRIVE\ Simulator.x86_64
```
*Note: Do not click connect yet.*

**Terminal 2 — Devkit Bridge (Docker)**
```bash
docker run --name autodrive_roboracer_api --rm -it \
    --network=host --ipc=host \
    -e ROS_DOMAIN_ID=48 \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env DISPLAY --privileged --gpus all \
    roboracer-localmap:latest
# Inside:
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 run autodrive_roboracer autodrive_bridge --ros-args -r /tf:=/tf_devkit_unused
```
*Wait for "Connected!" then click connect in the simulator.*

**Terminal 3 — Adapter Node (ForzaETH Container)**
```bash
cd /home/abhinav/Data_Drive/forzaeth/race_stack
./.docker_utils/main_attach_dock.sh
# Inside:
source ~/ws/install/setup.bash
ros2 run autodrive_adapter adapter_node --ros-args -r /tf:=/tf_adapter_unused
```

**Terminal 4 — Base System (ForzaETH Container)**
```bash
./.docker_utils/sec_dock.sh
# Inside:
source ~/ws/install/setup.bash
ros2 launch stack_master base_system_autodrive_launch.xml \
    map_name:=porto racecar_version:=AUTODRIVE autodrive:=True
```

**Terminal 5 — Time Trials (ForzaETH Container)**
```bash
./.docker_utils/sec_dock.sh
# Inside:
source ~/ws/install/setup.bash
ros2 launch stack_master time_trials_launch.xml \
    racecar_version:=AUTODRIVE LU_table:=SIM_linear ctrl_algo:=MAP
```

---

### 🏎️ F1TENTH Gym & RViz Setup (2 Terminals)

For running the native F1Tenth Gym and RViz-based simulator:

**Terminal 1 — Load Base System (Docker)**
```bash
cd /home/abhinav/Data_Drive/forzaeth/race_stack
./.docker_utils/main_attach_dock.sh
# Inside:
source ~/ws/install/setup.bash
ros2 launch stack_master base_system_launch.xml \
    map_name:=<map_name> sim:=true racecar_version:=SIM
```

**Terminal 2 — Load Racing System (Docker)**
```bash
./.docker_utils/sec_dock.sh
# Inside:
source ~/ws/install/setup.bash
ros2 launch stack_master time_trials_launch.xml \
    racecar_version:=SIM LU_table:=SIM_linear ctrl_algo:=MAP
```
*Note: `ctrl_algo` can be `MAP` or `PP` (Pure Pursuit).*

---

### 🗺️ Mapping Setup (5 Terminals)

1.  **Terminal 1 (ForzaETH Main):** Start `adapter_node` (remapped `/tf`).
2.  **Terminal 2 (ForzaETH Sec):** `ros2 launch stack_master mapping_autodrive_launch.xml map_name:=porto racecar_version:=AUTODRIVE`
3.  **Terminal 3 (ForzaETH Sec):** Teleop drive: `ros2 run key_teleop key_teleop --ros-args -p forward_rate:=0.5`
4.  **Terminal 4 (Devkit Container):** `ros2 launch autodrive_roboracer bringup_headless.launch.py` (Domain ID 48).
5.  **Terminal 5 (Host):** Launch AutoDRIVE Simulator binary.

---

### ✏️ Interactive Raceline Editor
Inside the ForzaETH container:
```bash
# 1. Run the editor
python3 ~/ws/src/race_stack/utilities/nodes/raceline_editor.py --map-dir ~/ws/src/race_stack/stack_master/maps/<map_name>

# 2. Rebuild the stack_master package to apply changes
cd ~/ws && colcon build --packages-select stack_master --symlink-install

# 3. Source the updated workspace
source ~/ws/install/setup.bash
```

---

## 🛠️ Tuning & Configuration

To fine-tune your racing performance and path generation, you can tinker with the following files:

### 🏎️ Speed Profile & Scaling
*   **File:** `stack_master/maps/<map_name>/speed_scaling.yaml`
*   **Usage:** Used to adjust the speed profile and apply ceilings to the maximum speed, both globally and for specific track sections.

### 📐 Raceline Generation (ETH Algorithm)
*   **File:** `stack_master/config/global_planner/global_planner_params.yaml`
*   **Usage:** Manage parameters for the initial raceline generation, including:
    *   **Safety Width:** Distance to keep from track boundaries.
    *   **Optimization Type:** Toggle between shortest path and minimum curvature algorithms.

---

## 📊 Live Tuning & Diagnostics

This stack includes tools for real-time performance analysis and runtime parameter tuning without restarting the car.

### 🎚️ GUI Tuning (rqt_reconfigure)
You can adjust controller parameters while the car is driving using a graphical interface.
1.  Launch the stack as normal.
2.  Open a new terminal and run: `ros2 run rqt_reconfigure rqt_reconfigure`
3.  Select `/controller`. You can now move sliders for:
    *   **`m_l1`:** Speed gain (Higher = smoother, Lower = aggressive).
    *   **`q_l1`:** Lookahead offset.
    *   **`t_clip_min`:** Minimum lookahead floor (Crucial for tight corners).
    *   **`lat_err_coeff`:** Speed penalty for high lateral error.

### ⌨️ Terminal Tuning (ros2 param)
If the GUI is unavailable, use standard ROS 2 commands:
```bash
# Example: Decrease lookahead floor to 0.5m for sharper turning
ros2 param set /controller t_clip_min 0.5
```
*Note: Ensure you include the decimal (e.g., `1.0` instead of `1`) to avoid Type Mismatch errors.*

### 📈 Controller Performance Plotter
The stack automatically logs high-resolution diagnostic data to `~/plot_datas/` in CSV format.
1.  **Run:** Launch with `ctrl_algo:=PP` or `ctrl_algo:=MAP`.
2.  **Stop:** The file is flushed and closed upon Ctrl+C.
3.  **Analyze:** Use the included visualizer script:
    ```bash
    # Run from the repository root
    python3 race_stack/utilities/nodes/plot_controller_data.py ~/plot_datas/plot_data_TIMESTAMP.csv
    ```
This will generate a 4-pane plot showing **Speed Accuracy**, **Lateral Error spikes**, **L1 Scaling**, and **Steering Saturation**.

---

# Original ForzaETH Race Stack 

ForzaETH Race Stack by the [D-ITET Center for Project Based Learning (PBL)](https://pbl.ee.ethz.ch/) at ETH Zurich. 

Accompanying this repository, a paper titled *ForzaETH Race Stack - Scaled Autonomous Head-to-Head Racing on Fully Commercial off-the-Shelf Hardware* is available on [Journal of Field Robotics](https://onlinelibrary.wiley.com/doi/pdf/10.1002/rob.22429), detailing the system's architecture, algorithms, and performance benchmarks.

**Note:** The results in the paper have been evaluated in the ROS1 version of the stack. As this is the ROS 2 version of the ForzaETH Race Stack, results may differ, as well as certain functionalities might be **missing** such as (might be added in the future):
- Baysian Optimization
- CPU Usage Measurements
- Car2Car Syncing
- SynPF integration
- Scan Alignment
- System Identification

**Note:** In general the ROS 2 stack is significantly less tested and explored than the ROS 1 version!

## Installation

We provide an installation guide [here](.docker_utils/README.md). 

Or check out our [video playlist on Youtube](https://www.youtube.com/playlist?list=PLMzSGo5LtaW9cgdwHB_FnX3qlAYx7P6JI):  
<a href="https://www.youtube.com/watch?v=A9Clg1n6rII">
  <img src="./base_system/misc/install_thumbnail.png" alt="Install" style="width: 33%;"/>
</a>
<a href="https://www.youtube.com/watch?v=6PtFzrRz1GU">
  <img src="./base_system/misc/simulator_thumbnail.png" alt="Simulation" style="width: 33%;"/>
</a>
<a href="https://www.youtube.com/watch?v=ACQdLD27v-k">
  <img src="./base_system/misc/hardware_thumbnail.png" alt="Car" style="width: 33%;"/>
</a>

**Note:** Click on the thumbnails to watch the videos.

## Getting started

After installation, the car (or the simulation environment) is ready to be tested. For examples on how to run the different modules on the car, refer to the [`stack_master` README](./stack_master/README.md).

## Contributing

In case you find our package helpful and want to contribute, please either raise an issue or directly make a pull request. To create pull request please follow the guidelines in [CONTRIBUTING](./CONTRIBUTING.md).

## Acknowledgement
This project would not be possible without the use of multiple great open-sourced code bases as listed below:

- [f1tenth_system](https://github.com/f1tenth/f1tenth_system)
- [F1TENTH Racecar Simulator](https://github.com/f1tenth/f1tenth_simulator)
- [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- [Cartographer](https://github.com/cartographer-project/cartographer)
- [Cartographer ROS Integration](https://github.com/cartographer-project/cartographer_ros)
- [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
- [RangeLibc](https://github.com/kctess5/range_libc)
- [BayesOpt4ROS](https://github.com/IntelligentControlSystems/bayesopt4ros)
- [cpu_monitor](https://github.com/alspitz/cpu_monitor)

### Problems
If you are having problem with the SIM (no car/scans showing for example), try the setup once again:
```bash
source ~/ws/src/race_stack/.install_utils/f110_sim_setup.sh
```

If your joystick is not working, try the following while the controller is connected:
```bash
sudo chmod 666 /dev/input/js0
sudo chmod 666 /dev/input/event*
```

## Citing ForzaETH Race Stack

If you found our race stack helpful in your research, we would appreciate if you cite it as follows:
```
@article{baumann2024forzaeth,
  title={ForzaETH Race Stack—Scaled Autonomous Head-to-Head Racing on Fully Commercial Off-the-Shelf Hardware},
  author={Baumann, Nicolas and Ghignone, Edoardo and K{\"u}hne, Jonas and Bastuck, Niklas and Becker, Jonathan and Imholz, Nadine and Kr{\"a}nzlin, Tobias and Lim, Tian Yi and L{\"o}tscher, Michael and Schwarzenbach, Luca and others},
  journal={Journal of Field Robotics},
  year={2024},
  publisher={Wiley Online Library}
}
```
