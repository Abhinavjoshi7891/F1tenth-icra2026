#!/bin/bash
set -e

NPZ_PATH="/home/abhinav/Data_Drive/forzaeth/race_stack/stack_master/maps/porto/porto_raceline.npz"
ADAPTER_SRC="/home/abhinav/Data_Drive/forzaeth/race_stack/utilities/nodes/autodrive_adapter"
TELEMETRY="/home/abhinav/Data_Drive/forzaeth/race_stack/telemetry"

if [ ! -f "$NPZ_PATH" ]; then
    echo "ERROR: porto_raceline.npz not found at $NPZ_PATH"
    exit 1
fi

mkdir -p "$TELEMETRY"

echo "========================================="
echo " LocalMapPP + Cartographer Racing Stack"
echo "========================================="
echo " NPZ     : $NPZ_PATH"
echo " ADAPTER : $ADAPTER_SRC"
echo ""

docker stop autodrive_devkit 2>/dev/null || true
docker rm -f autodrive_devkit 2>/dev/null || true

docker run --name autodrive_devkit --rm -it \
    --network=host --ipc=host \
    -e ROS_DOMAIN_ID=48 \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env DISPLAY \
    --privileged --gpus all \
    -v "$ADAPTER_SRC":/home/autodrive_devkit/src/autodrive_adapter \
    -v "$TELEMETRY":/tmp/telemetry \
    -v "$NPZ_PATH":/tmp/porto_raceline.npz:ro \
    --entrypoint /bin/bash \
    roboracer-localmap:latest \
    -c "
        cd /home/autodrive_devkit && \
        rm -rf build/autodrive_adapter install/autodrive_adapter && \
        source /opt/ros/humble/setup.bash && \
        colcon build --packages-select autodrive_adapter --symlink-install 2>&1 | tail -5 && \
        source install/setup.bash && \
        echo '[READY] Starting autodrive_bridge + local_map_pp_cartographer' && \
        ros2 launch autodrive_adapter local_map_pp_cartographer.launch.py
    "

echo "=== Telemetry saved to $TELEMETRY/ ==="