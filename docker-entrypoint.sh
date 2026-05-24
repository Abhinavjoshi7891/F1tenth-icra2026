#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Deploy runtime-generated LUT if it exists in the volume-mounted source tree
cp /ros2_ws/src/forzaeth/race_stack/system_identification/on_track_sys_id/models/DEXTER/DEXTER_pacejka_lookup_table.csv \
   /ros2_ws/install/steering_lookup/share/steering_lookup/cfg/ 2>/dev/null || true

exec "$@"
