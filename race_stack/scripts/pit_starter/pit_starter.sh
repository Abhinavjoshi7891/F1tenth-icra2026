#!/bin/bash
# ================================================================
# pit_starter.sh — ROS2 Pit Lane Setup (F1Tenth Race Stack)
#
# Sets ROS_DOMAIN_ID to match the car, sources the workspace,
# verifies car nodes are visible, and optionally launches RViz2.
#
# Usage:
#   ./pit_starter.sh <ROS_DOMAIN_ID> [rviz|rqt|rvizrqt]
#
# Examples:
#   ./pit_starter.sh 0              # domain only
#   ./pit_starter.sh 0 rviz         # domain + RViz2
#   ./pit_starter.sh 0 rvizrqt      # domain + RViz2 + rqt
# ================================================================

set -e

# ── Argument validation ───────────────────────────────────────────
if [ $# -lt 1 ]; then
    echo ""
    echo "ERROR: ROS_DOMAIN_ID required."
    echo "Usage: ./pit_starter.sh <ROS_DOMAIN_ID> [rviz|rqt|rvizrqt]"
    echo ""
    exit 1
fi

DOMAIN_ID=$1
GUI_ARG=${2:-none}

if ! [[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] || [ "$DOMAIN_ID" -gt 232 ]; then
    echo "ERROR: ROS_DOMAIN_ID must be an integer 0-232."
    exit 1
fi

# ── Environment ───────────────────────────────────────────────────
export ROS_DOMAIN_ID=$DOMAIN_ID

echo ""
echo "=================================================="
echo "  F1Tenth Pit Lane — ROS2"
echo "=================================================="
echo "  ROS_DOMAIN_ID : $ROS_DOMAIN_ID"

# Source ROS2 if not already done
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
    echo "  ROS2 sourced  : /opt/ros/humble"
fi

# Source workspace — walk up from this script's location to find install/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="$SCRIPT_DIR/../../../../install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
    echo "  Workspace     : $(realpath "$WS_SETUP")"
else
    echo "  [WARN] Workspace setup.bash not found at $WS_SETUP"
    echo "         Run 'colcon build' from your ws root first."
fi

echo ""

# ── Verify car is reachable ───────────────────────────────────────
echo "  Checking ROS2 node discovery (domain $DOMAIN_ID)..."
NODES=$(timeout 3 ros2 node list 2>/dev/null || true)
if [ -z "$NODES" ]; then
    echo "  [WARN] No nodes found on domain $DOMAIN_ID."
    echo "         Check: car powered on, same network, matching domain ID."
    echo "         Firewall: sudo ufw allow 7400/udp"
else
    echo "  [OK] Nodes visible:"
    echo "$NODES" | sed 's/^/        /'
fi

echo ""

# ── Find RViz config ──────────────────────────────────────────────
RVIZ_CONFIG="$SCRIPT_DIR/../../stack_master/config/SIM/sim.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "  [WARN] sim.rviz not found at $RVIZ_CONFIG. RViz2 will launch without config."
    RVIZ_CONFIG=""
fi

# ── Launch GUI tools ──────────────────────────────────────────────
case "$GUI_ARG" in
    rviz)
        echo "  Launching RViz2..."
        if [ -n "$RVIZ_CONFIG" ]; then
            rviz2 -d "$RVIZ_CONFIG" &
        else
            rviz2 &
        fi
        ;;
    rqt)
        echo "  Launching rqt..."
        rqt &
        ;;
    rvizrqt)
        echo "  Launching RViz2 + rqt..."
        if [ -n "$RVIZ_CONFIG" ]; then
            rviz2 -d "$RVIZ_CONFIG" &
        else
            rviz2 &
        fi
        rqt &
        ;;
    none)
        echo "  No GUI launched. Launch manually:"
        [ -n "$RVIZ_CONFIG" ] && echo "    rviz2 -d $RVIZ_CONFIG" || echo "    rviz2"
        ;;
    *)
        echo "  ERROR: Unknown GUI arg '$GUI_ARG'. Valid: rviz | rqt | rvizrqt"
        exit 1
        ;;
esac

echo ""
echo "  USEFUL COMMANDS:"
echo "    ros2 node list"
echo "    ros2 topic echo /lap_data"
echo "    ros2 topic hz /scan"
echo "    ros2 topic echo /state_machine"
echo "=================================================="
echo ""