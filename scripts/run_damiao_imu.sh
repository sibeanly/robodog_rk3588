#!/bin/bash
# Launch Damiao IMU ROS2 node + static tf (world->imu_link) for rviz visualization.
# Usage: ./scripts/run_damiao_imu.sh   (then run rviz2 separately)
set -e
cd "$(dirname "$0")/.."
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export PATH=/usr/bin:/usr/local/bin:/bin
export DISPLAY=${DISPLAY:-:1}
source /opt/ros/humble/setup.bash
source install/setup.bash   # provides imu_py

# static tf world -> imu_link (background)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world imu_link &
TFPID=$!

# IMU node (foreground; Ctrl+C stops both via trap)
trap "kill $TFPID 2>/dev/null; exit 0" INT TERM
/usr/bin/python3 scripts/damiao_imu_node.py
