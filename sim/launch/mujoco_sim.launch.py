# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2025-2026 Luo1imasi
#
# Launch the mevius2 mujoco sim: mujoco_bridge (physics) + sim_inference_node
# (Python policy). Run from a shell with conda deactivated and ROS sourced:
#   conda deactivate 2>/dev/null; unset PYTHONPATH PYTHONHOME
#   export PATH=/usr/bin:$PATH; source /opt/ros/jazzy/setup.bash
#   ros2 launch mujoco_sim.launch.py [use_viewer:=true]
#
# Drive the robot:
#   ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
#   ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
BRIDGE = os.path.join(SIM_DIR, "mujoco_bridge.py")
INFER = os.path.join(SIM_DIR, "sim_inference_node.py")


def generate_launch_description():
    use_viewer = LaunchConfiguration("use_viewer")

    return LaunchDescription([
        DeclareLaunchArgument("use_viewer", default_value="false"),
        # Viewer-enabled bridge (only when use_viewer:=true).
        ExecuteProcess(
            cmd=["/usr/bin/python3", BRIDGE],
            name="mujoco_sim_node",
            output="screen",
            condition=IfCondition(use_viewer),
        ),
        # Headless bridge (default).
        ExecuteProcess(
            cmd=["/usr/bin/python3", BRIDGE, "--no-viewer"],
            name="mujoco_sim_node",
            output="screen",
            condition=IfCondition(PythonExpression(
                ["'", use_viewer, "' == 'false'"])),
        ),
        ExecuteProcess(
            cmd=["/usr/bin/python3", INFER],
            name="sim_inference_node",
            output="screen",
        ),
    ])
