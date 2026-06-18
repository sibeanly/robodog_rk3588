# SPDX-License-Identifier: GPL-3.0
"""Launch Damiao IMU node + static tf (world->imu_link) + rviz2 for IMU visualization.

Run:
  source /opt/ros/jazzy/setup.bash
  source install_sp_d/setup.bash   # provides imu_py
  ros2 launch scripts/damiao_imu_rviz.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
RVIZ_CFG = os.path.join(HERE, os.pardir, "assets", "damiao_imu.rviz")
RVIZ_CFG = os.path.abspath(RVIZ_CFG)


def generate_launch_description():
    return LaunchDescription([
        # IMU publisher (wraps imu_py Damiao driver).
        ExecuteProcess(
            cmd=["/usr/bin/python3", os.path.join(HERE, "damiao_imu_node.py")],
            name="damiao_imu_node", output="screen"),
        # Static frame so rviz can place the IMU orientation axes.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="imu_tf",
            arguments=["0", "0", "0", "0", "0", "0", "world", "imu_link"]),
        # rviz with the IMU display config.
        ExecuteProcess(
            cmd=["rviz2", "-d", RVIZ_CFG],
            name="rviz2", output="screen"),
    ])
