#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
"""ROS2 node wrapping the Damiao DM-IMU-L1 driver (imu_py) for rviz visualization.

Publishes sensor_msgs/Imu on /imu (frame_id imu_link) with orientation (w,x,y,z),
angular_velocity (rad/s) and linear_acceleration (m/s^2). Spins a background
thread polling the driver at high rate so /imu reflects the latest frame.

Run:
  source install/setup.bash   # provides imu_py
  ros2 run ... or: /usr/bin/python3 scripts/damiao_imu_node.py

Verify: ros2 topic hz /imu ; ros2 topic echo /imu --once ; rviz2 -> add Imu
"""
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
import imu_py

DEV = os.environ.get("DAMIAO_DEV", "/dev/dm_imu")
BAUD = int(os.environ.get("DAMIAO_BAUD", "921600"))


class DamiaoImuNode(Node):
    def __init__(self):
        super().__init__("damiao_imu_node")
        self.declare_parameter("dev", DEV)
        self.declare_parameter("baud", BAUD)
        self.declare_parameter("frame_id", "imu_link")
        dev = self.get_parameter("dev").value
        baud = self.get_parameter("baud").value
        self.frame_id = self.get_parameter("frame_id").value

        self.get_logger().info(f"creating DAMIAO IMU on {dev}@{baud}")
        self.imu = imu_py.IMUDriver.create_imu(
            imu_id=0, interface_type="serial", interface=dev,
            imu_type="DAMIAO", baudrate=baud)

        # Use RELIABLE QoS so rviz2's default (Reliable) Imu display can
        # subscribe without a QoS mismatch ("error subscribing"). Best-Effort
        # would require the user to manually set the display to Best Effort.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Imu, "/imu", qos)

        self.msg = Imu()
        self.msg.header.frame_id = self.frame_id
        # Covariances: leave zero (unknown) — rviz still renders orientation.
        self._pub_count = 0
        self.running = True
        self._t = threading.Thread(target=self._read_loop, daemon=True)
        self._t.start()
        # Status log at 1Hz.
        self.create_timer(1.0, self._status)

    def _read_loop(self):
        # Cap publish rate at ~200 Hz. The Damiao module streams at 1000 Hz
        # internally, but an unthrottled Python loop would publish ~13 kHz,
        # flooding subscribers (rviz "error subscribing") and pegging CPU.
        # 200 Hz is plenty for visualization and status.
        period = 1.0 / 200.0
        next_t = time.monotonic()
        while self.running and rclpy.ok():
            try:
                q = self.imu.get_quat()        # (w, x, y, z)
                av = self.imu.get_ang_vel()    # rad/s
                la = self.imu.get_lin_acc()    # m/s^2
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"imu read failed: {e}", throttle_duration_sec=2.0)
                continue
            # only publish once we have a valid quaternion
            if q and math.sqrt(sum(c * c for c in q)) > 1e-6:
                now = self.get_clock().now().to_msg()
                self.msg.header.stamp = now
                self.msg.orientation.w = float(q[0])
                self.msg.orientation.x = float(q[1])
                self.msg.orientation.y = float(q[2])
                self.msg.orientation.z = float(q[3])
                self.msg.angular_velocity.x = float(av[0])
                self.msg.angular_velocity.y = float(av[1])
                self.msg.angular_velocity.z = float(av[2])
                self.msg.linear_acceleration.x = float(la[0])
                self.msg.linear_acceleration.y = float(la[1])
                self.msg.linear_acceleration.z = float(la[2])
                self.pub.publish(self.msg)
                self._pub_count += 1
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def _status(self):
        if self._pub_count == 0:
            self.get_logger().warn("no IMU data yet (waiting for frames)")
        else:
            q = self.imu.get_quat()
            self.get_logger().info(
                f"publishing /imu  q(w,x,y,z)=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})")


def main():
    rclpy.init()
    node = DamiaoImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node._t.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
