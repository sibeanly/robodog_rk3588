#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2025-2026 Luo1imasi
#
# mujoco_bridge.py - mujoco physics bridge for the mevius2 quadruped.
#
# Mirrors /home/orange5plus/code/mevius2/scripts/mevius2_main.py
# (mujoco_thread_func): keyframe reset, per-joint qpos/qvel mapping by name,
# PD torque into data.ctrl, mj_step, then publish joint_states + imu.
#
# Topics:
#   publish  /joint_states       sensor_msgs/JointState   (12, [FR,FL,BR,BL])
#   publish  /imu                sensor_msgs/Imu          (frame_id=imu_link,
#                       orientation wxyz->xyzw, ang_vel body-frame from gyro)
#   publish  /mujoco/base_pose   geometry_msgs/PoseStamped (base xyz+quat, monitor)
#   subscribe /joint_targets     std_msgs/Float32MultiArray (12, [FR,FL,BR,BL])
#
# PD: tau = kp*(target - qpos) + kd*(0 - qvel),  kp=50, kd=2  (200Hz).
# Spawn: mj_resetDataKeyframe(model, data, 0) -> STANDBY keyframe.
# Until the first /joint_targets arrives, the PD target is held at the
# keyframe qpos so the robot keeps its spawn pose.

import os
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

import mujoco
import mujoco.viewer

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
SCENE_XML = os.path.join(REPO_ROOT, "assets", "mujoco", "scene.xml")

# Policy joint order: [FR, FL, BR, BL] x [collar, hip, knee].
# (mujoco defines them in FR,FL,BR,BL body order, so we map by name.)
JOINT_NAMES = [
    "FR_collar_joint", "FR_hip_joint", "FR_knee_joint",
    "FL_collar_joint", "FL_hip_joint", "FL_knee_joint",
    "BR_collar_joint", "BR_hip_joint", "BR_knee_joint",
    "BL_collar_joint", "BL_hip_joint", "BL_knee_joint",
]

KP = 50.0
KD = 2.0
EFFORT_LIMIT = 60.0  # DCMotorCfg effort_limit; clip PD torque to +/-60
SIM_HZ = 200

# Height-scan ray grid: matches IsaacLab GridPatternCfg(resolution=0.1, size=[1.6,1.0]).
# 17 x 11 = 187 rays, yaw-aligned, shot straight down from base + (grid_xy, +20).
# Isaac mdp.height_scan = base_z - ray_hit_z - 0.5, clipped [-1,1].
HEIGHT_SCAN_OFFSET = 0.5
_HS_SX, _HS_SY, _HS_RES = 1.6, 1.0, 0.1
_hs_xs = np.arange(-_HS_SX / 2, _HS_SX / 2 + 1e-9, _HS_RES)
_hs_ys = np.arange(-_HS_SY / 2, _HS_SY / 2 + 1e-9, _HS_RES)
_HS_GX, _HS_GY = np.meshgrid(_hs_xs, _hs_ys, indexing="ij")  # 17x11
HS_GRID_LOCAL = np.stack([_HS_GX.ravel(), _HS_GY.ravel(), np.full(187, 20.0)], axis=1)  # (187,3)
HS_RAY_DIR = np.array([0.0, 0.0, -1.0])
HEIGHT_SCAN_HZ = 50  # scanner ticks at policy rate (Isaac: decimation*dt)


class MujocoSimNode(Node):
    def __init__(self, use_viewer: bool):
        super().__init__("mujoco_sim_node")

        self.model = mujoco.MjModel.from_xml_path(SCENE_XML)
        self.data = mujoco.MjData(self.model)
        # Reset to the STANDBY keyframe and take one step so derived quantities
        # (body poses, sensors) are populated, matching the reference.
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_step(self.model, self.data)

        if abs(self.model.opt.timestep - 1.0 / SIM_HZ) > 1e-6:
            self.get_logger().warn(
                f"model timestep {self.model.opt.timestep} != {1.0/SIM_HZ}; "
                f"using model timestep for stepping")

        # Per-joint qpos/qvel/actuator addresses, in JOINT_NAMES order.
        self.qpos_adr = np.zeros(12, dtype=int)
        self.qvel_adr = np.zeros(12, dtype=int)
        self.actuator_id = np.zeros(12, dtype=int)
        for i, name in enumerate(JOINT_NAMES):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"joint not found in mujoco model: {name}")
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if aid < 0:
                raise RuntimeError(f"actuator not found in mujoco model: {name}")
            self.qpos_adr[i] = self.model.jnt_qposadr[jid]
            self.qvel_adr[i] = self.model.jnt_dofadr[jid]
            self.actuator_id[i] = aid

        # Body-frame gyro sensor (angular velocity in the base_link site frame).
        # NB: a free joint's qvel[3:6] is in the GLOBAL frame, so the gyro sensor
        # is the correct body-frame source for the policy observation.
        self.gyro_adr = self._sensor_adr("body_gyro_sensor", 3)

        # Height-scan: put robot geoms in group 1 so downward rays (cast against
        # group 0 = floor + obstacles) ignore the robot's own legs. Matches
        # basemevius2/onnx_script/deploy_mujoco.py.
        for g in range(self.model.ngeom):
            bid = self.model.geom_bodyid[g]
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if bname not in (None, "world", "box"):  # robot body
                self.model.geom_group[g] = 1
        self._hs_geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self._hs_geomid = np.array([-1], dtype=np.int32)

        # Initial PD target = keyframe qpos (STANDBY) so the robot holds its
        # spawn pose until the first /joint_targets arrives.
        self.target = self.data.qpos[self.qpos_adr].copy()
        self.target_lock = threading.Lock()
        self.got_targets = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.js_pub = self.create_publisher(JointState, "/joint_states", qos)
        self.imu_pub = self.create_publisher(Imu, "/imu", qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/mujoco/base_pose", 10)
        self.hs_pub = self.create_publisher(Float32MultiArray,
                                            "/mujoco/height_scan", qos)
        self.create_subscription(Float32MultiArray, "/joint_targets",
                                 self.on_targets, qos)

        self.js_msg = JointState()
        self.js_msg.name = list(JOINT_NAMES)
        self.js_msg.position = [0.0] * 12
        self.js_msg.velocity = [0.0] * 12
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"

        self.viewer = None
        if use_viewer:
            try:
                # Official mujoco.viewer.launch_passive: rendering runs in its
                # own thread, so it does NOT block rclpy.spin (unlike the
                # third-party mujoco-python-viewer which froze the window).
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                self.viewer.cam.trackbodyid = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
                self.get_logger().info("mujoco viewer (launch_passive) enabled")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"viewer unavailable ({e}); headless")
                self.viewer = None

        self.running = True
        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.sim_thread.start()
        self.get_logger().info(
            f"mujoco bridge @ {SIM_HZ}Hz, 12 joints, scene={SCENE_XML}")

    def _sensor_adr(self, name: str, size: int) -> int:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            raise RuntimeError(f"sensor not found in mujoco model: {name}")
        adr = self.model.sensor_adr[sid]
        dim = self.model.sensor_dim[sid]
        if dim != size:
            raise RuntimeError(
                f"sensor {name} has dim {dim}, expected {size}")
        return adr

    def on_targets(self, msg: Float32MultiArray):
        if len(msg.data) >= 12:
            with self.target_lock:
                self.target = np.array(msg.data[:12], dtype=float)
                self.got_targets = True

    def sim_loop(self):
        period = 1.0 / SIM_HZ
        next_t = time.monotonic()
        log_counter = 0
        hs_counter = 0
        hs_every = SIM_HZ // HEIGHT_SCAN_HZ  # 200//50 = 4 -> 50Hz
        while self.running and rclpy.ok():
            self.step()
            log_counter += 1
            hs_counter += 1
            if hs_counter >= hs_every:
                hs_counter = 0
                self.publish_height_scan()
            if log_counter % SIM_HZ == 0:  # ~1s
                q = self.data.qpos
                self.get_logger().info(
                    f"base xyz=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f}) "
                    f"qw={q[3]:.3f} targets={'on' if self.got_targets else 'hold-standby'}")
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()  # fell behind; resync

    def publish_height_scan(self):
        """Isaac-consistent height_scan: 187 yaw-aligned down-rays, clipped [-1,1].

        height_scan[i] = base_z - ray_hit_z - 0.5, where ray_hit_z is the world z
        of the terrain hit point. Rays cast against terrain geoms (group 0 =
        floor + obstacles); robot geoms (group 1) are ignored via geomgroup so
        legs never pollute the scan. Misses (dist<0) -> ground z=0.
        """
        base_pos = self.data.qpos[0:3].copy()
        base_quat = self.data.qpos[3:7].copy()  # wxyz
        w, x, y, z = base_quat
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        cy, sy = np.cos(yaw), np.sin(yaw)
        gx = HS_GRID_LOCAL[:, 0]
        gy = HS_GRID_LOCAL[:, 1]
        wx = base_pos[0] + cy * gx - sy * gy
        wy = base_pos[1] + sy * gx + cy * gy
        wz = base_pos[2] + 5.0   # well above obstacles (deploy_mujoco uses +5)
        hit_z = np.empty(187, dtype=np.float64)
        for i in range(187):
            pnt = np.array([wx[i], wy[i], wz])   # wz is a scalar
            dist = mujoco.mj_ray(self.model, self.data, pnt, HS_RAY_DIR,
                                 self._hs_geomgroup, 1, -1, self._hs_geomid)
            if dist < 0:
                hit_z[i] = 0.0   # miss -> ground z=0
            else:
                hit_z[i] = wz - dist
        hs = np.clip(base_pos[2] - hit_z - HEIGHT_SCAN_OFFSET, -1.0, 1.0)
        msg = Float32MultiArray()
        msg.data = hs.astype(np.float32).tolist()
        self.hs_pub.publish(msg)

    def step(self):
        # PD using current (pre-step) qpos/qvel, recomputed every physics step
        # (200Hz) to match Isaac Lab's continuous DCMotor PD (avoids damping
        # starvation). Torque clipped to EFFORT_LIMIT (60 N.m, DCMotorCfg).
        with self.target_lock:
            tgt = self.target.copy()
        qpos = self.data.qpos[self.qpos_adr]
        qvel = self.data.qvel[self.qvel_adr]
        tau = np.clip(KP * (tgt - qpos) + KD * (-qvel), -EFFORT_LIMIT, EFFORT_LIMIT)
        for i, aid in enumerate(self.actuator_id):
            self.data.ctrl[aid] = tau[i]

        mujoco.mj_step(self.model, self.data)

        if self.viewer is not None:
            try:
                if not self.viewer.is_running():
                    self.get_logger().warn("viewer closed; stopping sim")
                    self.running = False
                    return
                self.viewer.sync()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"viewer sync failed: {e}")
                self.viewer = None

        # Read post-step state and publish.
        qpos = self.data.qpos[self.qpos_adr]
        qvel = self.data.qvel[self.qvel_adr]
        now = self.get_clock().now().to_msg()

        self.js_msg.header.stamp = now
        self.js_msg.position = qpos.tolist()
        self.js_msg.velocity = qvel.tolist()
        self.js_pub.publish(self.js_msg)

        wxyz = self.data.qpos[3:7]
        self.imu_msg.header.stamp = now
        self.imu_msg.orientation.w = float(wxyz[0])
        self.imu_msg.orientation.x = float(wxyz[1])
        self.imu_msg.orientation.y = float(wxyz[2])
        self.imu_msg.orientation.z = float(wxyz[3])
        gyro = self.data.sensordata[self.gyro_adr:self.gyro_adr + 3]
        self.imu_msg.angular_velocity.x = float(gyro[0])
        self.imu_msg.angular_velocity.y = float(gyro[1])
        self.imu_msg.angular_velocity.z = float(gyro[2])
        self.imu_pub.publish(self.imu_msg)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "world"
        pose.pose.position.x = float(self.data.qpos[0])
        pose.pose.position.y = float(self.data.qpos[1])
        pose.pose.position.z = float(self.data.qpos[2])
        pose.pose.orientation.w = float(wxyz[0])
        pose.pose.orientation.x = float(wxyz[1])
        pose.pose.orientation.y = float(wxyz[2])
        pose.pose.orientation.z = float(wxyz[3])
        self.pose_pub.publish(pose)


def main():
    use_viewer = "--no-viewer" not in sys.argv and bool(os.environ.get("DISPLAY", ""))
    rclpy.init(args=sys.argv)
    node = MujocoSimNode(use_viewer=use_viewer)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.sim_thread.join(timeout=2.0)
        # Do NOT call viewer.close() — launch_passive's viewer thread can
        # segfault on Jetson during cleanup. Let the process exit naturally;
        # the OS reaps the viewer thread. Set viewer=None so nothing else
        # touches it.
        node.viewer = None
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
