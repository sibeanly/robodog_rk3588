#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2025-2026 Luo1imasi
#
# sim_inference_node.py - Python alternative to the C++ inference_node for sim.
#
# Preferred path (avoids C++ sim_mode churn): replicates the observation math
# of src/inference/src/obs_manager.cpp and the action mapping of
# src/inference/src/inference_node.cpp::inference(), so the sim exercises the
# same policy_mevius2.onnx the real-hardware chain loads.
#
# Obs layout (34): ang_vel(3)*0.25 | gravity_b(3)=R(q_wxyz)^T*[0,0,-1] |
#                  cmd(3)*[2,2,0.25] | (dof_pos-default)(12)*sym |
#                  dof_vel(12)*0.05*sym | is_standing(1)
# Action: clamp raw to +-100; target = action * sym * 0.2 + default (act_alpha=1).
#
# Topics:
#   subscribe /joint_states   sensor_msgs/JointState   (12, [BL,BR,FL,FR])
#   subscribe /imu            sensor_msgs/Imu          (orientation xyzw, body ang_vel)
#   subscribe /cmd_vel        geometry_msgs/Twist
#   publish   /joint_targets  std_msgs/Float32MultiArray (12, [BL,BR,FL,FR])
#
# Runs the policy at 50Hz (CONTROL_DECIMATION=4 over the 200Hz PD).

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
from geometry_msgs.msg import Twist

import onnxruntime as ort

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
ONNX_PATH = os.path.join(REPO_ROOT, "src", "inference", "models",
                         "policy_mevius2.onnx")

# Policy joint order [BL, BR, FL, FR] x [collar, hip, knee].
JOINT_NAMES = [
    "BL_collar_joint", "BL_hip_joint", "BL_knee_joint",
    "BR_collar_joint", "BR_hip_joint", "BR_knee_joint",
    "FL_collar_joint", "FL_hip_joint", "FL_knee_joint",
    "FR_collar_joint", "FR_hip_joint", "FR_knee_joint",
]

# Constants from src/inference/config/inference_mevius2.yaml
# (scales) and mevius2-master/scripts/parameters.py (DEFAULT_ANGLE, sym).
DEFAULT_ANGLE = np.array(
    [0.0, 0.7, -1.2] * 4, dtype=np.float64)
DOF_SYM = np.array(
    [1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1], dtype=np.float64)
SCALE_ANG_VEL = 0.25
SCALE_LIN_VEL = 2.0
SCALE_DOF_POS = 1.0
SCALE_DOF_VEL = 0.05
ACTION_SCALE = 0.2
CLIP_OBS = 100.0
CLIP_ACTION = 100.0
# [vx_lo, vx_hi, vy_lo, vy_hi, wz_lo, wz_hi] from inference_mevius2.yaml clip_cmd.
CLIP_CMD = np.array([-0.4, 0.6, -0.4, 0.4, -0.8, 0.8], dtype=np.float64)

POLICY_HZ = 50
CMD_SCALE = np.array([SCALE_LIN_VEL, SCALE_LIN_VEL, SCALE_ANG_VEL],
                     dtype=np.float64)
GRAVITY_W = np.array([0.0, 0.0, -1.0], dtype=np.float64)


def quat_wxyz_to_rotmat(wxyz):
    """Rotation matrix R with v_world = R @ v_body for w-first quaternion."""
    w, x, y, z = wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


class SimInferenceNode(Node):
    def __init__(self):
        super().__init__("sim_inference_node")

        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        opts.intra_op_num_threads = 1
        self.session = ort.InferenceSession(
            ONNX_PATH, opts, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        out = self.session.get_outputs()[0]
        self.output_name = out.name
        self.get_logger().info(
            f"loaded ONNX {ONNX_PATH} in={self.session.get_inputs()[0].shape} "
            f"out={out.shape}")

        # State buffers (defaults keep the robot roughly upright until the first
        # /joint_states + /imu arrive).
        self.joint_pos = DEFAULT_ANGLE.copy()
        self.joint_vel = np.zeros(12, dtype=np.float64)
        self.quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.ang_vel = np.zeros(3, dtype=np.float64)  # body frame
        self.cmd = np.zeros(3, dtype=np.float64)       # [vx, vy, wz]
        self.name_to_idx = {n: i for i, n in enumerate(JOINT_NAMES)}
        self.state_lock = threading.Lock()
        self.has_js = False
        self.has_imu = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(JointState, "/joint_states", self.on_js, qos)
        self.create_subscription(Imu, "/imu", self.on_imu, qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.target_pub = self.create_publisher(Float32MultiArray,
                                                "/joint_targets", qos)

        self.running = True
        self.thread = threading.Thread(target=self.policy_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f"sim_inference_node @ {POLICY_HZ}Hz")

    def on_js(self, msg: JointState):
        with self.state_lock:
            for i, name in enumerate(msg.name):
                idx = self.name_to_idx.get(name)
                if idx is None:
                    continue
                self.joint_pos[idx] = msg.position[i]
                if i < len(msg.velocity):
                    self.joint_vel[idx] = msg.velocity[i]
            self.has_js = True

    def on_imu(self, msg: Imu):
        q = msg.orientation
        with self.state_lock:
            self.quat_wxyz = np.array(
                [q.w, q.x, q.y, q.z], dtype=np.float64)
            self.ang_vel = np.array(
                [msg.angular_velocity.x, msg.angular_velocity.y,
                 msg.angular_velocity.z], dtype=np.float64)
            self.has_imu = True

    def on_cmd(self, msg: Twist):
        vx = float(np.clip(msg.linear.x, CLIP_CMD[0], CLIP_CMD[1]))
        vy = float(np.clip(msg.linear.y, CLIP_CMD[2], CLIP_CMD[3]))
        wz = float(np.clip(msg.angular.z, CLIP_CMD[4], CLIP_CMD[5]))
        with self.state_lock:
            self.cmd = np.array([vx, vy, wz], dtype=np.float64)

    def build_obs(self) -> np.ndarray:
        with self.state_lock:
            quat = self.quat_wxyz.copy()
            ang_vel = self.ang_vel.copy()
            cmd = self.cmd.copy()
            dof_pos = self.joint_pos.copy()
            dof_vel = self.joint_vel.copy()
        # gravity_b = R(q_b2w)^T @ [0,0,-1]  (== q_w2b * gravity_w in the C++ node)
        gravity_b = quat_wxyz_to_rotmat(quat).T @ GRAVITY_W
        is_standing = 1.0 if float(np.linalg.norm(cmd[:3])) < 0.03 else 0.0
        obs = np.concatenate([
            ang_vel * SCALE_ANG_VEL,                                 # 3
            gravity_b,                                               # 3
            cmd * CMD_SCALE,                                         # 3
            (dof_pos - DEFAULT_ANGLE) * SCALE_DOF_POS * DOF_SYM,     # 12
            dof_vel * SCALE_DOF_VEL * DOF_SYM,                       # 12
            np.array([is_standing], dtype=np.float64),              # 1
        ]).astype(np.float32)
        return np.clip(obs, -CLIP_OBS, CLIP_OBS)

    def policy_loop(self):
        period = 1.0 / POLICY_HZ
        next_t = time.monotonic()
        log_counter = 0
        while self.running and rclpy.ok():
            if self.has_js and self.has_imu:
                obs = self.build_obs()
                action = self.session.run(
                    [self.output_name],
                    {self.input_name: obs.reshape(1, -1)})[0][0]
                action = np.clip(action, -CLIP_ACTION, CLIP_ACTION)
                target = action * DOF_SYM * ACTION_SCALE + DEFAULT_ANGLE
                msg = Float32MultiArray()
                msg.data = target.astype(np.float32).tolist()
                self.target_pub.publish(msg)

                log_counter += 1
                if log_counter % POLICY_HZ == 0:  # ~1s
                    # obs: [ang_vel(3) | gravity_b(3) | cmd(3) | dof_pos(12) |
                    #       dof_vel(12) | is_standing(1)]
                    self.get_logger().info(
                        f"gb=[{obs[3]:+.2f},{obs[4]:+.2f},{obs[5]:+.2f}] "
                        f"cmd=[{obs[6]:+.2f},{obs[7]:+.2f},{obs[8]:+.2f}] "
                        f"stand={obs[33]:.0f} "
                        f"act[0:3]=[{action[0]:+.2f},{action[1]:+.2f},{action[2]:+.2f}] "
                        f"tgt[0:3]=[{target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}]")
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()


def main():
    rclpy.init(args=sys.argv)
    node = SimInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
