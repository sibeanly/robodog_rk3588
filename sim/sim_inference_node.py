#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2025-2026 Luo1imasi
#
# sim_inference_node.py - Python inference for the basemevius2 rough-terrain
# policy (policy_21399.onnx, 232->12) in mujoco sim.
#
# Obs/action math lives in sim/obs_math.py (single source of truth, unit-tested).
#
# Obs layout (232): ang_vel(3)*0.25 | gravity_b(3) | cmd(3)*1.0 |
#   (dof_pos-default)(12)*1.0 | dof_vel(12)*0.05 | last_action(12) |
#   height_scan(187) clip[-1,1]   (zeros on flat ground)
# Action: target = default + clip(action,-1,1) * per_joint_scale, clamp to limits.
#
# Topics:
#   subscribe /joint_states   sensor_msgs/JointState   (12, policy order [FR,FL,BR,BL])
#   subscribe /imu            sensor_msgs/Imu          (orientation xyzw, body ang_vel)
#   subscribe /cmd_vel        geometry_msgs/Twist
#   publish   /joint_targets  std_msgs/Float32MultiArray (12, policy order)
#
# Runs the policy at 50Hz.

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

sys.path.insert(0, os.path.dirname(__file__))
from obs_math import (POLICY_JOINT_NAMES, DEFAULT_ANGLE, CLIP_CMD,
                      build_obs, action_to_targets)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
ONNX_PATH = os.path.join(REPO_ROOT, "src", "inference", "models",
                         "policy_21399.onnx")

POLICY_HZ = 50


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

        self.joint_pos = DEFAULT_ANGLE.copy()
        self.joint_vel = np.zeros(12, dtype=np.float64)
        self.quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.ang_vel = np.zeros(3, dtype=np.float64)  # body frame
        self.cmd = np.zeros(3, dtype=np.float64)       # [vx, vy, wz]
        self.last_action = np.zeros(12, dtype=np.float64)  # raw onnx output, init 0
        self.name_to_idx = {n: i for i, n in enumerate(POLICY_JOINT_NAMES)}
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

    def policy_loop(self):
        period = 1.0 / POLICY_HZ
        next_t = time.monotonic()
        log_counter = 0
        while self.running and rclpy.ok():
            if self.has_js and self.has_imu:
                with self.state_lock:
                    quat = self.quat_wxyz.copy()
                    ang_vel = self.ang_vel.copy()
                    cmd = self.cmd.copy()
                    dof_pos = self.joint_pos.copy()
                    dof_vel = self.joint_vel.copy()
                obs = build_obs(ang_vel, quat, cmd, dof_pos, dof_vel,
                                self.last_action)
                action = self.session.run(
                    [self.output_name],
                    {self.input_name: obs.reshape(1, -1)})[0][0]
                self.last_action = np.asarray(action, dtype=np.float64).copy()
                target = action_to_targets(action)
                msg = Float32MultiArray()
                msg.data = target.astype(np.float32).tolist()
                self.target_pub.publish(msg)

                log_counter += 1
                if log_counter % POLICY_HZ == 0:  # ~1s
                    self.get_logger().info(
                        f"gb=[{obs[3]:+.2f},{obs[4]:+.2f},{obs[5]:+.2f}] "
                        f"cmd=[{obs[6]:+.2f},{obs[7]:+.2f},{obs[8]:+.2f}] "
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
