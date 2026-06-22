# basemevius2 Rough-Terrain Model Adaptation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Adapt the Python mujoco sim and the C++ inference node to run the newly trained quadruped policy `policy_21399.onnx` (RSL-RL rough-terrain, 232→12), replacing the old 34→12 mevius2 model.

**Architecture:** Approach A — Sim-first verification then C++ port. Phase 1 rewrites the Python sim (`sim_inference_node.py` + `sim/mujoco_bridge.py`) to the new 232-dim obs / per-joint-action convention and verifies the robot stands/walks in mujoco. Phase 2 de-submodules `src/inference` into the main repo. Phase 3 ports the jeston-main mevius2 C++ base. Phase 4 extends the C++ node (per-joint action scale + `height_scan` zero-fill obs source + new config) and switches the launch default. The sim and C++ share one deployment-convention table but carry the values independently.

**Tech Stack:** ROS2 Humble, Python 3 (rclpy, onnxruntime, mujoco), C++17, onnxruntime 1.21.0 (CPU), CMake/colcon. Target aarch64 (Jetson) + x86_64.

## Global Constraints

- New model: `policy_21399.onnx`, input `obs` shape `[1,232]` float32, output `actions` shape `[1,12]` float32, opset 18, batch fixed at 1. No normalization inside the graph (`actor_obs_normalization=False`).
- Policy joint order: `[FR, FL, BR, BL] × [collar, hip, knee]` (NOT the old `[BL, BR, FL, FR]`, NOT Unitree's RR/RL).
- Obs 232 = `ang_vel×0.25(3) | projected_gravity(3) | cmd×1.0(3) | (joint_pos−default)×1.0(12) | joint_vel×0.05(12) | last_action(12) | height_scan clip[-1,1](187)`. `last_action` init = zeros, then raw onnx output each tick.
- Action: `target[i] = default[i] + clip(onnx_out[i], -1, 1) × per_joint_scale[i]`, scale `[0.125, 0.15, 0.30] × 4` in policy order. `default = [0.0, 0.7, -1.2] × 4`.
- Joint limits (URDF, all legs identical): collar `±0.7854`, hip `[-1.0472, 2.6180]`, knee `[-2.8508, -0.7812]`.
- PD: kp=50, kd=2 (all 12). Policy 50 Hz (decimation 4, sim/ctrl dt 0.005).
- `dof_sym_sign` = all +1 (new model has NO left/right symmetry sign, unlike old mevius2).
- `height_scan` = 187 zeros on flat ground (no terrain sensor; README-sanctioned).
- cmd ranges: vx∈[-0.8,0.8], vy∈[-0.5,0.5], wz∈[-0.8,0.8].
- No test framework in the wrapper repo (per CLAUDE.md); verification = Python unit test for obs math + `colcon build` + sim run. Never actuate real hardware without the user present.
- `src/inference` is currently a submodule of `Roboparty/roboparty_inference`; Phase 2 removes that submodule relationship. After de-submodule, C++ commits stay in `sibeanly/robodog_jeston`.
- The reference C++ base is the non-git snapshot at `/home/orange5plus/code/robodog_jeston-main/src/inference/` (a modified fork that already adds `is_standing`, `dof_sym_sign`, dynamic output-batch handling, `inference_mevius2.yaml`, and a 12-DOF `robot.yaml`).

---

## File Structure

**Phase 1 (sim, current repo `robodog_jeston`):**
- Modify `sim/sim_inference_node.py` — new 232-dim obs builder, per-joint action, FR/FL/BR/BL order, last_action state.
- Modify `sim/mujoco_bridge.py` — `JOINT_NAMES` → FR/FL/BR/BL order so `/joint_states` & `/joint_targets` arrays are in policy order.
- Create `sim/test_obs_builder.py` — unit test for the obs/action math (extracted pure functions).
- Copy `policy_21399.onnx` → `src/inference/models/policy_21399.onnx`.

**Phase 2 (de-submodule, current repo):**
- Modify `.gitmodules` — remove `[submodule "inference"]` block.
- Remove `.git/modules/src/inference`, `git rm src/inference`, replace dir with jeston-main copy, `git add`.

**Phase 3 (port C++ base):** no code edits; verify `colcon build`.

**Phase 4 (extend C++):**
- Modify `src/inference/src/inference_node.hpp` — `action_scale_` scalar→vector decl, `get_height_scan_obs` decl.
- Modify `src/inference/src/inference_node.cpp` — action mapping uses per-joint scale vector.
- Modify `src/inference/src/ros_interface.cpp` — declare/load `action_scale` as array (backward-compat scalar), declare `height_scan_size`.
- Modify `src/inference/src/obs_manager.cpp` — register `height_scan` source + `get_height_scan_obs` impl.
- Create `src/inference/config/inference_mevius2_rough.yaml`.
- Modify `src/inference/launch/inference.launch.py` — default → `inference_mevius2_rough.yaml`.

---

## Task 1: Stage the new model file

**Files:**
- Create: `src/inference/models/policy_21399.onnx`

**Interfaces:**
- Produces: `src/inference/models/policy_21399.onnx` — referenced by both `sim/sim_inference_node.py` (Task 3) and `config/inference_mevius2_rough.yaml` (Task 9). Input `[1,232]` float32 `obs`, output `[1,12]` float32 `actions`.

- [ ] **Step 1: Copy the model from the training repo**

```bash
cp /home/orange5plus/code/basemevius2/onnx_script/policy_21399.onnx \
   /home/orange5plus/code/robodog_jeston/src/inference/models/policy_21399.onnx
```

- [ ] **Step 2: Verify the signature matches the spec**

Run:
```bash
python3 -c "import onnx; m=onnx.load('/home/orange5plus/code/robodog_jeston/src/inference/models/policy_21399.onnx'); print('in',[(i.name,[d.dim_value for d in i.type.tensor_type.shape.dim]) for i in m.graph.input]); print('out',[(o.name,[d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])"
```
Expected: `in [('obs', [1, 232])]` and `out [('actions', [1, 12])]`.

- [ ] **Step 3: Commit**

```bash
git -C /home/orange5plus/code/robodog_jeston add src/inference/models/policy_21399.onnx
git -C /home/orange5plus/code/robodog_jeston commit -m "feat(inference): stage policy_21399.onnx rough-terrain model (232->12)"
```
Note: `src/inference` is still a submodule at this point; the file lands as untracked content inside the submodule. That is fine — Phase 2 absorbs the whole dir (including this file) into the main repo. Do not commit the submodule pointer change here.

---

## Task 2: Extract pure obs/action math into a testable module (TDD)

**Files:**
- Create: `sim/obs_math.py`
- Create: `sim/test_obs_builder.py`

**Interfaces:**
- Produces: `sim/obs_math.py` with:
  - `POLICY_JOINT_NAMES: list[str]` (FR/FL/BR/BL × collar/hip/knee)
  - `DEFAULT_ANGLE: np.ndarray` (12)
  - `PER_JOINT_ACTION_SCALE: np.ndarray` (12)
  - `JOINT_LIMITS: np.ndarray` (12×2)
  - `SCALE_ANG_VEL=0.25, SCALE_DOF_VEL=0.05, HEIGHT_SCAN_SIZE=187, CLIP_OBS=100.0, CLIP_ACTION=1.0`
  - `CLIP_CMD: np.ndarray` (6)
  - `build_obs(ang_vel, quat_wxyz, cmd, dof_pos, dof_vel, last_action, height_scan=None) -> np.ndarray` (232, float32)
  - `action_to_targets(action: np.ndarray) -> np.ndarray` (12, applies clip[-1,1] × scale + default, then clamp to limits)
- Consumes: nothing (pure functions; `sim_inference_node.py` will import these in Task 3).

- [ ] **Step 1: Write the failing test**

Create `sim/test_obs_builder.py`:
```python
#!/usr/bin/env python3
"""Unit tests for sim/obs_math.py — the new 232-dim rough-terrain convention."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from obs_math import (build_obs, action_to_targets, DEFAULT_ANGLE,
                      PER_JOINT_ACTION_SCALE, JOINT_LIMITS, HEIGHT_SCAN_SIZE,
                      SCALE_ANG_VEL, SCALE_DOF_VEL, CLIP_ACTION)

def test_obs_shape_and_segments():
    ang_vel = np.array([0.1, -0.2, 0.3])
    quat = np.array([1.0, 0.0, 0.0, 0.0])  # identity -> gravity_b = [0,0,-1]
    cmd = np.array([0.3, 0.0, 0.0])
    dof_pos = DEFAULT_ANGLE.copy()
    dof_vel = np.zeros(12)
    last_action = np.zeros(12)
    obs = build_obs(ang_vel, quat, cmd, dof_pos, dof_vel, last_action)
    assert obs.shape == (232,), obs.shape
    assert obs.dtype == np.float32
    # ang_vel scaled by 0.25
    np.testing.assert_allclose(obs[0:3], ang_vel * SCALE_ANG_VEL, atol=1e-6)
    # gravity_b at identity = [0,0,-1]
    np.testing.assert_allclose(obs[3:6], [0.0, 0.0, -1.0], atol=1e-6)
    # cmd scaled by 1.0
    np.testing.assert_allclose(obs[6:9], cmd, atol=1e-6)
    # dof_pos - default = 0 at default pose
    np.testing.assert_allclose(obs[9:21], 0.0, atol=1e-6)
    # dof_vel * 0.05 = 0
    np.testing.assert_allclose(obs[21:33], 0.0, atol=1e-6)
    # last_action zeros
    np.testing.assert_allclose(obs[33:45], 0.0, atol=1e-6)
    # height_scan zeros (flat ground default)
    np.testing.assert_allclose(obs[45:232], 0.0, atol=1e-6)
    assert (45 + HEIGHT_SCAN_SIZE) == 232

def test_last_action_echoed():
    last = np.full(12, 0.5)
    obs = build_obs(np.zeros(3), np.array([1.,0,0,0]), np.zeros(3),
                    DEFAULT_ANGLE.copy(), np.zeros(12), last)
    np.testing.assert_allclose(obs[33:45], 0.5, atol=1e-6)

def test_action_to_targets_default():
    # zero action -> targets = default
    act = np.zeros(12)
    tgt = action_to_targets(act)
    np.testing.assert_allclose(tgt, DEFAULT_ANGLE, atol=1e-6)

def test_action_to_targets_scale_and_clip():
    # action = +1 on collar (idx 0) -> default + 1*0.125
    act = np.zeros(12); act[0] = 1.0
    tgt = action_to_targets(act)
    assert abs(tgt[0] - (0.0 + 1.0 * PER_JOINT_ACTION_SCALE[0])) < 1e-6
    # action beyond clip 1.0 gets clamped to 1.0
    act2 = np.zeros(12); act2[1] = 5.0  # hip, idx 1
    tgt2 = action_to_targets(act2)
    assert abs(tgt2[1] - (0.7 + CLIP_ACTION * PER_JOINT_ACTION_SCALE[1])) < 1e-6

def test_action_clamped_to_joint_limits():
    # huge negative knee action must not exceed lower limit
    act = np.zeros(12); act[2] = -100.0  # knee idx 2
    tgt = action_to_targets(act)
    assert tgt[2] >= JOINT_LIMITS[2][0] - 1e-6, (tgt[2], JOINT_LIMITS[2])

def test_per_joint_scale_values():
    # collar=0.125, hip=0.15, knee=0.30 per leg
    expected = np.array([0.125, 0.15, 0.30] * 4)
    np.testing.assert_allclose(PER_JOINT_ACTION_SCALE, expected, atol=1e-6)

if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
```

- [ ] **Step 2: Run test to verify it fails (module missing)**

Run: `cd /home/orange5plus/code/robodog_jeston && python3 -m pytest sim/test_obs_builder.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'obs_math'`.

- [ ] **Step 3: Implement `sim/obs_math.py`**

Create `sim/obs_math.py`:
```python
# SPDX-License-Identifier: GPL-3.0
# Pure obs/action math for the basemevius2 rough-terrain policy (232->12).
# Single source of truth for the convention; sim_inference_node imports this.
# Convention verified against basemevius2/source/robot_lab/.../rough_env_cfg.py
# and assets/mevius2.py.
import numpy as np

# Policy joint order: [FR, FL, BR, BL] x [collar, hip, knee].
POLICY_JOINT_NAMES = [
    "FR_collar_joint", "FR_hip_joint", "FR_knee_joint",
    "FL_collar_joint", "FL_hip_joint", "FL_knee_joint",
    "BR_collar_joint", "BR_hip_joint", "BR_knee_joint",
    "BL_collar_joint", "BL_hip_joint", "BL_knee_joint",
]

# default joint pos per leg (collar, hip, knee), identical across legs.
DEFAULT_ANGLE = np.array([0.0, 0.7, -1.2] * 4, dtype=np.float64)

# per-joint action scale (policy order): collar=0.125, hip=0.15, knee=0.30.
PER_JOINT_ACTION_SCALE = np.array([0.125, 0.15, 0.30] * 4, dtype=np.float64)

# joint limits (policy order), [lower, upper] per joint — from mevius2.urdf.
JOINT_LIMITS = np.array(
    [[-0.7854, 0.7854], [-1.0472, 2.6180], [-2.8508, -0.7812]] * 4,
    dtype=np.float64)

SCALE_ANG_VEL = 0.25
SCALE_DOF_POS = 1.0
SCALE_DOF_VEL = 0.05
SCALE_CMD = 1.0  # cmd [vx, vy, wz] scaled by 1.0 (NOT [2,2,0.25] like old mevius2)
HEIGHT_SCAN_SIZE = 187
CLIP_OBS = 100.0
CLIP_ACTION = 1.0  # new model: clip raw onnx output to [-1, 1] before scaling
# [vx_lo, vx_hi, vy_lo, vy_hi, wz_lo, wz_hi]
CLIP_CMD = np.array([-0.8, 0.8, -0.5, 0.5, -0.8, 0.8], dtype=np.float64)

GRAVITY_W = np.array([0.0, 0.0, -1.0], dtype=np.float64)


def quat_wxyz_to_rotmat(wxyz):
    """R with v_world = R @ v_body for w-first quaternion."""
    w, x, y, z = wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def build_obs(ang_vel, quat_wxyz, cmd, dof_pos, dof_vel, last_action,
              height_scan=None):
    """Build the 232-dim observation vector (float32), policy joint order.

    ang_vel: (3,) body-frame angular velocity.
    quat_wxyz: (4,) base orientation, w-first.
    cmd: (3,) [vx, vy, wz] command (already clipped to CLIP_CMD by caller).
    dof_pos, dof_vel, last_action: (12,) in POLICY_JOINT_NAMES order.
    height_scan: (187,) terrain scan clipped to [-1,1], or None -> zeros.
    """
    if height_scan is None:
        height_scan = np.zeros(HEIGHT_SCAN_SIZE, dtype=np.float64)
    gravity_b = quat_wxyz_to_rotmat(quat_wxyz).T @ GRAVITY_W
    obs = np.concatenate([
        np.asarray(ang_vel, dtype=np.float64) * SCALE_ANG_VEL,        # 3
        gravity_b,                                                    # 3
        np.asarray(cmd, dtype=np.float64) * SCALE_CMD,                # 3
        (np.asarray(dof_pos, dtype=np.float64) - DEFAULT_ANGLE) * SCALE_DOF_POS,  # 12
        np.asarray(dof_vel, dtype=np.float64) * SCALE_DOF_VEL,        # 12
        np.asarray(last_action, dtype=np.float64),                    # 12
        np.asarray(height_scan, dtype=np.float64),                    # 187
    ])
    return np.clip(obs.astype(np.float32), -CLIP_OBS, CLIP_OBS)


def action_to_targets(action):
    """Map raw 12-dim onnx output to joint position targets (policy order).

    target = default + clip(action, -1, 1) * per_joint_scale,
    then clamped to JOINT_LIMITS.
    """
    a = np.clip(np.asarray(action, dtype=np.float64), -CLIP_ACTION, CLIP_ACTION)
    tgt = DEFAULT_ANGLE + a * PER_JOINT_ACTION_SCALE
    tgt = np.clip(tgt, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    return tgt
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/orange5plus/code/robodog_jeston && python3 -m pytest sim/test_obs_builder.py -v`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
git -C /home/orange5plus/code/robodog_jeston add sim/obs_math.py sim/test_obs_builder.py
git -C /home/orange5plus/code/robodog_jeston commit -m "feat(sim): pure 232-dim obs/action math for rough-terrain policy + tests"
```

---

## Task 3: Rewrite `sim_inference_node.py` to the new convention

**Files:**
- Modify: `sim/sim_inference_node.py`

**Interfaces:**
- Consumes: `sim/obs_math.py` (Task 2): `POLICY_JOINT_NAMES, DEFAULT_ANGLE, CLIP_CMD, build_obs, action_to_targets`.
- Consumes: `src/inference/models/policy_21399.onnx` (Task 1).
- Produces: publishes `/joint_targets` Float32MultiArray (12) in policy order `[FR,FL,BR,BL]`. The mujoco bridge (Task 4) must use the same order.

- [ ] **Step 1: Rewrite the file**

Replace the entire contents of `sim/sim_inference_node.py` with:
```python
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
```

- [ ] **Step 2: Sanity-check the obs math against the real model offline**

Run (a quick non-ROS check that the model accepts the obs shape and produces a sane action):
```bash
cd /home/orange5plus/code/robodog_jeston && python3 -c "
import sys, os, numpy as np
sys.path.insert(0, 'sim')
from obs_math import build_obs, DEFAULT_ANGLE, action_to_targets
import onnxruntime as ort
s = ort.InferenceSession('src/inference/models/policy_21399.onnx', providers=['CPUExecutionProvider'])
obs = build_obs(np.zeros(3), np.array([1.,0,0,0]), np.zeros(3), DEFAULT_ANGLE.copy(), np.zeros(12), np.zeros(12))
print('obs shape', obs.shape)
a = s.run(None, {s.get_inputs()[0].name: obs.reshape(1,-1)})[0][0]
print('action', np.round(a,3))
print('targets', np.round(action_to_targets(a),3))
"
```
Expected: `obs shape (232,)`, a 12-element action array, and targets near DEFAULT_ANGLE (±scale). No exception.

- [ ] **Step 3: Commit**

```bash
git -C /home/orange5plus/code/robodog_jeston add sim/sim_inference_node.py
git -C /home/orange5plus/code/robodog_jeston commit -m "feat(sim): rewrite sim_inference_node for 232-dim rough-terrain policy"
```

---

## Task 4: Switch `mujoco_bridge.py` joint order to FR/FL/BR/BL

**Files:**
- Modify: `sim/mujoco_bridge.py:43-50` (the `JOINT_NAMES` list)

**Interfaces:**
- Produces: `/joint_states` and consumes `/joint_targets` now in policy order `[FR,FL,BR,BL]×[collar,hip,knee]`, matching `sim_inference_node.py` (Task 3). The mujoco internal body order is unchanged; mapping is by `mj_name2id`.

- [ ] **Step 1: Update JOINT_NAMES**

In `sim/mujoco_bridge.py`, replace the `JOINT_NAMES` block (lines 43-50):
```python
# Policy joint order: [FR, FL, BR, BL] x [collar, hip, knee].
# (mujoco defines them in FR,FL,BR,BL body order, so we map by name.)
JOINT_NAMES = [
    "FR_collar_joint", "FR_hip_joint", "FR_knee_joint",
    "FL_collar_joint", "FL_hip_joint", "FL_knee_joint",
    "BR_collar_joint", "BR_hip_joint", "BR_knee_joint",
    "BL_collar_joint", "BL_hip_joint", "BL_knee_joint",
]
```

- [ ] **Step 2: Update the header docstring comment that names the order**

In `sim/mujoco_bridge.py` lines 12-16, the topic comments say `[BL,BR,FL,FR]`. Update them to `[FR,FL,BR,BL]`. Specifically change:
```
#   publish  /joint_states       sensor_msgs/JointState   (12, [BL,BR,FL,FR])
```
to
```
#   publish  /joint_states       sensor_msgs/JointState   (12, [FR,FL,BR,BL])
```
and the `/joint_targets` line likewise (`[BL,BR,FL,FR]` → `[FR,FL,BR,BL]`).

- [ ] **Step 3: Verify all joint names exist in the mujoco model**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston && python3 -c "
import mujoco
m = mujoco.MjModel.from_xml_path('assets/mujoco/scene.xml')
names = ['FR_collar_joint','FR_hip_joint','FR_knee_joint','FL_collar_joint','FL_hip_joint','FL_knee_joint','BR_collar_joint','BR_hip_joint','BR_knee_joint','BL_collar_joint','BL_hip_joint','BL_knee_joint']
for n in names:
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    assert jid >= 0, f'joint missing: {n}'
    assert aid >= 0, f'actuator missing: {n}'
print('all 12 joints + actuators present')
"
```
Expected: `all 12 joints + actuators present`. (The mevius2 MJCF already defines these names; only the array order changes.)

- [ ] **Step 4: Commit**

```bash
git -C /home/orange5plus/code/robodog_jeston add sim/mujoco_bridge.py
git -C /home/orange5plus/code/robodog_jeston commit -m "feat(sim): switch mujoco bridge joint order to policy [FR,FL,BR,BL]"
```

---

## Task 5: Run the sim end-to-end and verify standing + walking

**Files:** none (verification only).

- [ ] **Step 1: Build is not required for the Python sim, but source ROS2**

```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
```

- [ ] **Step 2: Launch the sim**

Run: `./sim/run_sim.sh`
Expected output: `[1/2] 启动 mujoco bridge...` then `bridge 已上线`, then `[2/2] 启动 sim_inference_node...` then `inference 已上线: ... loaded ONNX ... in=[1, 232] out=[1, 12]`.

- [ ] **Step 3: Verify the robot stands (no fall) for ~5 s**

Run (in another shell, sourced):
```bash
sleep 5 && tail -n 5 /tmp/sim_bridge.log
```
Expected: bridge log shows `base xyz=(...,...,~0.45)` roughly stable z (the spawn height) and `targets=on`. The robot should NOT collapse to z≈0. If it falls immediately, the obs convention is wrong — re-check Task 2/3 against `rough_env_cfg.py` (cmd scale, dof_vel scale, last_action).

- [ ] **Step 4: Verify the robot walks forward on command**

Run:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
sleep 4
tail -n 3 /tmp/sim_bridge.log
```
Expected: base x coordinate increasing over the log lines (robot moving forward). Then stop:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
```

- [ ] **Step 5: Stop the sim**

Run: `./sim/run_sim.sh stop`

- [ ] **Step 6: Record a short note in the commit message of the next task (no commit here — this is a verification gate).**

If standing + walking both pass, proceed to Phase 2. If not, fix the sim before touching C++ (that is the whole point of sim-first).

---

## Task 6: De-submodule `src/inference` into the main repo

**Files:**
- Modify: `.gitmodules`
- Replace: `src/inference/` (submodule → plain dir copied from jeston-main)

**Interfaces:**
- Produces: `src/inference/` as a plain tracked directory in `robodog_jeston`, containing the jeston-main mevius2 C++ base (with `is_standing` obs source, `dof_sym_sign`, dynamic output-batch handling, `inference_mevius2.yaml`, 12-DOF `robot.yaml`) PLUS `models/policy_21399.onnx` (from Task 1). All later C++ tasks edit files in this now-plain directory.

- [ ] **Step 1: Deinit the submodule**

```bash
cd /home/orange5plus/code/robodog_jeston
git submodule deinit -f src/inference
```

- [ ] **Step 2: Remove the submodule from git index and config**

```bash
git rm -f src/inference
rm -rf .git/modules/src/inference
```
Then edit `.gitmodules` to delete the entire `[submodule "inference"]` block (the 3 lines `path = src/inference` / `url = ...` and the header).

- [ ] **Step 3: Replace the directory with the jeston-main mevius2 C++ base**

```bash
rm -rf src/inference
cp -a /home/orange5plus/code/robodog_jeston-main/src/inference src/inference
# Re-stage the new model (the jeston-main copy has the OLD models; ensure 21399 is present)
cp /home/orange5plus/code/basemevius2/onnx_script/policy_21399.onnx src/inference/models/policy_21399.onnx
# Remove any stray .git inside the copied dir (jeston-main is not a repo, but be safe)
rm -rf src/inference/.git
```

- [ ] **Step 4: Verify the expected files are present**

Run:
```bash
ls src/inference/config/inference_mevius2.yaml src/inference/src/obs_manager.cpp src/inference/src/inference_node.cpp src/inference/models/policy_21399.onnx
grep -c "is_standing" src/inference/src/obs_manager.cpp
grep -c "dof_sym_sign_" src/inference/src/inference_node.cpp
```
Expected: all paths print; `grep -c "is_standing"` ≥ 1; `grep -c "dof_sym_sign_"` ≥ 1 (confirms the mevius2 base was copied, not the clean upstream submodule).

- [ ] **Step 5: Commit the de-submodule + ported base**

```bash
git add .gitmodules src/inference
git commit -m "refactor(inference): de-submodule src/inference, port jeston-main mevius2 C++ base

Brings src/inference into the main repo as a plain directory (was a
submodule of Roboparty/roboparty_inference). The jeston-main snapshot's
mevius2 additions are now tracked here: is_standing obs source,
dof_sym_sign, dynamic onnx output-batch handling, inference_mevius2.yaml,
12-DoF robot.yaml. Also stages policy_21399.onnx.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Task 7: Verify the ported C++ base builds

**Files:** none (verification only).

- [ ] **Step 1: Build the inference package**

```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select roboparty_inference
```
Expected: build succeeds, `Finished <<< roboparty_inference`. If it fails on missing onnxruntime tarballs, check `src/inference/thirdparty/` has the aarch64/x64 1.21.0 tarballs (copied from jeston-main in Task 6).

- [ ] **Step 2: Source and confirm the node is registered**

```bash
source install/setup.bash
ros2 pkg executables roboparty_inference
```
Expected: `roboparty_inference inference_node` listed.

- [ ] **Step 3: (No commit — gate passed.)** Proceed to Phase 4.

---

## Task 8: Add `height_scan` obs source to the C++ node

**Files:**
- Modify: `src/inference/src/inference_node.hpp` (add method decl)
- Modify: `src/inference/src/obs_manager.cpp` (register + impl)
- Modify: `src/inference/src/ros_interface.cpp` (declare `height_scan_size`)

**Interfaces:**
- Produces: a registered obs source `height_scan` that fills its segment with zeros (size driven by the layout string `height_scan:187`). Consumed by `inference_mevius2_rough.yaml` (Task 9). The obs_manager registry already pre-sizes `segment` to the layout count, so the impl just zero-fills.

- [ ] **Step 1: Add the method declaration to the header**

In `src/inference/src/inference_node.hpp`, find the line `void get_is_standing_obs(std::vector<float>& segment);` (around line 267) and add immediately after it:
```cpp
    void get_height_scan_obs(std::vector<float>& segment);
```

- [ ] **Step 2: Register the source in obs_manager.cpp**

In `src/inference/src/obs_manager.cpp`, find the registry (the `static const std::vector<ObsSourceDefinition> definitions{ ... }` block, around lines 43-58). Add a new entry after the `is_standing` line (`{"is_standing", &InferenceNode::get_is_standing_obs},`):
```cpp
        {"height_scan", &InferenceNode::get_height_scan_obs},
```

- [ ] **Step 3: Implement `get_height_scan_obs` in obs_manager.cpp**

In `src/inference/src/obs_manager.cpp`, immediately after the `get_is_standing_obs` definition (ends around line 216), add:
```cpp
void InferenceNode::get_height_scan_obs(std::vector<float>& segment) {
    // Flat-ground / no-terrain-sensor deployment: fill with zeros.
    // The segment is pre-sized to the layout count (e.g. height_scan:187 -> 187).
    // README-sanctioned: only stair-climbing ability degrades.
    std::fill(segment.begin(), segment.end(), 0.0f);
}
```

- [ ] **Step 4: Declare the height_scan_size param (informational, for future raycast) in ros_interface.cpp**

In `src/inference/src/ros_interface.cpp`, after the `declare_parameter<float>("gravity_z_upper", -0.5);` line (around line 32), add:
```cpp
    this->declare_parameter<int>("height_scan_size", 0);
```
(Not strictly consumed yet — the zero-fill uses the layout-derived segment size — but declaring it keeps the param explicit and avoids ROS2 "undeclared parameter" warnings if a config sets it.)

- [ ] **Step 5: Build to verify it compiles**

```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select roboparty_inference
```
Expected: `Finished <<< roboparty_inference`.

- [ ] **Step 6: Commit**

```bash
git add src/inference/src/inference_node.hpp src/inference/src/obs_manager.cpp src/inference/src/ros_interface.cpp
git commit -m "feat(inference): add height_scan obs source (zero-fill for flat ground)"
```

---

## Task 9: Per-joint action scale in the C++ node

**Files:**
- Modify: `src/inference/src/inference_node.hpp` (member type change)
- Modify: `src/inference/src/inference_node.cpp` (action mapping)
- Modify: `src/inference/src/ros_interface.cpp` (load as array, backward-compat scalar)

**Interfaces:**
- Produces: `action_scale_` becomes `std::vector<float>` (length `joint_num_`). The action mapping at `inference_node.cpp:316` changes from `* action_scale_` to `* action_scale_vec_[i]`. Config may give a scalar (broadcast) or an array (per-joint). Consumed by `inference_mevius2_rough.yaml` (Task 10) which supplies a 12-element array `[0.125,0.15,0.30]×4`.

- [ ] **Step 1: Change the member declaration in the header**

In `src/inference/src/inference_node.hpp`, find the line (around line 209):
```cpp
    float action_scale_, clip_actions_;
```
Split it into:
```cpp
    std::vector<float> action_scale_;  // per-joint (broadcast if scalar in config)
    float clip_actions_;
```

- [ ] **Step 2: Update the config loading in ros_interface.cpp**

In `src/inference/src/ros_interface.cpp`, find the block that loads `action_scale` (around line 57: `this->get_parameter("action_scale", action_scale_);`). Replace that single line with:
```cpp
    // action_scale: accept a scalar (broadcast to all joints) or a per-joint array.
    {
        rclcpp::Parameter as_param;
        if (this->get_parameter("action_scale", as_param)) {
            if (as_param.get_type() == rclcpp::PARAMETER_DOUBLE
                || as_param.get_type() == rclcpp::PARAMETER_INTEGER) {
                float s = static_cast<float>(as_param.as_double());
                action_scale_.assign(static_cast<size_t>(joint_num_), s);
            } else if (as_param.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY
                       || as_param.get_type() == rclcpp::PARAMETER_INTEGER_ARRAY) {
                auto vec = as_param.as_double_array();
                action_scale_.resize(vec.size());
                for (size_t i = 0; i < vec.size(); ++i)
                    action_scale_[i] = static_cast<float>(vec[i]);
            } else {
                action_scale_.assign(static_cast<size_t>(joint_num_), 0.3f);
            }
        } else {
            action_scale_.assign(static_cast<size_t>(joint_num_), 0.3f);
        }
    }
```
Also update the `declare_parameter` at line 25 from `declare_parameter<float>("action_scale", 0.3);` to declare it allowing both types. The simplest backward-compatible approach is to keep the float declaration but ALSO handle the array via the get_parameter type check above. **However**, ROS2 will reject setting a `PARAMETER_DOUBLE_ARRAY` into a parameter declared as `float`. So change line 25 to:
```cpp
    this->declare_parameter("action_scale", rclcpp::ParameterValue(0.3));
```
(undeclared-type declaration accepts any type the config provides.)

Also update the log line around `ros_interface.cpp:165` (`RCLCPP_INFO(... "action_scale: %f", action_scale_);`) — since it is now a vector, replace with:
```cpp
    print_vector<float>("action_scale", action_scale_);
```
(`print_vector` already exists and is used for `dof_sym_sign` at line 171.)

- [ ] **Step 3: Update the action mapping in inference_node.cpp**

In `src/inference/src/inference_node.cpp`, find line 316:
```cpp
                    act_[usd2urdf_[i]] = policy.ctx->output_buffer[i] * dof_sym_sign_[i] * action_scale_ + joint_default_angle_[usd2urdf_[i]];
```
Replace with:
```cpp
                    act_[usd2urdf_[i]] = policy.ctx->output_buffer[i] * dof_sym_sign_[i] * action_scale_[i] + joint_default_angle_[usd2urdf_[i]];
```
(`action_scale_[i]` is indexed by policy index `i`, matching how `dof_sym_sign_[i]` is indexed. `clip_actions_` clamp on the line above (315) is unchanged and will be set to 1.0 in the config.)

- [ ] **Step 4: Build to verify it compiles**

```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select roboparty_inference
```
Expected: `Finished <<< roboparty_inference`. If the humanoid `inference.yaml` (scalar `action_scale: 0.25`) is loaded, the broadcast branch handles it — verify no other code path indexes `action_scale_` as a scalar. Run `grep -n "action_scale_" src/inference/src/*.cpp src/inference/src/*.hpp` to confirm only the lines touched reference it.

- [ ] **Step 5: Commit**

```bash
git add src/inference/src/inference_node.hpp src/inference/src/inference_node.cpp src/inference/src/ros_interface.cpp
git commit -m "feat(inference): per-joint action scale (scalar broadcast + array support)"
```

---

## Task 10: Author `inference_mevius2_rough.yaml`

**Files:**
- Create: `src/inference/config/inference_mevius2_rough.yaml`

**Interfaces:**
- Consumes: `height_scan` obs source (Task 8), per-joint `action_scale` array (Task 9), `last_action` obs source (ported base), `policy_21399.onnx` (Task 1).
- Produces: the config the launch file (Task 11) loads. Obs total = 3+3+3+12+12+12+187 = 232, matching the model input.

- [ ] **Step 1: Create the config**

Create `src/inference/config/inference_mevius2_rough.yaml`:
```yaml
# basemevius2 rough-terrain quadruped policy config (12 DoF, 50Hz policy / 200Hz PD).
# Obs layout total = 3+3+3+12+12+12+187 = 232, matching policy_21399.onnx input.
# Convention from basemevius2/source/robot_lab/.../rough_env_cfg.py + assets/mevius2.py:
#   - no obs normalization (actor_obs_normalization=False)
#   - cmd scaled by 1.0 (NOT [2,2,0.25] like old mevius2)
#   - dof_sym_sign all +1 (new model has NO L/R symmetry sign)
#   - per-joint action scale: collar=0.125, hip=0.15, knee=0.30
#   - action clip [-1,1] (clip_actions=1.0), then default + clip*per_joint_scale
#   - height_scan = 187 zeros (flat ground; height_scan obs source zero-fills)
inference_node:
    ros__parameters:
        model_names: ["policy_21399.onnx"]
        obs_layouts:
          - "ang_vel:3, gravity_b:3, cmd_vel:3, dof_pos:12, dof_vel:12, last_action:12, height_scan:187"
        frame_stacks: [1]
        obs_stack_orders: ["frame_major"]
        act_alpha: 1.0
        joint_num: 12
        decimation: 4
        intra_threads: 1
        dt: 0.005
        obs_scales_lin_vel: 1.0
        obs_scales_ang_vel: 0.25
        obs_scales_dof_pos: 1.0
        obs_scales_dof_vel: 0.05
        obs_scales_gravity_b: 1.0
        clip_observations: 100.0
        # Per-joint action scale (policy order [FR,FL,BR,BL] x [collar,hip,knee]).
        action_scale: [0.125, 0.15, 0.30, 0.125, 0.15, 0.30, 0.125, 0.15, 0.30, 0.125, 0.15, 0.30]
        clip_actions: 1.0
        # New model: no L/R symmetry sign (all +1).
        dof_sym_sign: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        # usd2urdf: policy[i] -> motor index. Motor array order is [BL,BR,FL,FR]
        # (robot.yaml motor_id list), policy order is [FR,FL,BR,BL].
        # FR->9,10,11  FL->6,7,8  BR->3,4,5  BL->0,1,2.
        usd2urdf: [9, 10, 11, 6, 7, 8, 3, 4, 5, 0, 1, 2]
        clip_cmd:
            [-0.8, 0.8,
             -0.5, 0.5,
             -0.8, 0.8]
        # joint_default_angle in MOTOR order [BL,BR,FL,FR] x [collar,hip,knee]
        # (indexed by usd2urdf_[i] in the action mapping). All legs identical.
        joint_default_angle:
            [0.0, 0.7, -1.2,
             0.0, 0.7, -1.2,
             0.0, 0.7, -1.2,
             0.0, 0.7, -1.2]
        # Joint limits from mevius2.urdf (identical across all 4 legs), motor order.
        joint_limits:
            [-0.7854, 0.7854,
             -1.0472, 2.6180,
             -2.8508, -0.7812,
             -0.7854, 0.7854,
             -1.0472, 2.6180,
             -2.8508, -0.7812,
             -0.7854, 0.7854,
             -1.0472, 2.6180,
             -2.8508, -0.7812,
             -0.7854, 0.7854,
             -1.0472, 2.6180,
             -2.8508, -0.7812]
        gravity_z_upper: 1.0
        height_scan_size: 187
```
Note on `gravity_z_upper: 1.0`: the rough-terrain policy is allowed to be in任意姿态 during rough traversal; a tight fall-shutdown would misfire. The old humanoid config used -0.5; the old mevius2 used -0.5. Setting 1.0 effectively disables the gravity-based shutdown (gravity_b.z ranges in [-1,1], never > 1.0), which is safest for a sim/first-deploy rough policy. Real-hardware tuning is a later, user-supervised step.

- [ ] **Step 2: Verify the obs layout sums to 232 and matches the model**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston && python3 -c "
layout='ang_vel:3, gravity_b:3, cmd_vel:3, dof_pos:12, dof_vel:12, last_action:12, height_scan:187'
total=sum(int(p.split(':')[1]) for p in layout.split(','))
print('obs total', total)
assert total==232, total
import onnx
m=onnx.load('src/inference/models/policy_21399.onnx')
inp=[d.dim_value for d in m.graph.input[0].type.tensor_type.shape.dim]
print('model input', inp)
assert inp==[1,232], inp
print('OK')
"
```
Expected: `obs total 232` / `model input [1, 232]` / `OK`.

- [ ] **Step 3: Commit**

```bash
git add src/inference/config/inference_mevius2_rough.yaml
git commit -m "feat(inference): add inference_mevius2_rough.yaml for policy_21399 (232-dim)"
```

---

## Task 11: Switch the launch default to the rough config

**Files:**
- Modify: `src/inference/launch/inference.launch.py:15`

**Interfaces:**
- Consumes: `inference_mevius2_rough.yaml` (Task 10).

- [ ] **Step 1: Update the config filename in the launch file**

In `src/inference/launch/inference.launch.py`, change line 15 from `"inference_mevius2.yaml",` to:
```python
            "inference_mevius2_rough.yaml",
```

- [ ] **Step 2: Rebuild and confirm the config is installed**

```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select roboparty_inference
source install/setup.bash
ls $(ros2 pkg prefix roboparty_inference)/share/roboparty_inference/config/inference_mevius2_rough.yaml
```
Expected: the path prints (the config is installed into the share dir).

- [ ] **Step 3: Commit**

```bash
git add src/inference/launch/inference.launch.py
git commit -m "feat(inference): default launch to inference_mevius2_rough.yaml"
```

---

## Task 12: Verify the C++ node loads the model and runs one tick (no hardware)

**Files:** none (verification only).

- [ ] **Step 1: Launch the node without motors/IMU connected and watch startup**

```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 8 ros2 launch roboparty_inference inference.launch.py 2>&1 | tee /tmp/cpp_infer_launch.log
```
Expected: log shows the ONNX loaded (`model_names: policy_21399.onnx`), the action_scale vector printed as 12 values `[0.125, 0.15, 0.3, ...]`, `dof_sym_sign` all 1.0, and NO `ONNX input size mismatch` exception. It will likely then fail to open CAN/IMU devices (no hardware) — that is expected and NOT a failure of this task. The success criterion is: model loads, no obs-size throw.

- [ ] **Step 2: Confirm no input-size mismatch**

Run: `grep -i "mismatch\|input size\|Exception in inference" /tmp/cpp_infer_launch.log`
Expected: no matches (or only motor/IMU open errors, not obs-size). If "ONNX input size mismatch" appears, the config obs layout does not sum to 232 — re-check Task 10.

- [ ] **Step 3: (No commit — final gate.)**

---

## Self-Review

**1. Spec coverage:**
- Python sim 232-dim obs + per-joint action + FR/FL/BR/BL order → Tasks 2, 3, 4 ✓
- Sim verification (stand + walk) → Task 5 ✓
- De-submodule → Task 6 ✓
- Port jeston-main mevius2 C++ base → Task 6 (copy) + Task 7 (build) ✓
- Per-joint action scale (C++) → Task 9 ✓
- height_scan zero-fill obs source → Task 8 ✓
- last_action obs source → already in ported base (Task 6), used in config (Task 10) ✓
- New config inference_mevius2_rough.yaml → Task 10 ✓
- Model file staged → Task 1 ✓
- Launch default → Task 11 ✓
- C++ model-load verification → Task 12 ✓
- usd2urdf [9,10,11,6,7,8,3,4,5,0,1,2] → Task 10 ✓
- action clip via clip_actions=1.0 → Task 10 (reuses existing clamp at inference_node.cpp:315) ✓

**2. Placeholder scan:** No TBD/TODO. All code blocks are complete. The one "handle edge cases"-shaped step (Task 9 Step 2 type dispatch) gives full code.

**3. Type consistency:**
- `action_scale_` is `std::vector<float>` in Task 9 Step 1; indexed `action_scale_[i]` in Step 3; loaded as array in Step 2. Consistent.
- `get_height_scan_obs(std::vector<float>& segment)` declared (Task 8 Step 1), registered (Step 2), defined (Step 3). Consistent.
- `build_obs` / `action_to_targets` signatures in Task 2 match usage in Task 3. Consistent.
- `POLICY_JOINT_NAMES` (Task 2) matches `JOINT_NAMES` in Task 4 and the comment in Task 10. Consistent.
- `usd2urdf` semantics: `act_[usd2urdf_[i]]` means `usd2urdf[i]` = motor index for policy index i — matches the [9,10,11,...] mapping in Task 10. Consistent.

No gaps found. Plan is complete.
