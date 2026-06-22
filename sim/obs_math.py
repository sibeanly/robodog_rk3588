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
HEIGHT_SCAN_OFFSET = 0.5  # Isaac mdp.height_scan: base_z - ground_z - 0.5, clip [-1,1]
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
              height_scan=None, base_z=None, ground_z=0.0):
    """Build the 232-dim observation vector (float32), policy joint order.

    ang_vel: (3,) body-frame angular velocity.
    quat_wxyz: (4,) base orientation, w-first.
    cmd: (3,) [vx, vy, wz] command (already clipped to CLIP_CMD by caller).
    dof_pos, dof_vel, last_action: (12,) in POLICY_JOINT_NAMES order.
    height_scan: (187,) terrain scan already clipped to [-1,1]. If None,
        computed from base_z as (base_z - ground_z - HEIGHT_SCAN_OFFSET)
        clipped to [-1,1] -- matches Isaac mdp.height_scan on flat ground
        (offset=0.5). Pass base_z for the Isaac-consistent flat-ground value.
    base_z: base link world z, used only when height_scan is None.
    ground_z: ground plane z (0 for flat ground).
    """
    if height_scan is None:
        if base_z is None:
            height_scan = np.zeros(HEIGHT_SCAN_SIZE, dtype=np.float64)
        else:
            val = float(base_z) - float(ground_z) - HEIGHT_SCAN_OFFSET
            val = min(1.0, max(-1.0, val))  # clip [-1,1] like Isaac
            height_scan = np.full(HEIGHT_SCAN_SIZE, val, dtype=np.float64)
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
