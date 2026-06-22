#!/usr/bin/env python3
"""Unit tests for sim/obs_math.py — the new 232-dim rough-terrain convention."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from obs_math import (build_obs, action_to_targets, DEFAULT_ANGLE,
                      PER_JOINT_ACTION_SCALE, JOINT_LIMITS, HEIGHT_SCAN_SIZE,
                      SCALE_ANG_VEL, SCALE_DOF_VEL, CLIP_ACTION, CLIP_OBS)

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

def test_extreme_action_stays_within_limits():
    """A large action still yields targets within JOINT_LIMITS.

    NOTE: under CLIP_ACTION=1.0 the action clip in action_to_targets already
    keeps targets inside JOINT_LIMITS, so the downstream
    ``np.clip(tgt, JOINT_LIMITS[:,0], JOINT_LIMITS[:,1])`` is an UNREACHABLE
    safety net through the public API. This test documents that safety
    guarantee, not the clamp's execution -- it would still pass if the
    JOINT_LIMITS clamp line were deleted (the CLIP_ACTION clamp already binds).
    """
    # huge negative knee action must not exceed lower limit
    act = np.zeros(12); act[2] = -100.0  # knee idx 2
    tgt = action_to_targets(act)
    assert tgt[2] >= JOINT_LIMITS[2][0] - 1e-6, (tgt[2], JOINT_LIMITS[2])

def test_per_joint_scale_values():
    # collar=0.125, hip=0.15, knee=0.30 per leg
    expected = np.array([0.125, 0.15, 0.30] * 4)
    np.testing.assert_allclose(PER_JOINT_ACTION_SCALE, expected, atol=1e-6)

def test_projected_gravity_nonidentity_quaternion():
    """projected_gravity (R.T @ [0,0,-1]) under a non-trivial quaternion.

    Uses a 90 deg rotation about the x-axis: wxyz = [cos(pi/4), sin(pi/4),0,0].
    For this quaternion, build_obs's gravity_b = R.T @ [0,0,-1] = [0,-1,0]. A
    transposed rotation matrix (R @ v instead of R.T @ v) would give a
    different result, so this test catches that classic bug -- the
    identity-quaternion test cannot (R @ v == R.T @ v at the identity).
    """
    q = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])
    obs = build_obs(np.zeros(3), q, np.zeros(3),
                    DEFAULT_ANGLE.copy(), np.zeros(12), np.zeros(12))
    np.testing.assert_allclose(obs[3:6], [0.0, -1.0, 0.0], atol=1e-6)

def test_clip_obs_applied():
    """Large observations are clipped to +/- CLIP_OBS (100.0)."""
    # ang_vel * SCALE_ANG_VEL = [250, -250, 125] -> clipped to [100,-100,100]
    ang_vel = np.array([1000.0, -1000.0, 500.0])
    obs = build_obs(ang_vel, np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3),
                    DEFAULT_ANGLE.copy(), np.zeros(12), np.zeros(12))
    np.testing.assert_allclose(obs[0:3],
                               [CLIP_OBS, -CLIP_OBS, CLIP_OBS], atol=1e-6)

def test_height_scan_nonzero_passed_through():
    """An explicit height_scan is placed in obs[45:232] (clipped to +/-100)."""
    height_scan = np.full(HEIGHT_SCAN_SIZE, 0.5)
    obs = build_obs(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3),
                    DEFAULT_ANGLE.copy(), np.zeros(12), np.zeros(12),
                    height_scan=height_scan)
    np.testing.assert_allclose(obs[45:232], 0.5, atol=1e-6)

def test_height_scan_from_base_z_isaac_convention():
    """height_scan defaults to base_z - 0.5 (Isaac mdp.height_scan, offset=0.5),
    clipped to [-1,1]. At standing base_z=0.482 -> ~-0.018 (matches Isaac truth)."""
    from obs_math import HEIGHT_SCAN_OFFSET
    obs = build_obs(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3),
                    DEFAULT_ANGLE.copy(), np.zeros(12), np.zeros(12),
                    base_z=0.482)
    expected = 0.482 - HEIGHT_SCAN_OFFSET  # -0.018
    np.testing.assert_allclose(obs[45:232], expected, atol=1e-3)

def test_height_scan_clipped_to_limits():
    """height_scan from base_z is clipped to [-1,1] like Isaac."""
    # base_z far above ground -> base_z-0.5 > 1 -> clipped to 1.0
    obs = build_obs(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3),
                    DEFAULT_ANGLE.copy(), np.zeros(12), np.zeros(12),
                    base_z=5.0)
    np.testing.assert_allclose(obs[45:232], 1.0, atol=1e-6)
    # base_z below ground -> clipped to -1.0
    obs2 = build_obs(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3),
                     DEFAULT_ANGLE.copy(), np.zeros(12), np.zeros(12),
                     base_z=-1.0)
    np.testing.assert_allclose(obs2[45:232], -1.0, atol=1e-6)

if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
