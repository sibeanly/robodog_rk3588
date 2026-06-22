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
