#!/usr/bin/env python3
# Closed-loop sim (no ROS): policy + PD, log obs drift + action + tilt to find the
# destabilizing field. Replicates sim_inference_node + mujoco_bridge math exactly.
import sys, os, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
import mujoco
from obs_math import (build_obs, action_to_targets, DEFAULT_ANGLE,
                      POLICY_JOINT_NAMES)
import onnxruntime as ort

m = mujoco.MjModel.from_xml_path('assets/mujoco/scene.xml'); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)

JN = POLICY_JOINT_NAMES
qa=[]; qva=[]; aid=[]
for n in JN:
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    qa.append(m.jnt_qposadr[j]); qva.append(m.jnt_dofadr[j])
    aid.append(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n))
qa = np.array(qa); qva = np.array(qva); aid = np.array(aid)
sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, 'body_gyro_sensor')
gadr = m.sensor_adr[sid]

sess = ort.InferenceSession('src/inference/models/policy_21399.onnx', providers=['CPUExecutionProvider'])
inp = sess.get_inputs()[0].name

KP=50.0; KD=2.0
DECIMATION=4  # policy every 4 sim steps (50Hz over 200Hz)
last_action = np.zeros(12)
cmd = np.zeros(3)
target = DEFAULT_ANGLE.copy()  # hold until first policy tick

print("CLOSED-LOOP: policy@50Hz PD@200Hz, starting from standing keyframe")
print(f"{'step':>4} {'baseZ':>6} {'tilt':>6} {'gb_z':>6} {'dofpos_err_max':>13} {'act_max':>7} {'tgt012':>22}")
for i in range(400):
    # PD every sim step
    qpos = d.qpos[qa]; qvel = d.qvel[qva]
    tau = KP*(target - qpos) + KD*(-qvel)
    d.ctrl[aid] = tau
    mujoco.mj_step(m, d)

    if i % DECIMATION == 0:
        # build obs (snapshot current state)
        quat = d.qpos[3:7].copy()
        ang_vel = d.sensordata[gadr:gadr+3].copy()
        dof_pos = d.qpos[qa].copy()
        dof_vel = d.qvel[qva].copy()
        obs = build_obs(ang_vel, quat, cmd, dof_pos, dof_vel, last_action)
        a = sess.run(None, {inp: obs.reshape(1, -1)})[0][0]
        last_action = np.asarray(a, dtype=np.float64).copy()
        target = action_to_targets(a)

    if i % 20 == 0 or i == 399:
        z = d.qpos[2]; qw = d.qpos[3]
        tilt = np.degrees(2*np.arccos(np.clip(abs(qw), 0, 1)))
        gb_z = obs[5] if i % DECIMATION == 0 else float('nan')
        dpe = np.abs(d.qpos[qa] - DEFAULT_ANGLE).max()
        am = np.abs(last_action).max()
        print(f"{i:4d} {z:6.3f} {tilt:6.1f} {gb_z:6.2f} {dpe:13.3f} {am:7.2f} {np.array2string(target[:3], precision=2)}")
