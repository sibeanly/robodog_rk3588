#!/usr/bin/env python3
# PD-hold-only: load standing keyframe, hold targets=default, step, log base tilt.
import numpy as np, mujoco
m = mujoco.MjModel.from_xml_path('assets/mujoco/scene.xml'); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
JN = ["FR_collar_joint","FR_hip_joint","FR_knee_joint","FL_collar_joint","FL_hip_joint","FL_knee_joint","BR_collar_joint","BR_hip_joint","BR_knee_joint","BL_collar_joint","BL_hip_joint","BL_knee_joint"]
qa=[]; qva=[]; aid=[]
for n in JN:
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    qa.append(m.jnt_qposadr[j]); qva.append(m.jnt_dofadr[j])
    aid.append(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n))
qa = np.array(qa); qva = np.array(qva); aid = np.array(aid)
default = np.array([0, 0.7, -1.2]*4)
KP=50.0; KD=2.0
print("PD-HOLD test: hold joints at default for 600 steps @ dt=%.4f" % m.opt.timestep)
for i in range(600):
    qpos = d.qpos[qa]; qvel = d.qvel[qva]
    tau = KP*(default-qpos) + KD*(-qvel)
    d.ctrl[aid] = tau
    mujoco.mj_step(m, d)
    if i % 100 == 0 or i == 599:
        z = d.qpos[2]; qw = d.qpos[3]
        tilt_deg = np.degrees(2*np.arccos(np.clip(abs(qw), 0, 1)))
        print(f"  step {i:3d}: base z={z:.3f} qw={qw:+.3f} tilt~{tilt_deg:.1f}deg  joint_err_max={np.abs(qpos-default).max():.3f}")
