#!/usr/bin/env python3
# Closed-loop test of the OLD mevius2 model (policy_mevius2.onnx, 34-dim) with
# the OLD convention (DOF_SYM, cmd scale [2,2,0.25], [BL,BR,FL,FR], action clip
# +-100). Control: does the old model walk stably on this machine + MJCF?
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import mujoco
import onnxruntime as ort

m = mujoco.MjModel.from_xml_path('assets/mujoco/scene.xml'); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
# OLD model joint order [BL,BR,FL,FR]x[collar,hip,knee]
JN = ["BL_collar_joint","BL_hip_joint","BL_knee_joint","BR_collar_joint","BR_hip_joint","BR_knee_joint",
      "FL_collar_joint","FL_hip_joint","FL_knee_joint","FR_collar_joint","FR_hip_joint","FR_knee_joint"]
qa=[];qva=[];aid=[]
for n in JN:
    j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n); qa.append(m.jnt_qposadr[j]); qva.append(m.jnt_dofadr[j])
    aid.append(mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,n))
qa=np.array(qa);qva=np.array(qva);aid=np.array(aid)
sid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SENSOR,'body_gyro_sensor'); gadr=m.sensor_adr[sid]
# OLD convention
DEFAULT=np.array([0.0,0.7,-1.2]*4)
DOF_SYM=np.array([1,1,1,-1,1,1,1,1,1,-1,1,1],float)
SCALE_ANG=0.25; CMD_SCALE=np.array([2.0,2.0,0.25]); SCALE_DV=0.05; ACT_SCALE=0.2
GRAV=np.array([0,0,-1.])
def q2R(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
sess=ort.InferenceSession('src/inference/models/policy_mevius2.onnx',providers=['CPUExecutionProvider']); inp=sess.get_inputs()[0].name
KP=50;KD=2;DEC=4
# OLD keyframe: spawn tucked? No -- use standing for fair test. Set qpos to standing.
d.qpos[qa]=DEFAULT; d.qpos[2]=0.482; mujoco.mj_forward(m,d)
cmd=np.array([0.3,0,0]); target=DEFAULT.copy()
print('OLD model (34-dim) closed-loop, walk vx=0.3, standing spawn')
for i in range(2000):
    qpos=d.qpos[qa]; qvel=d.qvel[qva]
    d.ctrl[aid]=KP*(target-qpos)+KD*(-qvel); mujoco.mj_step(m,d)
    if i%DEC==0:
        quat=d.qpos[3:7].copy(); ang=d.sensordata[gadr:gadr+3].copy()
        gb=q2R(quat).T@GRAV
        is_stand=1.0 if np.linalg.norm(cmd)<0.03 else 0.0
        obs=np.concatenate([ang*SCALE_ANG,gb,cmd*CMD_SCALE,(d.qpos[qa]-DEFAULT)*DOF_SYM,d.qvel[qva]*SCALE_DV*DOF_SYM,[is_stand]]).astype(np.float32)
        obs=np.clip(obs,-100,100)
        a=sess.run(None,{inp:obs.reshape(1,-1)})[0][0]; a=np.clip(a,-100,100)
        target=a*DOF_SYM*ACT_SCALE+DEFAULT
    if i%400==0 or i==1999:
        z=d.qpos[2];qw=d.qpos[3];tilt=np.degrees(2*np.arccos(np.clip(abs(qw),0,1)))
        print(f'  step {i:4d}: x={d.qpos[0]:+.2f} z={z:.3f} tilt={tilt:4.1f}')
