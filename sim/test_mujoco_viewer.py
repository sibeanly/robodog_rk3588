#!/usr/bin/env python3
"""Minimal mujoco viewer test using the OFFICIAL mujoco.viewer (launch_passive).
More stable than the third-party mujoco-python-viewer on Jetson.
Run: /usr/bin/python3 sim/test_mujoco_viewer.py
"""
import os
import time
import mujoco
import mujoco.viewer

SCENE = os.path.join(os.path.dirname(__file__), os.pardir, "assets", "mujoco", "scene.xml")
SCENE = os.path.abspath(SCENE)

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

# launch_passive returns a manager; the viewer runs in its own thread.
with mujoco.viewer.launch_passive(model, data) as viewer:
    # track base_link
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = bid
    print("viewer launched. rendering for 5s. close the window to exit early.")
    start = time.monotonic()
    while viewer.is_running() and (time.monotonic() - start) < 5.0:
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)
print("done")
