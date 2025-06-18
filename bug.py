from pathlib import Path
import mujoco
from mujoco import viewer
import numpy as np

XML = Path("minimal.xml")

m = mujoco.MjModel.from_xml_path(str(XML))
d = mujoco.MjData(m)

qpos = np.array([
    -0.92501388,
    2.09100776,
    0.12541064,
    0.8635626912279962,
    0.03723404002261786,
    0.49949960030342155,
    0.05807972003528059,
    -1.0229925155693018,
    1.7923672889805318,
    0.10109852616199241,
    0.7735493978030474,
    0.5458780557966236,
    -0.31494960409367523,
    0.06667251487291104
])
d.qpos[:] = qpos
mujoco.mj_forward(m, d)

gid_caps = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "collision1")
gid_tire = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "collision2")
dist = mujoco.mj_geomDistance(m, d, gid_caps, gid_tire, 20.0, None)
print("SDF:", dist)
