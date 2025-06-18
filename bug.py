import mujoco
import numpy as np
from mujoco import MjModel, viewer

qpos = np.array([
    -0.2818793240587496, 1.969741491731251, 0.4881231533478665,
     0.00036932915014248704, -0.04109323615311549, 0.07045013990717668, 0.996668444030162,
    -0.0029311687736780865, -0.728221641108788, -0.40298279437995105,
    -1.0370951931328092, 1.818205634372612, 0.3522531446109742,
    -0.729808409984083, -0.5085413379121554, -0.23616474549469885, 0.3911414134765636
])

model_path = "robot.xml"
model = MjModel.from_xml_path(model_path)
data  = mujoco.MjData(model)

data.qpos[:] = qpos
mujoco.mj_forward(model, data)

gid_caps = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "collision1")
gid_tire = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "collision2")

fromto = np.zeros(6)
dist = mujoco.mj_geomDistance(model, data, gid_caps, gid_tire, 20, fromto)
print("SDF:", dist)

print("Visualizing to verify that the sdf should be positive")
center = 0.5 * (data.geom_xpos[gid_caps] + data.geom_xpos[gid_tire])
cam_pos = center + np.array([0.0, -1.0, 1.0])
cam_lookat = center

with viewer.launch_passive(model, data) as v:
    v.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    v.cam.lookat[:] = cam_lookat
    v.cam.distance = np.linalg.norm(cam_pos - cam_lookat)
    v.cam.azimuth  = 90
    v.cam.elevation = -35
    while v.is_running():
        with v.lock():
            scn = v.user_scn
            scn.ngeom = 0
            def _add_point(pos, rgba):
                idx = scn.ngeom
                mujoco.mjv_initGeom(
                    scn.geoms[idx],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.02, 0, 0],
                    pos=pos,
                    mat=np.eye(3).flatten(),
                    rgba=rgba
                )
                scn.ngeom += 1
            _add_point(fromto[:3], [1, 0, 0, 1])
            _add_point(fromto[3:],   [0, 0, 1, 1])
        v.sync()    