import mujoco
import mujoco.viewer as viewer
import os

model_path = "mujoco_menagerie/universal_robots_ur5e/scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
#mujoco.mj_step(model, data)
viewer.launch(model, data)