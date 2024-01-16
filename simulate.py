import dm_control.mujoco 
import mujoco.viewer

m = dm_control.mujoco.MjModel.from_xml_path(“example.xml”)
d = dm_control.mujoco.MjData(m)
viewer.launch_passive(m, d)
viewer.close()