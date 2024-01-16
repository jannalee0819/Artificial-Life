import dm_control.mujoco
import mujoco.viewer
import time
import numpy


m = dm_control.mujoco.MjModel.from_xml_path("example.xml")
d = dm_control.mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    actuators = m.nu
    wiggle = numpy.array([6, 2, 2, -1])
    d.ctrl[:actuators] = wiggle

    for i in range(10000):
        if viewer.is_running():
            if i%40 == 0:
                wiggle *= -1
            d.ctrl[:actuators] = wiggle 
            mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(0.01)
        else:
            break

    viewer.close()