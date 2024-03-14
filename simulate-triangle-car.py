import time

import mujoco
import mujoco.viewer
import math

m = mujoco.MjModel.from_xml_path('xml-triangle-car.xml')
d = mujoco.MjData(m)



with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  # Set an initial velocity for a joint (e.g., assuming joint index 0)
  initial_velocity = 1  # set your desired initial velocity
  print(m.actuator_user)
  #m.jnt_dofadr = initial_velocity
  #m.jnt_dofadr = -initial_velocity
  while viewer.is_running() and time.time() - start < 120:
    print('pos', d.qpos[0])
    step_start = time.time()
    print("Current time:", time.time() - start)
    if time.time() - start < 20:
        d.ctrl[0] = -initial_velocity*d.time
        d.ctrl[1] = initial_velocity*d.time
        d.ctrl[2] = -initial_velocity*d.time
    if 20 <= (time.time() - start) < 40:
        d.ctrl[0] = initial_velocity*d.time
        d.ctrl[1] = -initial_velocity*d.time
        d.ctrl[2] = initial_velocity*d.time
    if 40 <= (time.time() - start) < 60:
        d.ctrl[0] = initial_velocity*d.time
        d.ctrl[1] = initial_velocity*d.time
        d.ctrl[2] = -initial_velocity*d.time
    if 60 <= (time.time() - start) < 120:
        d.ctrl[0] = -initial_velocity*d.time
        d.ctrl[1] = initial_velocity*d.time
        d.ctrl[2] = initial_velocity*d.time

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
