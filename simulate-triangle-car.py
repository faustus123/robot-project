import time

import mujoco
import mujoco.viewer

# Load the Mujoco XML model for the Mecanum wheel robot
xml_content = """
<?xml version="1.0" encoding="utf-8"?>
<mujoco model="triangular_base">
	<compiler autolimits="true"/>

    <!-- set some defaults for units and lighting -->
    <compiler angle="degree" meshdir="../stl_exports/"/>

	<option timestep="0.001"/>

    <!-- import our stl files -->
    <asset>
        <mesh name="baseplate" file="baseplate_robot.stl" />
        <mesh name="wheel" file="wheel_for_mujoco.stl" />
    </asset>



    <!-- define our robot model -->
    <worldbody>
        <!-- set up a light pointing down on the robot -->
        <light directional="true" pos="-0.5 0.5 3" dir="0 0 -1" />

        <!-- add a floor so we don't stare off into the abyss -->
        <geom name="floor" pos="0 0 0" size="50 50 50" type="plane" rgba="1 0.83 0.61 0.5"/>

        <!-- start building our model -->
        <body name="base" pos="0 0 0">
		    <joint name="joint_base" type="free" axis="0 0 0" stiffness="0" damping="0" frictionloss="0"/>
            <geom name="baseplate_robot" type="mesh" mesh="baseplate" pos="0 0 1"/>
            <inertial pos="0 0 1" mass="500" diaginertia="1 1 1"/>

				<!-- Attach wheels to the main body -->
				<body name="wheel_1_body" pos=".24766 0 1.02">
				    <joint name="joint0" type="hinge" axis="-1 0 0" stiffness="0" frictionloss=".01"/>
				    <geom type="mesh" mesh="wheel" euler="0 0 90"/>
				    <inertial pos="0 0 0" mass="0.75" diaginertia="1 1 1"/>
				</body>

				<body name="wheel_2_body" pos="-.275 .265 1.02">
				    <joint name="joint1" type="hinge" axis="1 -2 0" stiffness="0" frictionloss=".01"/>
				    <geom type="mesh" mesh="wheel" euler="0 0 26.5"/>
				    <inertial pos="0 0 0" mass="0.75" diaginertia="1 1 1"/>
				</body>

				<body name="wheel_3_body" pos="-.275 -.265 1.02">
				    <joint name="joint2" type="hinge" axis="1 2 0" stiffness="0" frictionloss=".01"/>
				    <geom type="mesh" mesh="wheel" euler="0 0 -26.5"/>
				    <inertial pos="0 0 0" mass="0.75" diaginertia="1 1 1"/>
				</body>
        </body>
    </worldbody>

    <!-- attach actuators to joints -->
    <actuator>
        <motor name="joint0_motor" joint="joint0" ctrlrange="-15 15.0" ctrllimited="true"/>
        <damper name="damper0" joint="joint0" kv="10" ctrlrange="0 1"/>
        <motor name="joint1_motor" joint="joint1" ctrlrange="-15 15.0" ctrllimited="true"/>
        <damper name="damper1" joint="joint1" kv="10" ctrlrange="0 1"/>
        <motor name="joint2_motor" joint="joint2" ctrlrange="-15 15.0" ctrllimited="true"/>
        <damper name="damper2" joint="joint2" kv="10" ctrlrange="0 1"/>
    </actuator>

  <sensor>
    <actuatorfrc name="joint0_motor" actuator="joint0_motor"/>
    <actuatorfrc name="joint1_motor" actuator="joint1_motor"/>
    <actuatorfrc name="joint2_motor" actuator="joint2_motor"/>
    <actuatorfrc name="damper0" actuator="damper0"/>
    <actuatorfrc name="damper1" actuator="damper1"/>
    <actuatorfrc name="damper2" actuator="damper2"/>
    <jointactuatorfrc name="joint0" joint="joint0"/>
    <jointactuatorfrc name="joint1" joint="joint1"/>
    <jointactuatorfrc name="joint2" joint="joint2"/>
  </sensor>

</mujoco>
"""



m = mujoco.MjModel.from_xml_path('xml-triangle-car.xml')
d = mujoco.MjData(m)



with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  # Set an initial velocity for a joint (e.g., assuming joint index 0)
  initial_velocity = 2  # set your desired initial velocity
  d.qvel[1] = initial_velocity
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

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
