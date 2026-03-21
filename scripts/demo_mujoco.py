"""MuJoCo demo: load a built-in model, render it, verify GPU acceleration.

Usage:
    python scripts/demo_mujoco.py

Press ESC in the viewer to quit.
"""

import mujoco
import mujoco.viewer
import numpy as np

# Simple 6-DOF arm model in MJCF XML for testing
ARM_XML = """
<mujoco model="simple_arm">
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <worldbody>
    <!-- Ground plane -->
    <geom type="plane" size="1 1 0.01" rgba="0.8 0.8 0.8 1"/>

    <!-- Table -->
    <body name="table" pos="0.4 0 0.3">
      <geom type="box" size="0.3 0.3 0.02" rgba="0.6 0.4 0.2 1"/>
    </body>

    <!-- Object on table -->
    <body name="object" pos="0.4 0.1 0.34">
      <joint type="free"/>
      <geom type="box" size="0.02 0.02 0.02" rgba="1 0 0 1" mass="0.05"/>
    </body>

    <!-- Robot arm base -->
    <body name="base" pos="0 0 0">
      <geom type="cylinder" size="0.05 0.02" rgba="0.3 0.3 0.3 1"/>

      <!-- Joint 1: Base rotation (yaw) -->
      <body name="link1" pos="0 0 0.04">
        <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180"/>
        <geom type="cylinder" size="0.03 0.06" rgba="0.2 0.5 0.8 1"/>

        <!-- Joint 2: Shoulder (pitch) -->
        <body name="link2" pos="0 0 0.06">
          <joint name="joint2" type="hinge" axis="0 1 0" range="-90 90"/>
          <geom type="capsule" size="0.025" fromto="0 0 0 0 0 0.15" rgba="0.2 0.8 0.5 1"/>

          <!-- Joint 3: Elbow (pitch) -->
          <body name="link3" pos="0 0 0.15">
            <joint name="joint3" type="hinge" axis="0 1 0" range="-135 135"/>
            <geom type="capsule" size="0.02" fromto="0 0 0 0 0 0.12" rgba="0.8 0.5 0.2 1"/>

            <!-- Joint 4: Wrist pitch -->
            <body name="link4" pos="0 0 0.12">
              <joint name="joint4" type="hinge" axis="0 1 0" range="-90 90"/>
              <geom type="capsule" size="0.015" fromto="0 0 0 0 0 0.06" rgba="0.8 0.2 0.5 1"/>

              <!-- Joint 5: Wrist roll -->
              <body name="link5" pos="0 0 0.06">
                <joint name="joint5" type="hinge" axis="0 0 1" range="-180 180"/>
                <geom type="cylinder" size="0.018 0.01" rgba="0.5 0.2 0.8 1"/>

                <!-- Gripper (simplified) -->
                <body name="gripper_base" pos="0 0 0.01">
                  <joint name="gripper" type="slide" axis="1 0 0" range="0 0.03"/>
                  <geom type="box" size="0.005 0.015 0.02" pos="0.01 0 0.02" rgba="0.7 0.7 0.7 1"/>
                  <geom type="box" size="0.005 0.015 0.02" pos="-0.01 0 0.02" rgba="0.7 0.7 0.7 1"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="act1" joint="joint1" kp="50"/>
    <position name="act2" joint="joint2" kp="50"/>
    <position name="act3" joint="joint3" kp="50"/>
    <position name="act4" joint="joint4" kp="30"/>
    <position name="act5" joint="joint5" kp="20"/>
    <position name="act_gripper" joint="gripper" kp="10"/>
  </actuator>
</mujoco>
"""


def main() -> None:
    print("Loading MuJoCo model...")
    model = mujoco.MjModel.from_xml_string(ARM_XML)
    data = mujoco.MjData(model)

    print(f"MuJoCo version: {mujoco.__version__}")
    print(f"Model joints: {model.njnt}")
    print(f"Model actuators: {model.nu}")
    print(f"Timestep: {model.opt.timestep}")

    # Benchmark: run 1000 steps and measure speed
    print("\nBenchmarking physics simulation (1000 steps)...")
    import time

    start = time.time()
    for _ in range(1000):
        mujoco.mj_step(model, data)
    elapsed = time.time() - start
    print(f"1000 steps in {elapsed:.3f}s ({1000/elapsed:.0f} steps/sec)")

    # Reset and demo with simple sinusoidal motion
    mujoco.mj_resetData(model, data)

    print("\nLaunching interactive viewer...")
    print("Press ESC to quit. The arm will perform a simple demo motion.")

    step = 0

    def controller(model: mujoco.MjModel, data: mujoco.MjData) -> None:
        nonlocal step
        t = step * model.opt.timestep
        # Simple sinusoidal motion for each joint
        data.ctrl[0] = 0.5 * np.sin(0.5 * t)  # base rotation
        data.ctrl[1] = 0.3 * np.sin(0.3 * t)  # shoulder
        data.ctrl[2] = 0.5 * np.sin(0.4 * t)  # elbow
        data.ctrl[3] = 0.3 * np.sin(0.6 * t)  # wrist pitch
        data.ctrl[4] = 0.5 * np.sin(0.8 * t)  # wrist roll
        data.ctrl[5] = 0.015 + 0.015 * np.sin(t)  # gripper
        step += 1

    mujoco.viewer.launch(model, data, controller=controller)
    print("Viewer closed.")


if __name__ == "__main__":
    main()
