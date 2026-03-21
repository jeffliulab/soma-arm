"""SO-ARM100 MuJoCo simulation environment.

Gymnasium-compatible environment for 6-DOF robot arm simulation.
"""

from dataclasses import dataclass

import gymnasium as gym
import mujoco
import numpy as np

# SO-ARM100 model in MJCF XML
SO_ARM100_XML = """
<mujoco model="so_arm100">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <default>
    <joint damping="0.5" armature="0.1"/>
    <geom condim="4" friction="1 0.5 0.01"/>
  </default>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.8 0.8 0.8" rgb2="0.6 0.6 0.6"
             width="512" height="512"/>
    <material name="grid_mat" texture="grid" texrepeat="8 8" reflectance="0.1"/>
    <material name="arm_blue" rgba="0.2 0.5 0.8 1"/>
    <material name="arm_green" rgba="0.2 0.8 0.5 1"/>
    <material name="arm_orange" rgba="0.8 0.5 0.2 1"/>
    <material name="gripper_gray" rgba="0.7 0.7 0.7 1"/>
    <material name="target_red" rgba="1 0.2 0.2 0.5"/>
    <material name="object_red" rgba="0.9 0.1 0.1 1"/>
  </asset>

  <worldbody>
    <!-- Ground -->
    <geom type="plane" size="0.5 0.5 0.01" material="grid_mat"/>
    <light pos="0 0 1.5" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <light pos="0.5 0.5 1" dir="-0.5 -0.5 -1" diffuse="0.3 0.3 0.3"/>

    <!-- Table -->
    <body name="table" pos="0.25 0 0.15">
      <geom type="box" size="0.2 0.25 0.005" rgba="0.6 0.4 0.2 1" mass="10"/>
    </body>

    <!-- Target marker (visual only) -->
    <body name="target" pos="0.25 0 0.18" mocap="true">
      <geom type="sphere" size="0.015" material="target_red" contype="0" conaffinity="0"/>
    </body>

    <!-- Graspable object -->
    <body name="object" pos="0.25 0.05 0.18">
      <joint type="free"/>
      <geom type="box" size="0.015 0.015 0.015" material="object_red" mass="0.02"
            solref="0.01 1" solimp="0.95 0.95 0.01"/>
    </body>

    <!-- SO-ARM100 Robot -->
    <body name="base_link" pos="0 0 0">
      <geom type="cylinder" size="0.04 0.015" rgba="0.3 0.3 0.3 1" mass="0.5"/>

      <!-- Joint 1: Base rotation -->
      <body name="link1" pos="0 0 0.03">
        <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14 3.14" damping="1.0"/>
        <geom type="cylinder" size="0.025 0.04" material="arm_blue" mass="0.15"/>

        <!-- Joint 2: Shoulder -->
        <body name="link2" pos="0 0 0.04">
          <joint name="joint2" type="hinge" axis="0 1 0" range="-1.57 1.57" damping="0.8"/>
          <geom type="capsule" size="0.02" fromto="0 0 0 0 0 0.13" material="arm_green"
                mass="0.12"/>

          <!-- Joint 3: Elbow -->
          <body name="link3" pos="0 0 0.13">
            <joint name="joint3" type="hinge" axis="0 1 0" range="-2.36 2.36" damping="0.6"/>
            <geom type="capsule" size="0.018" fromto="0 0 0 0 0 0.1" material="arm_orange"
                  mass="0.1"/>

            <!-- Joint 4: Wrist pitch -->
            <body name="link4" pos="0 0 0.1">
              <joint name="joint4" type="hinge" axis="0 1 0" range="-1.57 1.57" damping="0.4"/>
              <geom type="capsule" size="0.015" fromto="0 0 0 0 0 0.05" material="arm_blue"
                    mass="0.06"/>

              <!-- Joint 5: Wrist roll -->
              <body name="link5" pos="0 0 0.05">
                <joint name="joint5" type="hinge" axis="0 0 1" range="-3.14 3.14" damping="0.3"/>
                <geom type="cylinder" size="0.015 0.008" material="arm_green" mass="0.04"/>

                <!-- Gripper: parallel jaw -->
                <body name="gripper_base" pos="0 0 0.01">
                  <!-- Left finger -->
                  <body name="finger_left" pos="0 0 0">
                    <joint name="gripper_left" type="slide" axis="0 1 0" range="0 0.025"
                           damping="0.2"/>
                    <geom type="box" size="0.004 0.002 0.02" pos="0 0.006 0.02"
                          material="gripper_gray" mass="0.01"/>
                  </body>
                  <!-- Right finger -->
                  <body name="finger_right" pos="0 0 0">
                    <joint name="gripper_right" type="slide" axis="0 -1 0" range="0 0.025"
                           damping="0.2"/>
                    <geom type="box" size="0.004 0.002 0.02" pos="0 -0.006 0.02"
                          material="gripper_gray" mass="0.01"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <!-- Camera for observation -->
  <visual>
    <global offwidth="640" offheight="480"/>
  </visual>

  <actuator>
    <position name="act_joint1" joint="joint1" kp="40" ctrlrange="-3.14 3.14"/>
    <position name="act_joint2" joint="joint2" kp="40" ctrlrange="-1.57 1.57"/>
    <position name="act_joint3" joint="joint3" kp="30" ctrlrange="-2.36 2.36"/>
    <position name="act_joint4" joint="joint4" kp="20" ctrlrange="-1.57 1.57"/>
    <position name="act_joint5" joint="joint5" kp="15" ctrlrange="-3.14 3.14"/>
    <position name="act_gripper_l" joint="gripper_left" kp="10" ctrlrange="0 0.025"/>
    <position name="act_gripper_r" joint="gripper_right" kp="10" ctrlrange="0 0.025"/>
  </actuator>

  <sensor>
    <jointpos name="jpos1" joint="joint1"/>
    <jointpos name="jpos2" joint="joint2"/>
    <jointpos name="jpos3" joint="joint3"/>
    <jointpos name="jpos4" joint="joint4"/>
    <jointpos name="jpos5" joint="joint5"/>
    <jointpos name="jpos_gl" joint="gripper_left"/>
    <jointpos name="jpos_gr" joint="gripper_right"/>
  </sensor>
</mujoco>
"""

# Joint name mapping
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]
GRIPPER_JOINT_NAMES = ["gripper_left", "gripper_right"]


@dataclass
class ArmEnvConfig:
    """Configuration for arm simulation environment."""

    render_mode: str | None = "human"  # "human", "rgb_array", or None
    control_freq: int = 20  # Hz
    sim_steps_per_control: int = 10  # physics steps per control step
    image_width: int = 640
    image_height: int = 480
    max_episode_steps: int = 500
    reward_type: str = "dense"  # "dense" or "sparse"


class ArmSimEnv(gym.Env):
    """Gymnasium environment for SO-ARM100 in MuJoCo.

    Observation: joint positions (7) + object position (3) + target position (3)
    Action: joint position targets (6) where action[5] controls both gripper fingers
    """

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, config: ArmEnvConfig | None = None) -> None:
        super().__init__()
        self.config = config or ArmEnvConfig()

        self.model = mujoco.MjModel.from_xml_string(SO_ARM100_XML)
        self.data = mujoco.MjData(self.model)

        # Observation: 7 joint pos + 3 object pos + 3 target pos = 13
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(13,), dtype=np.float32
        )

        # Action: 5 arm joints + 1 gripper (mapped to 2 fingers) = 6
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32
        )

        self._step_count = 0
        self._viewer = None
        self._renderer = None

        # Get body and joint IDs
        self._object_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "object")
        self._target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target"
        )

        # Actuator control ranges
        self._ctrl_ranges = self.model.actuator_ctrlrange.copy()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        # Randomize object position on table
        if self.np_random is not None:
            obj_x = self.np_random.uniform(0.15, 0.35)
            obj_y = self.np_random.uniform(-0.1, 0.1)
            obj_joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, "object"
            )
            obj_qpos_addr = self.model.jnt_qposadr[obj_joint_id]
            self.data.qpos[obj_qpos_addr] = obj_x
            self.data.qpos[obj_qpos_addr + 1] = obj_y
            self.data.qpos[obj_qpos_addr + 2] = 0.18

        mujoco.mj_forward(self.model, self.data)
        self._step_count = 0
        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        # Map action [-1, 1] to actuator control ranges
        ctrl = np.zeros(self.model.nu)
        for i in range(5):  # 5 arm joints
            low, high = self._ctrl_ranges[i]
            ctrl[i] = low + (action[i] + 1) * 0.5 * (high - low)
        # Gripper: action[5] controls both fingers symmetrically
        gripper_val = (action[5] + 1) * 0.5 * 0.025  # map to [0, 0.025]
        ctrl[5] = gripper_val  # left finger
        ctrl[6] = gripper_val  # right finger

        # Apply control and simulate
        self.data.ctrl[:] = ctrl
        for _ in range(self.config.sim_steps_per_control):
            mujoco.mj_step(self.model, self.data)

        self._step_count += 1
        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = False
        truncated = self._step_count >= self.config.max_episode_steps

        return obs, reward, terminated, truncated, {}

    def _get_obs(self) -> np.ndarray:
        """Get observation: joint positions + object pos + target pos."""
        joint_pos = self.data.sensordata[:7].copy()  # 5 arm + 2 gripper
        object_pos = self.data.xpos[self._object_body_id].copy()
        target_pos = self.data.xpos[self._target_body_id].copy()
        return np.concatenate([joint_pos, object_pos, target_pos]).astype(np.float32)

    def _compute_reward(self) -> float:
        """Compute reward based on end-effector to object distance."""
        # Get end-effector position (link5)
        ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link5")
        ee_pos = self.data.xpos[ee_id]
        obj_pos = self.data.xpos[self._object_body_id]
        target_pos = self.data.xpos[self._target_body_id]

        if self.config.reward_type == "sparse":
            # Sparse: +1 if object is near target
            return 1.0 if np.linalg.norm(obj_pos - target_pos) < 0.03 else 0.0

        # Dense reward
        dist_ee_obj = np.linalg.norm(ee_pos - obj_pos)
        dist_obj_target = np.linalg.norm(obj_pos - target_pos)
        return float(-dist_ee_obj - 2.0 * dist_obj_target)

    def render(self) -> np.ndarray | None:
        """Render the environment."""
        if self.config.render_mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(
                    self.model, self.config.image_height, self.config.image_width
                )
            self._renderer.update_scene(self.data)
            return self._renderer.render()
        return None

    def render_image(self) -> np.ndarray:
        """Render and return RGB image regardless of render_mode."""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, self.config.image_height, self.config.image_width
            )
        self._renderer.update_scene(self.data)
        return self._renderer.render()

    def get_joint_positions(self) -> np.ndarray:
        """Get current 5 arm joint positions + 1 average gripper position."""
        arm_pos = self.data.sensordata[:5].copy()
        gripper_pos = np.mean(self.data.sensordata[5:7])
        return np.append(arm_pos, gripper_pos)

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Set arm joint position targets directly (for teleop).

        Args:
            positions: Array of 6 values (5 arm joints + 1 gripper).
        """
        ctrl = np.zeros(self.model.nu)
        ctrl[:5] = positions[:5]
        ctrl[5] = positions[5]  # gripper left
        ctrl[6] = positions[5]  # gripper right (symmetric)
        self.data.ctrl[:] = ctrl
        for _ in range(self.config.sim_steps_per_control):
            mujoco.mj_step(self.model, self.data)

    def set_target_position(self, pos: np.ndarray) -> None:
        """Set target marker position."""
        self.data.mocap_pos[0] = pos

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def launch_viewer(self) -> None:
        """Launch interactive MuJoCo viewer."""
        mujoco.viewer.launch_passive(self.model, self.data)
