"""LEAP Hand v2 MuJoCo simulation environment.

Standalone hand environment for dexterous manipulation tasks.
"""

from dataclasses import dataclass

import gymnasium as gym
import mujoco
import numpy as np

# Simplified LEAP Hand v2 MJCF model
# 4 fingers (index, middle, ring, thumb), 4 DOF each = 16 DOF
LEAP_HAND_XML = """
<mujoco model="leap_hand_v2">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <default>
    <joint damping="0.1" armature="0.01"/>
    <geom condim="4" friction="1.5 0.5 0.01" solref="0.01 1" solimp="0.95 0.95 0.01"/>
  </default>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.9" rgb2="0.7 0.7 0.7"
             width="256" height="256"/>
    <material name="grid_mat" texture="grid" texrepeat="4 4"/>
    <material name="hand_mat" rgba="0.85 0.75 0.65 1"/>
    <material name="finger_mat" rgba="0.8 0.7 0.6 1"/>
    <material name="object_mat" rgba="0.2 0.6 0.9 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 0.5" dir="0 0 -1"/>
    <geom type="plane" size="0.2 0.2 0.01" material="grid_mat"/>

    <!-- Object to manipulate -->
    <body name="cube" pos="0 0 0.025">
      <joint type="free"/>
      <geom type="box" size="0.02 0.02 0.02" material="object_mat" mass="0.05"/>
    </body>

    <!-- LEAP Hand palm (fixed in space for standalone testing) -->
    <body name="palm" pos="0 0 0.15" euler="3.14 0 0">
      <geom type="box" size="0.04 0.05 0.01" material="hand_mat" mass="0.2"/>

      <!-- Index finger -->
      <body name="index_base" pos="0.025 -0.035 0.01">
        <joint name="index_abd" type="hinge" axis="1 0 0" range="-0.3 0.3"/>
        <geom type="capsule" size="0.008" fromto="0 0 0 0 0 0.04" material="finger_mat"/>
        <body name="index_mid" pos="0 0 0.04">
          <joint name="index_mcp" type="hinge" axis="1 0 0" range="0 1.57"/>
          <geom type="capsule" size="0.007" fromto="0 0 0 0 0 0.03" material="finger_mat"/>
          <body name="index_dist" pos="0 0 0.03">
            <joint name="index_pip" type="hinge" axis="1 0 0" range="0 1.57"/>
            <geom type="capsule" size="0.006" fromto="0 0 0 0 0 0.025" material="finger_mat"/>
            <body name="index_tip" pos="0 0 0.025">
              <joint name="index_dip" type="hinge" axis="1 0 0" range="0 1.2"/>
              <geom type="capsule" size="0.006" fromto="0 0 0 0 0 0.02" material="finger_mat"/>
            </body>
          </body>
        </body>
      </body>

      <!-- Middle finger -->
      <body name="middle_base" pos="0.025 -0.012 0.01">
        <joint name="middle_abd" type="hinge" axis="1 0 0" range="-0.3 0.3"/>
        <geom type="capsule" size="0.008" fromto="0 0 0 0 0 0.045" material="finger_mat"/>
        <body name="middle_mid" pos="0 0 0.045">
          <joint name="middle_mcp" type="hinge" axis="1 0 0" range="0 1.57"/>
          <geom type="capsule" size="0.007" fromto="0 0 0 0 0 0.032" material="finger_mat"/>
          <body name="middle_dist" pos="0 0 0.032">
            <joint name="middle_pip" type="hinge" axis="1 0 0" range="0 1.57"/>
            <geom type="capsule" size="0.006" fromto="0 0 0 0 0 0.027" material="finger_mat"/>
            <body name="middle_tip" pos="0 0 0.027">
              <joint name="middle_dip" type="hinge" axis="1 0 0" range="0 1.2"/>
              <geom type="capsule" size="0.006" fromto="0 0 0 0 0 0.02" material="finger_mat"/>
            </body>
          </body>
        </body>
      </body>

      <!-- Ring finger -->
      <body name="ring_base" pos="0.025 0.012 0.01">
        <joint name="ring_abd" type="hinge" axis="1 0 0" range="-0.3 0.3"/>
        <geom type="capsule" size="0.008" fromto="0 0 0 0 0 0.04" material="finger_mat"/>
        <body name="ring_mid" pos="0 0 0.04">
          <joint name="ring_mcp" type="hinge" axis="1 0 0" range="0 1.57"/>
          <geom type="capsule" size="0.007" fromto="0 0 0 0 0 0.028" material="finger_mat"/>
          <body name="ring_dist" pos="0 0 0.028">
            <joint name="ring_pip" type="hinge" axis="1 0 0" range="0 1.57"/>
            <geom type="capsule" size="0.006" fromto="0 0 0 0 0 0.023" material="finger_mat"/>
            <body name="ring_tip" pos="0 0 0.023">
              <joint name="ring_dip" type="hinge" axis="1 0 0" range="0 1.2"/>
              <geom type="capsule" size="0.006" fromto="0 0 0 0 0 0.018" material="finger_mat"/>
            </body>
          </body>
        </body>
      </body>

      <!-- Thumb -->
      <body name="thumb_base" pos="-0.02 -0.04 0.0">
        <joint name="thumb_abd" type="hinge" axis="0 0 1" range="-0.5 1.0"/>
        <geom type="capsule" size="0.009" fromto="0 0 0 0.02 0 0.02" material="finger_mat"/>
        <body name="thumb_mid" pos="0.02 0 0.02">
          <joint name="thumb_mcp" type="hinge" axis="1 0 0" range="0 1.2"/>
          <geom type="capsule" size="0.008" fromto="0 0 0 0.01 0 0.03" material="finger_mat"/>
          <body name="thumb_dist" pos="0.01 0 0.03">
            <joint name="thumb_pip" type="hinge" axis="1 0 0" range="0 1.2"/>
            <geom type="capsule" size="0.007" fromto="0 0 0 0.005 0 0.025" material="finger_mat"/>
            <body name="thumb_tip_body" pos="0.005 0 0.025">
              <joint name="thumb_dip" type="hinge" axis="1 0 0" range="0 1.0"/>
              <geom type="capsule" size="0.007" fromto="0 0 0 0 0 0.018" material="finger_mat"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Thumb: 4 DOF -->
    <position name="act_thumb_abd" joint="thumb_abd" kp="2"/>
    <position name="act_thumb_mcp" joint="thumb_mcp" kp="2"/>
    <position name="act_thumb_pip" joint="thumb_pip" kp="1.5"/>
    <position name="act_thumb_dip" joint="thumb_dip" kp="1"/>
    <!-- Index: 4 DOF -->
    <position name="act_index_abd" joint="index_abd" kp="2"/>
    <position name="act_index_mcp" joint="index_mcp" kp="2"/>
    <position name="act_index_pip" joint="index_pip" kp="1.5"/>
    <position name="act_index_dip" joint="index_dip" kp="1"/>
    <!-- Middle: 4 DOF -->
    <position name="act_middle_abd" joint="middle_abd" kp="2"/>
    <position name="act_middle_mcp" joint="middle_mcp" kp="2"/>
    <position name="act_middle_pip" joint="middle_pip" kp="1.5"/>
    <position name="act_middle_dip" joint="middle_dip" kp="1"/>
    <!-- Ring: 4 DOF -->
    <position name="act_ring_abd" joint="ring_abd" kp="2"/>
    <position name="act_ring_mcp" joint="ring_mcp" kp="2"/>
    <position name="act_ring_pip" joint="ring_pip" kp="1.5"/>
    <position name="act_ring_dip" joint="ring_dip" kp="1"/>
  </actuator>
</mujoco>
"""

HAND_JOINT_NAMES = [
    "thumb_abd", "thumb_mcp", "thumb_pip", "thumb_dip",
    "index_abd", "index_mcp", "index_pip", "index_dip",
    "middle_abd", "middle_mcp", "middle_pip", "middle_dip",
    "ring_abd", "ring_mcp", "ring_pip", "ring_dip",
]


@dataclass
class HandEnvConfig:
    """Configuration for hand simulation environment."""

    render_mode: str | None = "human"
    control_freq: int = 20
    sim_steps_per_control: int = 10
    image_width: int = 640
    image_height: int = 480
    max_episode_steps: int = 500
    task: str = "cube_rotate"  # "cube_rotate" or "grasp"


class HandSimEnv(gym.Env):
    """Gymnasium environment for LEAP Hand in MuJoCo.

    Observation: 16 joint positions + cube position (3) + cube orientation (4) = 23
    Action: 16 joint position targets
    """

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, config: HandEnvConfig | None = None) -> None:
        super().__init__()
        self.config = config or HandEnvConfig()

        self.model = mujoco.MjModel.from_xml_string(LEAP_HAND_XML)
        self.data = mujoco.MjData(self.model)

        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(23,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(16,), dtype=np.float32
        )

        self._step_count = 0
        self._renderer = None
        self._cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self._initial_cube_quat = np.array([1.0, 0, 0, 0])

        # Cache actuator ranges
        self._ctrl_ranges = self.model.actuator_ctrlrange.copy()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._step_count = 0

        # Store initial cube orientation for rotation reward
        cube_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "cube")
        qpos_addr = self.model.jnt_qposadr[cube_joint_id]
        self._initial_cube_quat = self.data.qpos[qpos_addr + 3 : qpos_addr + 7].copy()

        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        # Map action [-1, 1] to actuator ranges
        for i in range(self.model.nu):
            low, high = self._ctrl_ranges[i]
            self.data.ctrl[i] = low + (action[i] + 1) * 0.5 * (high - low)

        for _ in range(self.config.sim_steps_per_control):
            mujoco.mj_step(self.model, self.data)

        self._step_count += 1
        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = False
        truncated = self._step_count >= self.config.max_episode_steps

        return obs, reward, terminated, truncated, {}

    def _get_obs(self) -> np.ndarray:
        joint_pos = np.array([self.data.joint(name).qpos[0] for name in HAND_JOINT_NAMES])
        cube_pos = self.data.xpos[self._cube_body_id].copy()
        cube_quat = self.data.xquat[self._cube_body_id].copy()
        return np.concatenate([joint_pos, cube_pos, cube_quat]).astype(np.float32)

    def _compute_reward(self) -> float:
        cube_pos = self.data.xpos[self._cube_body_id]
        cube_quat = self.data.xquat[self._cube_body_id]

        if self.config.task == "cube_rotate":
            # Reward rotation around z-axis while keeping cube in hand
            quat_diff = self._quat_distance(cube_quat, self._initial_cube_quat)
            height_bonus = max(0, cube_pos[2] - 0.1)  # keep cube elevated
            drop_penalty = -1.0 if cube_pos[2] < 0.02 else 0.0
            return float(quat_diff * 0.5 + height_bonus * 2.0 + drop_penalty)

        # Default: grasp reward
        height = cube_pos[2]
        return float(max(0, height - 0.05) * 10.0)

    @staticmethod
    def _quat_distance(q1: np.ndarray, q2: np.ndarray) -> float:
        """Compute angular distance between two quaternions."""
        dot = np.abs(np.dot(q1, q2))
        dot = np.clip(dot, 0, 1)
        return float(2.0 * np.arccos(dot))

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Set joint position targets directly (for teleop)."""
        self.data.ctrl[:] = positions[:16]
        for _ in range(self.config.sim_steps_per_control):
            mujoco.mj_step(self.model, self.data)

    def render(self) -> np.ndarray | None:
        if self.config.render_mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(
                    self.model, self.config.image_height, self.config.image_width
                )
            self._renderer.update_scene(self.data)
            return self._renderer.render()
        return None

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
