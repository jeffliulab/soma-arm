"""Domain randomization for sim-to-real transfer.

Randomizes object properties, environment conditions, and robot parameters.
"""

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class RandomizationConfig:
    """Configuration for domain randomization ranges."""

    # Object properties
    object_pos_range: tuple[float, float] = (-0.08, 0.08)  # offset from default
    object_size_range: tuple[float, float] = (0.01, 0.025)  # half-size
    object_mass_range: tuple[float, float] = (0.01, 0.1)
    object_friction_range: tuple[float, float] = (0.5, 2.0)

    # Robot properties
    joint_damping_range: tuple[float, float] = (0.3, 1.5)  # multiplier
    actuator_kp_range: tuple[float, float] = (0.7, 1.3)  # multiplier

    # Visual properties
    light_pos_range: tuple[float, float] = (-0.3, 0.3)  # offset

    # Which randomizations to enable
    randomize_object: bool = True
    randomize_robot: bool = True
    randomize_visual: bool = True


class DomainRandomizer:
    """Applies domain randomization to MuJoCo environments.

    Modifies model parameters before each episode reset to improve
    sim-to-real transfer of trained policies.
    """

    def __init__(
        self,
        config: RandomizationConfig | None = None,
        seed: int | None = None,
    ) -> None:
        self.config = config or RandomizationConfig()
        self.rng = np.random.default_rng(seed)

        # Store original model parameters for resetting
        self._original_params: dict = {}

    def save_defaults(self, model: mujoco.MjModel) -> None:
        """Save original model parameters for resetting between episodes."""
        self._original_params = {
            "geom_size": model.geom_size.copy(),
            "body_mass": model.body_mass.copy(),
            "geom_friction": model.geom_friction.copy(),
            "dof_damping": model.dof_damping.copy(),
            "actuator_gainprm": model.actuator_gainprm.copy(),
            "light_pos": model.light_pos.copy(),
        }

    def restore_defaults(self, model: mujoco.MjModel) -> None:
        """Restore original model parameters."""
        for key, value in self._original_params.items():
            getattr(model, key)[:] = value

    def randomize(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Apply randomization to the model and data.

        Call this after mj_resetData but before the first mj_step.
        """
        if not self._original_params:
            self.save_defaults(model)

        # Restore defaults first
        self.restore_defaults(model)

        if self.config.randomize_object:
            self._randomize_object(model, data)
        if self.config.randomize_robot:
            self._randomize_robot(model)
        if self.config.randomize_visual:
            self._randomize_visual(model)

    def _randomize_object(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Randomize object position, size, mass, friction."""
        obj_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
        if obj_body_id < 0:
            return

        # Randomize position offset
        obj_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "object")
        if obj_joint_id >= 0:
            qpos_addr = model.jnt_qposadr[obj_joint_id]
            lo, hi = self.config.object_pos_range
            data.qpos[qpos_addr] += self.rng.uniform(lo, hi)
            data.qpos[qpos_addr + 1] += self.rng.uniform(lo, hi)

        # Randomize size
        geom_start = model.body_geomadr[obj_body_id]
        geom_count = model.body_geomnum[obj_body_id]
        for g in range(geom_start, geom_start + geom_count):
            lo, hi = self.config.object_size_range
            new_size = self.rng.uniform(lo, hi)
            model.geom_size[g, :3] = new_size

        # Randomize mass
        lo, hi = self.config.object_mass_range
        model.body_mass[obj_body_id] = self.rng.uniform(lo, hi)

        # Randomize friction
        for g in range(geom_start, geom_start + geom_count):
            lo, hi = self.config.object_friction_range
            model.geom_friction[g, 0] = self.rng.uniform(lo, hi)

    def _randomize_robot(self, model: mujoco.MjModel) -> None:
        """Randomize robot joint damping and actuator gains."""
        # Joint damping
        lo, hi = self.config.joint_damping_range
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name and name.startswith("joint"):
                multiplier = self.rng.uniform(lo, hi)
                model.dof_damping[model.jnt_dofadr[i]] = (
                    self._original_params["dof_damping"][model.jnt_dofadr[i]] * multiplier
                )

        # Actuator gains
        lo, hi = self.config.actuator_kp_range
        for i in range(model.nu):
            multiplier = self.rng.uniform(lo, hi)
            model.actuator_gainprm[i, 0] = (
                self._original_params["actuator_gainprm"][i, 0] * multiplier
            )

    def _randomize_visual(self, model: mujoco.MjModel) -> None:
        """Randomize lighting."""
        lo, hi = self.config.light_pos_range
        for i in range(model.nlight):
            offset = self.rng.uniform(lo, hi, size=3)
            model.light_pos[i] = self._original_params["light_pos"][i] + offset
