"""Launch MuJoCo simulation environments for testing.

Usage:
    python scripts/run_sim.py arm          # SO-ARM100 arm with sinusoidal demo motion
    python scripts/run_sim.py hand         # LEAP Hand with grasp demo
    python scripts/run_sim.py arm --random # Arm with domain randomization
"""

import argparse

import mujoco
import mujoco.viewer
import numpy as np

from src.simulation.arm_env import SO_ARM100_XML, ArmEnvConfig, ArmSimEnv
from src.simulation.domain_randomization import DomainRandomizer, RandomizationConfig
from src.simulation.hand_env import LEAP_HAND_XML


def run_arm_demo(use_randomization: bool = False) -> None:
    """Run SO-ARM100 arm demo with sinusoidal motion."""
    model = mujoco.MjModel.from_xml_string(SO_ARM100_XML)
    data = mujoco.MjData(model)

    randomizer = None
    if use_randomization:
        randomizer = DomainRandomizer(RandomizationConfig())
        randomizer.save_defaults(model)
        randomizer.randomize(model, data)
        print("Domain randomization applied")

    step = 0

    def controller(m: mujoco.MjModel, d: mujoco.MjData) -> None:
        nonlocal step
        t = step * m.opt.timestep
        d.ctrl[0] = 0.5 * np.sin(0.5 * t)  # base
        d.ctrl[1] = 0.3 * np.sin(0.3 * t)  # shoulder
        d.ctrl[2] = 0.5 * np.sin(0.4 * t)  # elbow
        d.ctrl[3] = 0.3 * np.sin(0.6 * t)  # wrist pitch
        d.ctrl[4] = 0.5 * np.sin(0.8 * t)  # wrist roll
        d.ctrl[5] = 0.012 + 0.012 * np.sin(t)  # gripper L
        d.ctrl[6] = 0.012 + 0.012 * np.sin(t)  # gripper R
        step += 1

    print("Launching SO-ARM100 demo viewer... Press ESC to quit.")
    mujoco.viewer.launch(model, data, controller=controller)


def run_hand_demo() -> None:
    """Run LEAP Hand demo with grasp motion."""
    model = mujoco.MjModel.from_xml_string(LEAP_HAND_XML)
    data = mujoco.MjData(model)

    step = 0

    def controller(m: mujoco.MjModel, d: mujoco.MjData) -> None:
        nonlocal step
        t = step * m.opt.timestep
        # Grasp/release cycle
        grasp = 0.5 + 0.5 * np.sin(0.3 * t)
        for i in range(m.nu):
            low = m.actuator_ctrlrange[i, 0]
            high = m.actuator_ctrlrange[i, 1]
            d.ctrl[i] = low + grasp * (high - low)
        step += 1

    print("Launching LEAP Hand demo viewer... Press ESC to quit.")
    mujoco.viewer.launch(model, data, controller=controller)


def run_gym_arm_demo() -> None:
    """Run arm environment with random actions via Gymnasium API."""
    env = ArmSimEnv(ArmEnvConfig(render_mode="rgb_array"))
    obs, _ = env.reset()

    print("Running Gymnasium arm env with random actions...")
    total_reward = 0
    for step in range(200):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward
        if terminated or truncated:
            break

    print(f"  Steps: {step+1}, Total reward: {total_reward:.2f}")
    env.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Launch MuJoCo simulation")
    parser.add_argument("env", choices=["arm", "hand", "gym"], help="Environment to launch")
    parser.add_argument("--random", action="store_true", help="Apply domain randomization")
    args = parser.parse_args()

    if args.env == "arm":
        run_arm_demo(use_randomization=args.random)
    elif args.env == "hand":
        run_hand_demo()
    elif args.env == "gym":
        run_gym_arm_demo()


if __name__ == "__main__":
    main()
