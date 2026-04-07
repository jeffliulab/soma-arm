# SmartRobotArm

**Language-driven smart robot arm — open-source.**

A fixed tabletop manipulator that listens to natural language ("put the red toy in the blue bin"), grounds the language to objects in the scene with open-vocabulary perception, dispatches imitation-learned (ACT) skill primitives, and validates each step with vision before reporting back. Built around the [ANIMA](https://github.com/jeffliulab/ANIMA_O1) cognitive framework (LLM-as-Parser, py_trees behavior tree, test-and-check validation, DIARC-inspired) and the LeRobot ecosystem.

> **Status:** pre-alpha, 8-week sprint in progress. APIs, package names, and launch files will change without notice. See [开发进度与待办事项.md](开发进度与待办事项.md) for the current sprint plan.

## What it is

```
   user: "put the red toy in the blue bin"
                       │
                       ▼
       ┌─────────────────────────────┐
       │  ANIMA LLM Parser           │  Claude API → structured TaskSpec
       └──────────────┬──────────────┘
                      ▼
       ┌─────────────────────────────┐
       │  Grounding DINO + SAM2      │  text → world coordinates
       └──────────────┬──────────────┘
                      ▼
       ┌─────────────────────────────┐
       │  py_trees behavior tree     │  task decomposition + skill dispatch
       └──────────────┬──────────────┘
                      ▼
       ┌─────────────────────────────┐
       │  ACT-trained primitives     │  pick / place / push, learned from
       │  (LeRobot, real-robot data) │  ~30 demos each via PDP gamepad teleop
       └──────────────┬──────────────┘
                      ▼
       ┌─────────────────────────────┐
       │  Test-and-check validator   │  vision-based success verification
       └──────────────┬──────────────┘
                      ▼
                  success / retry / report
```

## Hardware

- **Arm**: [Waveshare RoArm-M2-S](https://www.waveshare.com/roarm-m2-s.htm) — 4-DOF, ~28 cm reach, ~500 g payload, USB serial
- **Camera**: Logitech C922 Pro Stream — USB UVC, fixed overhead at ~60 cm
- **Teleop input**: PDP Wired Controller for Xbox — XInput / `xpad` driver
- **Compute**: Desktop with RTX 4090, Windows 11 + WSL2 Ubuntu 22.04, CUDA passthrough
- **Workspace**: ~30×40 cm tabletop with solid backdrop and small plush toys / bins
- **No mobile base, no Pi 5, no LiDAR** in v1 — see [开发进度与待办事项.md](开发进度与待办事项.md) for the rationale

## Repository layout

```
SmartRobotArm/
├── README.md
├── LICENSE                    # Apache-2.0
├── CLAUDE.md                  # Project instructions for Claude Code
├── 开发进度与待办事项.md       # 8-week sprint plan (single source of truth)
├── docs/
│   ├── DEVELOPMENT.md         # Step-by-step Chinese dev guide
│   └── FAQ-硬件与仿真.md       # Hardware + sim FAQ
└── src/                       # ROS 2 workspace
    ├── anima_node/            # ANIMA cognitive layer ROS 2 wrapper
    │   ├── anima_node/        #   - LLM parser node
    │   │   └── nodes/         #   - skill executor node
    │   ├── config/skills.yaml #   - skill registry with affordance checks
    │   └── launch/
    ├── arm_description/       # RoArm-M2-S URDF + Gazebo verification scene
    │   ├── urdf/
    │   ├── worlds/
    │   └── launch/            #   - display.launch.py (RViz)
    │                          #   - gazebo.launch.py (URDF + ANIMA dry-run)
    └── arm_bringup/           # Top-level launch (description + ANIMA)
        └── launch/full_system.launch.py
```

ROS 2 packages added later in the sprint:
- `arm_perception/` (Week 2): Grounding DINO + SAM2 + pixel→world reprojection
- `arm_manipulation/` (Week 3): MoveIt2 config + hardcoded primitive safety net

## Quick start

> Assumes WSL2 + Ubuntu 22.04 + ROS 2 Humble. See [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for the full setup.

```bash
# Build
cd ~/SmartRobotArm
colcon build --symlink-install
source install/setup.bash

# Sanity check: launch URDF in RViz
ros2 launch arm_description display.launch.py

# Or: launch arm + ANIMA mock parser
ros2 launch arm_bringup full_system.launch.py llm_backend:=mock

# Send a test instruction
ros2 topic pub /user_instruction std_msgs/String "data: 'put the red toy in the blue bin'"
```

## Project lineage

- **Cognitive layer**: built on [ANIMA_O1](https://github.com/jeffliulab/ANIMA_O1), an open-source cognitive framework for home robots, inspired by [DIARC](https://hrilab.tufts.edu/) (Tufts HRI Lab, where the developer was an RA in 2024–2025).
- **Migrated from**: [`soma_home_exp_v1`](https://github.com/jeffliulab/soma_home_exp_v1) (now archived). The original v1 plan was a mobile manipulation robot for garment sorting; on 2026-04-07 it was scoped down to a fixed tabletop manipulator. Shortly after, the repo was renamed/migrated to `SmartRobotArm` to better reflect the actual scope ("a smart arm, not a home robot"). The earlier commit on this repo (a brief SO-ARM100 + LEAP Hand + MediaPipe exploration) is unrelated and was rebuilt from scratch.
- **Long-term vision**: SmartRobotArm is the **manipulator capability layer** of the SOMA Homies family of home robots. Once the arm is reliable, it will be integrated onto a mobile platform to become Soma Home, the family's first complete robot. For the long-term design notes see the (private) `SOMA_HOME_EXP/` design folder.

## Industry alignment

The project deliberately follows the conventions of the embodied AI / IL community so that everything on the resume is legible to the field:

- **Real-robot teleop data collection** via LeRobot, published as a public LeRobotDataset on HuggingFace Hub
- **ACT (Action Chunking Transformer)** trained on real-robot data
- **Open-vocabulary perception** via Grounding DINO + SAM2 (no custom training)
- **LLM-as-Parser** task understanding (Claude API), not LLM-as-Translator
- **Behavior tree** task executor (py_trees)
- **Test-and-check validation** + failure recovery loop (DIARC signature feature)
- **ROS 2** system integration with multiple packages

What we are NOT doing in v1: RL, sim2real, VLA fine-tuning, mobile manipulation on real hardware, garment / cloth manipulation, bimanual, custom perception models. See [开发进度与待办事项.md](开发进度与待办事项.md) for the full in/out scope.

## License

[Apache License 2.0](LICENSE) — Copyright 2026 Jeff Liu Lab ([jeffliulab.com](https://jeffliulab.com), GitHub [@jeffliulab](https://github.com/jeffliulab)).

You may use, modify, and redistribute this code commercially or privately, provided you keep the copyright and license notices and document any changes. Contributors grant an explicit patent license; suing a contributor over patents in this work terminates your license.
