# SmartRobotArm — Project conventions

> This file is read by Claude Code at the start of every conversation. Keep it accurate and concise.

## What this project is

**SmartRobotArm** is a fixed tabletop manipulator powered by natural language. The user speaks an instruction, the [ANIMA](https://github.com/jeffliulab/ANIMA_O1) cognitive layer parses it via Claude API into a structured task spec, perception (Grounding DINO + SAM2) grounds language to world coordinates, a py_trees behavior tree dispatches ACT-trained skill primitives, and a test-and-check validator confirms success or triggers recovery.

This is a **portfolio / job-hunting project**, public Apache-2.0, written by Jeff Liu Lab. Optimize all decisions for **legibility to a hiring manager in the embodied AI / robotics space**, not for academic novelty or production hardening.

The long-term vision is for the manipulator to become the arm of a SOMA Home domestic robot. v1 deliberately scopes mobile manipulation, garment handling, bimanual, and force feedback OUT — those are v2+ goals.

## Repo lineage and current scope

- **Migrated from** [`soma_home_exp_v1`](https://github.com/jeffliulab/soma_home_exp_v1) (now archived) on 2026-04-07. ANIMA nodes, URDF, launch files, and the 8-week sprint plan all came from there.
- **Cognitive framework dependency**: [`ANIMA_O1`](https://github.com/jeffliulab/ANIMA_O1) — separate Python framework repo. SmartRobotArm wraps ANIMA in ROS 2 nodes.
- **Long-term design context**: lives in `SOMA_HOME_EXP/` and `ANIMA_FRAMEWORK/` design folders (private monorepo, not public). Some of those documents are STALE because they predate the pivot — check `SOMA_HOME_EXP/CLAUDE.md` for which files are current.
- **Earlier commit on this repo** is unrelated SO-ARM100 + LEAP Hand + MediaPipe exploration that was rebuilt from scratch. Don't reference any of it.

## Single source of truth for v1 execution

[`开发进度与待办事项.md`](开发进度与待办事项.md) is the **single source of truth** for the 8-week sprint plan. Always check it before recommending tasks. Update its progress tracker after completing meaningful work.

## Hardware (locked 2026-04-07)

- **Arm**: Waveshare RoArm-M2-S — 4-DOF, USB serial, hobby servo precision
- **Camera**: Logitech C922 Pro Stream Webcam — USB UVC, fixed overhead
- **Teleop input**: PDP Wired Controller for Xbox — XInput, user already owns
- **Compute**: Desktop with RTX 4090
- **No Pi 5, no mobile base, no LiDAR in v1** — everything plugs directly into the desktop PC
- **Object set**: small plush toys / figurines (user already owns)

## Development environment (locked 2026-04-07)

- **Host**: Windows 11 (user's primary OS, GPU known to perform at full capacity here)
- **Dev**: WSL2 + Ubuntu 22.04 — chosen over dual-boot Linux because the user's hardware suffers GPU performance loss on dual-boot. WSL2 + CUDA passthrough preserves 100% RTX 4090 performance.
- **USB**: `usbipd-win` forwards C922 + RoArm to WSL2's `/dev/video0` and `/dev/ttyUSB0`. **Each Windows reboot needs a fresh `usbipd attach`** — write an `attach_devices.bat` for convenience.
- **ROS 2**: Humble Hawksbill (LTS, best LeRobot compat)
- **Graphics**: WSLg (Gazebo / RViz2 work but slower — fine since they're not the bottleneck)

## Repository structure

```
SmartRobotArm/
├── README.md, LICENSE, CLAUDE.md, .gitignore
├── 开发进度与待办事项.md     # 8-week sprint plan (source of truth)
├── docs/
│   ├── DEVELOPMENT.md
│   └── FAQ-硬件与仿真.md
└── src/                     # ROS 2 workspace, build with colcon
    ├── anima_node/          # ANIMA cognitive layer ROS 2 wrapper
    ├── arm_description/     # URDF + Gazebo verification scene
    └── arm_bringup/         # Top-level launch
```

ROS 2 packages added later in the sprint (W2-W3):
- `arm_perception/` — Grounding DINO + SAM2
- `arm_manipulation/` — MoveIt2 config + hardcoded primitives

## Conventions

- **Language**: docs and comments are mostly Chinese (Simplified). Code, ROS package names, identifiers, and config keys stay in English. The 开发进度与待办事项.md follows this convention.
- **License headers**: not added to every source file in v1. The repo-level LICENSE is sufficient until we tag a release.
- **Naming**: ROS packages have NO `soma_` prefix anymore. They use `anima_node`, `arm_description`, `arm_bringup`, etc.
- **Author / copyright**: always **"Jeff Liu Lab"** (jeffliulab.com, GitHub @jeffliulab). Don't drop the "Lab" suffix.
- **Don't over-engineer**: this is pre-alpha. No CI, no packaging, no contribution guides, no code-of-conduct. Add them when there's a real reason.

## Common commands

```bash
# Build
cd ~/SmartRobotArm
colcon build --symlink-install
source install/setup.bash

# Visualize URDF
ros2 launch arm_description display.launch.py

# Launch arm + ANIMA mock parser
ros2 launch arm_bringup full_system.launch.py llm_backend:=mock

# Test ANIMA with a string
ros2 topic pub /user_instruction std_msgs/String "data: 'put the red toy in the blue bin'"

# USB device forwarding (Windows PowerShell, after reboot)
usbipd attach --wsl --busid <C922_BUSID>
usbipd attach --wsl --busid <ROARM_BUSID>
```

## What NOT to do (deliberately scoped out of v1)

When recommending features or tasks, push back if any of these come up — they are v2+ and would derail the 8-week sprint:

- ❌ Mobile manipulation on real hardware (Gazebo dry-run is allowed)
- ❌ Garment / cloth / cable / deformable object handling
- ❌ Bimanual / force feedback / 6-DOF arm
- ❌ Custom base / Nav2 / SLAM on real hardware
- ❌ Multi-machine ROS 2 DDS (everything is single PC)
- ❌ Pi 5 embedded deployment
- ❌ MuJoCo / Isaac Sim / Isaac Lab
- ❌ Sim2real transfer (real robot data only)
- ❌ Reinforcement learning of any kind
- ❌ Custom perception model training (Grounding DINO + SAM2 are pretrained)
- ❌ VLA fine-tuning (Week 8 stretch only)

If the user wants to do any of these, treat it as a scope change worth discussing — don't silently agree.

## License

[Apache-2.0](LICENSE), Copyright 2026 Jeff Liu Lab.
