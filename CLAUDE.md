# SmartRobotArm - Project Conventions

## Overview
Vision-based gesture teleoperation system: camera captures human arm/hand movements, robot arm + dexterous hand mirrors them in MuJoCo simulation. Models published to HuggingFace.

## Tech Stack
- Python 3.10+
- MuJoCo (simulation), MediaPipe (pose estimation), LeRobot + ACT (imitation learning)
- ikpy (arm IK), dex-retargeting (hand retargeting)
- PyTorch (training), Gymnasium (env API)

## Project Structure
- `src/` - Main source code (pose_estimation, retargeting, simulation, teleoperation, training)
- `config/` - YAML configs for arm, hand, camera, training
- `scripts/` - Entry-point scripts
- `models/` - URDF/MJCF files and trained checkpoints
- `data/` - Demonstration datasets
- `tests/` - Unit tests

## Code Style
- Linter: ruff
- Formatter: ruff format
- Type hints encouraged
- Tests: pytest

## Commands
- `ruff check src/ tests/` - Lint
- `ruff format src/ tests/` - Format
- `pytest tests/` - Run tests
- `python scripts/<script>.py` - Run entry-point scripts
