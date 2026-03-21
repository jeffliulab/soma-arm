# SmartRobotArm

Vision-based gesture teleoperation system for robot arm and dexterous hand control.

## Project Goal

Capture human arm and hand movements via camera (MediaPipe), and control a simulated robot arm (SO-ARM100) + dexterous hand (LEAP Hand v2) in MuJoCo to mirror those movements. Train autonomous manipulation policies using ACT (Action Chunking with Transformers) via the LeRobot framework.

## Architecture

```
Camera -> MediaPipe Holistic -> Pose Estimation (33 body + 21 hand landmarks)
    -> Arm IK (ikpy) -> 6-DOF SO-ARM100 joint angles
    -> Hand Retargeting (dex-retargeting) -> 8-DOF LEAP Hand joint angles
    -> MuJoCo Simulation (real-time visualization)
    -> LeRobot Data Collection -> ACT Policy Training -> Autonomous Execution
```

## Phases

- **Phase 0**: Environment setup and library verification
- **Phase 1**: SO-ARM100 simulation + vision-based arm control
- **Phase 2**: ACT imitation learning policy training
- **Phase 3**: LEAP Hand integration + hand retargeting
- **Phase 4**: Full system training + HuggingFace publication
- **Phase 5**: Hardware deployment guide

## Setup

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # Linux/Mac
# .venv\Scripts\activate   # Windows

# Install dependencies
pip install -e ".[all]"
```

## Tech Stack

| Component | Tool |
|-----------|------|
| Simulation | MuJoCo |
| Pose Estimation | MediaPipe Holistic |
| Robot Learning | LeRobot + ACT |
| Arm IK | ikpy |
| Hand Retargeting | dex-retargeting |
| Training | PyTorch on RTX 4090 |
