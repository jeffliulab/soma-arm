# SmartRobotArm 完整使用指南

## 一、项目概述

本项目实现**基于视觉的机器人遥操作系统**：通过摄像头捕捉人体手臂和手部动作，实时控制仿真中的机械臂（SO-ARM100）和灵巧手（LEAP Hand v2），并通过模仿学习（ACT）训练自主操作策略。

### 系统架构

```
┌──────────┐    ┌───────────────┐    ┌──────────────┐    ┌──────────────┐
│  摄像头   │───>│ MediaPipe     │───>│  逆运动学/    │───>│  MuJoCo      │
│  (USB)   │    │ Holistic      │    │  手部重定向    │    │  仿真环境     │
│          │    │ (33体+21手)   │    │  (ikpy/dex)  │    │  (渲染+物理)  │
└──────────┘    └───────────────┘    └──────────────┘    └──────────────┘
                                                               │
                                            ┌──────────────────┤
                                            │                  │
                                    ┌───────▼────┐    ┌───────▼────────┐
                                    │ 数据采集    │    │ 可视化显示      │
                                    │ (LeRobot格式)│   │ (摄像头+仿真)   │
                                    └───────┬────┘    └────────────────┘
                                            │
                                    ┌───────▼────────┐
                                    │ ACT策略训练     │
                                    │ (Transformer)  │
                                    └───────┬────────┘
                                            │
                                    ┌───────▼────────┐
                                    │ 自主策略执行     │
                                    │ (仿真评估)      │
                                    └────────────────┘
```

---

## 二、环境搭建

### 2.1 系统要求

- **操作系统**: Windows 11 / Ubuntu 22.04
- **Python**: 3.10+
- **GPU**: NVIDIA RTX 4090 (推荐) 或任何 8GB+ VRAM 的 NVIDIA GPU
- **摄像头**: USB 摄像头（笔记本内置摄像头也可以）

### 2.2 安装步骤

```bash
# 1. 克隆仓库
cd d:/Projects
git clone <repo_url> SmartRobotArm
cd SmartRobotArm

# 2. 创建虚拟环境
python -m venv .venv

# Windows:
.venv\Scripts\activate
# Linux/Mac:
# source .venv/bin/activate

# 3. 安装 PyTorch（根据 CUDA 版本选择）
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# 4. 安装项目依赖
pip install -e ".[dev]"

# 5. 安装可选依赖（LeRobot 和 dex-retargeting）
pip install lerobot          # LeRobot 框架
pip install dex-retargeting  # 手部重定向（Phase 3）

# 6. 验证安装
python -c "import mediapipe; print('MediaPipe OK')"
python -c "import mujoco; print(f'MuJoCo {mujoco.__version__} OK')"
python -c "import torch; print(f'PyTorch {torch.__version__}, CUDA: {torch.cuda.is_available()}')"
python -c "import gymnasium; print('Gymnasium OK')"
```

### 2.3 验证 Demo 脚本

```bash
# 验证 MediaPipe（需要摄像头，按 q 退出）
python scripts/demo_mediapipe.py

# 验证 MuJoCo（弹出 3D 窗口，按 ESC 退出）
python scripts/demo_mujoco.py

# 验证 LeRobot（打印库信息和管线结构）
python scripts/demo_lerobot.py
```

---

## 三、项目文件结构详解

```
SmartRobotArm/
├── src/                           # 核心源代码
│   ├── pose_estimation/           # 姿态估计模块
│   │   ├── body_tracker.py        # 身体姿态追踪（肩/肘/腕）
│   │   ├── hand_tracker.py        # 手部姿态追踪（21关键点）
│   │   └── holistic_tracker.py    # 联合身体+手部追踪
│   ├── retargeting/               # 动作重定向模块
│   │   ├── arm_ik.py              # 人体手臂→机械臂 逆运动学
│   │   ├── hand_retarget.py       # 人体手部→LEAP Hand 重定向
│   │   └── combined.py            # 手臂+手部联合重定向
│   ├── simulation/                # MuJoCo 仿真模块
│   │   ├── arm_env.py             # SO-ARM100 仿真环境 (Gymnasium)
│   │   ├── hand_env.py            # LEAP Hand 仿真环境 (Gymnasium)
│   │   └── domain_randomization.py # 域随机化（sim-to-real）
│   ├── teleoperation/             # 遥操作模块
│   │   ├── vision_teleop.py       # 视觉遥操作主循环
│   │   └── data_recorder.py       # 数据采集（LeRobot格式）
│   └── training/                  # 训练模块
│       ├── act_train.py           # ACT策略 + 训练管线
│       └── evaluate.py            # 策略评估 + 交互回放
├── scripts/                       # 入口脚本（直接运行）
│   ├── demo_mediapipe.py          # MediaPipe 演示
│   ├── demo_mujoco.py             # MuJoCo 演示
│   ├── demo_lerobot.py            # LeRobot 探索
│   ├── run_teleop.py              # 启动视觉遥操作
│   ├── collect_demos.py           # 采集示教数据
│   ├── train_act.py               # 训练 ACT 策略
│   ├── evaluate_policy.py         # 评估训练的策略
│   └── run_sim.py                 # 启动仿真环境
├── config/                        # 配置文件
│   ├── arm_config.yaml            # 机械臂参数
│   ├── camera_config.yaml         # 摄像头参数
│   ├── hand_config.yaml           # 灵巧手参数
│   └── training_config.yaml       # 训练超参数
├── models/                        # 模型文件
│   ├── so_arm100/                 # SO-ARM100 MJCF/URDF
│   ├── leap_hand/                 # LEAP Hand MJCF
│   └── trained/                   # 训练好的策略权重
├── data/                          # 数据集
│   └── demonstrations/            # 示教数据
├── tests/                         # 单元测试
│   ├── test_arm_ik.py
│   ├── test_hand_retarget.py
│   ├── test_simulation.py
│   └── test_act_train.py
└── docs/                          # 文档
    └── guide.md                   # 本文件
```

---

## 四、分阶段使用说明

### Phase 1: 机械臂仿真 + 视觉控制

这是最核心的阶段：用摄像头控制仿真中的 SO-ARM100 机械臂。

#### 4.1.1 查看仿真环境

```bash
# 启动 SO-ARM100 仿真（自动演示动作）
python scripts/run_sim.py arm

# 启动 LEAP Hand 仿真
python scripts/run_sim.py hand

# 带域随机化的仿真
python scripts/run_sim.py arm --random

# Gymnasium API 测试（随机动作）
python scripts/run_sim.py gym
```

#### 4.1.2 启动视觉遥操作

```bash
# 基础模式：摄像头控制仿真机械臂
python scripts/run_teleop.py

# 指定摄像头设备
python scripts/run_teleop.py --camera 1

# 启动时立即开始录制
python scripts/run_teleop.py --record
```

**操作说明：**
- 站在摄像头前，伸出右手臂
- 移动手臂 → 仿真机械臂跟随移动
- 捏合拇指和食指 → 仿真夹爪关闭/打开
- 屏幕左半边 = 摄像头画面（带骨架叠加）
- 屏幕右半边 = MuJoCo 仿真渲染
- 按 `r` 开始/停止录制，按 `s` 保存，按 `q` 退出

#### 4.1.3 关键代码说明

**姿态追踪** ([body_tracker.py](../src/pose_estimation/body_tracker.py))：
- 使用 MediaPipe Pose 提取 33 个身体关键点
- 核心关注：肩膀(12)、肘部(14)、手腕(16)
- 指数移动平均（EMA）平滑，减少抖动

**逆运动学** ([arm_ik.py](../src/retargeting/arm_ik.py))：
- 人体手臂坐标映射到机器人工作空间
- 优先使用 ikpy 库求解，备用几何 IK
- 速度限制防止突变，输出平滑

**仿真环境** ([arm_env.py](../src/simulation/arm_env.py))：
- 内置 SO-ARM100 MJCF 模型（6 自由度 + 平行夹爪）
- Gymnasium 兼容：支持 `reset()`, `step()`, `render()`
- 观测空间：7 关节位置 + 3 物体位置 + 3 目标位置 = 13 维
- 动作空间：5 臂关节 + 1 夹爪 = 6 维

---

### Phase 2: ACT 策略训练

用采集的示教数据训练自主操作策略。

#### 4.2.1 采集示教数据

```bash
# 采集 50 轮次的抓放示教数据
python scripts/collect_demos.py --num-episodes 50 --dataset-name so100_pick_place

# 自定义参数
python scripts/collect_demos.py --num-episodes 100 --max-steps 300 --camera 0
```

**操作流程：**
1. 启动后看到摄像头 + 仿真画面
2. 按**空格键**开始录制一个 episode
3. 用手臂操控机械臂完成抓放任务
4. 按**空格键**结束 episode
5. 重复直到采集足够数据
6. 按 `q` 退出并自动保存

数据保存在 `data/demonstrations/so100_pick_place/` 目录下。

#### 4.2.2 训练 ACT 策略

```bash
# 默认参数训练
python scripts/train_act.py

# 自定义参数
python scripts/train_act.py \
  --dataset data/demonstrations/so100_pick_place \
  --epochs 300 \
  --batch-size 64 \
  --chunk-size 20 \
  --lr 1e-4 \
  --device cuda
```

**训练说明：**
- ACT (Action Chunking with Transformers) 预测未来 20 步动作
- 在 RTX 4090 上约 2-3 小时完成 200 epochs
- 模型保存在 `models/trained/act_arm/`
- `policy_best.pt` = 最佳验证损失
- `policy_final.pt` = 最终 epoch

#### 4.2.3 评估策略

```bash
# 自动评估（100 轮次，计算成功率）
python scripts/evaluate_policy.py \
  --checkpoint models/trained/act_arm/policy_best.pt \
  --num-episodes 100

# 保存评估视频
python scripts/evaluate_policy.py \
  --checkpoint models/trained/act_arm/policy_best.pt \
  --save-video

# 交互式回放（在 MuJoCo 查看器中）
python scripts/evaluate_policy.py \
  --checkpoint models/trained/act_arm/policy_best.pt \
  --replay
```

**关键代码说明**

**ACT 策略** ([act_train.py](../src/training/act_train.py))：
- `SimpleACTPolicy`: Transformer encoder-decoder 架构
- 输入：当前关节位置 (6 维)
- 输出：未来 chunk_size 步的关节目标 (chunk_size x 6)
- 训练使用 MSE 损失 + AdamW + Cosine LR schedule

---

### Phase 3: LEAP Hand 灵巧手集成

在机械臂基础上加入 16 自由度灵巧手。

#### 4.3.1 手部仿真测试

```bash
# LEAP Hand 独立仿真
python scripts/run_sim.py hand
```

#### 4.3.2 启动手臂+手部联合遥操作

```bash
# 启用 LEAP Hand
python scripts/run_teleop.py --hand
```

**关键代码说明**

**手部重定向** ([hand_retarget.py](../src/retargeting/hand_retarget.py))：
- 从 MediaPipe 21 个手部关键点提取每根手指的弯曲角度
- 映射到 LEAP Hand 16 个自由度：4 根手指 x 4 关节
- 计算方式：相邻指节之间的弯曲角度（curl angle）

**联合控制** ([combined.py](../src/retargeting/combined.py))：
- Phase 1-2: 手臂 6 DOF + 夹爪（捏合手势控制）
- Phase 3+: 手臂 6 DOF + LEAP Hand 16 DOF = 22 DOF

---

### Phase 4: 域随机化 + 高级训练

#### 4.4.1 域随机化

```bash
# 查看域随机化效果
python scripts/run_sim.py arm --random
```

域随机化 ([domain_randomization.py](../src/simulation/domain_randomization.py)) 包括：
- **物体随机化**: 位置、大小、质量、摩擦力
- **机器人随机化**: 关节阻尼、执行器增益
- **视觉随机化**: 光照位置

#### 4.4.2 RL 训练（补充方案）

可以直接用 Gymnasium API 接入 SKRL/Stable-Baselines3：

```python
from src.simulation.arm_env import ArmSimEnv, ArmEnvConfig

env = ArmSimEnv(ArmEnvConfig(render_mode=None))
obs, _ = env.reset()
action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
```

---

### Phase 5: HuggingFace 发布

训练完成后，上传模型和数据集到 HuggingFace：

```bash
# 安装 huggingface_hub
pip install huggingface_hub

# 登录
huggingface-cli login

# 上传模型
python -c "
from huggingface_hub import HfApi
api = HfApi()
api.upload_folder(
    folder_path='models/trained/act_arm',
    repo_id='jeffliulab/smart-robot-arm-act',
    repo_type='model',
)
"

# 上传数据集
python -c "
from huggingface_hub import HfApi
api = HfApi()
api.upload_folder(
    folder_path='data/demonstrations/so100_pick_place',
    repo_id='jeffliulab/smart-robot-arm-demos',
    repo_type='dataset',
)
"
```

---

## 五、运行测试

```bash
# 运行所有测试
python -m pytest tests/ -v

# 运行特定测试
python -m pytest tests/test_arm_ik.py -v
python -m pytest tests/test_simulation.py -v
python -m pytest tests/test_act_train.py -v

# 代码检查
python -m ruff check src/ scripts/ tests/
python -m ruff format src/ scripts/ tests/
```

---

## 六、核心模块 API 速查

### 6.1 姿态估计

```python
from src.pose_estimation.holistic_tracker import HolisticTracker

tracker = HolisticTracker()
pose = tracker.detect(frame_bgr)  # -> HolisticPose

pose.arm_pose          # ArmPose: shoulder, elbow, wrist (各 3D)
pose.right_hand        # HandPose: 21 landmarks, pinch_distance
pose.left_hand         # HandPose
```

### 6.2 逆运动学

```python
from src.retargeting.arm_ik import ArmIK

ik = ArmIK()
angles = ik.solve(arm_pose)        # -> (6,) 关节角度
angles = ik.set_gripper(angles, 0.5)  # 设置夹爪开度
```

### 6.3 手部重定向

```python
from src.retargeting.hand_retarget import HandRetarget

retarget = HandRetarget()
hand_angles = retarget.retarget(hand_pose)  # -> (16,) LEAP Hand 关节角
gripper = retarget.compute_pinch_gripper(hand_pose)  # -> 0.0~1.0
```

### 6.4 联合控制

```python
from src.retargeting.combined import CombinedRetarget

combined = CombinedRetarget(enable_hand=True)  # Phase 3+
action = combined.retarget(holistic_pose)

action.arm_joints   # (6,) 手臂关节
action.hand_joints  # (16,) 手部关节（如果 enable_hand=True）
action.full_joints  # (22,) 全部关节拼接
```

### 6.5 仿真环境

```python
from src.simulation.arm_env import ArmSimEnv, ArmEnvConfig

env = ArmSimEnv(ArmEnvConfig(render_mode="rgb_array"))
obs, _ = env.reset()

# Gymnasium 标准接口
obs, reward, terminated, truncated, info = env.step(action)

# 直接关节控制（遥操作用）
env.set_joint_positions(np.array([0.1, 0.2, 0.3, 0.1, 0.0, 0.01]))

# 获取当前关节位置
positions = env.get_joint_positions()  # -> (6,)

# 渲染图像
image = env.render_image()  # -> (480, 640, 3) uint8
```

### 6.6 ACT 训练

```python
from src.training.act_train import ACTTrainer, ACTTrainConfig, load_policy

# 训练
trainer = ACTTrainer(ACTTrainConfig(dataset_path="data/demos", num_epochs=200))
save_path = trainer.train()

# 加载和推理
policy = load_policy("models/trained/act_arm/policy_best.pt")
state = torch.tensor(joint_positions, dtype=torch.float32).unsqueeze(0)
with torch.no_grad():
    action_chunk = policy(state)  # (1, 20, 6)
```

---

## 七、常见问题

### Q: MediaPipe 检测不到姿态？
- 确保光线充足，背景简洁
- 调低 `min_detection_confidence` 到 0.3
- 确保上半身（至少肩膀到手腕）在画面中

### Q: MuJoCo 渲染窗口打不开？
- Windows 上确保安装了 OpenGL 驱动
- 尝试 `render_mode="rgb_array"` 模式（离屏渲染）
- 更新 GPU 驱动

### Q: 训练时 CUDA out of memory？
- 减小 `batch_size`（32 → 16）
- 减小 `dim_model`（256 → 128）
- 使用 `--device cpu` 测试（会慢很多）

### Q: 仿真机械臂抖动严重？
- 增大 `smooth_factor`（0.3 → 0.7）
- 减小 `velocity_limit`（0.5 → 0.3）
- 确保摄像头画面稳定（固定摄像头位置）

### Q: 如何切换到左手控制？
- 修改 `config/camera_config.yaml` 中的 `use_right_arm: false`
- 或在代码中: `HolisticTrackerConfig(use_right_arm=False)`

---

## 八、硬件部署方案（Phase 5 参考）

当仿真验证通过后，可以购买以下硬件进行实际部署：

| 硬件 | 型号 | 价格 | 购买渠道 |
|------|------|------|---------|
| 机械臂 x2 | SO-ARM100 (主从) | ~$220 | WowRobo / Waveshare / Seeed Studio |
| 灵巧手 | LEAP Hand v2 | ~$2,000 | leaphand.com |
| 深度摄像头 | Intel RealSense D435 | ~$200 | Intel / Amazon |
| 电源 | 6V 5A (臂) + 12V (手) | ~$50 | 通用 |
| 3D打印 | 外包打印零件 | ~$100-200 | JLCPCB / Shapeways |
| **总计** | | **~$2,500-2,700** | |

部署步骤：
1. 组装 SO-ARM100（参考 [官方教程](https://huggingface.co/docs/lerobot/en/so100)）
2. 运行 LeRobot 标定脚本
3. 从 HuggingFace 下载训练好的模型
4. 运行 `scripts/deploy_real.py` 进行推理
5. 用少量真实数据微调以缩小 sim-to-real 差距

---

## 九、参考资源

| 资源 | 链接 |
|------|------|
| LeRobot 框架 | https://github.com/huggingface/lerobot |
| SO-ARM100 设计 | https://github.com/TheRobotStudio/SO-ARM100 |
| LEAP Hand v2 | https://v2.leaphand.com |
| MuJoCo 文档 | https://mujoco.readthedocs.io |
| MuJoCo Playground | https://playground.mujoco.org |
| ACT 论文 | https://arxiv.org/abs/2304.13705 |
| MediaPipe 文档 | https://developers.google.com/mediapipe |
| dex-retargeting | https://github.com/dexsuite/dex-retargeting |
| ikpy 文档 | https://github.com/Phylliade/ikpy |
