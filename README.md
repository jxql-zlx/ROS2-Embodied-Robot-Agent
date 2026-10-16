# ROS2 Multimodal Embodied Robot Agent System

> 状态：**P0–P7 已落地**：ROS2 闭环、视觉感知（YOLO）、
> Vision→Language→Action 闭环、分层 LLM Agent、SLAM + Nav2 自主导航。
> 面向具身智能的多模态机器人智能体系统，独立演进。

## 定位

面向**具身智能（Embodied Intelligence）**的多模态机器人智能体系统：

```text
能听 → 能看 → 能理解 → 能规划 → 能行动
```

技术栈：ROS2 + Vision + Speech + Multimodal AI + Embodied Intelligence。

## 与本科项目（AI-Robot-Demo）的关系

- **继承设计思想**：Agent 架构、Planner、Memory、Reflection、Recovery、
  Tool 调用、日志系统、实验方法（详见 [agent/](agent/README.md)）。
- **不复制旧项目代码**：本科项目冻结于 `bachelor-thesis-stable` 分支，
  本仓库仅继承设计思想。
- **执行层重新设计**：以 ROS2 为执行底座（Navigation2 / SLAM / 相机 / LiDAR）。

## 路线总览

| 模块 | 当前 | 未来 |
| --- | --- | --- |
| 执行 | ROS2 闭环 + Nav2/SLAM 导航（move / stop / rotate / navigate） | 机械臂 |
| 视觉 | YOLO + Ros2VisionTool（Vision→Action 闭环） | GPT Vision / Gemini / Qwen-VL |
| 语音 | 规划中 | ASR → 文本任务 |
| 模型 | 多模态模型接口（不绑定单一供应商） | GPT / Gemini / Claude / Qwen / DeepSeek |
| 网页 | Streamlit 控制台（web/） | 视觉面板实时显示 |

## 分层 Agent 架构（Hierarchical）

```text
DeepSeek Pro Planner（高层任务拆解）
        ↓
Task Manager（优先级排序 / 调度 / 失败重规划）
        ↓
DeepSeek Flash Executor（低层单任务执行）
        ↓
VisionTool / RobotTool / NavTool → ROS2 → Robot
```

- 简单任务（停止/前进/状态）走 Flash Executor 快速执行；
  复杂任务（找物+移动 / 导航）走 Pro Planner → Task Manager → Flash Executor。
- 优先级：`importance×0.4 + urgency×0.3 + dependency×0.2 + complexity×0.1`。
- 配置：`ros2/src/robot_agent/config/agent.yaml`（`mode` / `planner.model` / `executor.model`）。

## 导航（Navigation2 + SLAM）

```text
LiDAR (/scan) → slam_toolbox（建图 / 定位，发布 /map + map→odom）
      → Nav2（全局规划 / 局部控制 / 避障 / 恢复）
      → /cmd_vel → Gazebo 机器人
```

- 机器人 URDF 已带 360° 2D LiDAR（Gazebo ray sensor → `/scan`，frame `lidar_link`）。
- 启动导航：先 `ros2 launch robot_bringup sim.launch.py`，再
  `ros2 launch robot_bringup navigation.launch.py`。
- `Ros2RobotTool.navigate("x,y" | "x,y,yaw")` 通过 Nav2 `NavigateToPose` action
  在 `map` 系导航（例如 `导航到 1.0,0.0`）。

## 目录结构

```text
├── agent/        # Agent 设计思想（继承自本科项目，执行层重设计）
├── ros2/         # ROS2 工作区（interfaces / control / tools / agent / description / bringup）
├── vision/       # 视觉统一接口（BaseVision + YOLO，未来多模态）
├── speech/       # 语音输入 → ASR → 任务（规划）
├── web/          # Streamlit 网页控制台
├── models/       # 多模态模型统一接口（规划）
├── scripts/      # WSL 构建 / 测试 / 演示脚本
└── docs/         # 规划文档（roadmap、交接说明等）
```

## 快速开始（WSL）

```bash
# 构建工作区
bash scripts/wsl_build.sh

# 单元测试
bash scripts/run_tests.sh

# 启动仿真 Demo（Gazebo + RViz + robot_control + robot_agent）
bash scripts/run_demo.sh

# 另开终端，发送自然语言任务
ros2 run robot_agent send_task "让机器人向前移动"

# 视觉→语言→动作闭环 Demo（找杯子 → 机器人靠近）
bash scripts/wsl_vision_action_demo.sh

# 网页控制台（再开一个终端）
source ros2/install/setup.bash
python3 -m streamlit run web/app.py --server.port 8502
```

## 说明

- **视觉目标**：Gazebo 世界 `cup.world` 含一个 `model://cup` 贴图杯；
  运行视觉 demo 前脚本会自动设置 `GAZEBO_MODEL_PATH` / `GAZEBO_MODEL_DATABASE_URI`。
- **接近完成判定（simple 模式）**：目标在图像中居中，且 `/scan` 前方 ±10° 的有效距离
  达到 `approach_stop_distance`（默认距 `lidar_link` 0.65 米）时停止。每步动作最多 0.5 秒，
  动作结束后等待底盘稳定、三次一致激光读数和新视觉帧；无测距或读数异常时返回失败。
  此策略用于目标正前方无遮挡的 Demo，尚未将激光回波与具体物体做三维关联。
  Demo 脚本会重建相关包并检查服务成功、实际位移和最终测距。
- **LLM provider**：默认 `mock`（离线可用）。使用 DeepSeek 时设置环境变量
  `DEEPSEEK_API_KEY`，并 `ros2 launch ... llm_provider:=deepseek`；
  仓库**不提交任何 API key**，请勿把密钥写入代码或 `.env` 提交。

详见 [ros2/README.md](ros2/README.md)、[vision/README.md](vision/README.md)、
[web/README.md](web/README.md)、[docs/roadmap/embodied_ai_plan.md](docs/roadmap/embodied_ai_plan.md)。
