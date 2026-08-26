# ROS2 Embodied Robot Agent — 交接文档（2026-08-26）

> 面向后续开发者 / dsh。这是从 P0 到 P7（导航）的全部工作整理，
> 包含 git 提交、架构、运行方式、环境配置、已知问题与下一步路线。

## 一、项目概览

- 仓库：`F:\AI-Projects\ROS2-Embodied-Robot-Agent`
- 远程：`git@github.com:jxql-airobot/ROS2-Embodied-Robot-Agent.git`（公开，main）
- 技术栈：WSL2 + Ubuntu 22.04 + ROS2 Humble + Gazebo 11 + YOLO + DeepSeek
- 当前能力一句话：**能看（YOLO）→ 能规划（分层 LLM Agent）→ 能导航（Nav2+SLAM）→ 能行动（差速小车闭环）**
- 当前 HEAD：`5a4af27`（已推送 origin/main）

## 二、Git 提交与 Tag 总览（12 commit / 5 tag）

| commit | tag | 内容 |
| --- | --- | --- |
| `ba86948` | — | init 仓库 |
| `c9162f4` | — | docs: 首次 session handoff |
| `775e1ed` | — | feat(ros2): 首个闭环 demo（NL→LLM→动作→cmd_vel→运动） |
| `3ec431a` | — | feat(web): Streamlit 控制台（任务/响应/状态/odom） |
| `9b58b7a` | — | chore: 移除 ABB 相关内容 + legacy |
| `a8a5ed7` | — | chore(scripts): YOLO CPU 补丁 + 安装/冒烟/清理脚本 |
| `967d52d` | — | feat(robot_description): 前向摄像头 + Gazebo camera 插件 |
| `ddc3d4d` | `v0.1.0-vision-pipeline` | test(vision): 无头 camera→YOLO→detections 冒烟 |
| `760d7aa` | `v0.2.0-agent-vision` | feat: VisionTool + LLM 视觉感知闭环 |
| `79b2dce` | `v0.3.0-vision-action` | feat: Vision→Language→Action 闭环（找杯子→靠近） |
| `62ee4b1` | `v0.4.0-hierarchical-agent` | feat: 分层 LLM Agent（Pro/Task Manager/Flash） |
| `5a4af27` | `v0.5.0-navigation` | feat: Nav2 + slam_toolbox 导航 + navigate 落地 |

## 三、当前架构

```text
用户任务
  → HierarchicalAgent（复杂度判断）
      ├─ 简单（停止/前进/状态）→ Flash Executor → RobotTool → cmd_vel
      └─ 复杂（找物+移动/导航）→ Pro Planner → Task Manager → Flash Executor
          → VisionTool / RobotTool / NavTool → ROS2 → Gazebo Robot
```

- 视觉链路：`/camera/image_raw → YOLO → /vision/detections → Ros2VisionTool → Agent`
- 导航链路：`/scan（LiDAR）→ slam_toolbox（建图/定位）→ Nav2（规划/控制）→ /cmd_vel`

## 四、核心模块与关键文件

- `ros2/src/robot_interfaces`：`RobotCommand` / `RobotStatus` / `TaskExecute`
- `ros2/src/robot_control`：`/robot_command → /cmd_vel`（move/stop/rotate）
- `ros2/src/robot_tools`：
  - `ros2_robot_tool.py`（move/stop/rotate + **navigate 走 Nav2 NavigateToPose**）
  - `ros2_vision_tool.py`（订阅 `/vision/detections`，`find_object/get_objects`，含 direction）
- `ros2/src/robot_agent`：
  - `llm.py`（BaseLLM：mock / deepseek；`generate_action` / `plan_motion` / `plan_goal`）
  - `action_plan.py` / `prompt.py` / `vision_response.py`
  - `hierarchical_agent.py`（编排器 + 构建工厂）
  - `planner/`（Task/Plan schema、ProPlanner）、`task_manager/`（优先级/调度/状态/恢复）、`executor/`（FlashExecutor）
  - `robot_agent_node.py`（`mode=simple|hierarchical`）
- `ros2/src/robot_description`：差速小车 URDF（base + wheels + caster + camera + **LiDAR**）
- `ros2/src/robot_bringup`：launch / worlds（simple_world、cup.world）/ config（nav2、slam_toolbox）/ models（cup）
- `vision/`：`yolo_detector.py`、`image_utils.py`（rgb8→BGR）、`ros2_vision_node.py`
- `scripts/`：构建/测试/演示脚本（`wsl_build.sh`、`run_tests.sh`、`run_demo.sh`、`wsl_vision_action_demo.sh`）

## 五、运行方式

```bash
# 构建
bash scripts/wsl_build.sh

# 单元测试（ROS2 包：robot_control/robot_tools/robot_agent）
bash scripts/run_tests.sh

# 基础仿真 Demo
bash scripts/run_demo.sh            # Gazebo + RViz + robot_control + robot_agent
ros2 run robot_agent send_task "让机器人向前移动"

# 视觉→动作闭环 Demo（无头）
bash scripts/wsl_vision_action_demo.sh

# 导航（先仿真，再 SLAM+Nav2）
ros2 launch robot_bringup sim.launch.py
ros2 launch robot_bringup navigation.launch.py
ros2 run robot_agent send_task "导航到 1.0,0.0"
```

## 六、环境与配置要点

- **YOLO / torch**：`torch 2.6.0+cpu` + `ultralytics 8.4.115`。`torchvision 0.21.0` 与其 ABI 不兼容，
  已 patch `autobackend.py` 的 warmup（跳过坏 import）；重装 ultralytics 后重跑
  `scripts/wsl_patch_ultralytics.sh`。
- **API key**：DeepSeek key 在 `F:\AI-Projects\AI-Robot-Demo\.env`，**只运行时读取、绝不提交**。
  `robot_agent/config/agent.yaml` 里 `provider: mock`（默认离线）；切 deepseek 时设 `DEEPSEEK_API_KEY`。
  `deepseek-pro` / `deepseek-flash` 为占位模型名，真实可用 `deepseek-chat`。
- **Gazebo**：跑含 `model://cup` 的 demo 前需 `source /usr/share/gazebo/setup.sh` + 设
  `GAZEBO_MODEL_PATH`（指向 `ros2/src/robot_bringup/models`）+ `GAZEBO_MODEL_DATABASE_URI=""`
  （禁在线模型库，避免挂起）。演示脚本已内置。
- **sudo**：WSL 内 `sudo` 需要密码，可用 `wsl -u root` 直接以 root 执行。
- **apt 镜像**：若安装包报 404（镜像索引陈旧），先 `apt-get update`。
- **换行符**：仓库未加 `.gitattributes`，`core.autocrlf=true`；新 clone 后 `.sh/.py` 可能变 CRLF，
  建议补 `*.sh text eol=lf`、`*.py text eol=lf`。

## 七、测试情况

共 **73 个测试**（全部通过）：

- vision 6（schema + rgb8→BGR 颜色转换）
- robot_control 8（动作→速度映射/校验）
- robot_tools 16（接口契约 + VisionTool 查询 + 导航 goal 解析）
- robot_agent 43（LLM mock/DeepSeek、ActionPlan、分层 Agent 路由、优先级、失败恢复）

## 八、已知问题 / 坑

- **cup 模型是贴图 billboard**：程序化圆柱体 YOLO 识别不出，PIL 画的杯子贴图贴在薄盒子上才稳定识别。
- **`--symlink-install` 与新增 data 文件**：colcon symlink-install 对新增 world/模型可能报错，用普通
  `colcon build --packages-select <pkg>` 即可。
- **分层失败恢复**：当前是「旋转扩大搜索 + 重新观察」的简单策略，且有 `max_steps` 上限。
- **navigate() 非阻塞**：`Ros2RobotTool.navigate()` 只发送 Nav2 goal 并返回（fire-and-forget），
  结果在回调里打日志，Agent 不阻塞等待。
- **导航目标格式**：`"x,y"` 或 `"x,y,yaw"`（map 系）；命名地点（如 kitchen）尚未做映射。
- **AMCL 未启用**：当前用 slam_toolbox 提供 map→odom（SLAM 模式），未用预建地图 + AMCL。

## 九、下一步路线

1. **P7 语音**：`speech/` 目前是骨架，做 ASR → 文本任务 → Agent。
2. **P8 真实小车**：把 `Ros2RobotTool` 后端从 Gazebo 换成真车驱动。
3. **导航增强**：命名地点映射、地图保存/加载（map_server + AMCL）、Nav2 参数调优、Web 导航面板。
4. **工程收尾**：补 `.gitattributes`；把 vision 测试并入 `run_tests.sh`。

## 十、给 dsh 的建议

- 先看 `README.md` 和本文档，再跑 `bash scripts/run_tests.sh` 确认环境。
- 核心入口：`robot_agent/robot_agent_node.py` + `robot_agent/hierarchical_agent.py`。
- 工具层：`robot_tools/ros2_robot_tool.py`、`robot_tools/ros2_vision_tool.py`。
- 改代码前先跑测试；提交用 `feat:/fix:/refactor:/docs:/test:` 前缀；完成后 `git push origin main`。
