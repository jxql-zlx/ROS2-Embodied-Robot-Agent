# vision/ — 视觉感知层（YOLO 起步）

机器人"眼睛"。只负责感知，不负责决策：

```text
摄像头
  ↓
YOLO（BaseVision 接口）
  ↓
Scene{ objects: [{name, confidence, bbox, center}] }
  ↓
Agent（LLM 结合视觉做决策）
```

## 文件

| 文件 | 作用 |
| --- | --- |
| `vision_schema.py` | Detection / Scene 结构化数据（所有后端共用） |
| `base_vision.py` | BaseVision 抽象接口（未来换 GPT Vision / Gemini / Qwen-VL 不改造 Agent） |
| `yolo_detector.py` | ultralytics YOLO 检测器（开源，默认 yolov8n.pt 自动下载） |
| `vision_tool.py` | VisionTool：Agent 视角的 get_scene / find_object |
| `ros2_vision_node.py` | ROS2 节点：`/camera/image_raw` → YOLO → `/vision/detections`（JSON） |

## 快速使用（WSL）

```bash
pip3 install --user ultralytics          # 依赖 torch/cv2（已装）
python3 -c "
from vision.yolo_detector import YOLODetector
scene = YOLODetector().detect('test.jpg')   # 图片路径/numpy
print(scene.to_json())
"
```

## 与仿真联动

`simple_diffbot` URDF 已带摄像头（`/camera/image_raw`）。启动 Demo 后再开视觉节点：

```bash
ros2 launch robot_bringup demo.launch.py
# 另开终端：
source ros2/install/setup.bash
python3 vision/ros2_vision_node.py --ros-args -p use_sim_time:=true
ros2 topic echo /vision/detections
```

## 设计原则

- YOLO 不直接控制机器人；检测结果作为 LLM Agent 的环境输入。
- 视觉后端可替换：`BaseVision.detect(image) -> Scene`，Agent 不感知后端细节。
- 下一阶段：YOLO 结果进入 LLM 上下文 → LLM 规划 → ROS2 动作（找物/导航闭环）。
