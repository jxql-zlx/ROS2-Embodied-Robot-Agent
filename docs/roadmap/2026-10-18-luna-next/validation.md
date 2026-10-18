# 本轮实施验证记录

实施起点：`8835531`。只记录本轮实际运行的证据；未执行项明确保留。

## 验收矩阵

| 范围 | 要求 | 状态/证据 |
|---|---|---|
| 停止失败门禁 | approach 停止失败保持门禁，可显式重试恢复 | 通过：robot_agent 定向单测及两模式 Gazebo Nav2 取消场景 |
| 取消竞态 | 初次/最终停止间保留归属；自动停止不能释放显式取消门禁 | 通过：事件屏障单测覆盖最终 stop 失败、重试和任务代次隔离 |
| 命令回执 | 固定 ID、乱序/覆盖/迟到状态、无历史状态启动 | 通过：robot_tools 单测覆盖错 ID、终态保持、TTL/容量和 clock reset |
| 激光源时间 | 旧帧、未来帧、乱序、重复、ROS clock 回退与恢复 | 通过：robot_tools 单测覆盖源时间和仿真时间会话 |
| 接近动作 | 允许列表、参数/目标校验、剩余净空和最短运动 | 通过：robot_agent 单测及真实 YOLO Gazebo 成功/障碍场景 |
| 任务生命周期 | 同 UUID 重试、旧 UUID 隔离、迟到/重复请求和超时 | 通过：robot_agent 单测覆盖取消作用域、去重和服务路径 |
| 计划校验 | 直接动作、infer/replan 校验、复用状态隔离 | 通过：action/task schema 和 hierarchical 单测 |
| 单测总集 | `scripts/run_tests.sh` | 通过：208 项（control 11、tools 56、agent 135、vision 6） |
| ROS2 构建 | `scripts/wsl_build.sh`，6 个包 | 通过：6 packages finished |
| 导航取消 | simple/hierarchical CANCELED、停车观测、后续导航 | 通过：两模式均观察到 CANCELED；停止后 50/51 个新鲜 odom 样本，漂移 0.000 m，跟进导航成功 |
| 组合成功 | 导航→YOLO→接近净空→停车观测 | 通过：YOLO 找到 cup，前向净空 0.749 m；目标到达未确认；停车后漂移/偏航均 0 |
| 故障组合 | 导航失败、无目标、障碍、视觉/激光断流 | 通过：五种场景分别失败关闭；停止后最大观测漂移约 0.004 m |
| Git 交付 | diff 检查、提交、推送和状态 | 通过：实现提交 `66a2d7e` 已推送 `origin/main`；推送后工作区干净 |

## 集成运行日志

- Nav2 取消：`/tmp/ros2_navigation_cancel/run.3FzJ42`
- 组合成功：`/tmp/ros2_composed_navigation/success.H2uOnR`
- 导航失败：`/tmp/ros2_composed_navigation/navigation_failure.sXZenm`
- 目标缺失：`/tmp/ros2_composed_navigation/target_absent.tBjB9x`
- 近障碍：`/tmp/ros2_composed_navigation/blocked.Bkfapz`
- 视觉断流：`/tmp/ros2_composed_navigation/vision_dropout.6GrA6P`
- 激光断流：`/tmp/ros2_composed_navigation/range_dropout.XzI97P`

预期失败任务的客户端退出码为 2；外层场景脚本断言阶段结果、停车和漂移后返回 0。每次运行使用独立日志目录。

## 证据边界与未覆盖项

- 控制器 stop 确认和物理静止观测分别记录；物理静止必须有 odom 证据。本轮组合成功场景的结构化结果将 `physical_stop_confirmed` 保持为 `false`，外层脚本另外验证停车窗口。
- 激光净空不是杯子表面距离。本轮结果明确 `target_arrival_confirmed=false`、`target_association=unknown`。
- 相机 `observed_at` 使用 Unix 时间；相机 header 源采样时间和排队延迟映射仍未独立验证。
- 独立 `wsl_approach_scenarios.sh` 的 simple/hierarchical 16 场景矩阵本轮未运行；接近由单测和 6 个组合场景覆盖，不将未运行矩阵写成通过。
- 未连接在线模型，也未操作真机；模型故障使用离线模拟。
