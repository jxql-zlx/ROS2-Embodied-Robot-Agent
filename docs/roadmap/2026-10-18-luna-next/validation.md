# ROS2 加固实施验证记录

早期实施证据从 `8835531` 开始；本次追加复核基线为 `aae2bd4`（2026-10-18）。下表中的补充项目在该基线上重新运行。每个结论只覆盖列出的测试和仿真范围。

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
| 单测总集 | `scripts/run_tests.sh` | 本次最终复核通过：238 项（control 11、tools 66、agent 149、vision 12） |
| ROS2 构建 | `scripts/wsl_build.sh`，6 个包 | 本次复核通过：6 packages finished |
| 导航取消 | simple/hierarchical CANCELED、停车观测、后续导航 | 本次复核通过：两模式均观察到 CANCELED；停止后均有 51 个新鲜 odom 样本，漂移 0.000 m，follow-up 导航成功 |
| 组合成功 | 导航→YOLO→接近净空→停车观测 | 本次复核通过：YOLO 找到 cup；接近阶段安全停车并明确未确认目标到达；停车窗口 23 个新鲜样本，漂移/偏航均 0 |
| 接近场景矩阵 | simple/hierarchical × 8 个场景 | 本次复核通过：left、right、near、far、blocked、absent、vision_dropout、range_dropout，共 16 项 |
| 视觉 smoke | Gazebo camera→vision node→检测 JSON | 本次复核通过：`use_sim_time` 下获得 verified 源时间消息并解析 JSON；空场景对象数为 0，目标检测由组合场景另行验证 |
| 故障组合 | 导航失败、无目标、障碍、视觉/激光断流 | 通过：五种场景分别失败关闭；停止后最大观测漂移约 0.004 m |
| Git 交付 | diff 检查、提交、推送和状态 | 前一轮实现 `66a2d7e` 已在 `origin/main`；本次追加改动尚待提交和推送 |

## 集成运行日志

- 本次 Nav2 取消：`/tmp/ros2_navigation_cancel/run.qIfGU0`
- 本次组合成功：`/tmp/ros2_composed_navigation/success.UahCpz`
- simple 接近矩阵：`/tmp/approach_scenarios/run.7mK9Ni`
- hierarchical 接近矩阵：`/tmp/approach_scenarios/run.kPlqHD`
- 本次视觉 smoke：`/tmp/vision_smoke.log`、`/tmp/vision_node.log`
- 全量单测输出：`/tmp/ros2-tests-final.log`
- simple 中心场景初次启动曾在 clock warmup 中过早发任务；就绪门禁修复后的通过日志保存在 `/tmp/vision_action_demo/simple_center.4NqJbY`

早期集成运行日志（前一轮运行，不代表本次重跑）：
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
- 接近净空来自前向激光，不是杯子表面距离。结构化结果保持 `target_arrival_confirmed=false` 与 `target_association=unknown`；因此接近任务返回未完成，即便安全停车和控制层停止确认成功。
- `physical_stop_confirmed=false` 是任务回执中的明确值；仿真脚本单独通过连续 odom 窗口确认对应窗口内静止，不把该证据写进控制层结果。
- 相机源时间现用 ROS 图像 header stamp 与视觉节点 ROS clock 校验；Gazebo 端到端拿到 `source_time_status=verified`。外部硬件相机若使用不同时间域，需先映射，不能直接复用该验证结论。
- 前向 ROS clock jump 阈值从 10 ms 调至 1 s，避免将常规仿真 tick 当成跳变；显著回退和时钟源切换仍会重置缓存。运行时 `robot_control.update_rate` 可配置，而 approach 余量仍按默认 10 Hz 常量计算，本轮没有动态绑定此参数。
- 16 个独立接近场景本次已运行并通过；不覆盖这些故障的任意组合，也不证明真机性能。
- 未连接在线模型，也未操作真机；模型故障使用离线模拟。
