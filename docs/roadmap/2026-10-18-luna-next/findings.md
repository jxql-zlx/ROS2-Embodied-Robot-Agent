# 复查发现

基线：`292d3a7`，2026-10-18。以下“实施前缺陷”记录当时的静态复查；修复和本轮证据见 `validation.md`。

## 实施前已确认、随后修复
- 接近流程直接调用 robot.stop/confirmation，失败只进入消息，没有设置 node 的 cancellation_pending；handle_task finally 清 active 后可接受新任务。证据：`approach.py:32–57`、`robot_agent_node.py:446–461`；任务 1。
- Nav2 先终结时，初次 stop 确认可能清门禁，而取消代码随后还会执行最终 stop。窗口内新任务可进入，最终 stop 可影响它。证据：`robot_agent_node.py:183–201,251–258,326–348`；任务 2。
- 普通失败自动 stop 与显式取消的确认等待可并发，各自清同一布尔门禁；单槽 latest_status 还可能覆盖另一命令回执。证据：`robot_agent_node.py:204–259,300–303`、`ros2_robot_tool.py:100–103,176–231`；任务 2、3。
- 同 UUID 取消重试在原任务 finally 清 active_task_id 后早退为“已结束”，即使 pending cancellation 仍存在。证据：`robot_agent_node.py:139–147,339–348`；任务 7。
- scan 只检查非零/重复 stamp，并以接收年龄判定新鲜；递减 stamp 无条件被视作时钟重置。积压旧帧可成为当前净空证据。证据：`ros2_robot_tool.py:105–123,490–500`；任务 4。
- approach 仅特别处理 motion.action == stop，其余 ActionPlan 动作可到 dispatch；ActionPlan 允许 navigate/approach/vision。接近阶段尚无独立动作允许列表。证据：`approach.py:141–190`、`action_plan.py:12,37–52`；任务 5。
- 取消仿真脚本无论模式都发送组合任务，并按列表处理取消结果；简单模式需要独立导航输入和对象结果校验。仅更改输入不足以修复整个脚本。
- 服务客户端超时取消等待上限为 8 秒，而服务端可能依次等待初次停止确认 2 秒、Nav2 终结 8 秒、最终停止确认 2 秒；需统一期限和结果含义。
- topic 模式只能保证客户端调用一次 publish，不能据此宣称可靠送达或业务上只执行一次。

## 当前仍需保持的证据边界
- 最近前向激光只能提供净空，不能证明回波属于目标。本轮改为只报告安全停车和未确认目标到达；精准目标距离/相机雷达融合仍需传感器标定与独立设计。
- `RobotStatus stopped` 表示控制层处理了零速度命令，不代表轮子已物理静止。当前任务回执保留 `physical_stop_confirmed=false`；仿真脚本在独立连续 odom 窗口验证停止。
- 16 个 Gazebo 接近场景、两模式 Nav2 取消、组合导航成功和 vision smoke 均已验证；不覆盖任意多故障组合，不替代真机验证。

## 审查范围与限制
两名 Luna 分别复核取消/并发和感知路径，主代理完成整合测试和 Gazebo 场景运行。实施前的交错分析用于定位缺陷；修复后运行证据以 `validation.md` 所列实际结果为准。

## 前轮执行证据（本轮未重新运行）
- 全量单元测试：184 项通过。
- ROS2 Humble：6 个包构建通过。
- 层级导航取消仿真通过：`/tmp/ros2_navigation_cancel/run.GWNdcz`。
- 简单模式取消仿真两次失败：`/tmp/ros2_navigation_cancel/run.DMHVfR`、`/tmp/ros2_navigation_cancel/run.GPuBLl`。

## 2026-10-18 追加复核
- 实施前列出的取消、回执、激光时效、动作约束、目标语义、脚本和任务校验问题已按计划修复；定向测试和运行证据见 `validation.md`。
- 新发现：vision 与 robot_tools 的 `JumpThreshold.min_forward=10 ms` 会把 Gazebo 正常时钟步进当作时钟跳变，持续重置图像预热和传感器缓存；前跳阈值现为 1 s，并通过视觉 smoke 和 16 个 Gazebo 接近场景验证。显著回退和时钟源切换仍重置会话。
- 新发现：容差内小幅未来相机源时间会使组合年龄为负并被接近控制器拒绝；视觉工具现将容差范围内的年龄钳制为 0，超出容差仍失败关闭。
- 停止距离参数现要求有限正数；0、负数、NaN、Inf 均拒绝，失败路径只发布停止命令。
- 剩余限制：前向激光净空不能证明目标到达；`robot_control.update_rate` 可覆盖，但 approach 安全余量仍按默认 10 Hz 常量计算。
