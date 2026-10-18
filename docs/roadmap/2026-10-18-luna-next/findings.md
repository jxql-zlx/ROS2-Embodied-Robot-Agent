# 复查发现

基线：`292d3a7`，2026-10-18。以下结论来自静态代码复查；前轮执行证据单独注明。

## 已确认
- 接近流程直接调用 robot.stop/confirmation，失败只进入消息，没有设置 node 的 cancellation_pending；handle_task finally 清 active 后可接受新任务。证据：`approach.py:32–57`、`robot_agent_node.py:446–461`；任务 1。
- Nav2 先终结时，初次 stop 确认可能清门禁，而取消代码随后还会执行最终 stop。窗口内新任务可进入，最终 stop 可影响它。证据：`robot_agent_node.py:183–201,251–258,326–348`；任务 2。
- 普通失败自动 stop 与显式取消的确认等待可并发，各自清同一布尔门禁；单槽 latest_status 还可能覆盖另一命令回执。证据：`robot_agent_node.py:204–259,300–303`、`ros2_robot_tool.py:100–103,176–231`；任务 2、3。
- 同 UUID 取消重试在原任务 finally 清 active_task_id 后早退为“已结束”，即使 pending cancellation 仍存在。证据：`robot_agent_node.py:139–147,339–348`；任务 7。
- scan 只检查非零/重复 stamp，并以接收年龄判定新鲜；递减 stamp 无条件被视作时钟重置。积压旧帧可成为当前净空证据。证据：`ros2_robot_tool.py:105–123,490–500`；任务 4。
- approach 仅特别处理 motion.action == stop，其余 ActionPlan 动作可到 dispatch；ActionPlan 允许 navigate/approach/vision。接近阶段尚无独立动作允许列表。证据：`approach.py:141–190`、`action_plan.py:12,37–52`；任务 5。
- 取消仿真脚本无论模式都发送组合任务，并按列表处理取消结果；简单模式需要独立导航输入和对象结果校验。仅更改输入不足以修复整个脚本。
- 服务客户端超时取消等待上限为 8 秒，而服务端可能依次等待初次停止确认 2 秒、Nav2 终结 8 秒、最终停止确认 2 秒；需统一期限和结果含义。
- topic 模式只能保证客户端调用一次 publish，不能据此宣称可靠送达或业务上只执行一次。

## 证据不足的语义与待验证风险
- 最近前向扫描距离不能证明属于目标；目标居中和距离阈值不足以验证目标到达。当前 `approach.py:130–133,167–170` 会报告到达。任务 9 先明确语义，精确融合留待传感器条件确认。
- RobotStatus stopped 来自零 Twist 发布和控制状态，不含物理静止观测；上一轮仿真额外检查 odom，不等于生产路径已经使用同一证据。
- 取消先于执行请求被处理、相同 task ID 重复提交、复用层级 agent 后旧任务残留，需新增确定性复现，不在静态检查阶段声称已实验验证。
- 简单模式接近会经 node 默认等待运动完成后，再由 ApproachController 等待；应统一等待归属。当前终态仍在时可能立即通过，不把它单独认定为必然超时。
- 名称匹配允许子串，目标存在性与 objects[0] 方向不必指向同一对象；复查时应保持单一目标引用并覆盖错误适配器输出。

## 审查范围与限制
两名 Luna 分别独立只读复核取消/并发和感知路径，主代理核对代码与仿真脚本、客户端、计划校验。以上并发缺陷由代码交错分析确认可达，本轮没有跑新的复现程序、单元测试或仿真。不能用静态复查结论替代修复后的运行验收。

## 前轮执行证据（本轮未重新运行）
- 全量单元测试：184 项通过。
- ROS2 Humble：6 个包构建通过。
- 层级导航取消仿真通过：`/tmp/ros2_navigation_cancel/run.GWNdcz`。
- 简单模式取消仿真两次失败：`/tmp/ros2_navigation_cancel/run.DMHVfR`、`/tmp/ros2_navigation_cancel/run.GPuBLl`。
