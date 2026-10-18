# 规划进展

## 2026-10-18
- 用户要求详细下一步任务清单，交由 Luna 后续执行，并回查此前遗漏。
- 确认代码仓库 `F:/AI-Projects/ROS2-Embodied-Robot-Agent` 基线为 `292d3a7`，工作区原先干净。
- 本轮仅修改规划文档，不修改运行代码，不重跑 ROS/Gazebo 测试。
- 两项定向只读复核交由 Luna：取消并发；感知与接近流程。
- 两项交叉复核已收回；主代理核对了关键代码路径和竞争窗口。
- 形成 13 项下一轮任务，含优先级、依赖、改动范围、逐项验收和分批安排。
- 新增发现包括：approach 停止失败绕过门禁、Nav2 初次/最终停止之间的准入窗口、并发停止归属、单槽回执覆盖、scope 重试盲点、scan 源时间缺口。
- 上轮 184 项测试与 6 包构建、层级取消仿真通过作为历史证据保留；本轮不重复宣称新测试通过。
- 本轮改动仅为此目录的 task_plan.md、findings.md、progress.md；13 项编号连续性、文件存在性和工作区差异检查通过。未运行代码测试，待 Luna 实现后逐项验收；文档将单独提交推送。

## 用户授权开始实施
- 从 `8835531` 干净工作区继续，用户要求由 Luna 子代理执行。
- Luna execution：任务 1/2/7，随后处理同一核心文件上的 5/9。
- Luna robot_tools：任务 3/4，状态 ID、回执记录、scan 时间域与一致快照。
- Luna scripts/schema：任务 6/8、仿真启动时钟参数。
- 主代理：规划状态、交叉核对、测试/构建/仿真、文档及 Git 交付。
- 子代理只编辑各自负责文件；定向 ROS 测试分别使用域 61/62/63；全仿真统一串行执行。
- 当前没有 Sol 授权请求；未安排在线模型或真机操作。
- 状态/scan 第一轮定向测试 30 项通过；pytest 环境有插件冲突，关闭自动加载后正常。交叉检查要求补充状态时间戳在 ROS clock reset 后的恢复，最终结果待收回。
- 集成检查新增：同一取消初次/最终停车间持续保有门禁；未知 UUID 取消只记录其作用域；其他任务活动期间也需记住取消先到的 UUID；导航 params.timeout 必须传到执行层。
- 仿真启动将统一 agent/tool/control 的 use_sim_time；视觉 Unix observed_at 仍独立，不声称相机源采样时间已验证。
- 执行/取消第一批定向测试 32 项通过（ROS_DOMAIN_ID=61）；继续动作约束/结果证据及新的并发交错用例。
- robot_tools 完成状态 ROS clock epoch 重置与跨回退失败关闭；定向 33 项通过，另以 unittest discovery 核验 motion 12、range 13、navigation 8 项均通过。
- scripts/schema 定向首轮 44 项中 3 项失败来自测试 fake 未支持新增 command_id，正在更新测试接口；移动准备 predicate 的早完成误判也已要求修复。

## 本轮实施与验收结果
- Luna 执行/取消、robot_tools 状态与测距、脚本与 schema 三组改动已汇总；GPT-6.1 Sol 未调用，无待授权任务。
- 定向单测完整执行 `scripts/run_tests.sh`：robot_control 11、robot_tools 56、robot_agent 135、vision 6，共 208 项通过。
- `scripts/wsl_build.sh` 最终构建通过，6 个 ROS2 包均完成。
- `wsl_navigation_cancel_scenarios.sh simple hierarchical` 通过：Nav2 CANCELED、控制 stop 确认、停止后 odom 窗口和后续导航均验证。
- 组合 Gazebo 的 success、navigation_failure、target_absent、blocked、vision_dropout、range_dropout 六场景通过外层断言。成功结果保留 target_arrival_confirmed=false 和 physical_stop_confirmed=false；停车后观测由外层里程计单独核验。
- 集成首次暴露安全制动最短步长边界低估，已将停止边界改为目标停止距离+制动余量+两个控制周期距离。重新跑单测、构建和完整 success 场景通过；失败过程记录在 validation.md。
- 工作区交付前仍需最终 diff/status 检查，提交并推送；独立 16 项 approach 场景矩阵本轮未执行。
