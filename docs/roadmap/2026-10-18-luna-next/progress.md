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
- `66a2d7e fix: harden ROS2 task cancellation and approach safety` 已推送 `origin/main`，推送后检查工作区干净；独立 16 项 approach 场景矩阵本轮未执行。

## 2026-10-18 追加复核与验收
- 从 `aae2bd4`（与 `origin/main` 同步、工作区干净）开始本轮实现；Luna 子代理分别完成 approach 契约、取消生命周期、ROS/视觉时效性和视觉 smoke 启动参数。
- 全量测试最初发现 6 个旧用例仍把 safe-stop 当成 approach success；更新单项和 hierarchical 的结果断言后，最终 `scripts/run_tests.sh` 通过 236 项：robot_control 11、robot_tools 65、robot_agent 148、vision 12。
- 全量 `scripts/wsl_build.sh` 最终构建通过，6 个 ROS2 包完成。
- 修复额外发现的仿真时钟回调问题：10 ms 前跳阈值会把常规 10 Hz ROS clock tick 当作跳变并持续清缓存；vision 与 robot_tools 改用 1 s 前跳阈值，时钟源切换和显著回退仍触发新 epoch。
- 将容差内小幅未来相机时间的计算年龄归零，保留超过容差/过期数据拒绝；增加真实 Gazebo 可信帧就绪门禁，避免在 clock warmup 期间提前提交任务。
- simple 与 hierarchical 接近矩阵共 16 场景通过；包含 left/right 方向、近/远、阻挡、目标缺失、视觉断流、测距断流。日志分别为 `/tmp/approach_scenarios/run.7mK9Ni`、`/tmp/approach_scenarios/run.kPlqHD`。
- Gazebo vision smoke 通过；组合导航→YOLO→接近安全停车通过，结果明确目标到达未确认，连续停车窗口 23 个新鲜 odom 样本、漂移和偏航为 0。
- simple/hierarchical Nav2 cancel 回归通过：取消目标 CANCELED、follow-up 导航成功、停止后各 51 个新鲜 odom 样本且漂移 0；日志 `/tmp/ros2_navigation_cancel/run.qIfGU0`。
- 保留重要限制：前向激光净空不证明杯子到达；approach 周期余量仍按默认 10 Hz，未读取可覆盖的 `robot_control.update_rate`；未连接在线模型或真机。
- 最终复核发现停止距离参数需要额外输入校验；0、负数、NaN/Inf 现在失败关闭且不派发运动。`test_approach_scenarios.py` 定向 21 项通过；之后重跑全量单测共 237 项通过，6 包构建通过。
- 最后审查增加非对象视觉场景 fail-closed 校验及回归用例；视觉工具定向测试通过（14 项）。全量测试基于前述 237 项结果，加该新增用例共 238 项。
- 当前尚未创建本次提交或推送；待最终 review 后由主代理按要求完成 Git 收尾。无等待 GPT-6.1 Sol 授权项。
