# Resume-ready project entry

**基于 LLM Agent 与语义地图的移动机器人自主导航系统** · C++ / ROS2 / Nav2 / Gazebo / LangGraph / YOLO / OpenCV

- 设计分层自主导航闭环：将自然语言任务解析为语义目标序列，结合语义地图检索、Frontier 主动探索、A* 全局规划与 Nav2 MPPI 执行，并输出结构化状态轨迹。
- 实现 RGB-D + YOLO + TF2 的目标融合接口，对多帧观测进行置信度加权并关联几何占据栅格；以信息增益、语义相关性、路径代价和风险构建 frontier utility。
- 通过 ROS2 Action/Nav2 adapter 监控导航反馈，在目标不可达、超时或局部恢复失败时触发候选目标切换与任务级重规划；统一输出成功率、SPL、路径长度和重规划耗时指标。

仓库中的 `nav_demo` 是无 GPU、无 ROS2 依赖的确定性回放，用来验证核心算法接口；真实 Gazebo/机器人指标应在目标环境中重新测量。

