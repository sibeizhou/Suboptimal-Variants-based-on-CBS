# 面试讲解指南

## 30 秒版本

这个项目解决的是 Multi-Agent Path Finding：在有障碍物的网格地图里，为多个智能体规划从起点到终点的无碰撞路径。我实现并对比了 CBS、BCBS 和 ECBS。CBS 保证最优但搜索开销大；BCBS/ECBS 用 focal search 和冲突启发式在高层/低层搜索中做 bounded-suboptimal 取舍，用少量路径代价损失换更少的节点扩展和更快的运行时间。

## 可以重点讲的技术点

- **问题建模**：网格地图、起终点、障碍物、顶点冲突、边冲突。
- **CBS 高层搜索**：每个 constraint-tree 节点保存 constraints、paths、collisions、cost；发现冲突后分裂约束并只重规划受影响 agent。
- **低层规划**：A* 扩展到 space-time domain，状态包含 `(cell, timestep)`，允许 wait action 来处理时序冲突。
- **BCBS/ECBS 优化**：用 focal list 选择“足够接近最优”的节点，再用冲突启发式排序，降低搜索成本。
- **评估指标**：sum of costs、CPU time、expanded nodes、generated nodes。

## 面试官可能追问

**Q: CBS 为什么能保证最优？**  
高层 open list 按当前路径总代价排序，每次扩展最低 cost 的 constraint-tree 节点；低层 A* 返回满足当前约束的最短路径。因此第一个无冲突节点就是当前约束空间下的全局最优 MAPF 解。

**Q: BCBS/ECBS 为什么更快？代价是什么？**  
它们不总是扩展最低 cost 节点，而是在 `w * lower_bound` 范围内选一个启发式更好的节点。这样通常能更快减少冲突，但路径总代价可能高于最优 CBS。

**Q: 你如何验证路径正确？**  
项目里有 smoke tests，会加载小实例，运行 CBS/BCBS/ECBS，并断言每条路径从对应起点到终点且 `detect_collisions(paths)` 为空。

**Q: 如果继续改进，你会做什么？**  
我会先把实验输出改为结构化 CSV/JSON，补更多 benchmark 自动化；然后加入更强的 conflict selection 策略，比如 cardinal conflict 优先；最后把代码整理成 package，避免当前历史目录名中带空格。

## 简历项目描述

Implemented CBS, BCBS, and ECBS solvers for Multi-Agent Path Finding in Python, using space-time A* and focal-search heuristics to reduce constraint-tree expansion while preserving collision-free multi-agent paths.

## Demo 命令

```bash
cd "Final Project/code/code"
python run_experiments.py --instance "instances/test_40.txt" --solver CBS --method CBS --batch
python run_experiments.py --instance "instances/test_40.txt" --solver CBS --method BCBS --hc 1 --high-weight 1.5 --low-weight 1.5 --batch
```
