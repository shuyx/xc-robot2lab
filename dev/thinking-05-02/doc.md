---
date: 2026-05-25
tags: [祥承电子, Xc-Robot, 技术路线, RD2, 运动控制, QDD, 论文综合, 方法论]
machine: mac-minishu
agent: claude-code-opus47-via-sonnet46
session: xc-robot-skill
has_diagrams: false
type: 方法论文档
status: 草案待评审（17 篇论文 × 4 目标矩阵 + 三阶段路线）
---

# RD2 阶段技术路线 — 基于 12+13 论文综合

## 元信息

- **来源**：Sonnet 4.6 agent 通读 `03｜技术资产库/02｜技术资产/12｜运动控制论文`（3 篇）+ `13｜QDD关节控制论文MD`（13 篇 + 1 综述）共 17 篇，结合 XC-Robot 硬件软件基线评估
- **目标**：覆盖运动规划 / 控制架构 / 防抖 / 精度 4 个 RD2 阶段（2026-05~08）核心目标
- **约束**：不换硬件（灵足 RobStride 9:1 全系单磁编码器 + MIT mode）；不依赖关节力矩传感器；与现有 ROS2 + openarmx_xc_robot_2 栈兼容

## 一、17 篇论文 × 4 目标 — 适用矩阵

| 论文 | 运动规划 | 控制架构 | 防抖 | 精度 |
|------|---------|---------|------|------|
| **P3 Ruckig**（OTG，MIT 开源 C++）| ✅ OTG 插值层 | — | ✅ Jerk 限制消冲击 | 🟡 间接 |
| **P9 DynamicParamID**（OLS+SDP+CLIE）| — | ✅ 重力补偿前提 | — | ✅ 模型精度 |
| **P4 AdaptiveFriction**（Stribeck 在线辨识）| — | ✅ 摩擦补偿 | ✅ 低速 Stribeck | ✅ |
| **P1 CRISP**（ROS2 阻抗框架）| 🟡 | ✅ 阻抗一体化 | ✅ | ✅ |
| **P8 DualArm 摩擦观测器**（模型无关）| — | ✅ 双臂柔顺 | ✅ | ✅ |
| **P5 AhaRobot dithering** | — | 🟡 | ✅ 零速区静摩擦 | — |
| **P2 Hybrid-ID + LSTM** | — | ✅ | ✅ 极限 | ✅ |
| **P17 PINN-UKF** | — | 🟡 | 🟡 | 🟡 |

## 二、阶段化路线（短/中/长期 × 4 目标）

### 🟢 短期（1–3 周，零硬件改动，立即可启动）

#### S-1 集成 Ruckig OTG（P3）

- **解决目标**：抖动 / 运动规划
- **解决 D-1 根因**：#3「轨迹缺时间轴」+ #1「ROS2 二级规划过简」+ #2「未启用 MoveIt Servo 缺平滑」
- **为何有用**：当前 100 Hz 离散下发是抖动直接根因。Ruckig 是 MIT 开源 C++（已被 MoveIt Servo 内置），插入规划层只需 1–2 天。10 亿轨迹测试通过 + 纯运动学，**不依赖动力学模型，不依赖力矩传感器**，与 XC-Robot 硬件 0 冲突。
- **前置约束**：无。当前可直接做。
- **风险**：CAN 1 Mbps 在 200 Hz 上行 7 轴可能逼近带宽极限（NERO 实测 200 Hz 已为 CAN 1 Mbps 上限）— 上 500 Hz 前 **必须先做 CAN 带宽实测**。

#### S-2 gravity_comp 单臂 YAML（基于 P9 DynamicParamID 方法论 OLS+SDP+CLIE）

- **解决目标**：控制 / 精度
- **解决 D-1 根因**：#7「URDF 模型存在误差」
- **为何有用**：P9 专为 7-DoF 低成本臂（CRANE-X7 Dynamixel）设计，**仅用电流指令 + 编码器**，与 RobStride MIT 模式天然兼容，最终 RMSE ≈ 0.168 Nm。直接解决从 2026-05-23 持续挂起的 gravity_comp YAML。
- **前置约束**：需要采集 5–10 条结构化激励轨迹（单关节 + 相邻关节）。
- **同步动作**：把 URDF 动力学参数从 CAD 估算切到 P9 流水线实测值。

#### S-3 AhaRobot dithering 实验（P5 借鉴）

- **解决目标**：防抖（零速区静摩擦）
- **为何有用**：高频交替方波 u_d 克服静摩擦的原理通用，**与硬件无关**，可在 MIT mode 上以软件层注入。AhaRobot 在 66 Hz PID 都奏效，XC-Robot 100 Hz 完全可行。
- **风险**：可能与 RobStride 内部电流环交互产生新振，需小幅度试。
- **优先级**：在 S-1 完成后做（OTG 改善宏观抖动，dithering 处理零速区残留）。

### 🟡 中期（1–3 个月，需要采集数据 + 离线计算）

#### M-1 AdaptiveFriction 在线 Stribeck 辨识（P4）

- **解决目标**：防抖 / 精度
- **解决 D-1 根因**：#6「无双编码器无法准确反映电驱」的**软件层补救**（不靠双编码器，靠在线模型辨识）
- **为何有用**：只需位置+速度（RobStride 单磁编码器够），backstepping 自适应控制器在线运行；激励轨迹优化（条件数 5898→1819）保证辨识鲁棒；KUKA iiwa 实机验证。
- **前置**：需先有 gravity_comp（S-2 完成）。
- **配合**：S-3 dithering 是"硬补"，M-1 AdaptiveFriction 是"软调"，二者互补。

#### M-2 阻抗控制框架（**二选一**）

- **选 P1 CRISP**：ROS2 原生，重力 + Coriolis + 摩擦 + barrier 一体化，1 kHz 运行，Franka 实机。**优点**：最全面、可作为未来标准框架。**缺点**：需要先达到 500 Hz 才能跑出原文效果，移植到 100/200 Hz 需调参。
- **选 P8 DualArm 摩擦观测器**：模型无关摩擦观测器（比较名义角 vs 实测角差分估摩擦），**无需关节力矩传感器**，双臂场景天然匹配 XC-Robot。**优点**：工程量小，硬件契合度高。**缺点**：是局部组件不是完整框架。
- **主线程建议**：先 P8 摩擦观测器（轻量起步），结构稳定后再迁 P1 CRISP。

### 🔵 长期（3–6 个月或更久，需要 GPU + 大量数据）

#### L-1 P2 Hybrid-ID + LSTM 残差

- **何时启用**：S/M 阶段全做完，仍有 stick-slip 等滞后效应残留时
- **门槛**：需 GPU 离线训练 + 足量激励轨迹数据集
- **收益**：跟踪误差大幅降低，阻抗增益可降 60% 达同精度

#### L-2 P17 PINN-UKF（中等优先）

- **当前限制**：原文针对**高减速比谐波**机器人（ergoCub），9:1 行星摩擦特性差异大，不能直接迁移
- **何时再评估**：M-1 AdaptiveFriction 跑稳后，如果摩擦模型仍有未捕获模态再上 PINN

## 三、明确放弃 / 不投资源

| 论文 | 不适用原因 |
|------|-----------|
| **P7 摆线 QDD** | XC-Robot 用行星减速器，硬件路径完全不同 |
| **P13 行星减速器选型** | 9:1 RobStride 已锁死，无重选场景 |
| **P14 ProprioceptiveActuator** | 仅理论参考价值，不直接产出工作 |
| **P15 Blue 臂 QDD 操作** | 架构参考已读完，无更多可借鉴；Blue 项目本身已停滞 |
| **P16 QDD Hip Exoskeleton** | 同上 |

## 四、综合排序：本月（2026-05）启动建议

| 周次 | 动作 |
|------|------|
| 第 1 周 | **S-1 Ruckig 集成**（最低门槛、最高收益、立即降抖动）|
| 第 1–2 周并行 | 先做 **CAN 带宽实测**（决定能否从 100→200/500 Hz）|
| 第 2–3 周 | **S-2 gravity_comp YAML**（用 P9 方法论；解决最久挂起项）|
| 第 3–4 周 | **S-3 dithering** 小幅度试 + 准备 M-1 激励轨迹采集 |

## 五、为什么这套路线对 XC-Robot 一定有用

四条硬约束都满足：

1. **零硬件改动**（S/M 全程）
2. **不依赖关节力矩传感器**（XC-Robot 没有）
3. **不依赖双编码器**（XC-Robot 是单磁编码器；P4/P8 都用模型补救）
4. **与 MIT 模式 tau 前馈兼容**（P1/P4/P9 全部基于电流接口）

每一步都直接打中 D-1 八大根因之一，且**软件改造为主、不与 B-2/B-3 硬件决策冲突**。

## 六、参考资料

### 论文原文位置

- 12 文件夹（运动控制论文 3 篇）：`03｜技术资产库/02｜技术资产/12｜运动控制论文/`
  - `CRISP-ROS2-compliant-controllers-2509.06819.md`
  - `Hybrid-InverseDynamics-friction-2205.13804.md`
  - `Ruckig-jerk-trajectory-2105.04830.md`
- 13 文件夹（QDD 论文 13 + 综述 1）：`03｜技术资产库/02｜技术资产/13｜QDD关节控制论文MD/`
  - 子文件夹：AdaptiveFriction / AhaRobot / CartesianImpedance / CycloidalQDD / DualArm-compliant-control / DynamicParamID-LowCostArm / ImpedanceQDD / MiniCheetah / ODRI-torque-modular-robot / PlanetaryGearbox / ProprioceptiveActuator-MITCheetah / QDD-compliant-manipulation / QDD-hip-exoskeleton / Sensorless-torque-PINN-UKF
  - 综述：`QDD论文深度研读报告-机械臂运动控制与规划.md`

### 项目硬件软件基线

- xc-robot SKILL.md「技术决策史」节（B-1 主控 / B-2 模组 / B-3 机械臂 / B-4 VLA）
- xc-robot SKILL.md「判断框架」节（D-1 抖动八大候选根因）
- `06｜全库汇总总览/1-5_配置基线与版本说明.md`
- NotePlan `📌 项目概览 — 祥承 Xc-Robot.md`

### 待办挂起项追溯

- gravity_comp YAML 自 2026-05-23 持续挂起 — 见 `_会话脉络.md` 2026-05-24 / 25 多条记录
- update_rate 100→200 Hz — 已确认零代码改动，优先级最高
- DynamicParamID URDF 标定 — 参考 P9 论文，待排期

## 七、下次评审动作

- [ ] 用户评审本路线 → 决定先走 S-1 还是 S-2
- [ ] CAN 带宽实测排期
- [ ] 采集 P9 激励轨迹的硬件准备
- [ ] 若 P8 / P1 二选一未定，需做 ROS2 实测 demo 对比
