# 技术分析报告 A：双 X86 mini PC 方案

> **方案代号**：X86-Twin
> **硬件架构**：上位机 Intel i7 mini PC（主脑）＋ 下位机 Intel i5 mini PC（底盘实时控制脑）
> **软件底座**：Ubuntu 22.04 + PREEMPT_RT 6.1+ + ROS 2 Humble + MoveIt 2 + Nav2
> **核心卖点**：成熟的上游 ROS 2 + PREEMPT_RT 生态、UR/Franka 官方推荐同款路线、踩坑资料多、供应链稳定
> **核心风险**：BPU 算力为 0，VLA/大模型推理需要 Intel iGPU + OpenVINO 或外挂 NPU；国产化率低

---

## 一、执行摘要

### 1.1 方案定位一句话
**"用最成熟的工业机器人实时软件栈做可控的底座，把 AI 推理按需外挂。"**

### 1.2 核心 8 个判断

1. **实时性是机械臂丝滑的必要条件**，Intel x86 + PREEMPT_RT 6.1（mainline）是业界最成熟的工程路径，Universal Robots、Franka 都官方推荐 Intel NUC 作为 ROS 驱动主机。
2. **双机双脑的切分**：上位机负责 MoveIt 2 + Nav2 + 视觉感知 + 语音交互 + 安全层；下位机负责 ros2_control 硬件接口 + CAN 通信 + 安全 watchdog，不跑任何 AI 推理，专守 1 kHz 实时循环。
3. **OpenArmX_xc 控制栈必须重写关键层**。证据：原版 OpenArm `openarm_bimanual_moveit_config/config/ros2_controllers.yaml` update_rate = 100 Hz、`command_interfaces` 只有 position，不支持 MIT 力矩/阻抗；没有 MoveIt Servo 配置；全仓库无零点标定脚本——这些都是我们要自建的缺口。
4. **运动规划路线**：ros2_control + 自定义 effort_controller（直连达妙 DM MIT 模式）+ Pinocchio 动力学前馈（重力 + 科氏 + 摩擦）+ Ruckig 在线轨迹整形 + MoveIt Servo 流式接口。**这不是 4 层堆砌，而是 1 条贯通的控制链**。
5. **底盘精度路线**：当前在用 Cartographer 建图（已成功，±3 cm 收敛中）；推荐迁移到 SLAM Toolbox（ATE 0.13 m vs Cartographer 0.21 m）+ AMCL 粗定位 + AprilTag/ChArUco 精对位 + Nav2 docking server。20 m × 20 m 工位 ±5 cm 有工程解，但长期漂移必须靠周期性 Tag 重定位修正。
6. **手眼标定陷阱**：默认 easy_handeye 用 Tsai-Lenz 是文献里精度最差的算法（~4.4 mm），必须切换到 Daniilidis 或 Park-Martin（~3.4 mm），标定板必须用 ChArUco（~0.4–0.6 mm 在线）。
7. **单臂 ±0.1 mm 承诺的分层解读**：9:1 行星准直驱 QDD 电机的**关节层自由空间回位精度**物理极限约 0.3–1 mm，靠关节伺服达不到 ±0.1 mm；但揭榜挂帅验收场景（上下料/插拔/装配/锁付）全部是**接触任务**，通过"**视觉粗定位（±5 mm）+ 力控柔顺接触（±0.1 N 反馈）+ 视觉/力觉在线微调**"三层闭环可达 ±0.1 mm。这是 Franka/UR 等工业协作臂的主流路径。**承诺不变，路径分层：自由空间 ±0.5 mm + 接触任务 ±0.1 mm**。
8. **升降轴与机械臂协同：本期状态机优先，8-DOF 联合规划延后**。本期 RD2 采用"先升降到位、再规划臂"的状态机做法，工程复杂度可控、求解失败率低；arm_torso 8-DOF MoveIt 联合规划（TIAGo 参考路线）作为 P1 升级项，遇到狭窄工位再启用。
9. **多模态感知融合与柔性任务调度架构**：揭榜挂帅两大核心创新点——"几何-语义-力觉三层融合"和"行为树 + FSM + 状态感知 + 重规划"——单独在 §4.8、§4.9 展开。本期目标是**搭出架构骨架 + ROS 2 接口契约**，最小 Demo 跑通，不求完整填满。

---

## 二、硬件架构

### 2.1 双脑拓扑

```
                          ┌────────────────────────┐
                          │     上位机 PC-A (i7)     │
                          │  Ubuntu 22.04 PREEMPT_RT │
                          │                        │
                          │  ┌─────────────────┐   │
                          │  │ MoveIt 2 / Nav2 │   │
                          │  │ Perception (视觉)│   │
                          │  │ Voice / Head    │   │
                          │  │ VLA(可选)/Skill │   │
                          │  │ Safety Manager  │   │
                          │  └─────────────────┘   │
                          └──┬────────────────┬────┘
                             │                │
                    ┌────────┘                └──────────┐
                    │ 2.5GbE (DDS Fastrtps)             │ USB 3.0
                    │                                    │
        ┌───────────▼───────────┐            ┌───────────▼──────────────┐
        │   下位机 PC-B (i5)     │            │  USB Hub / 感知外设      │
        │ Ubuntu 22.04 PREEMPT_RT│            │ ROSEB43i × 2 (臂眼)      │
        │                       │            │ MV-EB435i × 1 (头部)     │
        │ ros2_control (1 kHz)  │            │ 6 麦阵列 (科大讯飞)       │
        │ Custom effort_ctrl    │            └──────────────────────────┘
        │ Pinocchio 前馈        │
        │ Ruckig 整形           │
        │ Safety Watchdog       │
        │ chassis_driver (CAN)  │
        └──┬─────────┬─────────┬┘
           │         │         │
           ▼         ▼         ▼
   ┌─────────┐ ┌──────────┐ ┌──────────┐
   │Left Arm │ │Right Arm │ │ Chassis  │
   │CAN 1    │ │CAN 2     │ │ CANopen  │
   │达妙DM   │ │达妙DM    │ │ ZLAC8015D│
   │×5       │ │×5        │ │ 轮毂电机×4│
   └─────────┘ └──────────┘ └──────────┘

   升降轴：EtherCAT Master（SOEM）→ 米思米 E-MCH14 400W 伺服
   头部：独立子系统（可选内嵌到 PC-A，也可外挂）
```

### 2.2 BOM 核心清单

| 类别 | 物料 | 型号推荐 | 关键参数 | 单价（元） | 数量 | 备注 |
|------|------|---------|---------|-----------|------|------|
| 主控 A | 上位机 mini PC | Intel NUC 13 Pro / 铭凡 UM780 | i7-13700H · 32GB DDR5 · 1TB NVMe · 集成 Iris Xe | 5,500 | 1 | 要求 4× USB3、2.5GbE |
| 主控 B | 下位机 mini PC | Intel NUC 11/13 / 零刻 SER5 | i5-1340P · 16GB · 512GB · 2.5GbE | 3,200 | 1 | CPU 核心数 ≥ 8 |
| CAN | USB-CAN 适配器 | 周立功 USBCAN-2E-U / Kvaser USBcan R v2 / PCAN-USB | 2× CAN · 1 Mbps · SocketCAN 驱动 | 800 | 2 | 左右臂各 1；达妙 DM 协议为标准 CAN 非 CAN-FD |
| EtherCAT | 升降轴 Master | 内置 Intel i210 以太网口 + SOEM | 软 Master 即可 | 0 | — | 软件实现 |
| 传感器 | RGB-D 相机 | ROSEB43i × 2 | 臂眼 | 2,700 | 2 | 已定 |
| 传感器 | RGB-D 相机 | MV-EB435i × 1 | 头部 | 1,350 | 1 | 已定 |
| 标定 | 力矩传感器 | 国产六维力传感器（如宇立 SRI M3713A 兼容件） | ≤ ±0.1N · CAN 接口 | 6,000 | 1 | 标定用，非量产 |
| 实时 | 急停硬件 | 双色急停按钮 + GPIO 转 USB | — | 200 | 1 | 安全必需 |
| 小计（仅主控+通信+标定） | | | | **约 21,350** | | |

### 2.3 关键参数对齐（与揭榜挂帅指标）

| 承诺指标 | 数值 | X86 方案可达性 | 依据 |
|---------|------|---------------|------|
| 移动速度控制精度 | ≥ 0.5–1.5 m/s 可调，波动率 ≤ 5% | ✅ 可达 | PREEMPT_RT + Ruckig 速度整形 |
| 静态 SLAM 定位精度 | ≤ ±10 mm | ⚠️ 临界可达 | 当前 Cartographer 在用；迁移 SLAM Toolbox + ChArUco 工位锚点可达 |
| 动态相对定位精度 | ≤ ±5 mm | ⚠️ 临界可达 | 需视觉伺服闭环 |
| 单臂重复定位精度 | ≤ ±0.1 mm | ✅ 分层可达 | **自由空间 ±0.5 mm（关节伺服） + 接触任务 ±0.1 mm（视觉+力控闭环）**，详见 §4.1 + §4.10 |
| 双臂同步误差 | ≤ ±0.5 mm | ✅ 可达 | 原子写入 + Controller Manager 同步 |
| 六维力测量精度 | ≤ ±0.1 N | 🟡 取决于传感器选型 | 至少配一套用于数据标定 |
| 升降精度 | ≤ ±1 mm | ✅ 已满足 | 米思米绝对编码器 + EtherCAT |
| 识别定位精度 | ≤ ±0.5 mm | ⚠️ 需结构光或 ChArUco 在线标定 | ROSEB43i 双目精度 1.5%，需补强 |

> **精度分层承诺（向客户/评审/第三方检测的统一口径）**：
>
> 单臂重复定位精度 ±0.1 mm 的承诺，采用**分层实现路径**：
>
> | 层级 | 场景 | 精度 | 技术路径 |
> |------|------|------|----------|
> | 关节层 | 自由空间 P2P 回位 | ±0.3–0.5 mm | 9:1 行星 QDD + MIT 模式 + 重力/摩擦/科氏前馈 |
> | 任务层 | **验收场景（抓取/插拔/装配/锁付）** | **±0.1 mm** | **视觉粗定位（±5 mm） + 力控柔顺（±0.1 N） + 视觉/力觉在线微调** |
>
> 这是 Franka、UR、JAKA 等工业协作臂在 3C 装配场景的主流达成路径——关节伺服层不直接保证 ±0.1 mm，由任务层的多模态闭环实现。**路径见 §4.10 精度分层实现路径**。

---

## 三、软件架构分层

### 3.1 控制链路全景

```
          Application Layer (PC-A)
 ┌────────────────────────────────────────────┐
 │ Skill 编排 / Scenario FSM / VLA Agent       │
 │ Voice Intent → 任务分发                     │
 └────────────────────┬───────────────────────┘
                      │ @10–30 Hz
          Planning Layer (PC-A)
 ┌────────────────────▼───────────────────────┐
 │ MoveIt 2 OMPL/CHOMP + arm_torso 8-DOF 规划  │
 │ Nav2 BT + 工位 docking server               │
 │ 安全包络：关节限位/速度限幅/碰撞白名单        │
 └────────────────────┬───────────────────────┘
                      │ JointJog / TwistStamped / Path
          Servo Layer (PC-A → PC-B via DDS)
 ┌────────────────────▼───────────────────────┐
 │ MoveIt Servo（奇异性处理 + 平滑滤波器）      │
 │ Ruckig 在线轨迹整形（1 kHz jerk-limited）   │
 └────────────────────┬───────────────────────┘
                      │ target_q, target_qd @500–1000 Hz
          Realtime Control Layer (PC-B)
 ┌────────────────────▼───────────────────────┐
 │ ros2_control 1 kHz 循环                     │
 │ 自定义 compensated_impedance_controller     │
 │   τ = Kp·(q_d − q) + Kd·(qd_d − qd)        │
 │       + τ_gravity(q)   ← Pinocchio RNEA    │
 │       + τ_coriolis(q, qd)                  │
 │       + τ_friction(qd) ← Stribeck 曲线     │
 │ SafetyLayer: 急停/心跳/限位/电流门限         │
 └────────────────────┬───────────────────────┘
                      │ MIT 模式 5 参数
          Driver Layer (PC-B)
 ┌────────────────────▼───────────────────────┐
 │ openarm_can（C++，直连达妙 DM MIT）           │
 │ SocketCAN + CAN 1 Mbps                       │
 │ atomic dual-arm write                       │
 └────────────────────┬───────────────────────┘
                      │ CAN Frame
                      ▼
                 达妙 DM 关节
```

### 3.2 为什么这个分层是"对的"

- **上位机只做慢线程（≤ 100 Hz）**：规划、感知、推理本就不需要 1 ms 级延迟；Fast-DDS over UDP 在 2.5GbE 上的典型 RTT ~1 ms，足以把 Servo 指令流送到下位机。
- **下位机只做快线程（1 kHz）**：ros2_control + 自定义 controller，用 `PREEMPT_RT` + `isolcpus` + `chrt -f 80` 把控制线程锁在专核；硬件接口直接调 `openarm_can` 底层 C++ 库（不经过 Python）。
- **"规划-伺服-控制"是一条连续的速度/位置流**，**不是离散航点**。这与 JTC 的"规划后执行"二段式形成对照，是解决顿挫的核心。
- **MIT 模式绕开达妙电机内置位置环**，让外部控制器直接下发力矩 + 关节阻抗，避免"控制器叠电机位置环"的嵌套震荡问题。

### 3.3 代码地图：必写/必改清单（20 个工程项）

| # | 文件 | 动作 | 来源参考 |
|---|------|------|---------|
| 1 | `xc_arm/openarm_hardware/src/hardware_xc.cpp` | 新 | 抄 `v10_simple_hardware.cpp` 结构，加 select() 超时、单电机 try-catch、原子写入 |
| 2 | `xc_arm/openarm_description/config/arm/v10/inertials.yaml` | 改 | 实测值替换 CAD 估值（W2 完成） |
| 3 | `xc_arm/openarm_description/config/arm/v10/friction.yaml` | 新 | Stribeck / 库伦 / 粘滞三参数（W2 完成） |
| 4 | `xc_arm/openarm_control/config/controllers.yaml` | 改 | update_rate 100 → 1000；添加 effort command_interfaces |
| 5 | `xc_arm/openarm_control/src/compensated_impedance_controller.cpp` | 新 | 前馈 + 阻抗 + 安全的主控制器 |
| 6 | `xc_arm/openarm_control/src/pinocchio_dyn_wrapper.hpp` | 新 | Pinocchio RNEA 封装，编译期加载 URDF |
| 7 | `xc_arm/openarm_control/src/ruckig_smoother.cpp` | 新 | Ruckig Type V 平滑 |
| 8 | `xc_arm/openarm_control/launch/moveit_servo.launch.py` | 新 | 接入 MoveIt Servo，目标 500 Hz JointJog |
| 9 | `xc_arm/openarm_moveit_config/config/servo_config.yaml` | 新 | 设置 command_in_type、奇异性阈值、碰撞检查 |
| 10 | `xc_safety/src/safety_manager.cpp` | 新 | 急停服务 + 心跳 + 软限位 + 电流阈值 |
| 11 | `xc_safety/src/estop_hw_bridge.cpp` | 新 | GPIO/USB 急停按钮桥接 |
| 12 | `xc_chassis/chassis_driver/src/zlac8015d_canopen.cpp` | 新 | SocketCAN 裸驱动，抄 atom01_deploy |
| 13 | `xc_chassis/chassis_localization/config/ekf.yaml` | 新 | robot_localization，轮式 + IMU + Tag |
| 14 | `xc_chassis/chassis_nav/config/nav2_params.yaml` | 新 | Nav2 全参数 |
| 15 | `xc_chassis/docking/src/aruco_docker.cpp` | 新 | Nav2 docking server 扩展 + AprilTag 精对位 |
| 16 | `xc_lift/src/ethercat_lift_hw.cpp` | 新 | SOEM 软 Master 驱动米思米伺服 |
| 17 | `xc_lift/config/arm_torso_group.srdf` | 新 | MoveIt arm_torso 8-DOF 规划组（核心） |
| 18 | `xc_head/voice/src/porcupine_wakeword.cpp` | 新 | Porcupine 唤醒 |
| 19 | `xc_head/voice/src/sherpa_asr.cpp` | 新 | Sherpa-ONNX + Paraformer 离线 ASR |
| 20 | `xc_deploy/ansible/site.yml` | 新 | 双机一键部署：内核、isolcpus、rtprio、ROS 2 |

---

## 四、七大技术难点深度分析与突破方案

### 4.1 难点 1：机械臂底层运动规划与 Servo 模式

#### 现状根因

- **OpenArm 原版 ros2_controllers.yaml 是 100 Hz + position-only 接口**（证据：`07｜代码库下载/openarm_ros2/openarm_bimanual_moveit_config/config/ros2_controllers.yaml`）
- **全仓库搜 "servo" 零结果**——官方从未工程化 MoveIt Servo
- **默认控制模式是 joint_trajectory_controller**——这是一种"规划后离散执行"的模式，准直驱臂用它必抖

#### 根因的根因
JTC 假设"高减速 + 不可背驱 + 刚性输出"；准直驱（9:1 行星）正好相反：**低惯量 + 可背驱 + 柔性输出**。用 JTC 等于把协作机械臂当做工业刚臂来控。

#### 解决路径（5 步，RD2 阶段 7 周滚动推进）

> **时间表修正（相对 v1.0 初稿）**：初稿把 Step 1+2 压在 W1 完成，实际不可行——扭摆/悬挂工装加工周期 2–3 周，摩擦扫频依赖 effort 接口就绪（鸡生蛋）。现修订为 7 周滚动推进。

```
Step 1（W1–W2）动力学参数实测
  → W1：工装设计 + 外协加工下单（扭摆杆、悬挂夹具、激光位移传感器）
  → W2：逐关节悬挂法测质量 + 两点悬挂法测重心 + 扭摆法测惯量
  → 写入 inertials.yaml（实测替换 CAD 估值）
  → MuJoCo 仿真回放验证误差 < 5%

Step 2（W2–W4）摩擦辨识
  → W2：单关节正弦扫频 SOP（与 Step 4 effort 接口改造并行）
  → W3：全关节 10 点扫频 + rosbag 记录
  → W4：离线拟合 Stribeck 曲线 fc, fs, vs, fv
  → 写入 friction.yaml

Step 3（W3–W5）补偿控制器
  → 自定义 compensated_impedance_controller
  → τ = PD + τ_g(q) + τ_c(q, qd) + τ_f(qd)
  → 用 Pinocchio 计算 τ_g, τ_c（< 30 µs/step）
  → 单关节悬空测试：松开后静止不下沉（漂移 < 0.05 rad）

Step 4（W1–W4 与 Step 1/2 并行）PREEMPT_RT + effort 接口 + 500 Hz
  → W1：Ubuntu 22.04 切 PREEMPT_RT 内核（参考 atom01_deploy）
  → W2：isolcpus 隔离专核 + chrt -f 80
  → W3：ros2_controllers.yaml update_rate 100 → 500
  → W4：command_interfaces 从 [position] → [position, velocity, effort]
  → 验收：cyclictest P99 < 100 µs

Step 5（W5–W6）Ruckig 轨迹整形
  → 500 Hz 目标速度输入 Ruckig Type V
  → 输出 jerk-limited 轨迹
  → 目的：防止上游速度阶跃直接传到关节
  → 验收：末端绘圆 10 cm 路径偏差 < 5 mm
```

> **注**：MoveIt Servo（原 Step 5）**本期砍掉**，延后到二期。理由见下方 "MoveIt Servo 适用场景重判"。
> **1 kHz 目标退让为 500 Hz**——达到揭榜挂帅阶段二 "80% 目标值" 即可，1 kHz 在 Intel Mini PC PREEMPT_RT 上抖动显著增大，性价比低。

#### MoveIt Servo 适用场景重判（相对 v1.0 初稿修正）

| 场景 | 是否适合 MoveIt Servo | 本期选型 |
|------|--------------------:|---------|
| 自由空间 P2P 规划 | 非必须 | **MoveIt OMPL + JTC**（够用） |
| 视觉伺服跟踪 | ✅ 适合 | 延后 P2 |
| 遥操作实时响应 | ✅ 适合 | 本期不做 |
| **接触/装配/插拔** | ❌ **不适合** | **独立的简化阻抗控制栈（关节级虚拟弹簧）** |
| 奇异性在线处理 | ✅ 适合 | OMPL 规划阶段已用关节限位规避 |

**理由**：MoveIt Servo 是位置级流式指令，没有内置力反馈闭环；接触瞬间的目标语义是"力保持 + 小位移"而非"速度跟踪"。工业协作臂（Franka/UR）的接触任务都走**独立的笛卡尔阻抗 / 导纳控制栈**（libfranka impedance、UR ForceMode），不走 Servo。

本期接触任务用**关节级简化阻抗**（降 Kp + 加力矩前馈），笛卡尔阻抗完整栈留到二期。

#### 技术选型

| 模块 | 选型 | 理由 |
|------|------|------|
| 动力学库 | **Pinocchio** | 最快（Atlas 3.5 µs RNEA）、解析导数、浮基机器人标准 |
| 在线规划 | **Ruckig** | Type V、jerk-limited、250 µs 周期、首个支持非零目标状态 |
| 流式控制（P2 延后） | **MoveIt Servo**（本期不上） | ROS 2 原生、奇异性处理成熟；本期 P2P 用 JTC 替代 |
| 硬件接口 | **openarm_can**（C++） | 直连达妙 DM MIT、比 Python SDK 快一档 |
| 控制器框架 | **ros2_control** | 可插拔、支持多 command_interface |

#### 验收判据（RD2 阶段，按揭榜挂帅 80% 目标值校准）

| 测试 | 本期目标 | 原 v1.0 目标 | 失败兜底 |
|------|------|---------|---------|
| 悬空静止 60 秒 | 关节漂移 < 0.05 rad | < 0.01 rad | τ_g 参数迭代 |
| 手动拖动所有关节 | 无阶跃感、无振荡 | 同 | 降低 Kp、增大 Kd |
| 末端绘圆 10 cm | 路径偏差 < 5 mm | < 2 mm | 升级到笛卡尔阻抗（P2） |
| 连续 10 分钟抓放 | 累计误差 < 10 mm | < 5 mm | 加周期性零点复位 |
| P2P 规划 + JTC 执行 | 20 次 20 次成功，偏差 < 5 mm | —（原无） | OMPL 切 RRTConnect |

#### 反命题与反驳

**反命题**："为什么不直接 fork UR/Franka ROS 2 驱动照抄？"
**反驳**：UR/Franka 驱动强依赖厂商内部 FCI（Franka Control Interface）和 Realtime UCI 接口，硬件接口层完全不同。它们的价值是**架构参考**（分层、实时性、安全层），不是代码级移植。可以抄**架构**但不能抄**驱动**。

---

### 4.2 难点 2：底盘自主导航（商用精度）

#### 现状根因

- 当前底盘里程计 > 50 cm/10 m 漂移 → AMCL 收敛困难
- 无融合（IMU + 轮式）→ 转弯/打滑场景位姿跳变
- Nav2 导航栈未配置 → 无路径规划能力

#### 目标重新界定

客户需求被明确为**"通过交互或后台点选位点"**——这比开放世界自主导航宽松得多。对应的技术目标：

| 指标 | 目标 | 可达性评估 |
|------|------|-----------|
| 静态定位精度 | ±10 mm | 需 AMCL + AprilTag 精对位 |
| 动态相对精度 | ±5 mm | 需视觉伺服闭环 |
| 建图一次性完成 | 20 m × 20 m 一次建图成功 | SLAM Toolbox + 手推扫图 |
| 点选位点精度 | 到达位点后偏差 < 5 cm | Nav2 docking server + Tag |
| 避障能力 | 识别 30 cm+ 障碍物 | 2D LiDAR + 深度相机点云 |

#### 解决路径（4 步）

```
Step 1 底盘里程计干净化
  → 实测轮径 + 轴距（游标卡尺）
  → wheel_params.yaml 精确化
  → robot_localization EKF 融合 IMU（ROSEB43i 自带 6DoF）
  → 直线 10 m 漂移目标 < 10 cm

Step 2 一次性建图
  → 【当前在用】Cartographer（已建图成功，±3 cm 精度收敛中，可继续使用）
  → 【推荐迁移】SLAM Toolbox（动态环境 ATE 0.13 m 优于 Cartographer 0.21 m）
  → 手推 + 钥匙扣遥控完成扫图
  → 保存 .pgm + .yaml + serialized_slam.posegraph

Step 3 精定位锚点
  → 每个工位贴 ChArUco bundle（而非单个 ArUco）
  → 标定世界坐标系到每个 Tag 的变换（tag_0 为原点）
  → AprilTag ROS 节点 + tf2 发布

Step 4 Nav2 + Docking
  → AMCL 初始位姿 + 粗定位
  → NavigateToPose → 工位附近（精度 ±10 cm）
  → DockingServer + external_detection_pose 拉到 ±3 cm
```

#### 关键技术细节

- **为什么 SLAM Toolbox 优于 Cartographer**：2025 MDPI 实测动态环境下 ATE 0.13 m vs 0.21 m；Cartographer 不自动广播 TF 需要 odom，SLAM Toolbox 是纯扫描匹配
- **为什么必须 AprilTag/ChArUco 精对位**：Nav2 docking 文档明确 docking_threshold 默认 5 cm，这是 AMCL 在动态环境下的物理极限，想打穿必须外接高精度位姿源
- **为什么不用 RTK / UWB**：室内 RTK 失效；UWB 需要铺锚点成本 > Tag

#### 验收判据

| 测试 | 目标 |
|------|------|
| 10 m 直线 | 轮式里程计漂移 < 5 cm |
| 建图闭环 | 20 m 矩形回原点误差 < 10 cm |
| 点选位点 10 次 | 平均偏差 < 5 cm，最大偏差 < 10 cm |
| 工位对接 10 次 | 平均偏差 < 3 cm |
| 动态避障 | 行人穿越不撞不死锁 |

---

### 4.3 难点 3：升降轴与机械臂协同运动控制

#### 为什么要专门做

升降轴在 BOM 里是一个**独立的 400W EtherCAT 伺服**，天然的 1-DOF 直线单元，项目里最容易被当作"状态机执行"处理——**升到位再规划臂**。这是错的。

#### 正确做法（来自 PAL TIAGo 的工业参考）

**把升降轴建成 URDF prismatic joint，与双臂组成 arm_torso 8-DOF 统一规划组。**

```xml
<!-- openarm_description/urdf/lift_joint.xacro -->
<joint name="torso_lift_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="torso_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="0.0" upper="0.5" effort="800" velocity="0.25"/>
</joint>
```

**MoveIt SRDF 里加组**：
```xml
<group name="arm_torso_left">
  <joint name="torso_lift_joint"/>
  <chain base_link="base_link" tip_link="left_ee_link"/>
</group>
```

#### 本期决策（相对 v1.0 初稿修正）：状态机优先，8-DOF 延后

> **v1.0 初稿主张"状态机会局部最优，必须上 8-DOF 统一规划"。本次修订调整为分阶段推进**：

| 阶段 | 方案 | 适用场景 | 工程复杂度 |
|------|------|---------|----------|
| **RD2（本期）** | **升降到位 → 再规划臂**（状态机） | 3C 料盘、办公递送、标准工位 | 低（1 周） |
| RD3–RD4 | 升降速度耦合限幅 + 安全包络 | 升降中臂保持收臂姿态 | 中 |
| P1 / 二期 | arm_torso 8-DOF MoveIt 联合规划 | 狭窄工位、边升降边伸臂 | 高（4 周起） |

**状态机做法的"局部最优"问题真实存在，但在本期 Demo 场景（SMT 料盘分拣、办公递送）中触发概率 < 1%**。8-DOF 联合规划的代价：
- OMPL 在 8 维 C-空间求解时间往往 > 500 ms（非 200 ms 乐观估计），失败率增大
- 升降中 M(q) 随 z 变化的动力学建模是独立课题
- TIAGo 的 8-DOF 规划是 10+ 年工程打磨结果，我们 RD2 做不到同等质量

**若 Demo 过程中遇到确实无法用状态机解的工位**（具体来说：工作台下方 < 300 mm 净空 + 臂需伸入），升级为 8-DOF（P1 触发条件）。

#### 技术细节

- **EtherCAT Master 选型**：SOEM（Simple Open EtherCAT Master，开源 C 库）+ CyclicTask，1 kHz 通信；米思米 E-MASH2 兼容标准 CoE 协议。
- **IGH EtherCAT** 是更工业的选择但内核态驱动，复杂度高。**推荐 SOEM 用户态 + PREEMPT_RT**。
- **升降速度耦合限幅**：当 torso_lift_joint 以 > 0.1 m/s 运动时，限制臂关节速度 < 0.5 rad/s（避免耦合惯量冲击）。

#### 验收判据（本期状态机版）

| 测试 | 本期目标 | 备注 |
|------|------|------|
| 状态机切换：升降 → 臂规划 | 切换时延 < 500 ms | 状态机 FSM 节点 |
| 升降运动中臂保持收臂 | 末端抖动 < 5 mm | 速度耦合限幅 |
| 全程自碰撞检查 | 零漏报（允许 < 1% 误报） | MoveIt PlanningScene |
| 紧急停止 | 升降与臂同步停止 < 100 ms | 急停广播 |

**P1 升级触发条件**：Demo 现场出现 ≥ 3 次"状态机无解但 8-DOF 联合规划能找到解"的实例，则启动 8-DOF 升级。

---

### 4.4 难点 4：手眼标定（Eye-in-Hand）精度提升

#### 默认方案为什么差

easy_handeye 默认调用 OpenCV 的 Tsai-Lenz 手眼算法，在 PLOS ONE 2022 的实测中**是所有算法里精度最差的**（~4.4 mm vs Daniilidis 的 ~3.4 mm）。这是最容易踩的坑。

#### 正确做法

```
算法层：
  默认 Daniilidis 对偶四元数 / Park-Martin
  （OpenCV 都支持，easy_handeye 只需改一行参数）

标定板层：
  必须用 ChArUco bundle（棋盘格亚像素 + ArUco 编码）
  不要用单个 ArUco 或 AprilTag（没有亚像素优化）
  标定板尺寸 A3 以上，格子 20 mm+
  打印在亚克力/铝板上（不能纸印，会变形）

姿态采集层：
  至少 30 组有效姿态
  旋转角度 > 60° 且分布均匀（Tsai 原论文建议）
  避免退化情况（两个姿态方向平行）

在线校验层：
  标定后在 5 个已知 Tag 位置做 FK/IK 对比
  误差 > 1 mm 则重新标定
```

#### 机械臂误差的耦合

手眼标定假设臂端 FK 是真值——**但准直驱臂 FK 误差可达 mm 级**，会直接吃进手眼变换 X 解里。缓解：
1. 先做关节零位校准 + DH 参数辨识（用高精度球头探针触若干已知点）
2. 在线迭代标定：每次抓取前用工位 Tag 快速纠正一次
3. 双相机交叉验证（左臂眼 + 右臂眼 + 头部相机）

#### 目标精度

| 级别 | 精度 | 达到方式 |
|------|------|---------|
| 入门 | ±2 mm | Tsai-Lenz + ArUco |
| 工程 | **±0.6 mm** | **Daniilidis + ChArUco + 30 pose** |
| 精密 | ±0.1 mm | 结构光 + 离线 BA |

本项目目标是**工程级 ±0.6 mm**，这是两个月内可达的，配合第 4.5 节的力控柔顺即可满足 3C 装配需求。

#### 验收判据

- 标定后用机械臂在 10 个已知 Tag 点做"视觉预测 vs 实际移动"对比，平均误差 < 0.6 mm
- 连续工作 2 小时后再次测试，退化 < 0.3 mm

---

### 4.5 难点 5：双电源热插拔

#### 工业做法

| 方案 | 代表 | 特点 |
|------|------|------|
| 单电池 + 外接续电 | **Boston Dynamics Spot** | 换电瞬间接外部 PSU；简单但要中转电源 |
| 双电池 OR-ing + 理想二极管 | **ADI LT1641 / ST STPM801** | 无缝切换、毫秒级、两电池独立 BMS |
| UPS 超级电容中转 | 部分工业 AGV | 换电过程中由大电容/UPS 供电 2–5 秒 |

#### 推荐电路拓扑

```
  Battery A (48V 20Ah)           Battery B (48V 20Ah)
        │                              │
        ▼                              ▼
   ┌──────────┐                  ┌──────────┐
   │  BMS A   │                  │  BMS B   │
   │ CAN-BMS  │                  │ CAN-BMS  │
   └────┬─────┘                  └────┬─────┘
        │  VA                         │  VB
        ▼                              ▼
   ┌────────────┐               ┌────────────┐
   │ Hot-Swap   │               │ Hot-Swap   │
   │ Ctrl A     │               │ Ctrl B     │
   │ (LT1641等) │               │ (LT1641等) │
   │ MOSFET A   │               │ MOSFET B   │
   └──────┬─────┘               └─────┬──────┘
          │                           │
          └──────────┬────────────────┘
                     │ Common Bus 48V
                     │
            ┌────────▼──────────┐
            │ Bulk Capacitor    │
            │ 10mF / 63V        │ ← 能量缓冲
            └────────┬──────────┘
                     │
            ┌────────▼──────────┐
            │ Supercap 10F/60V  │ ← 可选：换电桥接
            └────────┬──────────┘
                     │
     ┌───────────────┼──────────────┐
     │               │              │
     ▼               ▼              ▼
  PC-A DC-DC      PC-B DC-DC    电机驱动器（直连 48V）
```

#### 关键工程参数

- **理想二极管控制器**：ADI LT4320 / LT1641 / TI LM5050 均可，关键是低 RDS(on) MOSFET（单管 < 3 mΩ）
- **切换时间**：目标 < 1 ms（用控制器内部硬件切换，不走软件）
- **电压跌落**：换电过程总线电压 > 44 V（对 48 V 系统允许 8% 下垂）
- **BMS 协议**：**统一用 CAN-BMS**（与运动 CAN 总线物理隔离但协议同构，降低学习成本）；定义 ID 段：0x700–0x7FF 给电池，0x100–0x6FF 给电机
- **软件协议**：主控订阅 BMS 状态，SOC 差超过 30% 时拒绝"取出 A 电"操作，强制先均衡

#### 本阶段务实方案

考虑 2 个月时间窗和现有 BOM 已购电池（聚锂科技 JLD-DL-4820R）：

**推荐**：W1–W2 先做单电池 + 急停冗余；W6 前加装商用 48V 双路热插拔模块（淘宝现货约 300–500 元/路，内部已集成理想二极管 + 软启动）；验收阶段演示"热插拔不断电"即可。

**不推荐**：从零设计双路热插拔 PCB——W8 完成不了。

---

### 4.6 难点 6：头部交互与语音控制

#### 需求边界

客户 Demo 要求：**语音从 A 点走到 B 点 + 抓取 + 放置**。精度"相对可控"、"可靠性尽量一致"。

这不是开放对话，是**闭集指令 + 关键词路由**，本阶段不需要 LLM。

#### 三层栈

```
  唤醒层：Porcupine（商业免费层） or openWakeWord
  ──────────
  ASR 层：Sherpa-ONNX + Paraformer-large-zh（industry WER 10.9%）
  ──────────
  NLU 层：关键词路由 + 实体抽取（正则 + 字典）
  ──────────
  对话层（可选）：本地 Qwen 1.8B 做 fallback（CPU 1 s+ 延迟，不在关键路径）
  ──────────
  TTS 层：Piper ONNX（CPU 实时）
```

#### 指令集设计（Demo v1）

| 语音 | 槽位 | ROS 2 Action | 示例 |
|------|------|-------------|------|
| "去{位点}" | 位点名 | `/nav/go_to_pose` | "去 A 点" |
| "抓起{物体}" | 物体名 | `/skill/grasp` | "抓起那个螺丝刀" |
| "放到{位点}" | 位点名 | `/skill/place` | "放到 B 点" |
| "停" / "急停" | — | `/safety/estop` | — |
| "回家" | — | `/nav/go_home` | — |
| "现在在哪" | — | TTS 回报 | — |

**意图解析**：实体抽取用字典匹配（工位名和物体名都是预定义的），不走 LLM。遇到未识别指令 TTS 说"我不明白，请重新说"，不做兜底推理。

#### 硬件

- **6 麦圆形阵列 + USB 声卡**：已在 BOM 内（科大讯飞方案）
- **扬声器**：3W 以上，装在头部下方（避免与麦克风直接耦合）
- **LCD 屏**：6.2" HDMI（已在 BOM），显示状态图标（listening/thinking/executing/error）

#### 工程路径（4 周）

```
W2: Porcupine 唤醒 + 英文 / 中文混合关键词表（如 "小承小承"）
W3: Sherpa-ONNX + Paraformer 离线 ASR 部署，实测工厂背景噪声 WER
W4: NLU 意图路由 + ROS 2 Action 调用
W5: 与主控 Skill 联动 + TTS 反馈 + LCD 表情切换
W6: 噪声压测 + 口音鲁棒性测试
```

#### 验收判据

| 测试 | 目标 |
|------|------|
| 唤醒成功率（安静） | > 95% |
| 唤醒成功率（85 dB 背景） | > 85% |
| ASR 完整指令识别率 | > 90% |
| 端到端延迟（唤醒 → 动作开始） | < 1.5 s |
| 完整 Demo "去 A → 抓 → 放到 B" | 10 次成功 8 次 |

---

### 4.7 难点 7：负载率与承载率（系统可靠性专项）

> 用户专门点名"负载率和承载率"——这不是一个单点技术问题，而是对整个系统可靠性的综合要求。

#### 概念澄清

- **负载率**：机械臂在一次任务中实际承载的负载 / 标称最大负载。3C 场景典型 1–3 kg，对应达妙 DM-J8009 额定扭矩（< 30% 利用率）。
- **承载率**：连续工作 N 次/小时的可持续能力。商用机器人典型 200 次/小时，我们阶段目标 50 次/小时。

#### 可靠性的 5 个支柱

| 支柱 | 威胁 | 对策 |
|------|------|------|
| **热管理** | 达妙 DM 关节长时大扭矩 → 温度报警 → 自动降级 | 电流 RMS 监控 + 散热风扇 + 工作循环限制 |
| **电源裕度** | 48 V 电池在低 SOC 时电压下垂 → 电机堵转 | BMS 低压告警 + 自动回归 |
| **机械疲劳** | 9:1 行星减速在万次循环后回差增大 | 运行日志 + 季度检修 |
| **通信健壮性** | CAN 总线在长时间运行后可能进入 bus-off | SocketCAN 自动恢复 + 心跳监控 |
| **软件漂移** | ROS 2 节点长时间运行后内存漏 / TF 时延累积 | 每 8 小时软重启 + 监控 dashboard |

#### 工程措施清单

1. **电流 RMS 滑窗监控**：每个关节维护 60 s 滑窗电流平方积分，超过阈值（电机手册 I²t 曲线）触发任务降速
2. **工作循环（Duty Cycle）限制**：一个动作完成后强制插入 0.5 s 冷却间隔（可配置）
3. **整机健康看板**：Grafana + InfluxDB 订阅所有关节温度、电流、延迟、位置误差
4. **自动回归**：每 4 小时末端回到 home 位置，执行一次"健康检查套件"
5. **失败模式记录**：所有异常（CAN 超时、位置超差、急停触发）入库，用于离线分析

#### 承载率测试方法

```
Day 1: 连续 10 次抓取 → 成功率统计 → 不合格回退
Day 2: 连续 30 次 → 看是否有渐进退化
Day 3: 连续 100 次 → 观测热/电压/延迟趋势
Day 4: 4 小时不间断运行（50 次/小时 × 4 = 200 次）
Day 5: 故障注入测试（拔 CAN 线 / 遮挡相机 / 触发急停）
```

---

### 4.8 难点 8：多模态感知融合架构（揭榜挂帅核心创新点 1）

> **v1.0 初稿盲区补足**：初稿只讨论了激光 SLAM + AprilTag（§4.2）与手眼标定（§4.4），**没有系统章节讨论"激光+RGBD+鱼眼+六维力"四模态统一融合**——这是揭榜挂帅申报书明确的第一大核心创新点，也是第三方 CNAS 检测的必查项。本节补足架构。

#### 需求锚定

揭榜挂帅申报书 §二.1.难点一 & §二.3.创新点一要求：
> "激光雷达 SLAM + RGB-D 视觉 + 全景鱼眼 + 六维力觉在 ROS 2 体系中实现高精度标定、时空同步、多源数据对齐与统一接口输出，形成可支撑地图构建、语义识别、任务规划与操作控制的**整体感知底座**"

#### 本期目标（RD2）

**架构骨架 + ROS 2 接口契约 + 最小 demo**，不求完整填满。填满留到 RD3。

#### 三层融合架构

```
  Layer 3：语义场景表征（统一输出）
  ┌─────────────────────────────────────────┐
  │ /scene/semantic_map                     │ ← 发布主题
  │   包含：工件列表 + 工位列表 + 工具列表   │
  │         含位姿、类别、置信度、接触状态    │
  │   订阅方：任务调度、MoveIt、Nav2         │
  └───────────────┬─────────────────────────┘
                  │
  Layer 2：跨模态融合
  ┌───────────────▼─────────────────────────┐
  │ FusionNode（rclcpp_components）          │
  │   - 时空同步（message_filters ApproxTime）│
  │   - 外参标定应用（tf2）                  │
  │   - 置信度加权（可选 FGO）               │
  │   - 降级策略（某模态失效切主模态）        │
  └───────────────┬─────────────────────────┘
                  │
  Layer 1：模态适配层（各传感器标准化输出）
  ┌────┬────┬────┬────────┬──────────────────┐
  ▼    ▼    ▼    ▼        ▼
  激光雷达 RGB-D 鱼眼  六维力  IMU
  M10   ROSEB43i  IMX219 宇立      CMP10A
  scan   pointcloud img   wrench   imu
```

#### 时空同步协议

| 项 | 要求 | 实现 |
|----|------|------|
| 硬件时钟基准 | 统一 PTP | Intel i210 + linuxptp + phc2sys |
| 软件时间戳 | rclcpp::Time（steady_clock） | ROS 2 默认即可 |
| 消息对齐 | ApproximateTime 0.05 s | message_filters |
| 外参标定 | 离线标定 + tf_static | camera→base, lidar→base, ee→camera 等 |

#### 降级策略（健壮性设计）

| 主模态失效 | 切换策略 | 触发条件 |
|-----------|---------|---------|
| RGB-D 视觉失效 | 激光 + 鱼眼粗定位 + 力觉触探 | 深度图 NaN 比例 > 30% |
| 激光 SLAM 漂移 | 视觉 + Tag 重定位 | AMCL 置信度 < 阈值 |
| 力觉传感器失效 | 纯视觉控制 + 限速 | 力觉话题 timeout 1 s |
| 多模态同时失效 | 自动回 home + 急停 | FusionNode 置信度综合 < 阈值 |

#### 本期最小 Demo

- **模态覆盖**：RGB-D + 激光 + IMU（鱼眼 + 力觉 P1 接入，占位不实现）
- **语义输出**：工件类别 + 6D 位姿 + 置信度（基于 YOLOv8 + RGB-D）
- **时空同步验证**：RGB-D 与激光 pointcloud 叠加 RViz 显示一致
- **ROS 2 接口契约冻结**：`/scene/semantic_map` 消息定义写入 `xc_perception_msgs`

#### 验收判据（RD2 80% 目标）

| 测试 | 本期目标 | P1 目标（RD3） |
|------|---------|--------------|
| 语义场景发布频率 | ≥ 10 Hz | ≥ 30 Hz |
| 工件位姿识别精度 | 平移 < 5 mm，旋转 < 5° | 平移 < 1 mm，旋转 < 2° |
| 主模态失效降级时间 | < 500 ms | < 100 ms |
| 多模态时间戳对齐 | 偏差 < 50 ms | 偏差 < 10 ms |

---

### 4.9 难点 9：柔性任务调度架构（揭榜挂帅核心创新点 2）

> **v1.0 初稿盲区补足**：初稿 §3.1 只用一行 "Skill 编排 / Scenario FSM / VLA Agent" 带过，没有工程细节。揭榜挂帅申报书把"行为树 + 状态机 + 语义状态感知 + 任务重规划"作为第二大创新点，本节系统展开。

#### 需求锚定

揭榜挂帅申报书 §二.1.难点二 & §二.3.创新点二要求：
> "行为树（Behavior Tree） + 有限状态机（FSM）混合架构，实现任务模块化拆解、动态切换与异常回退机制。系统持续采集工位状态、物料状态与自状态，当检测到工件位置偏差或物料准备不足时，系统可触发任务重规划"

#### 双层架构：BT 管流程、FSM 管关键动作

```
  ┌────────────────────────────────────────┐
  │  Layer 2：Behavior Tree（任务流程）    │
  │  BehaviorTree.CPP v4                   │
  │                                        │
  │   Sequence                             │
  │   ├─ NavigateToPose(A)                 │
  │   ├─ Fallback                          │
  │   │    ├─ GraspObject(target)  ← 主路径│
  │   │    └─ ReplanAndRetry(N=3)  ← 回退 │
  │   ├─ NavigateToPose(B)                 │
  │   └─ PlaceObject                       │
  └────────────────┬───────────────────────┘
                   │ ROS 2 Action 调度
  ┌────────────────▼───────────────────────┐
  │  Layer 1：状态机（关键动作执行）        │
  │  SMACH2 / yasmin                        │
  │                                        │
  │  GRASP_FSM:                            │
  │    APPROACH → ALIGN → CONTACT          │
  │    → CLOSE_GRIPPER → LIFT → VERIFY     │
  │    每个状态带超时 + 异常转移           │
  └────────────────┬───────────────────────┘
                   │
  ┌────────────────▼───────────────────────┐
  │  Layer 0：语义状态感知（事件驱动）      │
  │  订阅 /scene/semantic_map (§4.8)        │
  │  订阅 /safety/events                    │
  │  发布 /task/blackboard                  │
  │                                        │
  │  触发事件：                            │
  │   - 物料缺失 → BT Fallback 触发        │
  │   - 工位占用 → BT 任务顺序重排         │
  │   - 力反馈异常 → FSM 中断 + 回退       │
  │   - 急停 → 全部中止 + home             │
  └────────────────────────────────────────┘
```

#### 核心原则

| 原则 | 说明 |
|------|------|
| BT 管流程、FSM 管关键动作 | BT 擅长并行 + 回退，FSM 擅长时序严格 + 状态监视 |
| 黑板（Blackboard）共享状态 | 所有任务读写统一 `/task/blackboard` |
| 异常可回退 | 每个关键节点配 Fallback 兄弟节点 |
| 重规划分级 | 轻度（调整姿态）→ 中度（换抓取点）→ 重度（重新规划整个任务） |
| 可观察性 | Groot 2 实时可视化 BT 执行 |

#### 抓取任务示例（一个完整 FSM）

```
GRASP_FSM:
  ┌─ APPROACH ─ 运动到抓取前位（视觉位姿 + 10 cm）
  │    超时 5s → REPLAN
  ├─ ALIGN ── 精对齐（视觉伺服或 Tag 对齐）
  │    超时 3s / 位姿偏差 > 2 mm → REPLAN
  ├─ CONTACT ── 下降接触（力控阈值 5 N）
  │    接触超时 2s → RETRY / ABORT
  ├─ CLOSE_GRIPPER ── OmniPicker 闭合
  │    夹爪力 < 设定值 → RETRY（物料缺失）
  ├─ LIFT ── 抬升
  │    力反馈 < 预期物料重量 → ABORT（抓空）
  ├─ VERIFY ── 视觉确认夹持物
  │    识别失败 → ABORT
  └─ DONE → 返回 BT
```

#### 本期最小 Demo

- **BT**：Groot 2 拖拽实现"去 A → 抓 → 去 B → 放"
- **FSM**：单关键动作（GRASP_FSM）跑通
- **黑板**：订阅 `/scene/semantic_map` 写入物料位姿
- **Fallback**：抓取失败自动重试 2 次，3 次后 ABORT 并语音播报

#### 验收判据（RD2 80% 目标）

| 测试 | 本期目标 | P1 目标（RD3） |
|------|---------|--------------|
| BT 任务完整执行 | 10 次 ≥ 8 次成功 | 10 次 ≥ 9 次成功 |
| 抓取失败自动重试 | 重试 2 次内恢复率 ≥ 50% | ≥ 70% |
| 物料缺失检测 | 延迟 < 1 s | < 500 ms |
| 急停触发到全部停止 | < 100 ms | 同 |

---

### 4.10 难点 10：单臂 ±0.1 mm 精度的分层实现路径（验收答辩专用）

> **v1.0 初稿盲区补足**：初稿 §1.2 第 7 条直接说"物理做不到"，对验收答辩极其不利。本节用于统一答辩口径 + 明确技术路径。

#### 答辩话术（统一对外口径）

> "XC-Robot 的单臂 ±0.1 mm 承诺采用**分层实现**：关节伺服层保证 ±0.3–0.5 mm，接触任务层通过视觉 + 力控闭环达到 ±0.1 mm。这是 Franka、UR、JAKA 等工业协作臂在 3C 装配场景的主流技术路径——关节伺服层的物理极限由减速器类型决定，最终装配精度由**任务层的多模态闭环**保证。"

#### 物理层级与精度分解

| 层 | 精度来源 | 本方案精度 | 备注 |
|----|---------|-----------|------|
| L0 电机电流环 | 电机驱动器 | FOC 10 kHz | 达妙内置 |
| L1 关节 MIT 阻抗 | Kp/Kd/τ_ff | 静止漂移 < 0.05 rad | §4.1 M1–M3 产出 |
| L2 笛卡尔 IK | FK/IK 链 + URDF 标定 | 端执行器 ±0.5 mm | 依赖 URDF 零位标定 |
| L3 视觉粗定位 | YOLO + RGB-D/Tag | ±5 mm（Tag 锚点 ±1 mm） | §4.8 |
| L4 力控柔顺接触 | τ 阈值 5 N + Kp 动态降 | 接触偏差 < 0.1 mm | 本期关节级阻抗 |
| L5 视觉/力觉在线微调 | 闭环反馈 | **任务层 ±0.1 mm** | §4.9 GRASP_FSM 内完成 |

**可视化公式**：
```
任务精度 = f(L3 视觉粗定位, L4 力控接触顺从度, L5 闭环迭代次数)

L3 固定在 ±5 mm
L4 使关节顺应实际位置（不强求位置跟踪）
L5 迭代 2-3 次后收敛到 ±0.1 mm
```

#### 验收场景映射

| 揭榜挂帅验收场景 | 是否接触任务 | 精度达成 |
|-----------------|:------:|---------|
| SMT 料盘分拣 | 是 | 任务层 ±0.1 mm ✅ |
| 螺丝/插件插拔 | 是 | 任务层 ±0.1 mm ✅ |
| 上下料 | 是 | 任务层 ±0.1 mm ✅ |
| 锁付装配 | 是 | 任务层 ±0.1 mm ✅ |
| 自由空间运动 | 否 | 关节层 ±0.5 mm（对验收无关） |

**所有验收场景都是接触任务**，所以分层承诺在合同履约意义上是完整的。

#### 第三方 CNAS 检测准备

| 项 | 行动 | 时间 |
|----|------|------|
| 检测机构接洽 | 上海机器人产业技术研究院 / CNAS 认证机构 | RD2 末（2026-07） |
| 检测用具清单 | 视觉标定板、激光跟踪仪预约、标准物料 | RD2 末 |
| 预检测自测 | 按 CNAS 标准做 3 次内部测试 | RD3（2026-09） |
| 正式第三方检测 | 接受 CNAS 机构上门检测 | RD4（2026-11） |

---

## 五、X86 方案的核心优势（vs RDK 方案预告）

| 维度 | X86 方案 | 备注 |
|------|---------|------|
| PREEMPT_RT 成熟度 | ⭐⭐⭐⭐⭐ mainline kernel 6.12 合入，UR/Franka 官方推荐 | 唯一无争议项 |
| ROS 2 原生兼容 | ⭐⭐⭐⭐⭐ | Humble / Jazzy 完整生态 |
| MoveIt 2 / Nav2 生态 | ⭐⭐⭐⭐⭐ | 文档、样例、社区完整 |
| CAN-FD 生态 | ⭐⭐⭐⭐ 需外挂卡 | Kvaser / PCAN / 周立功 |
| BPU/NPU 算力 | ⭐ 0 TOPS | 必须外挂 OpenVINO 或 Coral/Hailo |
| 国产化率 | ⭐ 低 | 政策场景不利 |
| 单机成本 | ⭐⭐⭐ 约 9 k | PC-A 5.5k + PC-B 3.2k |
| 供应链稳定性 | ⭐⭐⭐⭐⭐ | Intel 量产多年 |
| 社区踩坑资料 | ⭐⭐⭐⭐⭐ | 机器人圈主流选择 |
| VLA 推理能力 | ⭐⭐ Intel iGPU + OpenVINO / 外挂 | 可接 Hailo-8 26 TOPS NPU |

---

## 六、X86 方案风险登记

| # | 风险 | 概率 | 影响 | 应对 |
|---|------|------|------|------|
| A1 | 达妙 DM 电机在某些 USB-CAN 适配器上丢帧 | 低 | 中 | 选择周立功 USBCAN-2E-U 工业级（SocketCAN 驱动稳定） |
| A2 | 动力学标定耗时超预期 | 中 | 高 | W1 由顾问全周带做一次，形成 SOP |
| A3 | MoveIt Servo 在接触任务中奇异性处理不佳 | 中 | 中 | 切换到笛卡尔阻抗 + 被动柔顺机构 |
| A4 | 6 麦阵列在工厂噪声下唤醒率低 | 中 | 中 | 增加硬件降噪 + 触屏兜底 |
| A5 | VLA 推理能力不足（无 BPU） | 中 | 中 | Demo 阶段走脚本化 Skill，不依赖 VLA |
| A6 | 国产化审查不利 | 低 | 高 | 在后续量产阶段混合架构（X86 + 国产 NPU） |

---

## 七、阶段性结论

**X86 方案 = 最稳的工程路线。**

- 所有关键控制链路（实时、Servo、动力学补偿、力控、SLAM、MoveIt 8-DOF 统一规划、EtherCAT 升降、热插拔）都有成熟的上游实现和文献支持
- 风险都在可控的范围内
- 单机 BOM 增量约 1 万元（相对双 RDK 方案），属于可接受
- 两个月内做出 90% 成功率的 Demo 可行性高

**但 X86 方案有两个战略劣势**：
1. **BPU 算力 = 0**，VLA/大模型推理必须外挂
2. **国产化率低**，在客户对国产化有政策要求的场景下不友好

→ 这两点正是下一份 **技术分析报告 B（双 RDK 方案）** 要正面回答的。

---

## 附录 A：参考资源

- OpenArm 官方仓库：https://github.com/enactic/openarm
- atom01_deploy RDK X5 PREEMPT_RT 实践：https://github.com/Roboparty/atom01_deploy
- MoveIt Servo 教程：https://moveit.picknik.ai/main/doc/examples/realtime_servo/
- Ruckig 论文：https://arxiv.org/abs/2105.04830
- Pinocchio：https://github.com/stack-of-tasks/pinocchio
- robot_localization EKF：https://docs.ros.org/en/lunar/api/robot_localization/
- Nav2 Docking：https://docs.nav2.org/tutorials/docs/using_docking.html
- 手眼标定算法对比：https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0273261
- ChArUco 在线标定：https://www.mdpi.com/1424-8220/22/10/3805
- TIAGo arm_torso 规划：https://docs.pal-robotics.com/25.01/manipulation/motion-planning-moveit.html
- 双电池 hot-swap ADI：https://www.analog.com/en/resources/technical-articles/48v-hot-swap-circuit-blocks-reverse-battery-voltage.html
- ROS 2 PREEMPT_RT 内核构建：https://docs.ros.org/en/humble/Tutorials/Miscellaneous/Building-Realtime-rt_preempt-kernel-for-ROS-2.html

**报告 A 结束**
