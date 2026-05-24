---
date: 2026-05-21
tags: [祥承电子, Xc-Robot, 电机控制, MIT模式, FOC, 中间层, 前馈补偿, GitHub开源]
machine: mac-minishu
agent: opencode
---

# MIT ↔ FOC 中间层控制：GitHub 代码库与算法调研

## 背景

灵足时代 RobStride 关节模组的 FOC 电流环跑在 GD32F103 MCU 内部（固件闭源），无法修改底层 FOC 逻辑。但 MIT 模式（运控模式）通过 CAN 总线接收五元组指令：`pos, vel, Kp, Kd, torque(τ_ff)`。其中 `τ_ff`（力矩前馈）默认填 0，是突破上层控制的关键入口。

目标：**不改 FOC、不换硬件**，仅通过上层算法让电机输出更平滑、协调、稳定。

## 整体中间层架构

```text
运动规划层（MoveIt2 / 规划器）
        │
        ▼  coarse 路径点
┌─ Layer 1: 轨迹整形 ──────────────┐
│  Ruckig / TOTG                    │  ← 把粗点插成光滑S曲线
│  输出：pos(t), vel(t), acc(t)     │
└───────────────────────────────────┘
        │
        ▼  平滑后的 pos, vel
┌─ Layer 2: 动力学前馈 ──────────────┐
│  Pinocchio / Computed Torque      │  ← 计算重力+摩擦+科氏力矩
│  输出：τ_ff (注入MIT帧的torque字段) │
└───────────────────────────────────┘
        │
        ▼  pos + vel + τ_ff
┌─ Layer 3: 阻抗/顺应控制 ──────────┐
│  调节 Kp/Kd（关节或笛卡尔空间）    │  ← 让末端"软"或"硬"
└───────────────────────────────────┘
        │
        ▼  MIT 帧 (pos, vel, Kp, Kd, τ_ff)
    灵足 GD32 MCU（闭源 FOC）
        │
        ▼  电流 → 电机输出
```

## Layer 1：轨迹整形

### Ruckig（在线轨迹生成）

- **仓库**：`pantor/ruckig`（Stars: 2.7k+）
- **语言**：C++，有 Python binding
- **作用**：在线实时生成时间最优 S 曲线轨迹。位置/速度/加速度全场连续
- **MIT 模式意义**：MoveIt2 的粗粒度规划点 → Ruckig 插成平滑 pos(t)/vel(t) → 喂给 MIT 帧，关节不跳
- **在项目中的位置**：MoveIt2 已内置 Ruckig 插件，开箱可用

### MoveIt2 TOTG（Time-Optimal Trajectory Generation）

- **仓库**：`moveit/moveit2` → `moveit_core/trajectory_processing/`
- **作用**：MoveIt 自带的时间最优路径参数化
- **对比 Ruckig**：Ruckig 更灵活（在线、实时），TOTG 是后处理

### ROS2 Joint Trajectory Controller

- **仓库**：`ros-controls/ros2_controllers`
- **作用**：JTC 自带线性/三次样条插补，做基础平滑够用

## Layer 2：动力学前馈（核心）

MIT 帧的 `τ_ff` 字段默认填 0。填对值 = 提前补偿重力/摩擦/惯性，不等 PD 产生偏差再补。

### ⭐ Pinocchio（刚体动力学库）

- **仓库**：`stack-of-tasks/pinocchio`（Stars: 3.3k+）
- **语言**：C++/Python，ROS2 官方包（`apt install ros-humble-pinocchio`）
- **核心函数**：
  - `pinocchio::rnea(model, data, q, v, a)` → 计算逆动力学，得到重力 + 科氏力 + 惯性力矩
  - `pinocchio::computeGeneralizedGravity(model, data, q)` → 仅重力矩（常用作前馈）
- **XC 项目用法**：`τ_ff = gravityTorque(q)` → 写入 MIT 帧 torque 字段 → 实现重力补偿
- **预期效果**：静态 hold 误差从 ~25mm 降至 ~3mm（vault 已有估算）
- **参考示例**：Seeed Studio reBot Arm 官方案例即用 Pinocchio 做重力补偿

### ⭐ tmotor_ros2（SherbyRobotics）

- **仓库**：`SherbyRobotics/tmotor_ros2`（Python，ROS2）
- **核心价值**：最接近 XC 场景的可直接参考仓库。为 T-Motor（和灵足同代产品，MIT 协议族）实现完整中间层
- **代码结构**：

  ```text
  basic_2dof_controller/     ← 开环力矩/速度/位置控制
  ⭐ pyro_2dof_controller/   ← 重点
    ├── gravity_compensation     重力补偿
    ├── effector_pd_gravity      末端 PD + 重力补偿
    ├── computed_torque          计算力矩控制（PD + 动力学前馈）
    ├── trajectory_following     轨迹跟踪
    └── ...
  ```

- **可参考的控制模式**：
  | 模式 | 输入 | 输出 | 适用场景 |
  |------|------|------|----------|
  | gravity_compensation | 关节角度 | τ_ff | 零力拖动示教 |
  | computed_torque | 轨迹 + 动力学模型 | τ_ff + PD | 高精度轨迹跟踪 |
  | effector_pd_gravity | 末端位姿 | 笛卡尔 PD + τ_ff | 遥操作 |
- **运行方式**：`ros2 launch tmotor_ros2 start_pyro_robot_controller.launch.py`

### idra-lab/ros2_effort_controller

- **仓库**：`idra-lab/ros2_effort_controller`（Stars: 10, C++）
- **支持 ROS2**：Humble / Jazzy / Rolling
- **包结构**：

  ```text
  effort_controller_base/          ← 基类，与 ros2_control hardware interface 通信
  gravity_compensation/            ← 重力补偿控制器（独立包）
  joint_impedance_controller/      ← 关节空间阻抗控制
  cartesian_impedance_controller/  ← 笛卡尔空间阻抗控制
  debug_msg/
  ```

- **引用来源**：基于 `fzi-forschungszentrum-informatik/cartesian_controllers`
- **适用场景**：如果要从 ros2_control 层面做标准控制器开发，这是最佳参考

### 摩擦补偿

- **模型**：`τ_friction = μ_c * sign(vel) + μ_v * vel + μ_s * exp(-|vel|/ε) * sign(vel)`
  - Coulomb 摩擦（μ_c）
  - 黏性摩擦（μ_v）
  - Stribeck 效应（μ_s）
- **无独立仓库**，需要在硬件接口层自己实现（一两百行代码）
- **实现位置**：OpenArmX 的 `openarm_hardware::write()` 中，往 τ_ff 加摩擦项

## Layer 3：阻抗控制

### ROS2 Admittance Controller

- **仓库**：`ros-controls/ros2_controllers` 内置
- **作用**：基于力传感器信号做零力/顺应控制
- **依赖**：需要力矩传感器（灵足 QDD 的 proprioceptive 特性可部分替代）

### Cartesian / Joint Impedance Controller

- **仓库**：同上 `idra-lab/ros2_effort_controller`
- **作用**：在关节或笛卡尔空间设定刚度和阻尼，实现柔顺交互

## 底层 CAN 通信驱动

### mini-cheetah-tmotor-python-can（DFKI）

- **仓库**：`dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can`（Stars: 43）
- **语言**：Python，SocketCAN
- **实测性能**：PCAN-USB 约 800Hz，ESD CAN-USB/2 约 1500Hz（单电机往返）
- **关键函数**：`motor.send_rad_command(pos, vel, Kp, Kd, tau_ff)`
- **意义**：和灵足 MIT 帧格式完全一致。如果需要自研灵活控制器，可以直接用这个做底层驱动

### 灵足官方 SampleProgram

- **仓库**：`RobStride/SampleProgram`（Stars: 23, C, STM32）
- **内容**：STM32 HAL 例程，包含全部控制模式
- **API 列表**：
  - `RobStride_Motor_MIT_Control(angle, speed, kp, kd, torque)` — MIT 综合控制
  - `RobStride_Motor_current_control(current)` — 电流模式（run_mode=3）
  - `RobStride_Motor_move_control(t, angle, speed, kp, kd)` — 运控模式
  - `RobStride_Motor_Pos_control(speed, angle)` — 位置模式
  - `RobStride_Motor_Speed_control(speed, limit_cur)` — 速度模式

### sirwart/robstride

- **仓库**：`sirwart/robstride`（Python，轻量 SDK）
- **用途**：快速测试单电机，不做完整控制器

## 当前 OpenArmX 的 CAN 协议已确认能力

从 `rs_motor_constants.hpp` 和 `motor_control.py` 代码确认（OpenArmX 已纳入）：

| 能力 | 参数地址 | 状态 |
|------|---------|------|
| MIT 模式 | `run_mode=0`，功能码 `0x0100` | ✅ 常规使用 |
| 电流模式（run_mode=3） | `ParamIndex::RUN_MODE = 0x7005` | ✅ 支持 |
| iq_ref 电流指令 | `ParamIndex::IQ_REF = 0x7006` | ✅ 支持 |
| 电流环 Kp/Ki | `CUR_KP = 0x7010`, `CUR_KI = 0x7011` | ✅ 可调 |
| 电流滤波系数 | `CUR_FILT_GAIN = 0x7014` | ✅ 可调 |
| 转矩限制 | `LIMIT_TORQUE = 0x700B` | ✅ 可设 |

**结论**：灵足不是 CATL 那种完全锁定的模式。CAN 协议完整开放了电流模式入口。

## 综合建议实施路线

| 优先级 | 任务 | 依赖 | 预期效果 | 参考仓库 |
|--------|------|------|----------|----------|
| **P0** | 轨迹插补（Ruckig） | MoveIt2 已内置 | 消除阶跃跳变 | `pantor/ruckig` |
| **P1** | 重力补偿前馈 | URDF 动力学参数标定 | 静态误差 10× 改善 | `stack-of-tasks/pinocchio`, `SherbyRobotics/tmotor_ros2` |
| **P2** | 摩擦补偿前馈 | 摩擦辨识实验数据 | 消除低速爬行 | 自研（参考 vault FOC-vs-MIT-QDD分析）|
| **P3** | 科氏力/惯量前馈 | 完整动力学模型 | 高速运动更平稳 | `stack-of-tasks/pinocchio`(`rnea()`) |
| **P4** | 阻抗控制 | 前馈补偿稳定后 | 柔顺力控能力 | `idra-lab/ros2_effort_controller` |

每一层都只需要在 `openarm_hardware::write()` 中增加一行：把计算好的 τ_ff 填入 MIT 帧的 torque 字段。不改 FOC，不改固件，不换硬件。

## 关键决策记录

1. **不改 FOC**：固件闭源，也无法通过外部 MCU 替代（GD32 内部集成驱动 + 控制，没有外部电流环接口）
2. **不走替换路线**：不替换灵足为宇树（VLA 依赖力矩控制带宽，高减速比不可逆）
3. **不绕开 CAN**：即使开电流模式（run_mode=3），CAN 总线 1Mbps 的 ~1kHz 更新率瓶颈仍在。τ_ff 前馈在 1kHz 下效果够用，高频补偿依赖 GD32 本地自闭环
4. **开放度评估**：灵足的 CAN 协议开放度足够做上层控制，不属于 CATL 式锁定
