---
date: 2026-05-24
tags: [祥承电子, Xc-Robot, 重力补偿, OpenARM, 达妙, RobStride, 编码器, 前馈]
machine: mac-minishu
agent: claude-sonnet-4-6
session: xc-robot-skill
has_diagrams: true
---

# OpenARM 重力补偿架构与 XC-Robot 状态核查

> 本文基于对 openarm_can、openarm_teleop、openarmx_xc_robot_2 三个仓库的代码审查，
> 厘清三个核心问题：OpenARM 原版的电机减速方案、重力补偿的实际实现位置、
> 以及 XC-Robot 当前是否真正启用了重力补偿。

---

## 一、纠偏：OpenARM 原版电机类型

**错误预设**（来自 B-3 对比表）：OpenARM 原版使用谐波减速器 80:1+

**代码核查结果**：OpenARM 原版使用 **达妙（Damiao）DM4310，QDD 准直驱**，不是谐波减速器。

```cpp
// openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp
enum class MotorType : uint8_t {
    DM4310     = 1,   // ← OpenARM 实际使用，QDD 行星减速
    DM4310_48V = 2,
    DM4340     = 3,
    DMH3510    = 10,  // ← DMH 前缀才是谐波（Harmonic）系列
    DMH6215    = 11,  // ← OpenARM 未使用这个系列
};

// DM4310 参数
// tMax = 10 Nm,  vMax = 30 rad/s  → 典型 QDD 扭矩/速度特性
```

原版和 XC-Robot 都属于 QDD 准直驱路线，区别在于传动类型（达妙行星 vs RobStride 行星）和软件层实现，而不是减速器类型的本质差异。

---

## 二、电机编码器对比：达妙 vs RobStride

### 2.1 达妙 DM4310：有双位置寄存器

```cpp
// dm_motor_constants.hpp 寄存器定义
enum class RID : uint8_t {
    Gr   = 20,  // Gear Ratio（减速比写入寄存器）
    p_m  = 80,  // 电机侧位置（motor-side position）
    xout = 81,  // 输出侧位置（output-side position）
};
```

`p_m`（电机轴）和 `xout`（输出轴）是两个独立的寄存器，说明 DM 驱动器固件内部**可以同时读取两侧位置**。背隙补偿在 MCU 内部做掉，对上层 CAN 协议透明。

> **注意**：MIT 控制模式的反馈帧只返回一个综合位置值，内部补偿逻辑是否完整，
> 从协议层无法直接验证。此结论存疑，不能完全确认与"标准双编码器"等价。

### 2.2 RobStride：协议层单通道，但硬件含双芯片

> **[硬件纠正 2026-05-25]**：RobStride RS04 硬件含 2×AS5047P（PCB 双芯片），不是"单编码器"。当前 OpenARMX 驱动仅读 encoderRaw（电机侧），encoder2raw（输出侧）未激活 → **实际表现等同单编码器**。驱动改造后可获输出端位置反馈，但固件融合效果待实验验证。

```text
# 灵足时代通信协议（电机通信协议汇总.md）
# 通信类型2 反馈帧：
  Byte0~1: 当前角度（单一数值，无输出侧/电机侧区分）
  Byte2~3: 当前角速度
  Byte4~5: 当前力矩
  Byte6~7: 当前温度
  Bit16~21: 故障信息（含 bit19 磁编码故障）
```

协议层只有一个位置反馈字段。RS04 硬件实有 2×AS5047P（双芯片），但通信类型 2 反馈帧不区分输出侧/电机侧，encoder2raw（0x3007）需通过私有协议读取，当前驱动未激活——实际表现等同单编码器。这是 XC-Robot 精度劣于达妙双编码器闭环方案的软件层因素。

### 2.3 精度影响小结

| 维度 | 达妙 DM4310（原版）| RobStride RS04（XC-Robot）|
|------|-------------------|--------------------------|
| 编码器 | 单编码器 + 内部双位置寄存器 | 双芯片 2×AS5047P（硬件），但当前驱动仅激活电机侧，encoder2raw 输出侧未读取，实际表现等同单编码器 |
| 背隙补偿 | MCU 内部处理（透明）| 无 |
| 减速比 | 低减速比（QDD 行星）| 9:1 行星减速器 |
| 精度影响 | 较好（背隙部分消除）| 较差（背隙直接传递到输出）|

---

## 三、OpenARM 原版的重力补偿：只在遥操作模式下

> **[范围说明]**：以下内容描述 **OpenARM 原版**（`openarm_teleop` 包）的重力补偿实现，非 XC-Robot 当前状态（XC-Robot 暂未移植此功能）。

### 3.1 代码位置

重力补偿代码在 `openarm_teleop` 包，不在 ROS2 control 栈：

```text
openarm_teleop/
└── src/controller/
    ├── dynamics.hpp  ← KDL 动力学接口
    ├── dynamics.cpp  ← GetGravity() / GetCoriolis() / GetMassMatrixDiagonal()
    └── control.cpp   ← 遥操作控制循环（调用 dynamics）
```

`openarm_ros2` 的所有 launch 文件中：`grep "gravity" = 0`——ROS2 control 栈里完全没有重力补偿。

### 3.2 遥操作模式下的工作方式

```cpp
// control.cpp - 遥操作控制循环（每 200μs 执行一次）

// 1. 读当前关节位置和速度
for (size_t i = 0; i < arm_dof; ++i) {
    joint_arm_positions[i] = joint_arm_states[i].position;
    joint_arm_velocities[i] = joint_arm_states[i].velocity;
}

// 2. 计算重力矩 G(q) + 科氏矩 C(q,q̇) ← Leader 和 Follower 都做
if (role_ == ROLE_LEADER) {
    dynamics_l_->GetGravity(joint_arm_positions.data(), gravity.data());
    dynamics_l_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(), ...);
} else if (role_ == ROLE_FOLLOWER) {
    dynamics_f_->GetGravity(joint_arm_positions.data(), gravity.data());
    dynamics_f_->GetCoriolis(...);
}

// 3. 补偿力矩 = 重力矩 + 摩擦矩
joint_arm_states_ref[i].effort = gravity[i] + friction[i];

// 4. 打包进 MIT 五元组，作为 tau_ff 直接发给电机驱动器
arm_cmds.emplace_back(MITParam{
    Kp_[i], Kd_[i],
    motor_arm_states[i].position,  // pos_ref
    motor_arm_states[i].velocity,  // vel_ref
    motor_arm_states[i].effort     // tau_ff ← 重力 + 摩擦补偿
});
openarm_->get_arm().mit_control_all(arm_cmds);
```

**关键链路：**

```mermaid
flowchart LR
    JS["关节位置 q\n关节速度 q̇"] --> DYN["KDL 动力学计算\nG(q)+C(q,q̇)"]
    DYN --> TAU["tau_ff = G + C + 摩擦"]
    TAU --> MIT["MIT 五元组\n{Kp, Kd, pos_ref, vel_ref, tau_ff}"]
    MIT -->|"CAN 直发"| MOTOR["达妙电机驱动器\nFOC 电流环"]
```

这条路完全绕过 ROS2 control，直接在遥操作控制循环里操作 CAN。

### 3.3 非遥操作模式（普通 ROS2 位置控制）

```text
JTC → hardware_interface → CAN
       ↑
       tau_ff 永远为 0（JTC 没有接重力补偿信号）
```

重力全压在位置误差上，产生下垂。这就是原版在非遥操场景也需要 OpenarmX 的 `openarmx_gravity_comp` 包的根本原因。

---

## 四、XC-Robot 重力补偿现状

### 4.1 代码层面：包存在，实现完整

```text
openarmx_xc_robot_2/openarmx_ros2/openarmx_gravity_comp/
├── src/gravity_comp_node.cpp  ← 完整实现，含 Coriolis 补偿
├── src/dynamics.cpp           ← KDL 动力学
└── include/dynamics.hpp
```

功能完整：`g_scale=1.05`（重力缩放）、`enable_coriolis=true`（科氏补偿）、运行时可动态调参。

### 4.2 YAML 层面：双臂有，单臂无

```yaml
# openarmx_v10_bimanual_controllers.yaml（双臂）
left_forward_effort_controller:   # ✅ 已注册
  type: forward_command_controller/ForwardCommandController
right_forward_effort_controller:  # ✅ 已注册
  type: forward_command_controller/ForwardCommandController

# openarmx_v10_controllers.yaml（单臂）
# ← 完全没有 forward_effort_controller    ❌ 未注册
```

### 4.3 启动层面：开关默认关闭

```python
# openarmx.bimanual.launch.py
DeclareLaunchArgument(
    "enable_forward_effort",
    default_value="false",   # ← 默认关闭
)
```

```bash
# run_bimanual_moveit_with_can2.0.sh（唯一实际使用的启动脚本）
ros2 launch openarmx_bimanual_moveit_config demo.launch.py can_fd:=false
# ↑ 没有传 enable_forward_effort:=true
```

### 4.4 结论

| 层次 | 状态 |
|------|------|
| gravity_comp 代码 | ✅ 存在，含重力 + 科氏补偿 |
| 双臂 YAML 注册 | ✅ 已注册 |
| 单臂 YAML 注册 | ❌ 未注册（缺 forward_effort_controller）|
| 启动脚本 | ❌ 未传 enable_forward_effort:=true |
| **运行时补偿状态** | **❌ 完全未启用** |

**52mm 重力下垂误差（L1）完全来自这里**——代码准备好了，但开关从未打开。

---

## 五、修复方案

### Step 1：单臂 YAML 补全（`openarmx_v10_controllers.yaml`）

在 `controller_manager.ros__parameters` 下加：

```yaml
    forward_effort_controller:
      type: forward_command_controller/ForwardCommandController
```

文件末尾加控制器参数：

```yaml
forward_effort_controller:
  ros__parameters:
    joints:
      - openarmx_joint1
      - openarmx_joint2
      - openarmx_joint3
      - openarmx_joint4
      - openarmx_joint5
      - openarmx_joint6
      - openarmx_joint7
    interface_name: effort
    command_interfaces:
      - effort
    state_interfaces:
      - position
```

### Step 2：启动命令加参数

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  enable_forward_effort:=true \
  can_fd:=false
```

或在 `gravity_comp_node_launcher` 里把单侧 enable 做成条件判断，只启用实际连接的臂。

### Step 3：参数核查（运行时）

```bash
# 确认节点在跑
ros2 node list | grep gravity

# 检查实际参数
ros2 param get /gravity_comp_node g_scale
ros2 param get /gravity_comp_node enable_coriolis

# 实时查看补偿力矩（7 个关节）
ros2 topic echo /left_forward_effort_controller/commands
```

---

## 参考代码文件

| 文件 | 说明 |
|------|------|
| `Openarm Something/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp` | DM 电机型号和寄存器定义（p_m / xout / Gr）|
| `Openarm Something/openarm_teleop/src/controller/control.cpp` | 遥操作重力补偿调用位置（L154-L174）|
| `RobStride/EDULITE_A3/el_a3_sdk/docs/电机通信协议汇总.md` | RobStride 单编码器反馈协议确认 |
| `openarmx_xc_robot_2/openarmx_ros2/openarmx_gravity_comp/src/gravity_comp_node.cpp` | XC-Robot 重力补偿节点完整实现 |
| `openarmx_xc_robot_2/openarmx_ros2/openarmx_bringup/config/v10_controllers/openarmx_v10_controllers.yaml` | 单臂 YAML（缺 forward_effort_controller）|
| `openarmx_xc_robot_2/openarmx_ros2/openarmx_bringup/launch/openarmx.bimanual.launch.py` | enable_forward_effort 默认 false |
| `openarmx_xc_robot_2/openarmx_ros2/openarmx_bimanual_moveit_config/run_bimanual_moveit_with_can2.0.sh` | 实际启动脚本（未传 enable_forward_effort）|
