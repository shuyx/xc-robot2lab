---
date: 2026-05-22
tags: [祥承电子, Xc-Robot, 电机控制, 双编码器, 固件融合, 背隙补偿, 验证实验, 灵足, 达妙, 宇树]
machine: mac-minishu
agent: opencode
session: xc-robot-skill
has_diagrams: false
---

# 双编码器固件融合验证：灵足 vs 宇树/达妙

## 背景

确认以下事实：
- 宇树、达妙 -2EC 的双编码器闭环补偿在 **电机驱动板固件（MCU 层）** 内完成，上层 ROS2 代码不需要额外处理
- 达妙手册明确 "2EC = dual encoder, enabling backlash estimation"——固件已做融合
- 灵足 RS04 硬件有 2×AS5047P + `encoder2raw（0x3007）` + `chasu_angle_out（0x3034）` 寄存器，但**不确定固件是否做了主动融合补偿**

OpenARMX / xc_robot_2 当前只读 `mechPos（0x7019）`。如果灵足固件内部的 `chasu_angle_out` 已含补偿、而 OpenARMX 读的是不包含补偿的 `mechPos`，那改一行寄存器地址就能获益。反之，如果灵足固件根本没做融合，则需要反馈给灵足或自行在驱动层实现。

> **⚠️ 寄存器地址纠正（2026-05-25）**：`chasu_angle_out` 正确地址为 RS04=**0x3034**、RS03=**0x3044**（来自 260428 版手册）。旧版本写 0x3033 系错误，按旧地址采集的数据无效，实验前必须确认。

## 验证实验 SOP

### 目的

判断灵足 RS04 固件是否在内部对双编码器做了融合补偿。

### 前置条件

- 一台已连接 CAN 总线的 RS04 关节模块
- 能通过 CAN 发送 MIT 帧和读取私有协议寄存器
- 能用手或工具轻轻转动关节输出端

### 步骤

1. **关节上电使能**，用 MIT 模式保持在一个固定位置（比如 0 rad，Kp=50, Kd=2）

2. **读取三组数值**
   通过私有协议（通信类型 17）分别读取：

| 寄存器 | 含义 | 命令 |
|--------|------|------|
| `encoderRaw`（0x3004） | 电机侧原始值 | 读参数 |
| `encoder2raw`（0x3007） | 输出侧原始值 | 读参数 |
| `mechPos`（0x7019） | MIT 反馈位置（当前 OpenARMX 在读的） | 读参数 |
| `chasu_angle_out`（0x3034） | 输出端计算角度（RS04）/ RS03=0x3044 | 读参数 |

3. **施加外力**：用手轻轻转动关节输出端（正反向各转小角度，≤10°），让齿轮经过背隙空行程

4. **对比读数变化**
   - 记录外力施加过程中 4 个寄存器的变化
   - 计算 `encoderRaw / 减速比（9）` 与 `mechPos` 的差值 → 应接近零（`mechPos` 是软件估算）
   - 计算 `encoder2raw` 与 `chasu_angle_out` 的差值 → 判断 chasu_angle_out 是否只是简单换算

5. **判断依据**

| 现象 | 结论 |
|------|------|
| `chasu_angle_out` 与 `mechPos` 同步变化（趋势一致，差固定比例） | 固件未做独立补偿，`chasu_angle_out` 也是软件估算值 |
| `chasu_angle_out` 对输出端外力更敏感（响应更直接） | 固件在读 `encoder2raw`，`chasu_angle_out` 是输出端真实角度 |
| 外力消失后 `chasu_angle_out` 回到原位置，`mechPos` 有偏差 | 固件已做背隙补偿，`chasu_angle_out` 是补偿后位置 |

### 后续

- 如果确认 `chasu_angle_out` 是独立有效输出端位置：修改 `v10_simple_hardware.cpp`，将位置反馈从 `mechPos（0x7019）` 改为 `chasu_angle_out（0x3034）`（RS04）/ `0x3044`（RS03）
- 如果确认 `chasu_angle_out` 也是估算值：需联系灵足确认固件是否支持双编码器融合，或自行在驱动层实现 encoder2raw 独立读取和补偿

## 行业对标总结

| 厂商          | 硬件双编码器       | 固件融合                     | 用户层需改代码 |
| ----------- | ------------ | ------------------------ | ------- |
| 宇树 G1/H1    | ✅            | ✅ 固件内完成                  | ❌ 不需要   |
| 达妙 -2EC     | ✅            | ✅ 固件 backlash estimation | ❌ 不需要   |
| 因克斯 INX     | ✅            | ✅ 2000Hz 通信              | ❌ 不需要   |
| **灵足 RS04** | ✅（2×AS5047P） | ❓ 待验证                    | ❓ 见实验   |

达妙 -2EC 的补偿在固件内完成。原版 OpenARM 代码发标准 MIT 帧，不需要额外代码就能受益——但前提是用的达妙电机。OpenARMX 将电机从达妙换为灵足后，失去了这个固件层面的优势。

## 参考

- `robstride-kscale/python/parameter_map.py` — RS04 寄存器定义
- `v10_simple_hardware.cpp` — OpenARMX 当前仅读 mechPos
- 达妙 DM-J 系列手册：-2EC 双编码器规格说明
- 宇树 G1 拆解报告：双编码器固件融合
