---
date: 2026-05-21
time: 15:58
tags: [祥承电子, Xc-Robot, 电机控制, MIT协议, FOC]
machine: mac-minishu
agent: opencode
session: xc-robot-skill
---
有几个问题，我现在已经大概能理解 MIT 和 FOC 的关系了，也大概知道了 MIT 用户模式下“不言自明”的解释。

但是我想了解一下，这边提到上位机通过 CAN 帧实时注入 PD 参数，具体是什么意思：

1. 什么叫 PD 参数？是因为其中包含了 KP 和 KD 吗？
2. 这里的“位置环比例增益”和“速度环阻尼增益”具体指什么？
3. 什么是位置环，什么是速度环？
4. 这里的比例增益和阻尼增益分别代表什么意思？
# MIT 运控模式与五元组解释

## 背景

用户在学习 FOC 和 MIT 协议的关系，询问 MIT 运控模式的定义和五元组含义。

## 关键分析

### MIT 运控模式本质

MIT 运控模式不是换向策略，而是**上层通信协议 + 外环 PD 控制模式**。上位机通过 CAN 帧实时注入 PD 参数，驱动器退化为力矩执行器：

```text
τ_out = Kp·(pos_target - pos_actual) + Kd·(vel_target - vel_actual) + torque_ff
```

设计初衷（MIT Mini Cheetah）：让上层算法直接控制关节刚度/阻尼，而非被封闭 PID 环阻挡。

### 五元组含义

| 元素 | 含义 | 单位 |
|------|------|------|
| pos | 目标位置 | deg/rad |
| vel | 目标速度 | deg/s |
| Kp | 位置环比例增益（刚度） | N·m/deg |
| Kd | 速度环阻尼增益 | N·m·s/deg |
| torque_ff | 力矩前馈 | N·m |

### 灵足实现 vs MIT 原生差异

- MIT Mini Cheetah：PD 环跑在驱动板 MCU（40kHz），CAN 只传高层轨迹
- 灵足 RobStride：Kp/Kd 也在 CAN 帧里传（1kHz），PD 环被 CAN 带宽锁住

这是抖动根因之一：1kHz 更新率对 QDD 关节不够，外力扰动时相当于开环一个周期。

## 建议

查灵足 `run_mode=3`（纯力矩模式），上位机自己闭环，只发 iq_ref 绕过 CAN 瓶颈。
