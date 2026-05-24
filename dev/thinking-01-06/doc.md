---
date: 2026-05-21
time: 16:45
tags: [祥承电子, Xc-Robot, 电机控制, MIT协议, FOC, 扭矩]
machine: mac-minishu
agent: opencode
session: xc-robot-skill
---

# 传统伺服模式 vs MIT 模式，与扭矩生成

## 背景

接续前两篇笔记，补充传统驱动器与 MIT 模式的控制架构对比，以及 τ = Kt × i_q 的扭矩生成原理。

## 传统伺服驱动器模式

上位机只发「目标位置」，驱动器内部走完整的 PID 三环：

```text
上位机 ──pos_target──→ 位置环 ──→ 速度环 ──→ 电流环(FOC) ──→ 电机
                       封闭不可改     封闭不可改     可调参数有限
```

相当于只告诉司机"开到那个路口"，方向盘、油门、刹车全由车自己决定。上层无法干预中间环节的刚度/阻尼。

## MIT 运控模式

上位机通过 CAN 帧注入五元组，驱动器只算一步：

```text
τ_out = Kp·(pos_target - pos_actual) + Kd·(vel_target - vel_actual) + torque_ff
```

然后把 τ_out 转成电流执行。上层直接操控"方向盘的松紧"（Kp = 刚度）和"制动阻尼"（Kd）。

## 扭矩 τ 与电流的关系

归根结底，五元组最终都变成同一件事：**我要多大电流灌进电机线圈，来产生我想要的旋转力。**

对于表贴式 PMSM：

```text
τ = Kt × i_q

Kt = 1.5 × p × λ_pm
```

- τ — 扭矩 (N·m)
- Kt — 扭矩常数 (N·m/A)，电机本身物理参数
- p — 极对数
- λ_pm — 永磁体磁链 (Wb)
- i_q — Q 轴电流 (A)

驱动器收到 τ_out 后做：

```text
iq_ref = τ_out / Kt  →  送到 FOC 电流环执行
```

## FOC 坐标变换链路

```text
ia, ib, ic (三相静止)
    │
    ▼ Clark 变换
iα, iβ (两相静止)
    │
    ▼ Park 变换 (用转子角度 θ)
i_d, i_q (两相旋转)  ← 在这里调 i_d=0, i_q=iq_ref
    │
    ▼ 反 Park
vα, vβ
    │
    ▼ SVPWM
6路PWM → DRV8353(MOSFET) → 三相电压
```

Clark 变换把 3 个正弦波变 2 个；Park 变换用转子角度把这 2 个变到跟转子同步旋转的坐标轴上——这样 i_d 和 i_q 就成了**直流分量**（不再随时间正弦变化），可以用普通的 PI 控制器精准调节。

## 与前两篇的关系

- 04-MIT-五元组解释.md ← MIT 协议定义和外环
- 05-DQ轴与FOC原理.md ← D/Q 轴物理意义和弱磁
- 本篇 ← 传统 vs MIT 架构对比 + 扭矩如何转化为电流

