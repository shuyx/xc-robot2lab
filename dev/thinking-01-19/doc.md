---
date: 2026-05-22
tags: [祥承电子, Xc-Robot, 电机控制, 达妙, 灵足, RobStride, 双编码器, 成本评估, 方案对比]
machine: mac-minishu
agent: opencode
session: xc-robot-skill
has_diagrams: true
---

# 达妙 vs 灵足编码器分析与全方案成本评估

## 背景

基于代码仓库调研，确认 OpenARM 原版与 XC-Robot 在电机选型上的关键差异。综合笔记 14-18 的讨论，给出两个可行升级方案的成本评估。

---

## 一、代码实证：达妙 vs 灵足的编码器差距

### OpenARM 原版（达妙双编码器）

从 `Coding References/Openarm Something/openarm_driver/src/openarm_driver/config.yaml`：

```yaml
motor_config:
  types: ['DM8009', 'DM8009', 'DM4340', 'DM4340', 'DM4310', 'DM4310', 'DM4310', 'DM3507']
```

原版全用达妙（Damiao）电机。`openarm_can` 库有专门的 `damiao_motor/` 驱动目录（`dm_motor_control.cpp` 等）。**达妙 DM-J 全系标配双磁编码器（-2EC 后缀）**。

### OpenARMX（XC-Robot）= 灵足 RobStride

从 `Coding References/OpenarmX/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp`：

```cpp
#include <openarmx/robstride_motor/rs_motor_control.hpp>
```

硬件接口层全部使用 `robstride_motor::` 命名空间。电机从达妙换成了灵足 RobStride。编码器能力退化：

| | 原版 OpenARM（达妙） | XC-Robot（灵足 RS04） |
|---|---|---|
| 编码器数量 | **双编码器**（电机侧 + 输出侧） | **硬件 2×AS5047P，驱动未启用** |
| 输出端反馈 | 物理编码器直接测量 | 软件估算（÷减速比），驱动未读 encoder2raw |
| 背隙补偿 | 可闭环补偿 | 当前无法补偿（硬件有潜力，需改驱动） |
| 典型型号 | DM-J8009-2EC | RS04 |

**关键认知**：OpenARMX 团队替换电机时，编码器能力相比原版是**退化**的——虽然 RS04 硬件上有 2 颗 AS5047P 芯片（`encoder2raw` 寄存器 0x3007 存在），但 OpenARMX 驱动代码**未激活** `encoder2raw` 独立回读，实际效果等同单编码器。换回达妙不仅硬件更成熟（成品级双编码器支持），还可以复用原版 OpenARM 大量的驱动代码和调参经验。

### 灵足 RS04 当前的问题：驱动层未启用双编码器能力

从 `Coding References/RobStride/` 的 Python 和 C++ 代码确认：
- `encoderRaw（0x3004）` 是唯一的物理编码器读数（14bit 磁编码器）
- `mechPos（0x7019）` 是 `encoderRaw × 减速比` 的软件估算值
- 代码中只有 `MECHANICAL_OFFSET（0x2005）` 用于归零校准，没有背隙检测或补偿逻辑
- 校准仅含 `homing_offset`（零偏）和 `direction`（方向），和背隙无关

---

## 二、两个可行升级方案

### 方案 A：肩肘谐波 + 其余达妙双编码器行星（高价高精度）

| 关节 | 模组 | 推荐型号 | 成本（估） | 理由 |
|------|------|---------|-----------|------|
| **J1** | **谐波** 100:1 | 达妙 DM-JH14 / 绿的谐波一体 | ~¥4,000 | 背隙经全臂长放大 |
| **J2** | **谐波** 100:1 | 同上 | ~¥4,000 | 同上 |
| J3 | 行星双编码 | 达妙 DM-J8009-2EC | ~¥4,751 | 背隙不放大 |
| **J4** | **谐波** 80:1 | 达妙 DM-JH11 | ~¥3,000 | 背隙经前臂放大 |
| J5 | 行星双编码 | 达妙 DM-J4310-2EC | ~¥899 | 直接末端 |
| J6 | 行星双编码 | 同上 | ~¥899 | 直接末端 |
| J7 | 行星双编码 | 达妙 DM-J3507-2EC | ~¥700 | 最小模组 |
| **合计** | | | **~¥18,249/臂** | |

### 方案 B：全达妙双编码器行星（推荐，性价比最优）

| 关节 | 模组 | 推荐型号 | 成本（估） |
|------|------|---------|-----------|
| J1 | 行星双编码 | 达妙 DM-J8009-2EC（40 Nm 峰扭）| ~¥4,751 |
| J2 | 行星双编码 | 同上 | ~¥4,751 |
| J3 | 行星双编码 | 达妙 DM-J4340-2EC（27 Nm）| ~¥1,500 |
| J4 | 行星双编码 | 同上 | ~¥1,500 |
| J5 | 行星双编码 | 达妙 DM-J4310-2EC（12.5 Nm）| ~¥899 |
| J6 | 行星双编码 | 同上 | ~¥899 |
| J7 | 行星双编码 | 达妙 DM-J3507-2EC（3 Nm）| ~¥700 |
| **合计** | | | **~¥14,999/臂** |

---

## 三、四方案成本全景

| 方案 | 单臂成本 | 双臂成本 | 整车 BOM（原~6.4万） | 末端精度 |
|------|---------|---------|-------------------|---------|
| **① 当前（全 RS04 单编码）** | ~¥8,400 | ~¥16,800 | ~6.4 万（基线） | ±1-2mm |
| **② 全达妙双编码行星（B）** | ~¥15,000 | ~¥30,000 | +¥1.32 万 → ~7.72万 | ±0.3-0.5mm |
| **③ 谐波+达妙双编码（A）** | ~¥18,200 | ~¥36,400 | +¥1.96 万 → ~8.36万 | ±0.1-0.3mm |
| ④ 全谐波（参考） | ~¥28,000 | ~¥56,000 | +¥3.92 万 → ~10.32万 | ±0.05mm |

---

## 四、方案对比与推荐

| 维度 | 方案 A（谐波混合） | 方案 B（全达妙行星） |
|------|------------------|-------------------|
| 成本增量（双臂，vs 当前） | +¥1.96 万 | **+¥1.32 万**（推荐） |
| 精度改善 | 最优（谐波零背隙） | 良好（双编码器补偿） |
| 安装复杂度 | 高（谐波/行星混装） | **低（全行星，接口统一）** |
| 代码兼容性 | 中等（需适配） | **高（原版达妙驱动可复用）** |
| 供应链风险 | 谐波集中（绿的） | **分散（达妙/因克斯/高擎）** |
| 抗冲击 | 谐波寿命 ~10,000 hrs | **行星 >100,000 hrs** |

### 方案 B 精度对场景的覆盖

±0.3-0.5mm 末端精度：

| 场景 | 可行性 |
|------|--------|
| SMT 料盘分拣 | ✅ 完全可行 |
| 线边转运 | ✅ 完全可行 |
| 拆码垛 | ✅ 完全可行 |
| 仓储搬运 | ✅ 完全可行 |
| 精密插孔/装配 | 🟡 需视觉引导补偿 |
| 拖动示教/力控 | 🟡 精度够，缺力矩传感器 |

---

## 五、分阶段实施建议

```text
阶段 1（推荐先做）: 换全达妙双编码器行星 + 摩擦辨识标定
  → 成本增量 ¥1.32 万（双臂）
  → 目标精度 ±0.3-0.5mm
  → 验证是否覆盖 90% 应用场景

阶段 2（按需）: 如阶段 1 精度不够，再升级 J1/J2/J4 为谐波
  → 从方案 B 升级到方案 A
  → 仅再追加 ¥6,400（双臂）
  → 末端精度提升到 ±0.1mm
```

### 代码复用

原版 OpenARM 的达妙驱动代码可直接使用：

```text
openarm_can/src/openarm/damiao_motor/
├── dm_motor.cpp
├── dm_motor_control.cpp
├── dm_motor_device.cpp
└── dm_motor_device_collection.cpp
```

以及 PD 优化工具：
```text
dm_pd_optimizer/DM_Control_Python/
├── DM_CAN.py
├── DM_Motor_Test.py
└── BFGS_pd_finder.py
```

这些代码在 openarmx 移除达妙支持、换成 RobStride 后被弃用，但仍在仓库中。换回达妙后可恢复使用。

---

## 参考

- `Coding References/Openarm Something/openarm_driver/src/openarm_driver/config.yaml`
- `Coding References/OpenarmX/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp`
- `Coding References/Openarm Something/openarm_can/src/openarm/damiao_motor/`
- `Coding References/Openarm Something/dm_pd_optimizer/`
- [[18-肩肘谐波方案与OpenARM关节分配设计]]
- [[16-行星双编码器谐波vs冲击与行业调研]]
