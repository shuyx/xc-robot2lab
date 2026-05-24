---
date: 2026-05-22
tags: [祥承电子, Xc-Robot, 电机控制, 编码器, RS04, RS03, RS00, AS5047P, 双编码器, encoder2raw, 驱动层]
machine: mac-minishu
agent: opencode
session: xc-robot-skill
has_diagrams: false
---

# 编码器事实纠正：硬件双芯片 vs 驱动单通道

## 背景

之前多次分析中我错误地将 RS04 描述为"单编码器"或"单磁编码器"。用户从技术资产库文档中发现 RS00/RS03/RS04 的电气特性都标注"磁编码器: 2 pcs"，要求重新检视。

## 纠正后的正确事实

### 硬件层（产品规格书 + BOM 确认）

> **[精度修正 2026-05-25]**：
> - **RS03/RS04**：2×AS5047P 确认（BOM + 代码实证 encoder2raw=0x3007）
> - **RS00 type2（新版）**：2×AS5047P，encoder2raw 暂不暴露 ⚠️
> - **RS00 type1（旧版）**：单编码器（1 PCS），encoder2raw 不可用
> - 来源：`灵足时代产品规格介绍.md:144` + `21-AS5047P双芯片物理架构.md:39-44`

- **RS03/RS04** 驱动器 PCB 上有 **2 颗 AS5047P 磁编码器芯片**（14bit 单圈绝对值），`encoder2raw` 寄存器已暴露 ✅
- **RS00 type2（新版）**：2 颗 AS5047P ✅，但 encoder2raw 功能暂不暴露 ⚠️
- **RS00 type1（旧版）**：仅 **1 颗** AS5047P，encoder2raw 功能不可用 ⚠️
- `灵足时代产品规格介绍.md` 电气特性表：磁编码器 = 2 pcs（注：特指新版；旧版 RS00 type1 例外）
- `RS04使用说明书260428.pdf` BOM 表：序号 3 "磁编码器芯片 AS5047P" = 2 PCS（BOM 记录存在版本差异，不能断言主板一定是 2 颗，但模块级双编码器功能成立，硬件层已有实证）
- `RS03使用说明书260428.pdf` BOM 表：同样 2 PCS

### 寄存器层（代码确认）

- **RS03/RS04**：`encoderRaw（0x3004）` + `encoder2raw（0x3007）` 两个独立寄存器，分别对应电机侧和输出侧的磁性编码器读数
- **RS00**：虽然 PCB 上有 2 颗芯片，但 `encoder2raw` 寄存器未暴露，功能上等同单编码器

### 驱动层（OpenARMX 现状）

```text
v10_simple_hardware.cpp:
- 读取 encoderRaw（电机侧）→ 软件计算 mechPos = encoderRaw ÷ 9（减速比）
- 未读取 encoder2raw（输出侧独立位置）
- 未做闭环背隙补偿
```

### 核心区分

| 层面 | 事实 | 影响 |
|------|------|------|
| **硬件** | 2×AS5047P 芯片（全部型号） | 双编码器物理基础存在 |
| **寄存器** | encoder2raw (0x3007) 可在 RS03/RS04 读取 | 输出侧独立位置可获取 |
| **驱动** | OpenARMX 未读 encoder2raw | 当前实际表现等同单编码器 |
| **达妙 DM-J-2EC** | 成品级支持双编码器独立回读 | 拿来就能用，不需要改驱动 |

### 对之前分析的更正

之前所有声称"RS04 是单编码器"的描述都需要修正为：

> RS04/RS03 硬件上有 2 颗 AS5047P 芯片（`encoder2raw` 寄存器可读输出侧位置），但 OpenARMX 驱动代码**未激活**第二路编码器独立回读——`mechPos` 仍是 `encoderRaw ÷ 9` 的软件估算值。瓶颈在驱动层，不在硬件。

## 对后续方案的影响

- 如果继续用灵足电机：可以改驱动层激活 `encoder2raw`，走"软件升级"路径获得双编码器能力（不需要换模组）
- 如果换达妙：达妙 -2EC 的成品级双编码器**不需改驱动**，拿来即用
- RS00 因寄存器未暴露，功能上受限

## 批量修正记录

本次在以下位置同步更新：

### 思考过程笔记（07｜思考过程/01-电机控制/）

| 笔记 | 修正内容 |
|------|---------|
| 03 | 标题改为"全部型号 PCB 上有 2 颗 AS5047P 芯片" |
| 14 | Q1 结论改为"硬件双芯片，驱动未启用" |
| 15 | 示意图标注修正 |
| 16 | 方案描述修正 |
| 17 | 最关键的整段重写 |
| 19 | 对比表+说明文本修正 |

### 网站文章（lab-website/dev/）

| 文档 | 修正内容 |
|------|---------|
| industry-harmonic-planetary-research | 分析结论 + TikZ 图更新 |
| dual-encoder-cost-evaluation | 对比表 + callout 更新 |
| shoulder-elbow-harmonic-plan | 约束描述更新 |
| dev/index.html | 摘要更新 |

## 参考

- `03｜技术资产库/02｜技术资产/10｜灵足时代Robstride/灵足时代产品规格介绍.md` — 磁编码器 2 pcs
- `RS04使用说明书260428.pdf` — BOM 表 AS5047P × 2
- `RS03使用说明书260428.pdf` — BOM 表 AS5047P × 2
- `robstride-kscale/python/parameter_map.py` — encoder2raw (0x3007) 寄存器
- `openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp` — 当前仅读 encoderRaw
