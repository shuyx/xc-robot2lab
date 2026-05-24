---
date: 2026-05-22
tags: [祥承电子, Xc-Robot, 电机控制, AS5047P, 磁编码器, 双编码器, encoder2raw, 硬件架构]
machine: mac-minishu
agent: opencode
session: xc-robot-skill
has_diagrams: false
---

# AS5047P 双芯片物理架构与 `encoder2raw` 寄存器分析

## 背景

核心实际问题不是“有没有双编码器”本身，而是：

1. 输入端 / 输出端相关编码器量能否被外部读取？
2. 如果能读，是否足以拿来做输出端回差闭环补偿？

本笔记按 **灵足官网产品页 + RobStride GitHub 官方手册 + AS5047P 官方手册** 重新整理，并修正原文中已与当前证据不一致的表述。

## 先说结论

- **RS03**：高置信度是“电机侧一颗 + 输出/低速侧一颗”的双编码器架构，输出侧相关量在协议层可读。
- **RS04**：高置信度存在双编码器功能和输出侧相关可读量，但公开资料对“2 颗 AS5047P 是否都在主驱动板 BOM 内”存在冲突，不能再写死为“驱动板上明确 2 颗”。
- **对控制最重要的结论**：新版官方手册里，`encoderRaw`、`encoder2raw`、`chasu_angle_out`、`theta_mech_1` 都是只读量，所以**输出侧反馈并非读不到**；主要风险在于 **手册 / 固件版本漂移**，而不是硬件侧完全没暴露。
- **是否能拿来做回差闭环**：可以作为外环输入自己做，但我没有找到灵足官方公开的“encoder2 回差补偿应用笔记”或现成控制模式说明。

## 官方资料之间的冲突与一致部分

### 一致部分

- [RobStride 03 产品页](https://www.robstride.com/products/robStride03) 和 [RobStride 04 产品页](https://www.robstride.com/products/robStride04) 都写了 `2` 个磁编码器，且文案明确是“驱动、电机、行星减速器、双编码器四合一”。
- [灵足时代产品规格介绍 20250626](https://github.com/RobStride/Product_Information/blob/master/%E7%81%B5%E8%B6%B3%E6%97%B6%E4%BB%A3%E4%BA%A7%E5%93%81%E8%A7%84%E6%A0%BC%E4%BB%8B%E7%BB%8D%20RobStride%20Product%20Specification%20Document%2020250626.pdf) 中，`RS03` 和 `RS04` 都标 `Magnetic Encoder 2 pcs`。
- `RS03` / `RS04` 新版官方手册都暴露了第二路编码器相关寄存器，且把相关量命名为“差速磁编码器”“低速角度”“输出端位置”这一类语义。

### 冲突部分

- [RS03 使用说明书 260428](https://github.com/RobStride/Product_Information/blob/master/%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/RS03/RS03%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6260428.pdf) 的“主要器件及规格”明确列 `AS5047P 2 PCS`。
- [RS04 使用说明书 260428](https://github.com/RobStride/Product_Information/blob/master/%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/RS04/RS04%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6260428.pdf) 的“主要器件及规格”却只列 `AS5047P 1 PCS`。

因此当前最稳的说法是：

- `RS03`：可以比较稳地写成“电机侧 + 输出/低速侧”双编码器。
- `RS04`：可以比较稳地写成“模块级双编码器功能成立，第二路输出侧相关量可读”；但**不能再写死“两颗 AS5047P 都明确在主驱动板 BOM 里”**。

## 关键寄存器：以 2026-04-28 官方手册为准

### RS03

| 地址 | 名称 | 手册备注 | 当前判断 |
|------|------|----------|----------|
| `0x3004` | `encoderRaw` | 磁编码器采样值 | 电机侧编码器原始值 |
| `0x3007` | `encoder2raw` | 差速磁编码器采样值 | 第二路 / 输出侧相关原始值 |
| `0x3027` | `as_angle` | 磁编初始角 | 第一颗编码器链路 |
| `0x3028` | `cs_angle` | 差速磁编初始角 | 第二颗编码器链路 |
| `0x3029` | `chasu_angle` | 差速角度 | 输出侧相关中间量 |
| `0x303e` | `theta_mech_1` | 类型 2 低速角度 | 输出 / 低速侧角度量 |
| `0x3042` | `position` | 初始化位置 | 位置结果量 |
| `0x3044` | `chasu_angle_out` | 电机位置判定参数 | 输出侧相关角度结果量 |

### RS04

| 地址 | 名称 | 手册备注 | 当前判断 |
|------|------|----------|----------|
| `0x3004` | `encoderRaw` | 磁编码器采样值 | 电机侧编码器原始值 |
| `0x3007` | `encoder2raw` | 差速磁编码器采样值 | 第二路 / 输出侧相关原始值 |
| `0x3028` | `as_angle` | 磁编初始角 | 第一颗编码器链路 |
| `0x3029` | `cs_angle` | 差速磁编初始角 | 第二颗编码器链路 |
| `0x302a` | `chasu_angle` | 差速角度 | 输出侧相关中间量 |
| `0x3032` | `position` | 电机位置判定参数 | 位置结果量 |
| `0x3034` | `chasu_angle_out` | 电机位置判定参数 | 输出侧相关角度结果量 |
| `0x3044` | `theta_mech_1` | 类型 2 低速角度 | 输出 / 低速侧角度量 |

### 重要修正：地址会漂移

原文把 `chasu_angle_out` 固定写成 `0x3033`，这在当前官方新版手册里已经不对：

- `RS03 260428`：`chasu_angle_out = 0x3044`
- `RS04 260428`：`chasu_angle_out = 0x3034`

同理，`0x3007` 在旧资料里也可能被列成 `vBus(mv)`。所以读寄存器时必须绑定：

1. **具体型号**
2. **具体手册版本**
3. **具体固件版本**

## 这些输入 / 输出端编码器量能不能读取？

结论：**能读。**

在当前官方新版手册中：

- `encoderRaw` 是只读
- `encoder2raw` 是只读
- `chasu_angle_out` 是只读
- `theta_mech_1` 是只读

这说明：

1. **电机侧相关量可读**
2. **输出 / 低速侧相关量也可读**
3. 真正的风险不是“没有接口”，而是 **旧固件 / 旧寄存器表可能对不上**

需要注意的版本前提：

- `RS03` 读参数命令要求固件至少到 `0.3.1.41`
- `RS04` 读参数命令要求固件至少到 `0.4.1.29`

如果固件太旧，`0x3007` 之类地址可能仍按旧表工作，不能直接套用新版寄存器定义。

## `encoder2raw` 与输出端误差补偿的关系

当前证据支持以下判断：

- `encoderRaw` 与 `encoder2raw` 不是文档层定义上的“固定比例镜像”
- `encoder2raw` 更像第二颗编码器链路的原始采样值
- `chasu_angle_out` / `theta_mech_1` 更像输出 / 低速侧的角度结果量

因此，如果目标是做输出端回差补偿，协议层面至少已经具备读数基础：

- 内环参考：`encoderRaw`
- 外环参考：`encoder2raw`、`chasu_angle_out`、`theta_mech_1`

但还不能把它写成“官方已经给了现成回差闭环方案”。我没有找到灵足官方公开的技术文档或应用笔记，专门说明如何使用 `encoder2` 做 backlash compensation。

## 当前最稳的物理架构表述

### RS03

高置信度可以表述为：

- 第一颗 AS5047P 在电机转子侧
- 第二颗 AS5047P 在减速器输出 / 低速侧

理由不是单靠营销文案，而是：

- 官方产品页与规格书都写双编码器
- 手册 BOM 列 `AS5047P 2 PCS`
- 手册寄存器语义明确区分“磁编码器”和“差速磁编码器”
- 示波器描述把“转子（编码器）位置”和“输出端位置”分开

### RS04

当前更稳的写法应当保守一点：

- 模块级双编码器功能成立
- 第二路输出 / 低速侧相关反馈量在协议层可读
- 但公开文档对主驱动板 BOM 数量存在冲突，因此不能只凭 BOM 断言“两颗 AS5047P 都明确在驱动板上”

更可能的解释是：

- 一颗在主驱动板 / 电机侧
- 另一颗在输出侧的小板、结构件或未被该 BOM 计入的位置

这比“两颗都在电机侧做冗余”更符合当前寄存器语义。

## 还有哪些点没有完全钉死？

- `RS04` 第二颗 AS5047P 的**物理安装位置**，仍缺正式拆解图或原理图确认
- `encoder2raw` 到输出轴物理角度的**精确换算式**，官方公开资料里未给出
- 官方没有公开 **encoder2 回差闭环补偿应用笔记**

## 参考

- [RobStride 03 产品页](https://www.robstride.com/products/robStride03)
- [RobStride 04 产品页](https://www.robstride.com/products/robStride04)
- [灵足时代产品规格介绍 20250626](https://github.com/RobStride/Product_Information/blob/master/%E7%81%B5%E8%B6%B3%E6%97%B6%E4%BB%A3%E4%BA%A7%E5%93%81%E8%A7%84%E6%A0%BC%E4%BB%8B%E7%BB%8D%20RobStride%20Product%20Specification%20Document%2020250626.pdf)
- [RS03 使用说明书 260428](https://github.com/RobStride/Product_Information/blob/master/%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/RS03/RS03%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6260428.pdf)
- [RS04 使用说明书 260428](https://github.com/RobStride/Product_Information/blob/master/%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/RS04/RS04%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6260428.pdf)
- [AS5047P 官方产品页](https://ams-osram.com/products/sensor-solutions/position-sensors/ams-as5047p-high-resolution-position-sensor)
- [AS5047P 官方数据手册 PDF](https://look.ams-osram.com/m/d05ee39221f9857/original/AS5047P-DS000324.pdf)
- [[20-编码器事实纠正硬件双芯片vs驱动单通道]]
