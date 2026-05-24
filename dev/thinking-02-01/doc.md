---
date: 2026-05-23
tags: [祥承电子, Xc-Robot, 机械臂控制]
machine: mac-minishu
session: xc-robot-skill
has_diagrams: false
---

# 机械臂控制专题索引

本目录存放 XC-Robot 机械臂控制相关技术讨论记录。

## 目录规划

| 序号 | 文件 | 主题 |
|------|------|------|
| 01 | `01-机械臂控制.md` | 本索引 |
| 02~ | 按讨论主题递增 | — |

## 技术背景

- **硬件**：OpenARM 七自由度开源臂魔改版（灵足 RobStride RS04/RS03/RS00，行星减速器 9:1）
- **代码基**：`openarmx_xc_robot_2`（当前实际在跑，`~/Obsidian/Coding References/Openarmx_xc_robot/openarmx_xc_robot_2/`）
- **已知退化**：相比 OpenARM 原版，砍掉力控 / 控制频率降至 100Hz / 无安全机制（见 `06｜全库汇总总览/3-2_OpenARM原版与xc架构对比.md`）
- **核心痛点**：关节抖动、轨迹不平滑、缺重力补偿前馈

## 关联文档

- `01-电机控制/` — FOC/MIT 原理、PD 参数、前馈补偿信号链（底层控制基础）
- `06｜全库汇总总览/2-3_机械臂OpenARM.md`
- `06｜全库汇总总览/2-4_机械臂控制.md`
- `06｜全库汇总总览/2-5_运动控制.md`
- `06｜全库汇总总览/3-2_OpenARM原版与xc架构对比.md`
