---
date: 2026-05-21
time: "15:45"
tags: [祥承电子, Xc-Robot, 电机控制, Robstride, 编码器, 行星减速器, QDD]
machine: mac-minishu
agent: claude-code
session: xc-robot-skill
source_repo: Openarm代码库/robstride-kscale
source_type: code_review
---

# Robstride RS03/RS04 编码器数量与减速器结构

## 背景

调试 OpenArmX 机械臂抖动问题，需厘清灵足关节模组的硬件结构：单编码器还是双编码器？单行星还是双行星齿轮？

## 关键分析

### 编码器：全部型号 PCB 上有 2 颗 AS5047P 芯片

来源：`robstride-kscale/python/parameter_map.py` 寄存器定义

| 寄存器 | RS00/RS02 | RS03/RS04 | 含义 |
|--------|-----------|-----------|------|
| `0x3004 encoderRaw` | ✅ | ✅ | 电机转子侧编码器 |
| `0x3007 encoder2raw` | ❌ | ✅ **有** | 输出轴侧第二路编码器 |
| `0x3030 motor_mech_angle` | ❌ | ✅ | 电机侧机械角 |
| `0x3033 chasu_angle_out` | ❌ | ✅ | 输出端角度 |

两个编码器均为 AS5047P（14-bit 单圈绝对值磁编）：
- 编码器1 → FOC 换向（转子磁场角）+ 电机侧速度估计
- 编码器2 → 输出轴位置反馈（全闭环控制潜力）

### 减速器：单级行星齿轮（9:1），非宇树双行星

- 灵足 RS04/RS03：单级行星 9:1，QDD 准直驱设计
- 宇树关节：两级行星或摆线针轮，36:1~100:1
- 9:1 低减速比 → 高力矩透明度 + 高回驱能力，但对扰动敏感

### 关键发现：encoder2 在 OpenArmX 中未被激活

<mark style="background: #BBFABBA6;">OpenArmX 代码当前仅使用 `mechPos`（= 电机侧位置 ÷ 9），并未读取 `encoder2raw` 做全闭环。
减速器齿隙和摩擦的输出误差未被输出侧编码器补偿，是低速抖动的潜在根因之一。</mark>

## 结论 / 决策

1. RS03/RS04 是双编码器架构（encoder1 转子侧 + encoder2 输出侧）
2. 减速器是单级行星齿轮（9:1），不是双行星
3. 当前控制只用了 encoder1，encoder2 未激活 → 全闭环控制是可探索的改进方向

## 待办事项

- [ ] 确认 OpenArmX 代码中 encoder2 相关接口是否已暴露（`openarmx_ros2` 硬件接口层）
- [ ] 评估激活 encoder2 全闭环的工程代价（需要修改 hardware interface 和控制器）
- [ ] 与灵足确认 encoder2 的精度指标（是否与 encoder1 相同规格）

## 参考

- `Openarm代码库/robstride-kscale/python/parameter_map.py`（寄存器 0x3004/0x3007 对比）
- `06｜全库汇总总览/2-3_OpenArm技术资料汇编上.md` §4.1（RS04/RS03/RS00 产品线）
- `06｜全库汇总总览/2-5_openarmx代码问题_运动控制篇.md`（当前抖动根因）
