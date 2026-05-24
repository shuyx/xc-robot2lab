---
date: 2026-05-21
time: "15:30"
tags: [祥承电子, Xc-Robot, 项目管理, 知识管理, skill, 专家模式]
machine: mac-minishu
agent: claude-code
session: xc-robot-skill
---

# XC-Robot 技术专家 Skill 建立 — 约定与架构

## 背景

此前通过 OpenCode 的 primary mode agent 建立了 XC-Robot 项目的对话入口，但存在三个根本性不足：知识静态快照、无语义检索、跨工具孤岛。本次讨论确立了基于 Claude Code skill 的新机制。

## 核心设计决策

### 方案 A vs OpenCode Agent 的选择

方案 A（扩展 AGENTS.md + skill）优于 OpenCode primary mode 的核心原因：
- Claude Code 可直接调用 ace-tool 做语义检索，不限于静态 system prompt
- 笔记写入、wiki 生成均在同一工具链内，无跨工具孤岛
- `06｜全库汇总总览/` 20 个汇总文档已是压缩后的精华，可按需加载

### xc-robot skill 架构

```text
~/.claude/skills/xc-robot/
├── SKILL.md          # 模式激活 + 专家人格 + 7技术域 + 会话规则
├── DOMAINS.md        # 可动态更新的技术域配置（独立于SKILL）
└── scripts/
    ├── build-wiki.py         # 扫描笔记文件夹 → 生成 _Wiki.md
    └── publish-feishu-wiki.sh # Obsidian Wiki → 飞书知识库（待实现）
```

### 会话写入约定

| 约定项 | 内容 |
|--------|------|
| 写入目录 | `02｜研发进展跟踪/07｜规划 5 月份任务/（04）思考过程/<技术域>/` |
| 文件名 | `YYYYMMDD-HHmm-主题关键词.md` |
| 触发条件 | 有实质技术分析/决策/新信息时自动写入，不询问 |
| 例外 | 纯聊天/无实质内容跳过 |

### 技术域映射（初始版）

9 个技术域文件夹：01-电机控制 / 02-机械臂控制 / 03-底盘运动 / 04-视觉感知 / 05-导航规划 / 06-VLA与AI / 07-系统集成 / 08-供应链选型 / 09-项目管理

## 结论 / 决策

- `/xc-robot` 命令激活专家模式，即日起在祥承电子目录下有效
- 外部代码仓库工作区固定为 `~/Obsidian/Openarm代码库/`（现有 20+ 个 OpenARM 相关仓库）
- Wiki 生成目标平台：飞书知识库（待后续实现）
- 会话胶囊目录：`HS-TX/`（将在祥承电子目录下建立）

## 待办事项

- [ ] 建立 HS-TX 会话胶囊文件夹并配置结构
- [ ] 实现飞书 Wiki 发布脚本（lark-wiki CLI）
- [ ] 建立 DOMAINS.md 可编辑域配置文件
- [ ] 测试 `/xc-robot` 激活序列是否正常读取 KB 文件

## 参考

- Skill 文件：`~/.claude/skills/xc-robot/SKILL.md`
- 项目指令：`🔌 祥承电子/00｜项目治理与申报/AGENTS.md`
- 知识库：`🔌 祥承电子/06｜全库汇总总览/`（20 个汇总文档）
