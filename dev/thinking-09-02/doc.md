---
date: 2026-05-21
tags: [祥承电子, Xc-Robot, 项目管理, SKILL更新]
machine: mac-minishu
agent: opencode
session: xc-robot-skill
has_diagrams: false
---

# XC-Robot Skill v2.1 能力更新

## Q1：用户把之前写好的完整移植规格发来，要求更新 v2.1

### 分析

v2.1 的核心变更：
1. 图示铁律：禁止 ASCII 拼图，一律 Mermaid
2. draw.io MCP 集成：先 Mermaid 确认 → 再生成 .drawio
3. 笔记规范：frontmatter 加 has_diagrams 字段，ASCII 图入库前 MUST 转 Mermaid
4. 确认工具边界：只写 07｜思考过程，不走时间胶囊/NotePlan 反馈

### 结论

SKILL.md 已升级到 v2.1，同时写入 🔌 祥承电子/.rules/ 供 OpenCode 自动加载。

## 待办事项

- [ ] draw.io MCP 重启会话后确认 mcp__drawio__* 工具可用
