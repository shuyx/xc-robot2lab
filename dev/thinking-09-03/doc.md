---
date: 2026-05-21
tags: [祥承电子, Xc-Robot, 项目管理, SKILL更新]
machine: mac-minishu
agent: claude-sonnet-4-6
session: xc-robot-skill
has_diagrams: false
---

# XC-Robot Skill v2.2 能力更新

## Q1：含公式的图必须用 TikZ，数学内容审查能力如何实现？

### 分析

原 v2.1 规则：Mermaid 节点禁放公式，复杂图用 TikZ。但判断标准不清晰，存在"这个图要不要 TikZ"的歧义。

v2.2 新铁律：**判断标准明确化**——

触发 TikZ 的条件（满足任一即走 TikZ）：
- 节点/标注里有 `τ`、`ζ`、`ω`、下标、分式、`=` 等数学符号
- 必须公式 + 流程结合才能讲清楚原理（如 PD 控制环路、传递函数框图、状态方程链路）

不需要 TikZ 的情况：
- 纯架构流程图（模块 A → 模块 B，没有公式）
- 状态机（过阻尼 / 临界 / 欠阻尼，文字描述即可）

**数学审查 python 脚本**：发布 lab-website 前在 `doc.md` 所在目录执行，检查两类问题：
1. Mermaid 代码块内残留 `$`、`\frac`、`\tau` 等符号
2. 正文中 `τ=`、`ζ=` 等裸 Unicode 公式（未被 `$...$` 包裹）

### 结论

- 图含公式 → TikZ（无例外）
- 发布前必须跑审查脚本，exit code 非 0 则不允许 `git add`

---

## Q2：lark-cli 飞书内容写作，callout 等富文本手法如何集成？

### 分析

lark-doc SKILL.md 是断链（`~/.agents/skills/lark-doc` 未安装），但从 memory 文件和 write-skill 里提炼了完整的飞书格式规范：

**飞书 Callout 语义规范**（必须用，不能只写纯文字）：

| 类型 | 语法 | 场景 |
|-----|------|------|
| 💡 提示 | `> 💡 **提示** ...` | 背景、定义、扩展 |
| ⚠️ 警告 | `> ⚠️ **注意** ...` | 参数边界、风险 |
| ✅ 推荐 | `> ✅ **推荐** ...` | 最优实践 |
| ❌ 禁止 | `> ❌ **禁止** ...` | 错误做法 |
| 🔑 关键 | `> 🔑 **结论** ...` | 核心决策依据 |

**飞书排版铁律**：
- 标题 ≤3 层（`###` 以下禁止）
- 加粗克制，每段 1-2 处
- Mermaid 图放节首，先图后文
- 表格优先代替无序列表堆砌
- 飞书不渲染 KaTeX，公式改 Unicode 近似写法（`τ = Kp·e_pos + Kd·e_vel`）
- 文档 >2000 字：Claude 规划结构，Gemini 撰写正文

**发布命令**：

```bash
lark-cli docs +create \
  --title "<标题>" \
  --markdown "<内容>" \
  --folder-token HRrxfpBQMlujVtd9kVSccdCUn1d
```

### 结论

飞书输出规范已写入 SKILL.md「飞书输出规范」章节，触发词「写飞书/发飞书」时自动应用。

---

## Q3：多通道输出路由如何设计？

### 分析

xc-robot 内容此前只有两条出口：07｜思考过程笔记 + lab-website。用户要求统一管理五条出口：

```text
做 PPT / 演示     → /pptskill（内容路由 + DeckIR + guizang/beamer）
发布到网站         → Lab-Website 发布流程（doc.md + index.html + git push）
导出 Word/docx    → officecli（必须带 word-reference.docx 统一模板）
写飞书文档         → lark-cli + Callout 规范
记录一下 / 写笔记  → 07｜思考过程 技术笔记（本 SKILL.md 规范）
```

pptskill 是独立的三层 agent（内容路由 → DeckIR 编译 → 渲染选型），xc-robot skill 不重复实现，直接 `/pptskill` 触发即可。

officecli 必须加 `--reference "$HOME/.claude/templates/word-reference.docx"`，禁止裸转换（历史教训：默认模板排版过素）。

### 结论

多通道路由表已写入 SKILL.md「多通道内容输出路由」章节 + v2.2 覆盖规则第 7 条，与主 CLAUDE.md 路由约定保持一致。

---

## Q4：如何通知 OpenCode / Hermes / Codex 适配 v2.2 变更？

### 分析

三端发送机制：
- **Hermes**：`hermes send --to "discord:#xc-robot-5月" "..."` + `hermes send --to "telegram:Kevin Yuan (dm)" "..."`
- **OpenCode**：`cd ~/Obsidian/kevinob && ~/Library/pnpm/opencode run "..."` — OpenCode 会加载当前目录的 skill，可直接通过 prompt 传递通知
- **Codex**：`codeagent-wrapper --backend codex "..."` — 同上，prompt 作为 context 传入

v2.1 通知时 OpenCode 回复「已收到」，v2.2 通知时 OpenCode 额外输出「Skill "xc-robot"」，说明它自动加载了更新后的 SKILL.md。

### 结论

三端均已收到并确认。标准通知 prompt 结构：**变更清单（编号）+ 无需执行操作 + 回复「已收到」**。

---

## 待办事项

- [ ] academic.css 补全 `.callout-key` / `.callout-warn` / `.callout-ok` / `.callout-info` 样式（当前 HTML callout div 已在 doc.md 中使用，需确认 CSS 已定义）
- [ ] lark-doc SKILL.md 断链问题：`~/.agents/skills/lark-doc` 未安装，若后续需要完整飞书功能需要修复软链接

## 参考

- SKILL.md：`~/.claude/skills/xc-robot/SKILL.md`（v2.2，982 行）
- write-skill：`~/.claude/skills/write-skill/SKILL.md`（第 129-248 行，飞书/Callout 规范）
- memory：`~/.claude/projects/.../memory/feedback_mermaid_no_formulas.md`
- lab-website 文章：`🔌 祥承电子/lab-website/dev/foc-mit-control-deep-dive/doc.md`
