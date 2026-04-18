# Transcribe Box Lab · 实验室门户站点

祥承智能机器人实验室（XC-Robot）内部研究资料门户。静态站点部署于 GitHub Pages。

**访问地址**：<https://shuyx.github.io/xc-robot2lab/>
**访问方式**：需要访问密码（联系实验室管理员）

---

## 站点结构

```
lab-website/
├── index.html                 # 首页（hero + 三栏目入口）
├── login.html                 # 登录墙
├── about.html                 # 关于实验室
├── reports/                   # 栏目 1：汇报资料
│   ├── index.html             # 汇报列表
│   └── 20260419-xc-robot-phase-review/
│       ├── index.html         # Reveal.js 演示页（侧边栏 + 键盘 + 全屏）
│       └── slides/            # 34 页 SVG
├── repos/                     # 栏目 2：代码仓库索引
├── dev/                       # 栏目 3：开发内容（技术分析/BOM/架构）
├── assets/
│   ├── css/academic.css       # 学院风全站样式
│   ├── js/auth.js             # 登录页逻辑（SHA-256 + sessionStorage）
│   ├── js/auth-guard.js       # 受保护页面的守卫（未登录自动跳 login）
│   └── img/hero-robot.png     # 主视觉图
├── scripts/
│   └── add-report.py          # 新增汇报的脚手架脚本
└── README.md
```

## 新增一份汇报

```bash
python3 scripts/add-report.py \
    --date 2026-05-10 \
    --slug phase-review-may \
    --title "XC-Robot 2026-05 阶段汇报" \
    --desc "动力学标定完成 + Nav2 导航调通" \
    --svg-dir /path/to/svg-folder
```

脚本会：
1. 在 `reports/` 下新建 `20260510-phase-review-may/` 目录
2. 拷贝 SVG 到 `slides/` 子目录
3. 生成 Reveal.js 演示页
4. 在 `reports/index.html` 首项插入条目

然后 `git add -A && git commit -m '...' && git push` 即可。

## 修改访问密码

编辑 `assets/js/auth.js`，替换 `PASSWORD_HASH`：

```bash
# 生成新密码的 SHA-256
python3 -c "import hashlib; print(hashlib.sha256('NEW-PASSWORD'.encode()).hexdigest())"
```

然后 commit + push。

## 本地预览

```bash
cd lab-website
python3 -m http.server 8000
# 浏览器打开 http://localhost:8000
```

## GitHub Pages 部署

首次部署步骤（仓库创建后）：

1. 仓库 **Settings → Pages**
2. Source 选 `Deploy from a branch`
3. Branch 选 `main` / `/ (root)`
4. 保存后等 1-2 分钟，站点即上线

---

© 2026 Transcribe Box Lab · 祥承智能机器人实验室 · Internal Research Portal
