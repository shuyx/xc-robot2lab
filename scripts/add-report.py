#!/usr/bin/env python3
"""add-report.py — 快速为 Transcribe Box Lab 站点添加一份新的进度汇报。

示例：
  python3 scripts/add-report.py \\
      --date 2026-05-10 \\
      --slug phase-review-may \\
      --title "XC-Robot 2026-05 阶段汇报" \\
      --desc "机械臂动力学标定完成 + 底盘 EKF 融合落地 + Nav2 点到点导航调通" \\
      --svg-dir ~/Obsidian/kevinob/🔌\\ 祥承电子/02｜研发进展跟踪/xxx/svg

动作：
  1. 在 reports/ 下新建 YYYYMMDD-<slug>/ 目录
  2. 拷贝 svg-dir 里的所有 slide_*.svg 到 slides/ 子目录
  3. 基于模板（复用 20260419-xc-robot-phase-review/index.html 结构）生成新的 index.html
  4. 在 reports/index.html 的列表顶部插入新汇报条目（时间倒序）
"""
import argparse
import shutil
import re
from pathlib import Path
from datetime import datetime

ROOT = Path(__file__).resolve().parent.parent   # lab-website/
REPORTS_DIR = ROOT / "reports"
TEMPLATE_DIR = REPORTS_DIR / "20260419-xc-robot-phase-review"


def build_reveal_html(n_slides: int, title: str) -> str:
    """复用模板 index.html，只替换标题与 section 列表。"""
    tpl = (TEMPLATE_DIR / "index.html").read_text(encoding="utf-8")
    # 1. 替换 <title>
    tpl = re.sub(
        r"<title>.*?</title>",
        f"<title>{title} · Transcribe Box Lab</title>",
        tpl,
        count=1,
    )
    # 2. 替换顶部工具栏 brand 里的文字
    tpl = re.sub(
        r"XC-ROBOT · 2026-04 PHASE REVIEW",
        title.upper(),
        tpl,
        count=1,
    )
    # 3. 替换 section 列表
    sections = "\n        ".join(
        f'<section><img class="slide-svg" src="slides/slide_{i:02d}.svg"></section>'
        for i in range(n_slides)
    )
    tpl = re.sub(
        r"<!-- 35 个 section 由 JS 动态生成，或静态写好；此处静态写 -->.*?</div>\s*</div>\s*</div>",
        f"<!-- 动态生成 -->\n        {sections}\n      </div>\n    </div>\n  </div>",
        tpl,
        count=1,
        flags=re.DOTALL,
    )
    return tpl


def update_reports_index(date: str, slug: str, title: str, desc: str, n_slides: int):
    """在 reports/index.html 的 item-list 首项之前插入新条目。"""
    idx = REPORTS_DIR / "index.html"
    text = idx.read_text(encoding="utf-8")

    new_item = (
        f'    <li class="item">\n'
        f'      <div class="date">{date}</div>\n'
        f'      <div>\n'
        f'        <a class="title" href="{slug}/index.html">{title}</a>\n'
        f'        <p class="desc">{desc}</p>\n'
        f'      </div>\n'
        f'      <div class="meta">\n'
        f'        <span class="badge">{n_slides} 页</span><br>\n'
        f'        <span style="display:inline-block;margin-top:6px">阶段汇报</span>\n'
        f'      </div>\n'
        f'    </li>\n\n'
    )

    # 插入到 <ul class="item-list"> 打开标签之后
    text = re.sub(
        r'(<ul class="item-list">\n)',
        r'\1' + new_item,
        text,
        count=1,
    )
    idx.write_text(text, encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", required=True, help="YYYY-MM-DD")
    ap.add_argument("--slug", required=True, help="URL 友好的短标识（英文-连字符）")
    ap.add_argument("--title", required=True, help="汇报中文标题")
    ap.add_argument("--desc", required=True, help="一句话描述")
    ap.add_argument("--svg-dir", required=True, type=Path, help="含 slide_*.svg 的目录")
    args = ap.parse_args()

    # 校验
    try:
        datetime.strptime(args.date, "%Y-%m-%d")
    except ValueError:
        raise SystemExit("日期格式必须 YYYY-MM-DD")

    svg_dir = args.svg_dir.expanduser()
    svgs = sorted(svg_dir.glob("slide_*.svg"))
    if not svgs:
        raise SystemExit(f"在 {svg_dir} 未找到 slide_*.svg 文件")

    # 目录命名：YYYYMMDD-<slug>
    dir_name = args.date.replace("-", "") + "-" + args.slug
    target = REPORTS_DIR / dir_name
    target.mkdir(parents=True, exist_ok=False)

    # 拷贝 SVG
    slides_dir = target / "slides"
    slides_dir.mkdir()
    for s in svgs:
        shutil.copy2(s, slides_dir / s.name)

    # 生成 index.html
    html = build_reveal_html(len(svgs), args.title)
    (target / "index.html").write_text(html, encoding="utf-8")

    # 更新 reports/index.html
    update_reports_index(args.date, dir_name, args.title, args.desc, len(svgs))

    print(f"✅ 新汇报已创建：reports/{dir_name}/")
    print(f"   · {len(svgs)} 页 SVG 已拷贝")
    print(f"   · reports/index.html 已插入条目")
    print(f"   · 下一步：git add -A && git commit -m 'add: {args.title}' && git push")


if __name__ == "__main__":
    main()
