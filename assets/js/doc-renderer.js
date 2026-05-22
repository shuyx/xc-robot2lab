/* Transcribe Box Lab · Markdown + Mermaid + KaTeX 文档渲染器
   Usage: 在 doc 页面引入本脚本 + CDN (marked + mermaid + katex + auto-render)，调用
     renderDoc('doc.md', document.getElementById('doc-root'))
   - ```mermaid 代码块 → Mermaid 图
   - $$...$$ 独立公式 / $...$ 行内公式 → KaTeX 渲染
   - Excalidraw 图：导出 SVG 后 doc.md 里 ![alt](./fig.svg) 直接引用 */
(function (global) {
  /** 剥离 YAML frontmatter（---...---） */
  function stripFrontmatter(md) {
    if (!md.startsWith('---')) return md;
    var end = md.indexOf('---', 3);
    if (end === -1) return md;
    return md.slice(end + 3).replace(/^\n+/, '');
  }

  /** marked 对 code block 内容做了 HTML escape；
   *  把 Mermaid 代码还原成原始文本再交给 Mermaid 解析。 */
  function unescapeHtml(s) {
    return s
      .replace(/&amp;/g, '&')
      .replace(/&lt;/g, '<')
      .replace(/&gt;/g, '>')
      .replace(/&quot;/g, '"')
      .replace(/&#39;/g, "'");
  }

  async function renderDoc(url, container) {
    const resp = await fetch(url);
    if (!resp.ok) {
      container.innerHTML = '<p style="color:#b94a48">文档加载失败 · HTTP ' + resp.status + '</p>';
      return;
    }
    let md = await resp.text();

    // 1. 剥离 frontmatter（Obsidian 源文档通常含 ---...--- 头）
    md = stripFrontmatter(md);

    // 1.5 在 marked 处理前，把数学公式块提取出来，
    //     防止 marked 把 \{ \} \; 等当 Markdown 转义序列处理
    const mathStore = [];
    const MPRE = '⧫MATH', MPOST = 'MATH⧫'; // ⧫ 不在 Markdown 语法字符集中
    // 先处理块公式 $$...$$（多行）
    md = md.replace(/\$\$([\s\S]+?)\$\$/g, function (match) {
      const i = mathStore.push(match) - 1;
      return MPRE + i + MPOST;
    });
    // 再处理行内公式 $...$（单行）
    md = md.replace(/\$([^\$\n]+?)\$/g, function (match) {
      const i = mathStore.push(match) - 1;
      return MPRE + i + MPOST;
    });

    // 2. 自定义 renderer：把 ```mermaid 转为 <div class="mermaid">
    const renderer = new marked.Renderer();
    const origCode = renderer.code.bind(renderer);
    renderer.code = function (token) {
      const code = (typeof token === 'object') ? token.text : token;
      const lang = (typeof token === 'object') ? token.lang : arguments[1];
      if (lang === 'mermaid') {
        // marked 会把代码块内容做 HTML escape；反转义后交给 Mermaid
        return '<div class="mermaid">' + unescapeHtml(code) + '</div>';
      }
      return origCode.apply(this, arguments);
    };
    marked.use({ renderer, gfm: true, breaks: false, async: false });

    // 3. 渲染
    container.innerHTML = marked.parse(md);

    // 3.5 还原被提取的数学公式（marked 处理完之后，KaTeX 之前）
    if (mathStore.length > 0) {
      let html = container.innerHTML;
      mathStore.forEach(function (block, i) {
        html = html.split(MPRE + i + MPOST).join(block);
      });
      container.innerHTML = html;
    }

    // 4. 外链加 target=_blank
    container.querySelectorAll('a[href^="http"]').forEach(function(a) {
      a.target = '_blank';
      a.rel = 'noopener noreferrer';
    });

    // 5. 生成 TOC
    buildTOC(container);

    // 6. 渲染数学公式（KaTeX auto-render）
    //    需要页面已加载 katex.min.js + contrib/auto-render.min.js
    if (global.renderMathInElement) {
      global.renderMathInElement(container, {
        delimiters: [
          { left: '$$', right: '$$', display: true  },
          { left: '$',  right: '$',  display: false }
        ],
        throwOnError: false,
        // 跳过代码块，避免误处理 `$var` 变量名
        ignoredTags: ['script', 'noscript', 'style', 'textarea', 'pre', 'code', 'option']
      });
    }

    // 7. 渲染 Mermaid
    if (global.mermaid) {
      try {
        await global.mermaid.run({
          nodes: container.querySelectorAll('.mermaid'),
          suppressErrors: true   // 渲染失败时降级显示原文而非崩溃
        });
      } catch (e) {
        console.warn('Mermaid 渲染异常：', e);
      }
    }
  }

  function buildTOC(container) {
    const tocEl = document.getElementById('doc-toc');
    if (!tocEl) return;
    const headings = container.querySelectorAll('h2, h3');
    if (headings.length === 0) return;
    const ul = document.createElement('ul');
    headings.forEach(function(h, i) {
      h.id = 'h-' + i;
      const li = document.createElement('li');
      li.className = 'toc-' + h.tagName.toLowerCase();
      const a = document.createElement('a');
      a.href = '#h-' + i;
      a.textContent = h.textContent.replace(/^\d+\.\s*/, '');
      li.appendChild(a);
      ul.appendChild(li);
    });
    tocEl.appendChild(ul);
  }

  global.renderDoc = renderDoc;
})(window);
