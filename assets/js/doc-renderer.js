/* Transcribe Box Lab · Markdown + Mermaid 文档渲染器
   Usage: 在 doc 页面引入本脚本 + CDN (marked + mermaid)，调用
     renderDoc('doc.md', document.getElementById('doc-root'))
   自动把 ```mermaid 代码块渲染成 Mermaid 图，其余保持普通 Markdown。 */
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

    // 4. 外链加 target=_blank
    container.querySelectorAll('a[href^="http"]').forEach(function(a) {
      a.target = '_blank';
      a.rel = 'noopener noreferrer';
    });

    // 5. 生成 TOC
    buildTOC(container);

    // 6. 渲染 Mermaid
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
