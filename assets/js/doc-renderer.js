/* Transcribe Box Lab · Markdown + Mermaid 文档渲染器
   Usage: 在 doc 页面引入本脚本 + CDN (marked + mermaid)，调用
     renderDoc('doc.md', document.getElementById('doc-root'))
   自动把 ```mermaid 代码块渲染成 Mermaid 图，其余保持普通 Markdown。 */
(function (global) {
  async function renderDoc(url, container) {
    const resp = await fetch(url);
    if (!resp.ok) {
      container.innerHTML = `<p style="color:#b94a48">文档加载失败 · HTTP ${resp.status}</p>`;
      return;
    }
    const md = await resp.text();

    // 自定义 renderer：把 ```mermaid 代码块转为 <div class="mermaid">
    const renderer = new marked.Renderer();
    const origCode = renderer.code.bind(renderer);
    renderer.code = function (token) {
      const code = (typeof token === 'object') ? token.text : token;
      const lang = (typeof token === 'object') ? token.lang : arguments[1];
      if (lang === 'mermaid') {
        // 保留原始文本，交给 mermaid.run 处理
        return `<div class="mermaid">${code}</div>`;
      }
      return origCode.apply(this, arguments);
    };
    marked.use({
      renderer,
      gfm: true,
      breaks: false,
      async: false
    });

    // 解析
    container.innerHTML = marked.parse(md);

    // 外链加 target=_blank
    container.querySelectorAll('a[href^="http"]').forEach(a => {
      a.target = '_blank';
      a.rel = 'noopener noreferrer';
    });

    // 生成目录（TOC）
    buildTOC(container);

    // 渲染 mermaid
    if (global.mermaid) {
      try {
        await global.mermaid.run({
          nodes: container.querySelectorAll('.mermaid'),
          suppressErrors: false
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
    headings.forEach((h, i) => {
      const id = 'h-' + i;
      h.id = id;
      const li = document.createElement('li');
      li.className = 'toc-' + h.tagName.toLowerCase();
      const a = document.createElement('a');
      a.href = '#' + id;
      a.textContent = h.textContent.replace(/^\d+\.\s*/, '');
      li.appendChild(a);
      ul.appendChild(li);
    });
    tocEl.appendChild(ul);
  }

  global.renderDoc = renderDoc;
})(window);
