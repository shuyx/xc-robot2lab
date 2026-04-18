/* Transcribe Box Lab · 图片 Lightbox
   点击竞品图集 / 文档内图片 → 全屏浮层放大 · 支持 ESC / 点击背景 / 点击 × 关闭 */
(function () {
  var SELECTOR = '.competitor-gallery img, .competitor-hero img, .doc-prose img';

  function openLightbox(src, alt) {
    // 已有则先关闭
    var exist = document.querySelector('.lightbox');
    if (exist) exist.remove();

    var box = document.createElement('div');
    box.className = 'lightbox';

    var backdrop = document.createElement('div');
    backdrop.className = 'lightbox-backdrop';

    var img = document.createElement('img');
    img.src = src;
    img.alt = alt || '';

    var closeBtn = document.createElement('button');
    closeBtn.className = 'lightbox-close';
    closeBtn.innerHTML = '×';
    closeBtn.setAttribute('aria-label', '关闭');

    box.appendChild(backdrop);
    box.appendChild(img);
    box.appendChild(closeBtn);

    if (alt) {
      var cap = document.createElement('div');
      cap.className = 'lightbox-caption';
      cap.textContent = alt;
      box.appendChild(cap);
    }

    document.body.appendChild(box);
    document.body.style.overflow = 'hidden';

    function close() {
      box.remove();
      document.body.style.overflow = '';
      document.removeEventListener('keydown', onEsc);
    }
    function onEsc(e) { if (e.key === 'Escape') close(); }

    backdrop.addEventListener('click', close);
    img.addEventListener('click', close);
    closeBtn.addEventListener('click', close);
    document.addEventListener('keydown', onEsc);
  }

  // 全局点击代理
  document.addEventListener('click', function (e) {
    var t = e.target;
    if (t && t.tagName === 'IMG' && t.matches(SELECTOR)) {
      e.preventDefault();
      openLightbox(t.src, t.alt);
    }
  });
})();
