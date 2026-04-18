/* Transcribe Box Lab · auth guard
   所有受保护页面 <head> 顶端引入本文件。
   未登录立即跳转到 /login.html；登录后 sessionStorage 标记 tbl_auth=1。 */
(function () {
  if (sessionStorage.getItem('tbl_auth') === '1') return;
  // 计算当前页面到站点根的相对路径
  var path = location.pathname;
  // GitHub Pages 项目站形式：/<repo>/... 或本地直开
  var parts = path.split('/').filter(Boolean);
  // 去掉最后一个文件名；再看是否 index.html
  if (parts.length > 0 && parts[parts.length - 1].endsWith('.html')) parts.pop();
  // 相对于当前页面的 login.html
  var up = parts.length > 0 ? '../'.repeat(parts.length - (path.startsWith('/' + parts[0] + '/') && parts[0].includes('.') ? 0 : 0)) : '';
  // 简化：只看从当前文件到 site root 需要几层 ..
  // 我们已知站内所有页面相对于 root 的层级：
  //   index.html / login.html / about.html      → 0 层（不加 ../）
  //   reports/index.html / repos/index.html     → 1 层（../）
  //   reports/<slug>/index.html                 → 2 层（../../）
  var file = path.substring(path.lastIndexOf('/') + 1) || 'index.html';
  var dirParts = path.replace(/\/[^/]*$/, '').split('/').filter(Boolean);
  // 找到站点根：如果 URL 是 /xc-robot2lab/reports/x/index.html，则根目录是 /xc-robot2lab/
  // 我们通过查找已知的根标记文件不可行；改为约定：
  //   约定栏目目录名仅这几个：reports / repos / dev
  var siteSections = ['reports', 'repos', 'dev', 'competitors'];
  var depth = 0;
  for (var i = dirParts.length - 1; i >= 0; i--) {
    if (siteSections.indexOf(dirParts[i]) >= 0) {
      depth = dirParts.length - i;
      break;
    }
  }
  var prefix = depth > 0 ? '../'.repeat(depth) : '';
  location.replace(prefix + 'login.html');
})();
