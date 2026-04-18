/* Transcribe Box Lab · login handler
   SHA-256 of the set password is stored here.
   To change the password: run in your terminal
     python3 -c "import hashlib; print(hashlib.sha256('NEW-PASSWORD'.encode()).hexdigest())"
   and replace PASSWORD_HASH below. */
(function () {
  // 当前密码：sjtu-xc2026
  var PASSWORD_HASH = '8564e5c9bd1adf53fb51a20e1a16de293fe03231154a60d703084c39c6cb48f6';

  async function sha256(str) {
    var buf = new TextEncoder().encode(str);
    var hash = await crypto.subtle.digest('SHA-256', buf);
    return Array.from(new Uint8Array(hash))
      .map(b => b.toString(16).padStart(2, '0'))
      .join('');
  }

  function onReady(fn) {
    if (document.readyState !== 'loading') fn();
    else document.addEventListener('DOMContentLoaded', fn);
  }

  onReady(function () {
    // 若已登录，直接跳 index
    if (sessionStorage.getItem('tbl_auth') === '1') {
      location.replace('index.html');
      return;
    }
    var form = document.getElementById('login-form');
    var input = document.getElementById('password');
    var err = document.getElementById('err');
    if (!form) return;

    form.addEventListener('submit', async function (e) {
      e.preventDefault();
      err.textContent = '';
      var pwd = input.value;
      if (!pwd) return;
      try {
        var h = await sha256(pwd);
        if (h === PASSWORD_HASH) {
          sessionStorage.setItem('tbl_auth', '1');
          // 如果 URL 有 ?next=xxx，跳回去
          var params = new URLSearchParams(location.search);
          var next = params.get('next') || 'index.html';
          location.replace(next);
        } else {
          err.textContent = '密码不正确';
          input.select();
        }
      } catch (e2) {
        err.textContent = '浏览器不支持 SHA-256（请用 Chrome/Safari）';
      }
    });

    input.focus();
  });
})();
