(function () {
  'use strict';

  var MANIFEST_URL = 'https://ok1hra.github.io/IC-705_Interface/manifest.json';
  var FLASHER_URL  = 'https://ok1hra.github.io/IC-705_Interface/';

  var localRev  = null;
  var remoteRev = null;
  var el        = null;

  function injectStyles() {
    var s = document.createElement('style');
    s.textContent =
      'a.fw-version-link{color:inherit;text-decoration:none;}' +
      'a.fw-version-link:hover{text-decoration:underline;}' +
      'a.fw-update-link{color:#d97706;font-weight:700;text-decoration:none;margin-left:4px;}' +
      'a.fw-update-link:hover{color:#b45309;text-decoration:underline;}';
    document.head.appendChild(s);
  }

  function render() {
    if (!el || localRev === null) return;
    el.innerHTML = '';
    var a = document.createElement('a');
    a.href      = FLASHER_URL;
    a.target    = '_blank';
    a.rel       = 'noopener';
    a.className = 'fw-version-link';
    var hasUpdate = remoteRev !== null && Number(remoteRev) > Number(localRev);
    a.title     = hasUpdate ? 'New firmware available — click to open web installer'
                            : 'Open web installer';
    a.textContent = 'FW ' + localRev;
    el.appendChild(a);
    if (hasUpdate) {
      var upd = document.createElement('a');
      upd.href      = FLASHER_URL;
      upd.target    = '_blank';
      upd.rel       = 'noopener';
      upd.className = 'fw-update-link';
      upd.title     = 'New firmware available — click to open web installer';
      upd.textContent = ' → ' + remoteRev + ' ▲';
      el.appendChild(upd);
    }
  }

  // log-manager.js calls this instead of writing el.textContent directly
  window.setFwRev = function (rev) {
    localRev = String(rev);
    render();
  };

  function fetchLocal() {
    fetch('/state')
      .then(function (r) { return r.json(); })
      .then(function (d) {
        if (d && d.fwRev) {
          localRev = String(d.fwRev);
          render();
        }
      })
      .catch(function () {});
  }

  function fetchRemote() {
    fetch(MANIFEST_URL)
      .then(function (r) { return r.json(); })
      .then(function (d) {
        if (d && d.version) {
          remoteRev = String(d.version);
          render();
        }
      })
      .catch(function () {});
  }

  function init() {
    el = document.getElementById('topbarFw');
    if (!el) return;
    injectStyles();

    var dataRev = el.getAttribute('data-rev');
    if (dataRev) {
      localRev = String(dataRev);
      render();
    } else {
      fetchLocal();
    }
    fetchRemote();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
}());
