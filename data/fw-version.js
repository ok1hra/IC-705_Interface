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
      'a.fw-update-link{color:#d97706;font-weight:700;text-decoration:none;margin-left:4px;}' +
      'a.fw-update-link:hover{color:#b45309;text-decoration:underline;}';
    document.head.appendChild(s);
  }

  function render() {
    if (!el || localRev === null) return;
    var text = 'FW ' + localRev;
    if (remoteRev !== null && Number(remoteRev) > Number(localRev)) {
      var a = document.createElement('a');
      a.href      = FLASHER_URL;
      a.target    = '_blank';
      a.rel       = 'noopener';
      a.className = 'fw-update-link';
      a.title     = 'New firmware available — click to open web installer';
      a.textContent = '→ ' + remoteRev + ' ▲';
      el.textContent = text;
      el.appendChild(a);
    } else {
      el.textContent = text;
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
