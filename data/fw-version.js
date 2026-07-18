(function () {
  'use strict';

  var MANIFEST_URL = 'https://ok1hra.github.io/IC-705_Interface/manifest.json';
  var FLASHER_URL  = 'https://ok1hra.github.io/IC-705_Interface/';

  var localRev  = null;
  var remoteRev = null;
  var wifiRssi  = null;
  var el        = null;
  var offline   = false;
  var bdSupported = null;
  var failCount = 0;
  var OFFLINE_AFTER = 2;      // consecutive failed polls before warning
  var FETCH_TIMEOUT = 4000;   // ms; hung request counts as a failed poll

  function injectStyles() {
    var s = document.createElement('style');
    s.textContent =
      'a.fw-version-link{color:inherit;text-decoration:none;}' +
      'a.fw-version-link:hover{text-decoration:underline;}' +
      'a.fw-update-link{color:#d97706;font-weight:700;text-decoration:none;margin-left:4px;}' +
      'a.fw-update-link:hover{color:#b45309;text-decoration:underline;}' +
      '.topbar-wifi-rssi{color:inherit;}' +
      '.topbar-wifi-rssi-bad{color:#dc2626;font-weight:700;}' +
      '.topbar-esp-offline{color:#fff;background:#dc2626;font-weight:700;' +
        'padding:1px 7px;border-radius:4px;animation:espOfflineBlink 1s step-end infinite;}' +
      '@keyframes espOfflineBlink{50%{opacity:0.45;}}' +
      '.topbar-fw-sep{color:inherit;margin:0 7px;}';
    s.textContent +=
      '.tab.tab-cat-muted{opacity:.55;}' +
      '.tab.tab-cat-muted:hover{opacity:.82;}';
    document.head.appendChild(s);
  }

  function renderHardwareNavigation() {
    document.querySelectorAll('.bd-nav').forEach(function (link) {
      link.hidden = bdSupported !== true;
    });
  }

  function render() {
    if (!el || (localRev === null && !offline)) return;
    el.innerHTML = '';

    var rssi = document.createElement('span');
    if (offline) {
      rssi.className = 'topbar-esp-offline';
      rssi.textContent = '⚠ OFFLINE';
      rssi.title = 'Page lost connection to the interface';
    } else {
      rssi.className = 'topbar-wifi-rssi';
      if (wifiRssi !== null && wifiRssi > -999) {
        rssi.textContent = wifiRssi + ' dBm';
        if (wifiRssi <= -70) {
          rssi.className += ' topbar-wifi-rssi-bad';
        }
      } else {
        rssi.textContent = '-- dBm';
      }
    }
    el.appendChild(rssi);

    if (localRev === null) return;

    var sep = document.createElement('span');
    sep.className = 'topbar-fw-sep';
    sep.textContent = '|';
    el.appendChild(sep);

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

  window.setWifiRssi = function (rssi) {
    var n = Number(rssi);
    wifiRssi = Number.isFinite(n) ? n : null;
    render();
  };

  function fetchLocal() {
    var ctrl = (typeof AbortController !== 'undefined') ? new AbortController() : null;
    var timer = ctrl ? window.setTimeout(function () { ctrl.abort(); }, FETCH_TIMEOUT) : null;
    fetch('/state', ctrl ? { signal: ctrl.signal, cache: 'no-store' } : { cache: 'no-store' })
      .then(function (r) {
        if (!r.ok) throw new Error('http ' + r.status);
        return r.json();
      })
      .then(function (d) {
        failCount = 0;
        offline = false;
        if (d && d.fwRev) {
          localRev = String(d.fwRev);
        }
        if (d && d.wifiRssi !== undefined) {
          var n = Number(d.wifiRssi);
          wifiRssi = Number.isFinite(n) ? n : null;
        }
        if (d && d.bdSupported !== undefined) {
          bdSupported = d.bdSupported === true;
          renderHardwareNavigation();
        }
        render();
      })
      .catch(function () {
        failCount++;
        if (failCount >= OFFLINE_AFTER && !offline) {
          offline = true;
          render();
        }
      })
      .finally(function () {
        if (timer !== null) window.clearTimeout(timer);
      });
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
    renderHardwareNavigation();

    var dataRev = el.getAttribute('data-rev');
    if (dataRev) {
      localRev = String(dataRev);
      render();
    } else {
      fetchLocal();
    }
    window.setInterval(fetchLocal, 5000);
    fetchRemote();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
}());
