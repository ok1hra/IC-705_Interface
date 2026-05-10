'use strict';
/**
 * log-manager.js — contest log creation, selection, export (CSV + ADIF)
 * Depends on: log-db.js (LogDB)
 */

(function (global) {

  // ── Helpers ────────────────────────────────────────────────────────────────

  function todayYyyymmdd() {
    const d = new Date();
    const y = d.getUTCFullYear();
    const m = String(d.getUTCMonth() + 1).padStart(2, '0');
    const day = String(d.getUTCDate()).padStart(2, '0');
    return `${y}${m}${day}`;
  }

  // ── State ──────────────────────────────────────────────────────────────────

  let _activeLog    = null;   // current log object
  let _onLogChanged = null;   // callback(log) when active log changes

  function getActiveLog()        { return _activeLog; }
  function onLogChanged(handler) { _onLogChanged = handler; }

  function _notifyChange() {
    window._logSettings = _activeLog;   // make available to log.js / dxcc lookup
    if (_onLogChanged) _onLogChanged(_activeLog);
  }

  // ── Activate a log ─────────────────────────────────────────────────────────

  function activateLog(log) {
    _activeLog = log;
    LogDB.setSetting('activeLogId', log ? log.id : null).catch(() => {});
    _notifyChange();
  }

  // ── Restore last active log on startup ────────────────────────────────────

  function restoreActiveLog() {
    return LogDB.getAllSettings().then(cfg => {
      const id = cfg.activeLogId;
      if (!id) return null;
      return LogDB.getLog(id).then(log => {
        if (log) activateLog(log);
        return log;
      });
    });
  }

  // ── Increment QSO number in active log ────────────────────────────────────

  function bumpQsoNumber() {
    if (!_activeLog) return Promise.resolve(null);
    _activeLog.nextQsoNumber++;
    return LogDB.updateLog(_activeLog);
  }

  // ── CSV export ─────────────────────────────────────────────────────────────

  function escCsv(v) {
    const s = String(v == null ? '' : v);
    if (s.includes(',') || s.includes('"') || s.includes('\n')) {
      return '"' + s.replace(/"/g, '""') + '"';
    }
    return s;
  }

  const CSV_HEADERS = [
    'qsoNumber','dateUtc','timeUtc','call','rstSent','rstReceived',
    'exchangeReceived','frequencyHz','frequencyDisplay','mode','trx',
    'continent','country','dxccPrefix','cqZone','ituZone',
    'utcOffset','qrbKm','azimuthDeg','locatorReceived',
  ];

  function qsoToCsvRow(q) {
    const d = q.dxcc || {};
    return [
      q.qsoNumber, q.qsoDateUtc, q.timeOnUtc,
      q.call, q.rstSent, q.rstReceived, q.exchangeReceived,
      q.frequencyHz, q.frequencyDisplay, q.mode, q.trx,
      d.continent, d.country, d.mainPrefix, d.cqZone, d.ituZone,
      d.utcOffset, d.qrbKm, d.azimuthDeg,
      q.locatorReceived || '',
    ].map(escCsv).join(',');
  }

  function exportCsv(logId) {
    return LogDB.getQsosForLog(logId).then(qsos => {
      qsos.sort((a,b) => a.qsoNumber - b.qsoNumber);
      const lines = [CSV_HEADERS.join(','), ...qsos.map(qsoToCsvRow)];
      return lines.join('\r\n');
    });
  }

  function downloadCsv(logObj) {
    exportCsv(logObj.id).then(csv => {
      const date = logObj.createdAtUtc.slice(0, 10);
      const name = date + '-' + logObj.contestName.replace(/[^A-Z0-9]/gi, '') + '.csv';
      _triggerDownload(csv, name, 'text/csv;charset=utf-8;');
    });
  }

  // ── ADIF export ────────────────────────────────────────────────────────────

  function adifField(tag, val) {
    if (val == null || val === '') return '';
    const s = String(val);
    return '<' + tag + ':' + s.length + '>' + s;
  }

  function freqToAdifMhz(hz) {
    return (hz / 1e6).toFixed(6);
  }

  function freqToBand(hz) {
    if (hz >= 1800000   && hz <= 2000000)   return '160m';
    if (hz >= 3500000   && hz <= 4000000)   return '80m';
    if (hz >= 5351500   && hz <= 5366500)   return '60m';
    if (hz >= 7000000   && hz <= 7300000)   return '40m';
    if (hz >= 10100000  && hz <= 10150000)  return '30m';
    if (hz >= 14000000  && hz <= 14350000)  return '20m';
    if (hz >= 18068000  && hz <= 18168000)  return '17m';
    if (hz >= 21000000  && hz <= 21450000)  return '15m';
    if (hz >= 24890000  && hz <= 24990000)  return '12m';
    if (hz >= 28000000  && hz <= 29700000)  return '10m';
    if (hz >= 50000000  && hz <= 54000000)  return '6m';
    if (hz >= 70000000  && hz <= 71000000)  return '4m';
    if (hz >= 144000000 && hz <= 148000000) return '2m';
    if (hz >= 222000000 && hz <= 225000000) return '1.25m';
    if (hz >= 420000000 && hz <= 450000000) return '70cm';
    if (hz >= 902000000 && hz <= 928000000) return '33cm';
    return '';
  }

  function adifModeMap(mode) {
    const m = (mode || '').toUpperCase();
    if (m === 'CW' || m === 'CWR' || m === 'CW-R') return 'CW';
    if (m === 'USB') return 'SSB';
    if (m === 'LSB') return 'SSB';
    if (m === 'FM')  return 'FM';
    if (m === 'AM')  return 'AM';
    if (m === 'RTTY' || m === 'RTTY-R' || m === 'RTTYR') return 'RTTY';
    if (m === 'FSK') return 'RTTY';
    if (m === 'DV')  return 'DIGITALVOICE';
    return mode;
  }

  function qsoToAdif(q, stationCall, myLocator) {
    const fields = [
      adifField('FREQ',             freqToAdifMhz(q.frequencyHz)),
      adifField('BAND',             freqToBand(q.frequencyHz)),
      adifField('QSO_DATE',         (q.qsoDateUtc || '').replace(/-/g,'')),
      adifField('TIME_ON',          (q.timeOnUtc  || '').replace(':','')),
      adifField('CALL',             q.call),
      adifField('MODE',             adifModeMap(q.mode)),
      adifField('RST_SENT',         q.rstSent),
      adifField('STX',              String(q.qsoNumber).padStart(3,'0')),
      adifField('RST_RCVD',         q.rstReceived),
      adifField('SRX',              q.exchangeReceived),
      adifField('STATION_CALLSIGN', stationCall),
      adifField('MY_GRIDSQUARE',    myLocator),
      adifField('GRIDSQUARE',       q.locatorReceived || ''),
      '<EOR>',
    ];
    return fields.filter(Boolean).join(' ') + '\r\n';
  }

  function exportAdif(logObj) {
    return LogDB.getQsosForLog(logObj.id).then(qsos => {
      qsos.sort((a,b) => a.qsoNumber - b.qsoNumber);
      const header = 'Generated by IC-705 Interface contest log\r\n' +
                     adifField('PROGRAMID','IC705-Log') + ' ' +
                     adifField('PROGRAMVERSION','1.0') + ' ' +
                     '<EOH>\r\n\r\n';
      const records = qsos.map(q =>
        qsoToAdif(q, logObj.stationCall, logObj.myLocator)
      ).join('');
      return header + records;
    });
  }

  function downloadAdif(logObj) {
    exportAdif(logObj).then(adif => {
      const date = logObj.createdAtUtc.slice(0, 10);
      const name = date + '-' + logObj.contestName.replace(/[^A-Z0-9]/gi,'') + '.adi';
      _triggerDownload(adif, name, 'text/plain;charset=utf-8;');
    });
  }

  function _triggerDownload(content, filename, mime) {
    const blob = new Blob(['﻿' + content], { type: mime });
    const url  = URL.createObjectURL(blob);
    const a    = document.createElement('a');
    a.href     = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 500);
  }

  // ── Storage info popup (shared, created once) ─────────────────────────────

  function _ensureStoragePopup() {
    if (document.getElementById('lmStorageInfoPopup')) return;
    const d = document.createElement('div');
    d.className = 'si-popup';
    d.id = 'lmStorageInfoPopup';
    d.innerHTML = `
      <p class="si-popup-head">Storage info <button class="si-popup-close" id="lmSiClose">✕</button></p>
      <div class="si-section">
        <p class="si-h">Usage &amp; quota — where to look</p>
        <div class="si-row"><span class="si-browser">Chrome / Edge</span><span>DevTools <kbd>F12</kbd> → Application → Storage — shows exact usage and quota for this origin.</span></div>
        <div class="si-row"><span class="si-browser">Firefox</span><span>Address bar: <code>about:storage</code> — lists all origins with size. Or DevTools <kbd>F12</kbd> → Storage → IndexedDB.</span></div>
        <div class="si-row"><span class="si-browser">Safari</span><span>Develop → Web Inspector → Storage. (Enable Develop menu in Preferences → Advanced.)</span></div>
      </div>
      <div class="si-section">
        <p class="si-h">Persistent storage — protection from auto-eviction</p>
        <div class="si-row"><span class="si-browser">Chrome / Edge</span><span>Granted automatically when you bookmark this page or install it as a PWA (⋮ → "Install app"). No action usually needed.</span></div>
        <div class="si-row"><span class="si-browser">Firefox</span><span>Settings → Privacy &amp; Security → Cookies and Site Data → Manage Data → find this origin → check "Persist". Requires HTTPS.</span></div>
        <div class="si-row"><span class="si-browser">Safari</span><span>Managed automatically by the browser; explicit requests from code are not supported.</span></div>
      </div>
      <p class="si-note">This app runs over plain HTTP, so the Storage API (requires HTTPS) is unavailable to JavaScript. Desktop browsers rarely evict IndexedDB data unless the disk is critically full — especially for bookmarked pages.</p>`;
    document.body.appendChild(d);
    document.getElementById('lmSiClose').addEventListener('click', e => {
      e.stopPropagation();
      d.classList.remove('si-open');
    });
    document.addEventListener('click', e => {
      if (!d.contains(e.target)) d.classList.remove('si-open');
    });
  }

  // ── Log manager modal UI ───────────────────────────────────────────────────

  let _allLogs   = [];
  let _allCounts = [];
  let _delAllTimer = null;

  function buildModal() {
    const el = document.createElement('div');
    el.id        = 'logMgrModal';
    el.className = 'lm-modal lm-hidden';
    el.innerHTML = `
      <div class="lm-box">
        <div class="lm-header">
          <span class="lm-title">QRPLog</span>
          <button class="lm-close" id="lmClose" type="button">✕</button>
        </div>

        <div class="lm-body">
        <section class="lm-section" id="lmListSection">
          <div class="lm-list-header">
            <span class="lm-section-title" style="margin:0">Saved logs</span>
            <input id="lmSearch" class="lm-search" type="search" placeholder="filter…" autocomplete="off" spellcheck="false">
          </div>
          <div id="lmLogList" class="lm-log-list"></div>
          <div class="lm-del-all-row">
            <button id="lmDeleteAll" class="lm-btn lm-btn-sm lm-btn-del" type="button">Delete all</button>
          </div>
        </section>

        <section class="lm-section">
          <div class="lm-section-title">New log</div>
          <form id="lmNewForm" class="lm-form" autocomplete="off">
            <label class="lm-row">
              <span>Contest</span>
              <input id="lmContest" maxlength="40" placeholder="CQWW" required>
            </label>
            <label class="lm-row">
              <span>My call</span>
              <input id="lmMyCall" maxlength="16" placeholder="OK1ABC" required>
            </label>
            <label class="lm-row">
              <span>Exchange</span>
              <input id="lmExch" maxlength="10" placeholder="NR / JO70FD">
            </label>
            <label class="lm-row">
              <span>My locator</span>
              <input id="lmLoc" maxlength="6" placeholder="JO70FD">
            </label>
            <label class="lm-row">
              <span>Start QSO#</span>
              <input id="lmStartNr" type="number" min="1" value="1">
            </label>
            <div class="lm-actions">
              <button type="submit" class="lm-btn lm-btn-primary">Create &amp; activate</button>
            </div>
          </form>
        </section>
        </div>
      </div>
    `;
    document.body.appendChild(el);

    document.getElementById('lmClose').addEventListener('click', closeModal);
    el.addEventListener('click', e => { if (e.target === el) closeModal(); });
    document.getElementById('lmNewForm').addEventListener('submit', onNewFormSubmit);
    document.getElementById('lmLogList').addEventListener('click', onLogListClick);
    document.getElementById('lmSearch').addEventListener('input', applyFilter);
    document.getElementById('lmDeleteAll').addEventListener('click', onDeleteAll);

    // uppercase fields
    ['lmContest','lmMyCall','lmExch','lmLoc'].forEach(id => {
      const inp = document.getElementById(id);
      inp.addEventListener('input', () => {
        const pos = inp.selectionStart;
        inp.value = inp.value.toUpperCase();
        inp.setSelectionRange(pos, pos);
      });
    });

    return el;
  }

  function renderLogList() {
    const container = document.getElementById('lmLogList');
    if (!container) return;
    LogDB.getLogs().then(logs => {
      logs.sort((a,b) => b.createdAtUtc.localeCompare(a.createdAtUtc));
      Promise.all(logs.map(log =>
        LogDB.getQsosForLog(log.id).then(qsos => qsos.filter(q => !q.deleted).length)
      )).then(counts => {
        _allLogs   = logs;
        _allCounts = counts;
        applyFilter();
      });
    });
  }

  function applyFilter() {
    const container = document.getElementById('lmLogList');
    if (!container) return;
    const searchEl = document.getElementById('lmSearch');
    const query    = searchEl ? searchEl.value.trim().toLowerCase() : '';

    const filtered = _allLogs.reduce((acc, log, i) => {
      const match = !query
        || log.contestName.toLowerCase().includes(query)
        || log.stationCall.toLowerCase().includes(query);
      if (match) acc.push({ log, i });
      return acc;
    }, []);

    container.innerHTML = '';

    if (!filtered.length) {
      container.innerHTML = '<div class="lm-empty">' + (_allLogs.length ? 'No match.' : 'No logs yet.') + '</div>';
      return;
    }

    filtered.forEach(({ log, i }) => {
      const count    = _allCounts[i] || 0;
      const isActive = _activeLog && _activeLog.id === log.id;
      const row      = document.createElement('div');
      row.className  = 'lm-log-row' + (isActive ? ' lm-log-active' : '');
      row.innerHTML  = `
        <div class="lm-log-info">
          <span class="lm-log-name">${_esc(log.contestName)}</span>
          <span class="lm-log-meta">${_esc(log.stationCall)}${log.myLocator ? ' | ' + _esc(log.myLocator) : ''}${log.defaultExchange ? ' | ' + _esc(log.defaultExchange) : ''} | #${count}</span>
        </div>
        <div class="lm-log-btns">
          ${!isActive ? `<button class="lm-btn lm-btn-sm" data-act="open" data-id="${_esc(log.id)}">Open</button>` : '<span class="lm-active-tag">active</span>'}
          <button class="lm-btn lm-btn-sm lm-btn-export" data-act="csv"  data-id="${_esc(log.id)}">CSV</button>
          <button class="lm-btn lm-btn-sm lm-btn-export" data-act="adif" data-id="${_esc(log.id)}">ADIF</button>
          <button class="lm-btn lm-btn-sm lm-btn-del"    data-act="del"  data-id="${_esc(log.id)}">Del</button>
        </div>
      `;
      container.appendChild(row);
    });

    const total    = _allCounts.reduce((s, c) => s + c, 0);
    const filteredTotal = filtered.reduce((s, { i }) => s + (_allCounts[i] || 0), 0);
    const estKb  = total * 650;
    const estStr = estKb >= 1048576 ? '~' + (estKb / 1048576).toFixed(1) + ' MB'
                                    : '~' + (estKb / 1024).toFixed(0) + ' kB';
    const footer = document.createElement('div');
    footer.className = 'lm-log-total';
    const footerText = query
      ? `${filtered.length} / ${_allLogs.length} logs · ${filteredTotal} / ${total} QSO · ${estStr}`
      : `${_allLogs.length} logs · ${total} QSO · ${estStr}`;
    footer.innerHTML = `${footerText} <button class="si-btn" id="lmStorageInfo" type="button" title="Jak zkontrolovat storage v prohlížeči">ⓘ</button>`;
    footer.querySelector('#lmStorageInfo').addEventListener('click', e => {
      e.stopPropagation();
      _ensureStoragePopup();
      const p = document.getElementById('lmStorageInfoPopup');
      if (p) p.classList.toggle('si-open');
    });
    container.appendChild(footer);
  }

  function onLogListClick(e) {
    const btn = e.target.closest('[data-act]');
    if (!btn) return;
    const id  = btn.dataset.id;
    const act = btn.dataset.act;
    if (act === 'open') {
      LogDB.getLog(id).then(log => { if (log) { activateLog(log); closeModal(); } });
    } else if (act === 'csv') {
      LogDB.getLog(id).then(log => { if (log) downloadCsv(log); });
    } else if (act === 'adif') {
      LogDB.getLog(id).then(log => { if (log) downloadAdif(log); });
    } else if (act === 'del') {
      if (!confirm('Delete log ' + id + ' and all its QSOs?')) return;
      LogDB.getQsosForLog(id)
        .then(qsos => Promise.all(qsos.map(q => LogDB.deleteQso(q.id))))
        .then(() => LogDB.deleteLog(id))
        .then(() => {
          if (_activeLog && _activeLog.id === id) activateLog(null);
          renderLogList();
        });
    }
  }

  function onDeleteAll() {
    const btn = document.getElementById('lmDeleteAll');
    if (!btn) return;
    if (btn.dataset.confirm !== '1') {
      // first click — arm
      btn.dataset.confirm = '1';
      btn.textContent = '⚠ Confirm delete ALL?';
      btn.classList.add('lm-btn-del-confirm');
      clearTimeout(_delAllTimer);
      _delAllTimer = setTimeout(() => {
        btn.dataset.confirm = '';
        btn.textContent = 'Delete all';
        btn.classList.remove('lm-btn-del-confirm');
      }, 3000);
      return;
    }
    // second click — execute
    clearTimeout(_delAllTimer);
    btn.dataset.confirm = '';
    btn.textContent = 'Deleting…';
    btn.disabled = true;
    LogDB.openDb().then(db => new Promise((resolve, reject) => {
      const t = db.transaction(['qso', 'logs'], 'readwrite');
      t.objectStore('qso').clear();
      t.objectStore('logs').clear();
      t.oncomplete = resolve;
      t.onerror    = e => reject(e.target.error);
    })).then(() => {
      activateLog(null);
      btn.disabled = false;
      btn.textContent = 'Delete all';
      btn.classList.remove('lm-btn-del-confirm');
      renderLogList();
    }).catch(err => {
      btn.disabled = false;
      btn.textContent = 'Delete all';
      alert('Error: ' + err.message);
    });
  }

  function onNewFormSubmit(e) {
    e.preventDefault();
    const contestName     = document.getElementById('lmContest').value.trim();
    const stationCall     = document.getElementById('lmMyCall').value.trim();
    const defaultExchange = document.getElementById('lmExch').value.trim();
    const myLocator       = document.getElementById('lmLoc').value.trim();
    const startQsoNumber  = parseInt(document.getElementById('lmStartNr').value, 10) || 1;

    if (!contestName || !stationCall) return;

    LogDB.createLog({ contestName, stationCall, defaultExchange, myLocator, startQsoNumber })
      .then(log => {
        activateLog(log);
        closeModal();
      });
  }

  function _esc(s) {
    return String(s || '')
      .replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
  }

  // ── Open / close ───────────────────────────────────────────────────────────

  function openModal() {
    let modal = document.getElementById('logMgrModal');
    if (!modal) modal = buildModal();
    modal.classList.remove('lm-hidden');
    const inp = document.getElementById('lmContest');
    if (inp && !inp.value) inp.value = todayYyyymmdd() + '-';
    renderLogList();
  }

  function closeModal() {
    const modal = document.getElementById('logMgrModal');
    if (modal) modal.classList.add('lm-hidden');
  }

  // ── Export ─────────────────────────────────────────────────────────────────

  global.LogManager = {
    getActiveLog, onLogChanged, activateLog, restoreActiveLog,
    bumpQsoNumber, openModal, closeModal,
    downloadCsv, downloadAdif,
  };

}(window));
