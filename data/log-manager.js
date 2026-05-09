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

  // ── Log manager modal UI ───────────────────────────────────────────────────

  function buildModal() {
    const el = document.createElement('div');
    el.id        = 'logMgrModal';
    el.className = 'lm-modal lm-hidden';
    el.innerHTML = `
      <div class="lm-box">
        <div class="lm-header">
          <span class="lm-title">Contest Log</span>
          <button class="lm-close" id="lmClose" type="button">✕</button>
        </div>

        <section class="lm-section" id="lmListSection">
          <div class="lm-section-title">Saved logs</div>
          <div id="lmLogList" class="lm-log-list"></div>
        </section>

        <section class="lm-section">
          <div class="lm-section-title">New log</div>
          <form id="lmNewForm" class="lm-form" autocomplete="off">
            <label class="lm-row">
              <span>Contest</span>
              <input id="lmContest" maxlength="30" placeholder="CQWW" required>
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
    `;
    document.body.appendChild(el);

    document.getElementById('lmClose').addEventListener('click', closeModal);
    el.addEventListener('click', e => { if (e.target === el) closeModal(); });
    document.getElementById('lmNewForm').addEventListener('submit', onNewFormSubmit);

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
      if (!logs.length) {
        container.innerHTML = '<div class="lm-empty">No logs yet.</div>';
        return;
      }
      logs.sort((a,b) => b.createdAtUtc.localeCompare(a.createdAtUtc));
      container.innerHTML = '';
      logs.forEach(log => {
        const isActive = _activeLog && _activeLog.id === log.id;
        const row = document.createElement('div');
        row.className = 'lm-log-row' + (isActive ? ' lm-log-active' : '');
        row.innerHTML = `
          <div class="lm-log-info">
            <span class="lm-log-name">${_esc(log.contestName)}</span>
            <span class="lm-log-meta">${_esc(log.stationCall)} &nbsp;·&nbsp; ${_esc(log.id.slice(0,10))} &nbsp;·&nbsp; QSO# ${log.nextQsoNumber - 1}</span>
          </div>
          <div class="lm-log-btns">
            ${!isActive ? `<button class="lm-btn lm-btn-sm" data-act="open" data-id="${_esc(log.id)}">Open</button>` : '<span class="lm-active-tag">active</span>'}
            <button class="lm-btn lm-btn-sm lm-btn-export" data-act="csv"  data-id="${_esc(log.id)}">CSV</button>
            <button class="lm-btn lm-btn-sm lm-btn-export" data-act="adif" data-id="${_esc(log.id)}">ADIF</button>
            <button class="lm-btn lm-btn-sm lm-btn-del"  data-act="del"  data-id="${_esc(log.id)}">Del</button>
          </div>
        `;
        container.appendChild(row);
      });
      container.addEventListener('click', onLogListClick);
    });
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
