'use strict';

// ── App state ─────────────────────────────────────────────────────────────────

const app = {
  runMode:    'RUN',   // 'RUN' | 'SP'
  activeTrx:  1,       // 1 | 2 | 3
  trxLabels:  ['IC-705', 'TRX2', 'TRX3'],
  trxOi3:     [false, false, false],  // OI3 mode per TRX
  trxIps:     ['', '', ''],           // backend IP per TRX

  // TRX state from /state polling
  connected:  false,
  frequency:  0,
  mode:       'USB',
  tx:         false,
  fwRev:      '',

  // RST field: true if operator manually edited it for the current QSO
  rstDirty:   false,

  // Form state machine
  formState:  'IDLE',  // IDLE | CALL_ENTERED | EXCHANGE_ENTERED | LOGGED
  prevCallSent: '',    // call as sent in TXEXCH, for TU/CALL-TU decision

  _pollTimer:  null,
  _hintTimer:  null,
};

// ── Auto-backup state ─────────────────────────────────────────────────────────

let _backupIdleTimer = null;
let _qsoSinceBackup  = 0;
const BACKUP_IDLE_MS = 30 * 60 * 1000;

// ── DOM references ────────────────────────────────────────────────────────────

const inpCall     = document.getElementById('inpCall');
const inpRst      = document.getElementById('inpRst');
const inpExch     = document.getElementById('inpExch');
const btnRunMode  = document.getElementById('btnRunMode');
const logHint     = document.getElementById('logHint');
const logEmptyHint = document.getElementById('logEmptyHint');

const trxButtons  = [
  document.getElementById('btnTrx1'),
  document.getElementById('btnTrx2'),
  document.getElementById('btnTrx3'),
];

const sbTime      = document.getElementById('sbTime');
const sbFreq      = document.getElementById('sbFreq');
const sbMode      = document.getElementById('sbMode');
const sbContinent = document.getElementById('sbContinent');
const sbCountry   = document.getElementById('sbCountry');
const sbPrefix    = document.getElementById('sbPrefix');
const sbCq        = document.getElementById('sbCq');
const sbItu       = document.getElementById('sbItu');
const sbUtc       = document.getElementById('sbUtc');
const sbQrb       = document.getElementById('sbQrb');
const sbAz        = document.getElementById('sbAz');

// ── Utilities ─────────────────────────────────────────────────────────────────

function formatFreq(hz) {
  const n = Math.round(Number(hz));
  if (!n) return '-.---.--';
  const mhz  = Math.floor(n / 1_000_000);
  const khz  = Math.floor((n % 1_000_000) / 1000);
  const tens = Math.floor((n % 1000) / 10);
  return `${mhz}.${String(khz).padStart(3, '0')}.${String(tens).padStart(2, '0')}`;
}

function rstDefault(mode) {
  if (!mode) return '59';
  switch (mode.toUpperCase()) {
    case 'SSB': case 'LSB': case 'USB': case 'FM': case 'AM': case 'DV':
      return '59';
    default:
      return '599';
  }
}

function utcHHMM() {
  const d = new Date();
  return String(d.getUTCHours()).padStart(2, '0') + ':' + String(d.getUTCMinutes()).padStart(2, '0');
}

// ── UTC clock (fires every second, aligned to wall-clock seconds) ─────────────

function startClock() {
  function tick() {
    sbTime.textContent = utcHHMM();
    const msUntilNextSecond = 1000 - (Date.now() % 1000);
    setTimeout(tick, msUntilNextSecond);
  }
  tick();
}

// ── /state polling ────────────────────────────────────────────────────────────

function pollState() {
  clearTimeout(app._pollTimer);
  const trxIdx = app.activeTrx - 1;
  const isOi3  = app.trxOi3[trxIdx] && trxIdx > 0 && app.trxIps[trxIdx];
  const url    = isOi3 ? '/oi3/state?ip=' + encodeURIComponent(app.trxIps[trxIdx]) : '/state';
  fetch(url)
    .then(r => {
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return r.json();
    })
    .then(data => {
      applyState(data);
      app._pollTimer = setTimeout(pollState, isOi3 ? 500 : 250);
    })
    .catch(() => {
      applyDisconnected();
      app._pollTimer = setTimeout(pollState, 1000);
    });
}

function applyState(data) {
  app.connected = !!data.connected;
  if (data.fwRev && !app.fwRev) {
    app.fwRev = String(data.fwRev);
    if (window.setFwRev) {
      window.setFwRev(app.fwRev);
    } else {
      const el = document.getElementById('topbarFw');
      if (el) el.textContent = 'FW ' + app.fwRev;
    }
  }

  // Update frequency/mode/tx only when radio is actually connected —
  // when disconnected, preserve manually entered values
  if (app.connected) {
    app.frequency = data.frequency || 0;
    app.tx        = !!data.tx;
    const newMode = (data.mode || 'USB').trim();
    if (newMode !== app.mode) {
      app.mode = newMode;
      if (!app.rstDirty) inpRst.value = rstDefault(app.mode);
    }
  }

  renderStatusBar();
  renderConnStatus();
}

function applyDisconnected() {
  app.connected = false;
  renderStatusBar();
  renderConnStatus();
}

// ── Status bar rendering ──────────────────────────────────────────────────────

function renderStatusBar() {
  if (app.connected) {
    sbFreqModeGroup.style.display = '';
    sbManualGroup.style.display   = 'none';
    sbFreq.textContent = formatFreq(app.frequency);
    sbMode.textContent = app.mode || '---';
  } else {
    sbFreqModeGroup.style.display = 'none';
    sbManualGroup.style.display   = '';
  }
  // Preserve DXCC while call field has content; clear only when empty
  if (inpCall.value.trim()) {
    updateDxccFromCall();
  } else {
    renderDxccStatus(null);
  }
}

const sbDxccGroup      = document.getElementById('sbDxccGroup');
const azIndicator      = document.getElementById('azIndicator');
const sbFreqModeGroup  = document.getElementById('sbFreqModeGroup');
const sbManualGroup    = document.getElementById('sbManualGroup');
const sbManualFreq     = document.getElementById('sbManualFreq');
const sbManualMode     = document.getElementById('sbManualMode');

function renderDxccStatus(dxcc) {
  if (!dxcc) {
    sbDxccGroup.style.display = 'none';
    azIndicator.classList.remove('az-active');
    return;
  }
  sbDxccGroup.style.display = '';
  if (dxcc.azimuthDeg != null) {
    azIndicator.textContent = '↑';
    azIndicator.style.transform = 'rotate(' + dxcc.azimuthDeg + 'deg)';
    azIndicator.classList.add('az-active');
  } else {
    azIndicator.classList.remove('az-active');
  }
  // Phase 2 will call renderDxccStatus(result) with populated data:
  // { continent, country, mainPrefix, cqZone, ituZone, utcOffset, qrbKm, azimuthDeg }
  sbContinent.textContent = dxcc.continent   || '--';
  sbCountry.textContent   = dxcc.country     || '--';
  sbPrefix.textContent    = dxcc.mainPrefix  || '--';
  sbCq.textContent        = dxcc.cqZone ? 'CQ-' + dxcc.cqZone : 'CQ--';
  sbItu.textContent       = dxcc.ituZone ? 'ITU-' + dxcc.ituZone : 'ITU--';
  sbUtc.textContent       = dxcc.utcOffset != null ? 'utc ' + dxcc.utcOffset : 'utc --';
  sbQrb.textContent       = dxcc.qrbKm  != null ? 'QRB ' + dxcc.qrbKm + 'km' : 'QRB --';
  sbAz.textContent        = dxcc.azimuthDeg != null ? 'Az ' + dxcc.azimuthDeg + '°' : 'Az --';
}

// ── Connection status indicator ───────────────────────────────────────────────

function renderConnStatus() {
  trxButtons.forEach((b, i) => {
    b.classList.remove('trx-conn-ok', 'trx-conn-off');
    if (i === app.activeTrx - 1) {
      b.classList.add(app.connected ? 'trx-conn-ok' : 'trx-conn-off');
    }
  });
}

// ── Hint message (2 s auto-clear) ─────────────────────────────────────────────

function showHint(msg) {
  logHint.textContent = msg;
  clearTimeout(app._hintTimer);
  app._hintTimer = setTimeout(() => { logHint.textContent = ''; }, 2000);
}

// ── Auto-backup: helpers, idle timer, close warning (C + D) ──────────────────

function _backupFileTs(d) {
  const p = n => String(n).padStart(2, '0');
  return String(d.getUTCFullYear()) + p(d.getUTCMonth() + 1) + p(d.getUTCDate()) +
         '-' + p(d.getUTCHours()) + p(d.getUTCMinutes());
}

function _idbGetAllStore(db, storeName) {
  return new Promise((resolve, reject) => {
    const req = db.transaction(storeName, 'readonly').objectStore(storeName).getAll();
    req.onsuccess = () => resolve(req.result);
    req.onerror   = e  => reject(e.target.error);
  });
}

async function backupDb() {
  const now = new Date();
  const cdb = await LogDB.openDb();
  const [logs, qso, settings] = await Promise.all([
    _idbGetAllStore(cdb, 'logs'),
    _idbGetAllStore(cdb, 'qso'),
    _idbGetAllStore(cdb, 'settings'),
  ]);

  let syncState = [], devices = [];
  try {
    const sdb = await new Promise((res, rej) => {
      const r = indexedDB.open('datasyncDb', 1);
      r.onsuccess      = e => res(e.target.result);
      r.onerror        = e => rej(e.target.error);
      r.onupgradeneeded = () => {};
    });
    [syncState, devices] = await Promise.all([
      _idbGetAllStore(sdb, 'sync_state').catch(() => []),
      _idbGetAllStore(sdb, 'devices').catch(() => []),
    ]);
  } catch (_) {}

  const fname  = _backupFileTs(now) + '-QSO-database.json';
  const backup = {
    export_version: 1,
    exported_at:    now.toISOString(),
    stores:         { logs, qso, settings, sync_state: syncState, devices },
  };
  const blob = new Blob([JSON.stringify(backup, null, 2)], { type: 'application/json' });
  const url  = URL.createObjectURL(blob);
  const a    = document.createElement('a');
  a.href = url; a.download = fname; a.click();
  URL.revokeObjectURL(url);
  return fname;
}

function _beforeUnloadHandler(e) {
  e.preventDefault();
  e.returnValue = '';
}

function _disarmBeforeUnload() {
  window.removeEventListener('beforeunload', _beforeUnloadHandler);
}

function _onBackupDone() {
  _qsoSinceBackup = 0;
  _disarmBeforeUnload();
  document.getElementById('btnBackup').classList.remove('btn-backup-pending');
}

function _resetBackupTimer() {
  clearTimeout(_backupIdleTimer);
  _backupIdleTimer = setTimeout(async () => {
    try {
      const fname = await backupDb();
      showHint('Auto-backup: ' + fname);
      _onBackupDone();
    } catch (err) {
      showHint('Auto-backup failed: ' + (err.message || err));
    }
  }, BACKUP_IDLE_MS);
}

function _onQsoBackupHook() {
  _qsoSinceBackup++;
  _resetBackupTimer();
  if (_qsoSinceBackup === 1) {
    document.getElementById('btnBackup').classList.add('btn-backup-pending');
    window.addEventListener('beforeunload', _beforeUnloadHandler);
  }
}

function _initBackup() {
  document.getElementById('btnBackup').addEventListener('click', async () => {
    try {
      const fname = await backupDb();
      showHint('Backup: ' + fname);
      _onBackupDone();
    } catch (err) {
      showHint('Backup error: ' + (err.message || err));
    }
  });
}

// ── RUN / S&P toggle ──────────────────────────────────────────────────────────

function setRunMode(mode) {
  app.runMode = mode;  // 'RUN' | 'SP'
  btnRunMode.innerHTML = mode === 'RUN' ? 'R<u>U</u>N' : 'S&amp;P';
  // RUN: show *?, hide check  /  S&P: show check, hide *?
  btnCallQ.classList.toggle('btn-action-hidden', mode !== 'RUN');
  btnCheck.classList.toggle('btn-action-hidden', mode !== 'SP');
}

btnRunMode.addEventListener('click', () => {
  setRunMode(app.runMode === 'RUN' ? 'SP' : 'RUN');
  inpCall.focus();
});

// ── Global hotkeys ────────────────────────────────────────────────────────────

document.addEventListener('keydown', e => {
  // Esc — close QSO edit dialog if open
  if (e.key === 'Escape') {
    const modal = document.getElementById('qsoEditModal');
    if (modal && !modal.classList.contains('lm-hidden')) {
      closeQsoEdit();
      return;
    }
  }
  // Alt+U — toggle RUN / S&P
  if (e.altKey && !e.ctrlKey && !e.shiftKey && e.key === 'u') {
    e.preventDefault();
    setRunMode(app.runMode === 'RUN' ? 'SP' : 'RUN');
    inpCall.focus();
    return;
  }
  // Alt+W — clear all input fields
  if (e.altKey && !e.ctrlKey && !e.shiftKey && e.key === 'w') {
    e.preventDefault();
    clearForm();
    renderDxccStatus(null);
    inpCall.focus();
  }
});

// ── TRX selection ─────────────────────────────────────────────────────────────

function updateTrxHeader() {
  const hdr = document.getElementById('jhdrTrx');
  if (hdr) hdr.textContent = app.trxLabels[app.activeTrx - 1] || 'TRX';
}

function updateCatTabVisibility() {
  const trxIdx = app.activeTrx - 1;
  const isOi3  = app.trxOi3[trxIdx] && trxIdx > 0;
  const catTab = document.querySelector('.tabs a[href="/"]');
  if (catTab) catTab.style.display = isOi3 ? 'none' : '';
}

trxButtons.forEach(btn => {
  btn.addEventListener('click', () => {
    app.activeTrx = Number(btn.dataset.trx);
    trxButtons.forEach(b => b.classList.toggle('btn-trx-active', b === btn));
    updateTrxHeader();
    updateCatTabVisibility();
    renderConnStatus();
    clearTimeout(app._pollTimer);
    pollState();
    inpCall.focus();
  });
});

// ── Input: uppercase normalisation ────────────────────────────────────────────

function normaliseUpper(inp) {
  inp.addEventListener('input', () => {
    const pos = inp.selectionStart;
    inp.value = inp.value.toUpperCase();
    inp.setSelectionRange(pos, pos);
  });
}

normaliseUpper(inpCall);
normaliseUpper(inpExch);

inpRst.addEventListener('input', () => { app.rstDirty = true; });

// ── DXCC lookup on Call input ─────────────────────────────────────────────────

inpCall.addEventListener('input', () => {
  updateDxccFromCall();
  if (!inpCall.value.trim()) clearDupePanel();
});

function updateDxccFromCall() {
  const call = inpCall.value.trim();
  if (!call || !window.DXCC) {
    renderDxccStatus(null);
    return;
  }

  const dxcc = DXCC.lookupDxcc(call);
  if (!dxcc) {
    renderDxccStatus(null);
    return;
  }

  // Attach QRB/azimuth if we have our locator from settings
  const myLoc = (window._logSettings && window._logSettings.myLocator) || '';
  if (myLoc) {
    const myPos = DXCC.locatorToLatLon(myLoc);
    if (myPos) {
      const { qrbKm, azimuthDeg } = DXCC.calculateQrbAzimuth(
        myPos.lat, myPos.lon, dxcc.latitude, dxcc.longitude
      );
      dxcc.qrbKm     = qrbKm;
      dxcc.azimuthDeg = azimuthDeg;
    }
  }

  renderDxccStatus(dxcc);
}

// ── Dupe panel reference (needed by clearForm below) ─────────────────────────

const dupePanel = document.getElementById('dupePanel');

function clearDupePanel() {
  dupePanel.textContent = '';
  dupePanel.classList.add('dupe-panel-hidden');
  document.querySelectorAll('.qso-row.qso-dupe').forEach(r => r.classList.remove('qso-dupe'));
}

// ── Form state machine helpers ────────────────────────────────────────────────

function formStateOf() {
  const call = inpCall.value.trim();
  const exch = inpExch.value.trim();
  if (!call && !exch) return 'IDLE';
  if (call  && !exch) return 'CALL_ENTERED';
  return 'EXCHANGE_ENTERED';
}

function clearForm() {
  inpCall.value    = '';
  inpExch.value    = '';
  inpRst.value     = rstDefault(app.mode);
  app.rstDirty     = false;
  app.prevCallSent = '';
  app.formState    = 'IDLE';
  renderDxccStatus(null);
  clearDupePanel();
  setPrevExchVisible(false);
}

// ── Macro context builder ─────────────────────────────────────────────────────

function macroCtx(overrides) {
  const log    = LogManager.getActiveLog() || {};
  const trxIdx = app.activeTrx - 1;
  return Object.assign({
    mode:         app.mode,
    freqHz:       app.frequency,
    stationCall:  log.stationCall     || '',
    call:         inpCall.value.trim(),
    exchangeType: (log.defaultExchange === 'NR' || log.defaultExchange === 'NRUTC')
                    ? log.defaultExchange : 'STATIC',
    exchange:     log.defaultExchange || '',
    qsoNumber:    log.nextQsoNumber   || 1,
    prevQsoNumber:(log.nextQsoNumber  || 1) - 1,
    myLocator:    log.myLocator       || '',
    _oi3:         app.trxOi3[trxIdx] && trxIdx > 0,
    _trxIp:       app.trxIps[trxIdx] || '',
  }, overrides);
}

// ── Send CW/RTTY macro via WebSocket ─────────────────────────────────────────

function sendMacroText(macroType) {
  if (!window.LogMacros) return;
  const mg = LogMacros.modeGroup(app.mode);
  if (mg === 'PHONE') {
    showHint('Phone mode — send manually');
    return;
  }
  const trxIdx = app.activeTrx - 1;
  const isOi3  = app.trxOi3[trxIdx] && trxIdx > 0 && app.trxIps[trxIdx];
  if (!app.connected && !isOi3) {
    showHint('TRX not connected');
    return;
  }
  LogMacros.sendMacro(macroType, macroCtx()).then(ok => {
    if (!ok) showHint('Send failed');
  });
}

// ── Enter key workflow ────────────────────────────────────────────────────────

inpCall.addEventListener('keydown', handleCallEnter);
inpExch.addEventListener('keydown', handleExchEnter);

function handleCallEnter(e) {
  if (e.key === ' ') {
    e.preventDefault();
    checkDupe(inpCall.value.trim());
    return;
  }
  if (e.key !== 'Enter') return;
  e.preventDefault();

  const call  = inpCall.value.trim();
  const state = formStateOf();

  if (state === 'IDLE') {
    if (app.runMode === 'RUN') {
      sendMacroText('CQ');
    } else {
      showHint('Enter callsign');
    }
    return;
  }

  if (state === 'CALL_ENTERED') {
    app.prevCallSent = call;
    checkDupe(call);
    if (app.runMode === 'RUN') {
      sendMacroText('TXEXCH');
    } else {
      sendMacroText('TXEXCHSP');
    }
    inpExch.focus();
    return;
  }

  inpExch.focus();
}

// ── Dupe check ────────────────────────────────────────────────────────────────

function checkDupe(call) {
  const log = LogManager.getActiveLog();
  if (!log || !call) { clearDupePanel(); return; }
  const normCall = call.toUpperCase();
  const chkGlobal = document.getElementById('chkGlobalSearch');
  const isGlobal  = !!(chkGlobal && chkGlobal.checked);

  const dupeProm    = isGlobal ? LogDB.findDupesGlobal(normCall)      : LogDB.findDupes(log.id, normCall);
  const partialProm = call.length >= 2
    ? (isGlobal ? LogDB.findPartialGlobal(normCall) : LogDB.findPartial(log.id, normCall))
    : Promise.resolve([]);
  const logsProm    = isGlobal ? LogDB.getLogs() : Promise.resolve(null);

  Promise.all([dupeProm, partialProm, logsProm]).then(([dupes, partials, allLogs]) => {
    const activeDupes    = dupes.filter(d => !d.deleted);
    const activePartials = partials.filter(d => !d.deleted && d.call.toUpperCase() !== normCall);
    if (!activeDupes.length && !activePartials.length) { clearDupePanel(); return; }

    function esc(s) { return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }
    function hlCall(c, frag) {
      const idx = c.indexOf(frag);
      if (idx < 0) return esc(c);
      return esc(c.slice(0, idx))
        + '<span class="dp-partial-hl">' + esc(c.slice(idx, idx + frag.length)) + '</span>'
        + esc(c.slice(idx + frag.length));
    }

    const logMap = Object.fromEntries((allLogs || []).map(l => [l.id, l]));
    const myCall  = log.stationCall || '';
    const hasDupe = activeDupes.some(d => d.logId === log.id);
    let htmlPartial = '';
    let htmlDupe    = '';

    if (activePartials.length) {
      if (!isGlobal) {
        const uniq = [...new Set(activePartials.map(d => d.call))].sort();
        htmlPartial += '<div class="dp-line dp-partial">PARTIAL: '
          + uniq.map(c => hlCall(c, normCall)).join('  ') + '</div>';
      } else {
        const pAct = [], pSame = [], pOther = [];
        activePartials.forEach(d => {
          if (d.logId === log.id) { pAct.push(d); return; }
          const sc = (logMap[d.logId] || {}).stationCall || '';
          (sc === myCall ? pSame : pOther).push(d);
        });
        const uAct   = [...new Set(pAct.map(d => d.call))].sort();
        const uSame  = [...new Set(pSame.map(d => d.call))].sort();
        const uOther = [...new Set(pOther.map(d => d.call))].sort();
        const cells = [];
        if (uOther.length) cells.push(uOther.map(c => esc(c)).join('  '));
        if (uSame.length)  cells.push('<span class="dp-partial-green">' + uSame.map(c => esc(c)).join('  ') + '</span>');
        if (uAct.length)   cells.push(uAct.map(c => hlCall(c, normCall)).join('  '));
        if (cells.length) htmlPartial += '<div class="dp-line dp-partial">PARTIAL: ' + cells.join('  ') + '</div>';
      }
    }

    if (activeDupes.length) {
      const fmtD = d => 'DUPE: #' + String(d.qsoNumber).padStart(3,'0') + ' '
        + esc(d.call) + ' ' + esc(d.timeOnUtc||'') + ' '
        + esc(d.frequencyDisplay||'') + ' ' + esc(d.mode||'');

      if (!isGlobal) {
        activeDupes.forEach(d => { htmlDupe += '<div class="dp-line dp-dupe">' + fmtD(d) + '</div>'; });
      } else {
        const cat1 = [], cat2 = [], cat3 = [];
        activeDupes.forEach(d => {
          if (d.logId === log.id) { cat3.push(d); return; }
          const sc = (logMap[d.logId] || {}).stationCall || '';
          (sc === myCall ? cat2 : cat1).push(d);
        });
        cat1.forEach(d => { htmlDupe += '<div class="dp-line dp-dupe-other">'   + fmtD(d) + '</div>'; });
        cat2.forEach(d => { htmlDupe += '<div class="dp-line dp-dupe-samecall">' + fmtD(d) + '</div>'; });
        cat3.forEach(d => { htmlDupe += '<div class="dp-line dp-dupe">'          + fmtD(d) + '</div>'; });
      }
    }

    const html = htmlPartial + htmlDupe;
    if (!html) { clearDupePanel(); return; }

    dupePanel.innerHTML = html;
    dupePanel.className = 'dupe-panel' + (hasDupe ? '' : ' dupe-panel-similar');
    dupePanel.classList.remove('dupe-panel-hidden');
    document.querySelectorAll('.qso-row').forEach(row => {
      row.classList.toggle('qso-dupe', row.dataset.call === normCall);
    });
  }).catch(() => {});
}

function handleExchEnter(e) {
  if (e.key !== 'Enter') return;
  e.preventDefault();

  const call = inpCall.value.trim();
  const exch = inpExch.value.trim();

  if (!call) {
    inpCall.focus();
    return;
  }

  if (!exch) {
    if (app.runMode === 'RUN') {
      sendMacroText('TXEXCH');
    } else {
      sendMacroText('TXEXCHSP');
    }
    showHint('Enter exchange');
    return;
  }

  // Both filled → log QSO
  logQso(call, exch);
}

// ── QSO logging ───────────────────────────────────────────────────────────────

function logQso(call, exch) {
  const log = LogManager.getActiveLog();
  if (!log) {
    showHint('No log open — press LOG to create one');
    LogManager.openModal();
    return;
  }

  const now        = new Date();
  const dateUtc    = now.toISOString().slice(0, 10);
  const timeUtc    = String(now.getUTCHours()).padStart(2,'0') + ':' +
                     String(now.getUTCMinutes()).padStart(2,'0');
  const rstSent    = rstDefault(app.mode);
  const rstRcvd    = inpRst.value.trim() || rstSent;

  // DXCC lookup
  const dxcc = window.DXCC ? DXCC.lookupDxcc(call) : null;
  if (dxcc) {
    const myLoc = log.myLocator || '';
    if (myLoc && window.DXCC) {
      const myPos = DXCC.locatorToLatLon(myLoc);
      if (myPos) {
        const { qrbKm, azimuthDeg } = DXCC.calculateQrbAzimuth(
          myPos.lat, myPos.lon, dxcc.latitude, dxcc.longitude
        );
        dxcc.qrbKm      = qrbKm;
        dxcc.azimuthDeg = azimuthDeg;
      }
    }
  }

  // VHF/UHF: extract Maidenhead locator from Exch text
  let locatorReceived = '';
  if (app.frequency > 140_000_000) {
    const m = exch.match(/\b([A-R]{2}\d{2}[A-X]{2})\b/i);
    if (m) locatorReceived = m[1].toUpperCase();
    // if we got a locator from exchange, recalculate QRB more precisely
    if (locatorReceived && dxcc && window.DXCC) {
      const myLoc = log.myLocator || '';
      const dxPos = DXCC.locatorToLatLon(locatorReceived);
      const myPos = myLoc ? DXCC.locatorToLatLon(myLoc) : null;
      if (dxPos && myPos) {
        const { qrbKm, azimuthDeg } = DXCC.calculateQrbAzimuth(
          myPos.lat, myPos.lon, dxPos.lat, dxPos.lon
        );
        dxcc.qrbKm      = qrbKm;
        dxcc.azimuthDeg = azimuthDeg;
      }
    }
  }

  const qso = {
    logId:            log.id,
    qsoNumber:        log.nextQsoNumber,
    qsoDateUtc:       dateUtc,
    timeOnUtc:        timeUtc,
    timestampUtc:     now.toISOString(),
    call:             call,
    rstSent:          rstSent,
    rstReceived:      rstRcvd,
    exchangeReceived: exch,
    frequencyHz:      app.frequency,
    frequencyDisplay: formatFreq(app.frequency),
    mode:             app.mode,
    trx:              app.trxLabels[app.activeTrx - 1],
    dxcc:             dxcc || null,
    locatorReceived:  locatorReceived,
    bandClass:        app.frequency > 140_000_000 ? 'VHF_PLUS' : 'HF',
    note:             '',
  };

  LogDB.addQso(qso)
    .then(saved => {
      LogManager.bumpQsoNumber();
      appendJournalRow(saved);
      _onQsoBackupHook();
      // Phase 5: send TU macro + reset RIT
      sendTuAndResetRit();
      clearForm();
      // Show "prev exch" button in S&P mode after logging
      if (app.runMode === 'SP') setPrevExchVisible(true);
      inpCall.focus();
    })
    .catch(err => {
      // Do NOT clear form or advance QSO number on error
      showHint('DB error: ' + (err.message || err));
    });
}

function sendTuAndResetRit() {
  // Send TU macro
  if (app.runMode === 'RUN') {
    sendMacroText('TU');
  }
  // Reset RIT via HTTP POST /cmd
  if (app.connected) {
    fetch('/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ type: 'setRitClear' }),
    }).catch(() => {});
  }
}

// ── Journal: append a QSO row (Phase 4 will call this) ───────────────────────

function appendJournalRow(qso, displayNr) {
  const hint = document.getElementById('logEmptyHint');
  if (hint) hint.remove();

  const body = document.getElementById('logJournalBody');
  const row  = document.createElement('div');
  row.className = qso.deleted ? 'qso-row qso-deleted' : 'qso-row';
  row.dataset.logId = qso.logId || '';
  row.dataset.call  = qso.call  || '';
  row.dataset.qsoId = qso.id    != null ? String(qso.id) : '';

  const dxccText = (() => {
    if (qso.deleted) return '';
    const d = qso.dxcc;
    if (!d) return '';
    const parts = [];
    if (d.country) parts.push(d.country);
    if (d.qrbKm != null) parts.push(d.qrbKm + 'km');
    return parts.join(' | ');
  })();

  const cols = [
    { cls: 'jcol-nr',   text: String(displayNr != null ? displayNr : qso.qsoNumber).padStart(3, '0') },
    { cls: 'jcol-date', text: qso.deleted ? '' : (qso.qsoDateUtc  || '') },
    { cls: 'jcol-time', text: qso.deleted ? '' : (qso.timeOnUtc   || '--:--') },
    { cls: 'jcol-call', text: qso.deleted ? '' : (qso.call        || '') },
    { cls: 'jcol-freq', text: qso.deleted ? '' : (qso.frequencyDisplay || '') },
    { cls: 'jcol-mode', text: qso.deleted ? '' : (qso.mode        || '') },
    { cls: 'jcol-snt',  text: qso.deleted ? '' : (qso.rstSent     || '') },
    { cls: 'jcol-rcv',  text: qso.deleted ? '' : (qso.rstReceived || '') },
    { cls: 'jcol-exch', text: qso.deleted ? '' : (qso.exchangeReceived || '') },
    { cls: 'jcol-trx',  text: qso.deleted ? '' : (qso.trx         || '') },
    { cls: 'jcol-dxcc', text: dxccText },
  ];

  cols.forEach(({ cls, text }) => {
    const span = document.createElement('span');
    span.className = 'jcol ' + cls;
    span.textContent = text;
    row.appendChild(span);
  });

  if (!qso.deleted) row.title = 'Click to edit';

  body.appendChild(row);
  row.scrollIntoView({ block: 'end' });
}

// ── QSO row click → edit dialog ───────────────────────────────────────────────

document.getElementById('logJournalBody').addEventListener('click', e => {
  const row = e.target.closest('.qso-row[data-qso-id]');
  if (!row || !row.dataset.qsoId) return;
  openQsoEdit(Number(row.dataset.qsoId));
});

document.getElementById('logJournalBody').addEventListener('contextmenu', e => {
  const callSpan = e.target.closest('.jcol-call');
  if (!callSpan) return;
  const row = callSpan.closest('.qso-row');
  if (!row || !row.dataset.call) return;
  e.preventDefault();
  window.open('https://www.google.com/search?q=' + encodeURIComponent(row.dataset.call), '_blank');
});

// ── Call search ───────────────────────────────────────────────────────────────

document.getElementById('jhdrCall').addEventListener('click', () => {
  const row = document.getElementById('logCallSearchRow');
  const inp = document.getElementById('logCallSearchInp');
  row.classList.remove('ds-hidden');
  inp.value = '';
  inp.focus();
  applyCallSearch('');
});

document.getElementById('logCallSearchClose').addEventListener('click', closeCallSearch);

document.getElementById('logCallSearchInp').addEventListener('input', e => {
  applyCallSearch(e.target.value.trim());
});

document.getElementById('logCallSearchInp').addEventListener('keydown', e => {
  if (e.key === 'Escape') closeCallSearch();
});

function closeCallSearch() {
  document.getElementById('logCallSearchRow').classList.add('ds-hidden');
  applyCallSearch('');
}

function applyCallSearch(query) {
  const q = query.toUpperCase();
  const rows = document.querySelectorAll('#logJournalBody .qso-row');
  let first = null;
  rows.forEach(row => {
    const callSpan = row.querySelector('.jcol-call');
    const rawCall  = row.dataset.call || '';
    if (!q) {
      row.classList.remove('qso-search-dim', 'qso-search-hl');
      if (callSpan) callSpan.textContent = rawCall;
    } else {
      const idx = rawCall.toUpperCase().indexOf(q);
      const match = idx !== -1;
      row.classList.toggle('qso-search-dim', !match);
      row.classList.toggle('qso-search-hl',   match);
      if (callSpan) {
        if (match) {
          const pre  = rawCall.slice(0, idx);
          const mid  = rawCall.slice(idx, idx + q.length);
          const post = rawCall.slice(idx + q.length);
          callSpan.innerHTML = esc(pre) + '<mark class="call-search-mark">' + esc(mid) + '</mark>' + esc(post);
        } else {
          callSpan.textContent = rawCall;
        }
      }
      if (match && !first) first = row;
    }
  });
  if (first) first.scrollIntoView({ block: 'center' });
}

let _qsoEditQso = null;

function openQsoEdit(qsoId) {
  LogDB.getQso(qsoId).then(qso => {
    if (!qso) { console.warn('openQsoEdit: QSO not found, id=', qsoId); return; }
    _qsoEditQso = qso;
    let modal = document.getElementById('qsoEditModal');
    if (!modal) modal = buildQsoEditModal();
    fillQsoEditForm(qso);
    modal.classList.remove('lm-hidden');
    document.getElementById('qeCall').focus();
  }).catch(err => { console.error('openQsoEdit error:', err); });
}

function buildQsoEditModal() {
  const el = document.createElement('div');
  el.id = 'qsoEditModal';
  el.className = 'lm-modal lm-hidden';
  el.innerHTML = `
    <div class="lm-box" style="width:min(440px,calc(100vw - 24px))">
      <div class="lm-header">
        <span class="lm-title" id="qeTitle">Edit QSO</span>
        <button class="lm-close" id="qeClose">×</button>
      </div>
      <div class="lm-section">
        <div class="lm-form">
          <label class="lm-row"><span>Call</span><input id="qeCall" maxlength="16" autocapitalize="characters" spellcheck="false" autocomplete="off"></label>
          <label class="lm-row"><span>Date UTC</span><input id="qeDate" type="date"></label>
          <label class="lm-row"><span>Time UTC</span><input id="qeTime" type="time"></label>
          <label class="lm-row"><span>Freq (kHz)</span><input id="qeFreq" inputmode="decimal" spellcheck="false" autocomplete="off"></label>
          <label class="lm-row"><span>Mode</span>
            <select id="qeMode" class="lm-row-select">
              <option value="USB">USB</option>
              <option value="LSB">LSB</option>
              <option value="CW">CW</option>
              <option value="CW-R">CW-R</option>
              <option value="RTTY">RTTY</option>
              <option value="FM">FM</option>
              <option value="AM">AM</option>
            </select>
          </label>
          <label class="lm-row"><span>RST Sent</span><input id="qeSnt" maxlength="3" autocomplete="off"></label>
          <label class="lm-row"><span>RST Rcvd</span><input id="qeRcv" maxlength="3" autocomplete="off"></label>
          <label class="lm-row"><span>Exchange</span><input id="qeExch" maxlength="20" autocomplete="off"></label>
        </div>
      </div>
      <div class="lm-section qe-actions">
        <button id="qeDelete" class="lm-btn lm-btn-del">Delete</button>
        <div class="qe-actions-right">
          <button id="qeCancel" class="lm-btn">Cancel</button>
          <button id="qeSave" class="lm-btn lm-btn-primary">Save</button>
        </div>
      </div>
    </div>
  `;
  document.body.appendChild(el);

  document.getElementById('qeClose').addEventListener('click', closeQsoEdit);
  document.getElementById('qeCancel').addEventListener('click', closeQsoEdit);
  el.addEventListener('click', e => { if (e.target === el) closeQsoEdit(); });
  document.getElementById('qeSave').addEventListener('click', saveQsoEdit);
  document.getElementById('qeDelete').addEventListener('click', deleteQsoEdit);

  const callInp = document.getElementById('qeCall');
  callInp.addEventListener('input', () => {
    const p = callInp.selectionStart;
    callInp.value = callInp.value.toUpperCase();
    callInp.setSelectionRange(p, p);
  });

  return el;
}

function fillQsoEditForm(qso) {
  document.getElementById('qeTitle').textContent = 'Edit QSO #' + String(qso.qsoNumber).padStart(3, '0');
  document.getElementById('qeCall').value  = qso.call          || '';
  document.getElementById('qeDate').value  = qso.qsoDateUtc    || '';
  document.getElementById('qeTime').value  = qso.timeOnUtc     || '';
  document.getElementById('qeFreq').value  = qso.frequencyHz   ? (qso.frequencyHz / 1000).toString() : '';
  const modeEl = document.getElementById('qeMode');
  modeEl.value = Array.from(modeEl.options).find(o => o.value === qso.mode) ? qso.mode : 'USB';
  document.getElementById('qeSnt').value   = qso.rstSent          || '';
  document.getElementById('qeRcv').value   = qso.rstReceived      || '';
  document.getElementById('qeExch').value  = qso.exchangeReceived || '';
}

function closeQsoEdit() {
  const modal = document.getElementById('qsoEditModal');
  if (modal) modal.classList.add('lm-hidden');
  _qsoEditQso = null;
}

function saveQsoEdit() {
  if (!_qsoEditQso) return;
  const call = document.getElementById('qeCall').value.trim().toUpperCase();
  if (!call) return;
  const hz = Math.round(parseFloat(document.getElementById('qeFreq').value.replace(',', '.')) * 1000) || _qsoEditQso.frequencyHz || 0;

  _qsoEditQso.call             = call;
  _qsoEditQso.qsoDateUtc       = document.getElementById('qeDate').value;
  _qsoEditQso.timeOnUtc        = document.getElementById('qeTime').value;
  _qsoEditQso.frequencyHz      = hz;
  _qsoEditQso.frequencyDisplay = formatFreq(hz);
  _qsoEditQso.mode             = document.getElementById('qeMode').value;
  _qsoEditQso.rstSent          = document.getElementById('qeSnt').value.trim();
  _qsoEditQso.rstReceived      = document.getElementById('qeRcv').value.trim();
  _qsoEditQso.exchangeReceived = document.getElementById('qeExch').value.trim();
  _qsoEditQso.deleted          = false;

  LogDB.updateQso(_qsoEditQso).then(() => {
    updateJournalRow(_qsoEditQso);
    closeQsoEdit();
  });
}

function deleteQsoEdit() {
  if (!_qsoEditQso) return;
  _qsoEditQso.deleted = true;
  LogDB.updateQso(_qsoEditQso).then(() => {
    updateJournalRow(_qsoEditQso);
    closeQsoEdit();
  });
}

function updateJournalRow(qso) {
  const row = document.querySelector('.qso-row[data-qso-id="' + qso.id + '"]');
  if (!row) return;
  if (qso.deleted) {
    row.className = 'qso-row qso-deleted';
    Array.from(row.children).forEach(span => {
      if (!span.classList.contains('jcol-nr')) span.textContent = '';
    });
  } else {
    row.className = 'qso-row';
    const dxccText = (() => {
      const d = qso.dxcc;
      if (!d) return '';
      const parts = [];
      if (d.country) parts.push(d.country);
      if (d.qrbKm != null) parts.push(d.qrbKm + 'km');
      return parts.join(' | ');
    })();
    const vals = [
      String(qso.qsoNumber).padStart(3, '0'),
      qso.qsoDateUtc || '',
      qso.timeOnUtc  || '--:--',
      qso.call       || '',
      qso.frequencyDisplay || '',
      qso.mode       || '',
      qso.rstSent    || '',
      qso.rstReceived || '',
      qso.exchangeReceived || '',
      qso.trx        || '',
      dxccText,
    ];
    Array.from(row.children).forEach((span, i) => { span.textContent = vals[i] || ''; });
  }
}

// ── Action buttons (*?, nr?, prev exch, check) ────────────────────────────────

const btnCallQ  = document.getElementById('btnCallQ');
const btnNrQ    = document.getElementById('btnNrQ');
const btnPrevEx = document.getElementById('btnPrevEx');
const btnCheck  = document.getElementById('btnCheck');

btnCallQ.addEventListener('click', () => {
  if (window.LogMacros) {
    LogMacros.sendMacro('TXEXCHSP', macroCtx({ call: (inpCall.value.trim() || '') + '?' }));
  }
  checkDupe(inpCall.value.trim());
  inpCall.focus();
});

btnNrQ.addEventListener('click', () => {
  if (window.LogMacros) {
    const type = app.runMode === 'RUN' ? 'NR?' : 'NR?';
    // send a literal "nr?" text
    LogMacros.sendMacro('TXEXCHSP', macroCtx({ exchange: 'nr?' }));
  }
  inpExch.focus();
});

btnPrevEx.addEventListener('click', () => {
  if (window.LogMacros) {
    LogMacros.sendMacro('TXEXCHSP2', macroCtx());
  }
  inpCall.focus();
});

btnCheck.addEventListener('click', () => {
  checkDupe(inpCall.value.trim());
  inpCall.focus();
});

function setPrevExchVisible(visible) {
  btnPrevEx.classList.toggle('btn-action-hidden', !visible);
}

// ── Active log display ────────────────────────────────────────────────────────

const btnOpenLog = document.getElementById('btnOpenLog');
btnOpenLog.addEventListener('click', () => LogManager.openModal());

function onActiveLogChanged(log) {
  if (!log) {
    btnOpenLog.textContent = 'LOG';
    btnOpenLog.classList.remove('btn-trx-active');
    return;
  }
  btnOpenLog.textContent = 'LOG: ' + (log.contestName || '—');
  btnOpenLog.classList.add('btn-trx-active');
  // reload journal rows for the active log
  loadJournalFromDb(log.id);
}

LogManager.onLogChanged(onActiveLogChanged);

// ── Load journal from IndexedDB ───────────────────────────────────────────────

function loadJournalFromDb(logId) {
  const body = document.getElementById('logJournalBody');
  body.innerHTML = '';
  const spacer = document.createElement('div');
  spacer.className = 'log-journal-spacer';
  body.appendChild(spacer);

  LogDB.getQsosForLog(logId).then(qsos => {
    qsos.sort((a, b) => {
      const ta = (a.timestampUtc || a.qsoDateUtc + 'T' + (a.timeOnUtc||'') + 'Z') || '';
      const tb = (b.timestampUtc || b.qsoDateUtc + 'T' + (b.timeOnUtc||'') + 'Z') || '';
      return ta < tb ? -1 : ta > tb ? 1 : 0;
    });
    if (!qsos.length) {
      body.innerHTML = '<div class="log-empty" id="logEmptyHint">No QSO logged yet.</div>';
      return;
    }
    qsos.forEach((q, i) => appendJournalRow(q, i + 1));
  }).catch(() => {});
}

// ── Initialisation ────────────────────────────────────────────────────────────

function init() {
  setRunMode('RUN');
  inpRst.value = rstDefault(app.mode);
  startClock();
  pollState();

  // Load global LOG settings from ESP32 /log-config (shared across browsers)
  fetch('/log-config').then(r => r.json()).then(cfg => {
    if (cfg.trx1Label) app.trxLabels[0] = cfg.trx1Label;
    if (cfg.trx2Label) app.trxLabels[1] = cfg.trx2Label;
    if (cfg.trx3Label) app.trxLabels[2] = cfg.trx3Label;
    if (cfg.trx2Ip)   app.trxIps[1]   = cfg.trx2Ip;
    if (cfg.trx3Ip)   app.trxIps[2]   = cfg.trx3Ip;
    app.trxOi3[1] = !!cfg.trx2Oi3;
    app.trxOi3[2] = !!cfg.trx3Oi3;
    trxButtons.forEach((b, i) => { b.textContent = app.trxLabels[i]; });
    // Hide TRX2/TRX3 buttons when no IP is configured
    trxButtons[1].style.display = app.trxIps[1] ? '' : 'none';
    trxButtons[2].style.display = app.trxIps[2] ? '' : 'none';
    // If active TRX is now hidden, fall back to TRX1
    if ((app.activeTrx === 2 && !app.trxIps[1]) || (app.activeTrx === 3 && !app.trxIps[2])) {
      app.activeTrx = 1;
      trxButtons.forEach((b, i) => b.classList.toggle('btn-trx-active', i === 0));
    }
    updateTrxHeader();
    updateCatTabVisibility();
    renderConnStatus();
  }).catch(() => {});

  // Manual freq/mode inputs (active when TRX disconnected)
  sbManualFreq.addEventListener('input', () => {
    const hz = Math.round(parseFloat(sbManualFreq.value.replace(',', '.')) * 1000);
    if (hz > 0) {
      app.frequency = hz;
      if (inpCall.value.trim()) updateDxccFromCall();
    }
  });
  sbManualFreq.addEventListener('blur', () => {
    const hz = Math.round(parseFloat(sbManualFreq.value.replace(',', '.')) * 1000);
    if (hz > 0) sbManualFreq.value = (hz / 1000).toString();
    else sbManualFreq.value = '';
  });
  sbManualMode.addEventListener('change', () => {
    app.mode = sbManualMode.value;
    if (!app.rstDirty) inpRst.value = rstDefault(app.mode);
  });

  // Start WebSocket for macros
  if (window.LogMacros) LogMacros.init();

  _initBackup();

  // Restore last active log, then focus Call
  LogManager.restoreActiveLog().then(() => {
    inpCall.focus();
  }).catch(() => {
    inpCall.focus();
  });
}

init();
