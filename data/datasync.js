'use strict';

(function () {

  // ── Config ───────────────────────────────────────────────────────────────────
  const SYNC_DB_NAME    = 'datasyncDb';
  const SYNC_DB_VER     = 1;
  const CONTEST_DB_NAME = 'contestLogDb';
  const CONTEST_DB_VER  = 1;
  const PROTOCOL_VER    = 1;
  const APP_VER         = '1.0.0';
  const MAX_ITEMS       = 200;
  const MAX_BYTES       = 32768;
  const BUFFER_HIGH     = 1024 * 1024;

  // ── State ────────────────────────────────────────────────────────────────────
  let syncDb    = null;
  let contestDb = null;
  let pc        = null;
  let dc        = null;
  let scanStream = null;
  let scanRaf    = null;
  let sessionId  = null;
  let phase      = 'idle';
  let helloReceived = false;
  let doneSent      = false;
  let pendingReqIds = new Set();
  let stats = { sent: 0, received: 0, batches: 0, errors: 0 };

  // ── Device ID ────────────────────────────────────────────────────────────────

  function uuidv4() {
    if (crypto.getRandomValues) {
      const b = new Uint8Array(16);
      crypto.getRandomValues(b);
      b[6] = (b[6] & 0x0f) | 0x40;
      b[8] = (b[8] & 0x3f) | 0x80;
      const h = Array.from(b, x => x.toString(16).padStart(2, '0'));
      return h[0]+h[1]+h[2]+h[3]+'-'+h[4]+h[5]+'-'+h[6]+h[7]+'-'+h[8]+h[9]+'-'+h[10]+h[11]+h[12]+h[13]+h[14]+h[15];
    }
    return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, c => {
      const r = Math.random() * 16 | 0;
      return (c === 'x' ? r : (r & 0x3 | 0x8)).toString(16);
    });
  }

  function deviceId() {
    let id = localStorage.getItem('ds_device_id');
    if (!id) { id = uuidv4(); localStorage.setItem('ds_device_id', id); }
    return id;
  }

  function deviceLabel() {
    return localStorage.getItem('ds_device_label') || ('Device ' + deviceId().slice(0, 8));
  }

  // ── Filename helper ──────────────────────────────────────────────────────────

  function fileTs(d) {
    const p = n => String(n).padStart(2, '0');
    return String(d.getFullYear()) + p(d.getMonth() + 1) + p(d.getDate()) +
           '-' + p(d.getHours()) + p(d.getMinutes());
  }

  // ── Database: sync metadata (sync_state, devices) ────────────────────────────

  function openSyncDb() {
    return new Promise((resolve, reject) => {
      if (syncDb) { resolve(syncDb); return; }
      const req = indexedDB.open(SYNC_DB_NAME, SYNC_DB_VER);
      req.onupgradeneeded = e => {
        const db = e.target.result;
        if (!db.objectStoreNames.contains('sync_state')) {
          db.createObjectStore('sync_state', { keyPath: 'source_device_id' });
        }
        if (!db.objectStoreNames.contains('devices')) {
          db.createObjectStore('devices', { keyPath: 'device_id' });
        }
      };
      req.onsuccess = e => { syncDb = e.target.result; resolve(syncDb); };
      req.onerror   = e => reject(e.target.error);
    });
  }

  // ── Database: contest log (QSOs, logs, settings) ─────────────────────────────
  // Local QSOs have numeric autoIncrement id.
  // Remote QSOs have string id: "source_device_id:source_seq".
  // In IndexedDB key order numbers sort before strings, so ranges on numeric
  // keys safely exclude remote string keys.

  function openContestDb() {
    return new Promise((resolve, reject) => {
      if (contestDb) { resolve(contestDb); return; }
      const req = indexedDB.open(CONTEST_DB_NAME, CONTEST_DB_VER);
      req.onupgradeneeded = e => {
        // Mirror of log-db.js v1 schema — only fires when DB is brand new.
        const db = e.target.result;
        if (!db.objectStoreNames.contains('logs')) {
          db.createObjectStore('logs', { keyPath: 'id' })
            .createIndex('active', 'active', { unique: false });
        }
        if (!db.objectStoreNames.contains('qso')) {
          const qs = db.createObjectStore('qso', { keyPath: 'id', autoIncrement: true });
          qs.createIndex('logId',           'logId',                   { unique: false });
          qs.createIndex('logId_qsoNumber', ['logId','qsoNumber'],     { unique: false });
          qs.createIndex('logId_call',      ['logId','call'],          { unique: false });
          qs.createIndex('logId_timestamp', ['logId','timestampUtc'],  { unique: false });
          qs.createIndex('logId_freq',      ['logId','frequencyHz'],   { unique: false });
          qs.createIndex('logId_mode',      ['logId','mode'],          { unique: false });
        }
        if (!db.objectStoreNames.contains('settings')) {
          db.createObjectStore('settings', { keyPath: 'key' });
        }
        if (!db.objectStoreNames.contains('runtimeState')) {
          db.createObjectStore('runtimeState', { keyPath: 'key' });
        }
      };
      req.onsuccess = e => { contestDb = e.target.result; resolve(contestDb); };
      req.onerror   = e => reject(e.target.error);
    });
  }

  // ── Generic IDB helpers ──────────────────────────────────────────────────────

  function idbGetAll(db, store, index, query) {
    return new Promise((resolve, reject) => {
      const t   = db.transaction(store, 'readonly');
      const s   = t.objectStore(store);
      const src = index ? s.index(index) : s;
      const req = query !== undefined ? src.getAll(query) : src.getAll();
      req.onsuccess = e => resolve(e.target.result);
      req.onerror   = e => reject(e.target.error);
    });
  }

  function idbCount(db, store, range) {
    return new Promise((resolve, reject) => {
      const req = db.transaction(store, 'readonly').objectStore(store).count(range || null);
      req.onsuccess = e => resolve(e.target.result);
      req.onerror   = e => reject(e.target.error);
    });
  }

  function idbPut(db, store, record) {
    return new Promise((resolve, reject) => {
      const t = db.transaction(store, 'readwrite');
      t.objectStore(store).put(record);
      t.oncomplete = resolve;
      t.onerror    = e => reject(e.target.error);
    });
  }

  // ── Version vector ───────────────────────────────────────────────────────────

  async function getLocalVector() {
    const [cdb, sdb] = await Promise.all([openContestDb(), openSyncDb()]);

    // Max local (numeric) QSO id — numbers sort before strings in IDB,
    // so bound(1, MAX_SAFE_INTEGER) reliably skips remote string keys.
    let localMax = 0;
    await new Promise((resolve, reject) => {
      const range = IDBKeyRange.bound(1, Number.MAX_SAFE_INTEGER);
      const req   = cdb.transaction('qso', 'readonly')
                       .objectStore('qso')
                       .openCursor(range, 'prev');
      req.onsuccess = e => { if (e.target.result) localMax = e.target.result.key; resolve(); };
      req.onerror   = e => reject(e.target.error);
    });

    const vector = {};
    if (localMax > 0) vector[deviceId()] = localMax;

    const states = await idbGetAll(sdb, 'sync_state');
    states.forEach(s => { vector[s.source_device_id] = s.max_seq_seen; });
    return vector;
  }

  // ── QSO read: local (numeric ids) ────────────────────────────────────────────

  function getLocalQsosRange(fromSeq, toSeq) {
    return openContestDb().then(cdb => new Promise((resolve, reject) => {
      const range = IDBKeyRange.bound(fromSeq, toSeq);
      const req   = cdb.transaction('qso', 'readonly').objectStore('qso').getAll(range);
      req.onsuccess = e => {
        resolve(e.target.result.map(q => {
          const r = Object.assign({}, q);
          r.source_device_id = deviceId();
          r.source_seq       = q.id;         // preserve numeric local id
          r.id               = deviceId() + ':' + q.id; // string key for transport
          return r;
        }));
      };
      req.onerror = e => reject(e.target.error);
    }));
  }

  // ── QSO read: remote (string ids, for relay) ──────────────────────────────────

  async function getRemoteQsosRange(devId, fromSeq, toSeq) {
    const cdb  = await openContestDb();
    const all  = await idbGetAll(cdb, 'qso');
    return all.filter(q =>
      typeof q.id === 'string' &&
      q.source_device_id === devId &&
      q.source_seq >= fromSeq &&
      q.source_seq <= toSeq
    );
  }

  // ── QSO write: remote QSOs into contestLogDb.qso ─────────────────────────────
  // id is set to "source_device_id:source_seq" (string) so it never collides
  // with local autoIncrement numeric ids. put() is idempotent.

  async function insertRemoteQsos(qsos, remoteLogs) {
    const cdb = await openContestDb();
    return new Promise((resolve, reject) => {
      const stores = remoteLogs && remoteLogs.length ? ['qso', 'logs'] : ['qso'];
      const t = cdb.transaction(stores, 'readwrite');
      if (remoteLogs && remoteLogs.length) {
        const ls = t.objectStore('logs');
        remoteLogs.forEach(l => ls.put(l));
      }
      const qs = t.objectStore('qso');
      let stored = 0;
      qsos.forEach(q => {
        q.id = q.source_device_id + ':' + q.source_seq;
        qs.put(q);
        stored++;
      });
      t.oncomplete = () => resolve(stored);
      t.onerror    = e  => reject(e.target.error);
    });
  }

  async function getLogsForDevice() {
    const cdb  = await openContestDb();
    return idbGetAll(cdb, 'logs');
  }

  async function updateSyncState(devId, maxSeq) {
    const sdb = await openSyncDb();
    await idbPut(sdb, 'sync_state', {
      source_device_id: devId,
      max_seq_seen:     maxSeq,
      updated_at:       new Date().toISOString(),
    });
  }

  async function upsertDevice(devId, label) {
    const sdb = await openSyncDb();
    const now = new Date().toISOString();
    return new Promise((resolve, reject) => {
      const t   = sdb.transaction('devices', 'readwrite');
      const s   = t.objectStore('devices');
      const req = s.get(devId);
      req.onsuccess = e => {
        const rec = e.target.result || { device_id: devId, first_seen_at: now };
        rec.last_seen_at = now;
        if (label) rec.label = label;
        s.put(rec);
        t.oncomplete = resolve;
        t.onerror    = ev => reject(ev.target.error);
      };
      req.onerror = e => reject(e.target.error);
    });
  }

  // ── Compression ──────────────────────────────────────────────────────────────

  function compress(text) {
    return Promise.resolve(
      btoa(unescape(encodeURIComponent(text)))
        .replace(/\+/g,'-').replace(/\//g,'_').replace(/=/g,'')
    );
  }

  function decompress(b64url) {
    return Promise.resolve(
      decodeURIComponent(escape(atob(b64url.replace(/-/g,'+').replace(/_/g,'/'))))
    );
  }

  // ── QR display ───────────────────────────────────────────────────────────────

  function showQR(containerId, text) {
    const el = document.getElementById(containerId);
    el.innerHTML = '';
    if (typeof QRCode !== 'undefined') {
      try { new QRCode(el, { text, width: 256, height: 256, correctLevel: QRCode.CorrectLevel.L }); return; }
      catch (_) {}
    }
    el.innerHTML = '<p class="ds-qr-note">QR library not loaded. Use copy/paste below.</p>';
  }

  // ── QR scanning (BarcodeDetector API) ────────────────────────────────────────

  async function startScan(videoId, wrapId, onResult) {
    const video = document.getElementById(videoId);
    const wrap  = document.getElementById(wrapId);

    if (!('BarcodeDetector' in window)) {
      log('BarcodeDetector not supported in this browser — use paste fallback.');
      return;
    }

    try {
      scanStream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment' } });
      video.srcObject = scanStream;
      await video.play();
      wrap.classList.remove('ds-hidden');
      const detector = new BarcodeDetector({ formats: ['qr_code'] });
      async function tick() {
        if (video.readyState === video.HAVE_ENOUGH_DATA) {
          try {
            const codes = await detector.detect(video);
            if (codes.length > 0) { stopScan(); wrap.classList.add('ds-hidden'); onResult(codes[0].rawValue); return; }
          } catch (_) {}
        }
        scanRaf = requestAnimationFrame(tick);
      }
      scanRaf = requestAnimationFrame(tick);
    } catch (err) {
      log('Camera unavailable: ' + err.message + '. Use paste fallback.');
    }
  }

  function stopScan() {
    if (scanRaf)    { cancelAnimationFrame(scanRaf); scanRaf = null; }
    if (scanStream) { scanStream.getTracks().forEach(t => t.stop()); scanStream = null; }
  }

  // ── WebRTC ───────────────────────────────────────────────────────────────────

  function makePC() {
    if (pc) { try { pc.close(); } catch (_) {} }
    pc = new RTCPeerConnection({ iceServers: [] });
    pc.oniceconnectionstatechange = () => {
      log('ICE: ' + pc.iceConnectionState);
      if (pc.iceConnectionState === 'failed')       setPhase('failed');
      if (pc.iceConnectionState === 'disconnected') { setPhase('idle'); log('Disconnected.'); }
    };
    return pc;
  }

  function waitIce(conn) {
    if (conn.iceGatheringState === 'complete') return Promise.resolve();
    return new Promise(resolve => {
      const t = setTimeout(resolve, 6000);
      conn.onicegatheringstatechange = () => {
        if (conn.iceGatheringState === 'complete') { clearTimeout(t); resolve(); }
      };
    });
  }

  async function createOffer() {
    try {
      setPhase('creating-offer');
      sessionId = uuidv4();
      makePC();
      dc = pc.createDataChannel('datasync', { ordered: true });
      setupDC(dc);
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);
      await waitIce(pc);
      const payload = {
        type: 'datasync-offer', encoding: 'base64url',
        protocol_version: PROTOCOL_VER, device_id: deviceId(),
        device_label: deviceLabel(), session_id: sessionId,
        created_at: new Date().toISOString(),
        data: await compress(pc.localDescription.sdp),
      };
      const str = JSON.stringify(payload);
      document.getElementById('offerText').value = str;
      showQR('qrOfferContainer', str);
      setPhase('waiting-answer');
      log('Offer ready. Show QR to Device B, then scan their answer.');
    } catch(e) {
      log('createOffer ERROR: ' + e.name + ': ' + e.message);
    }
  }

  async function processOffer(raw) {
    let payload;
    try {
      payload = JSON.parse(raw.trim());
      if (payload.type !== 'datasync-offer')         throw new Error('Not a datasync-offer payload');
      if (payload.protocol_version !== PROTOCOL_VER) throw new Error('Protocol version mismatch');
    } catch (e) { log('Error: ' + e.message); return; }

    setPhase('connecting');
    sessionId = payload.session_id;
    await upsertDevice(payload.device_id, payload.device_label);
    makePC();
    pc.ondatachannel = e => { dc = e.channel; setupDC(dc); };
    const sdp = payload.data ? await decompress(payload.data) : payload.sdp;
    await pc.setRemoteDescription({ type: 'offer', sdp });
    const answer = await pc.createAnswer();
    await pc.setLocalDescription(answer);
    await waitIce(pc);
    const ans = {
      type: 'datasync-answer', encoding: 'base64url',
      protocol_version: PROTOCOL_VER, device_id: deviceId(),
      device_label: deviceLabel(), session_id: sessionId,
      created_at: new Date().toISOString(),
      data: await compress(pc.localDescription.sdp),
    };
    const str = JSON.stringify(ans);
    document.getElementById('answerText').value = str;
    document.getElementById('qrAnswerSection').classList.remove('ds-hidden');
    showQR('qrAnswerContainer', str);
    log('Answer ready. Show QR to Device A.');
  }

  async function processAnswer(raw) {
    let payload;
    try {
      payload = JSON.parse(raw.trim());
      if (payload.type !== 'datasync-answer') throw new Error('Not a datasync-answer payload');
      if (payload.session_id !== sessionId)   throw new Error('Session ID mismatch');
    } catch (e) { log('Error: ' + e.message); return; }
    await upsertDevice(payload.device_id, payload.device_label);
    const sdp = payload.data ? await decompress(payload.data) : payload.sdp;
    await pc.setRemoteDescription({ type: 'answer', sdp });
    setPhase('connecting');
    log('Answer accepted — waiting for DataChannel…');
  }

  // ── DataChannel ──────────────────────────────────────────────────────────────

  function setupDC(channel) {
    channel.onopen = () => {
      setPhase('connected');
      log('DataChannel open — starting sync…');
      resetStats();
      helloReceived = false;
      doneSent      = false;
      pendingReqIds = new Set();
      sendHello();
    };
    channel.onclose  = () => { if (phase !== 'done') { log('DataChannel closed.'); setPhase('idle'); } };
    channel.onerror  = e  => log('DataChannel error: ' + ((e.error && e.error.message) || 'unknown'));
    channel.onmessage = e => {
      try { handleMsg(JSON.parse(e.data)); } catch (err) { log('Invalid message: ' + err.message); }
    };
  }

  function sendMsg(msg) {
    if (dc && dc.readyState === 'open') dc.send(JSON.stringify(msg));
  }

  // ── Sync protocol ────────────────────────────────────────────────────────────

  async function sendHello() {
    setPhase('syncing');
    const [vector, logMeta] = await Promise.all([getLocalVector(), getLogsForDevice()]);
    sendMsg({
      type: 'hello', protocol_version: PROTOCOL_VER,
      device_id: deviceId(), device_label: deviceLabel(),
      app_version: APP_VER, db_schema_version: 1, vector, log_meta: logMeta,
    });
  }

  async function handleMsg(msg) {
    switch (msg.type) {
      case 'hello':        await onHello(msg);       break;
      case 'request_logs': await onRequestLogs(msg); break;
      case 'logs_batch':   await onLogsBatch(msg);   break;
      case 'ack_batch':    onAckBatch(msg);           break;
      case 'sync_done':    onSyncDone(msg);           break;
      default: log('Unknown message type: ' + msg.type);
    }
  }

  async function onHello(msg) {
    helloReceived = true;
    log('Hello from ' + (msg.device_label || msg.device_id.slice(0, 8)));
    document.getElementById('stRemoteDevice').textContent = msg.device_label || msg.device_id.slice(0, 8);
    if (msg.protocol_version !== PROTOCOL_VER) { log('Protocol version mismatch'); return; }
    await upsertDevice(msg.device_id, msg.device_label);
    if (msg.log_meta && msg.log_meta.length) {
      const cdb = await openContestDb();
      await new Promise((resolve, reject) => {
        const t = cdb.transaction('logs', 'readwrite');
        const s = t.objectStore('logs');
        msg.log_meta.forEach(l => s.put(l));
        t.oncomplete = resolve;
        t.onerror = e => reject(e.target.error);
      });
    }
    const localVec = await getLocalVector();
    let needCount  = 0;
    for (const [devId, remoteMax] of Object.entries(msg.vector)) {
      const localMax = localVec[devId] || 0;
      if (remoteMax > localMax) {
        const rid = uuidv4();
        pendingReqIds.add(rid);
        needCount++;
        log('Requesting ' + devId.slice(0, 8) + '… seq ' + (localMax + 1) + '–' + remoteMax);
        sendMsg({
          type: 'request_logs', request_id: rid, source_device_id: devId,
          from_seq: localMax + 1, to_seq: remoteMax,
          max_batch_bytes: MAX_BYTES, max_items: MAX_ITEMS,
        });
      }
    }
    if (needCount === 0) { log('Already in sync — nothing to request.'); checkDone(); }
  }

  async function onRequestLogs(msg) {
    log('Remote requests ' + msg.source_device_id.slice(0, 8) + '… seq ' + msg.from_seq + '–' + msg.to_seq);
    await sendBatches(msg.source_device_id, msg.from_seq, msg.to_seq,
                      msg.request_id, msg.max_items || MAX_ITEMS);
  }

  async function sendBatches(devId, fromSeq, toSeq, requestId, maxItems) {
    let cur = fromSeq;
    let firstBatch = true;
    while (cur <= toSeq) {
      while (dc && dc.bufferedAmount > BUFFER_HIGH) await new Promise(r => setTimeout(r, 50));
      const end  = Math.min(cur + maxItems - 1, toSeq);
      const qsos = devId === deviceId()
        ? await getLocalQsosRange(cur, end)
        : await getRemoteQsosRange(devId, cur, end);
      const logMeta = firstBatch ? await getLogsForDevice() : [];
      firstBatch = false;
      if (qsos.length === 0) {
        sendMsg({
          type: 'logs_batch', request_id: requestId, batch_id: uuidv4(),
          source_device_id: devId, from_seq: cur, to_seq: toSeq,
          count: 0, is_last: true, logs: [], log_meta: logMeta,
        });
        break;
      }
      const actualEnd = qsos[qsos.length - 1].source_seq;
      const isLast    = actualEnd >= toSeq;
      sendMsg({
        type: 'logs_batch', request_id: requestId, batch_id: uuidv4(),
        source_device_id: devId, from_seq: cur, to_seq: actualEnd,
        count: qsos.length, is_last: isLast, logs: qsos, log_meta: logMeta,
      });
      stats.sent += qsos.length; stats.batches++; updateStats();
      log('Sent ' + qsos.length + ' QSOs (' + devId.slice(0, 8) + '… ' + cur + '–' + actualEnd + ')');
      cur = actualEnd + 1;
      if (isLast) break;
    }
  }

  async function onLogsBatch(msg) {
    let stored = 0;
    if (msg.logs && msg.logs.length > 0) stored = await insertRemoteQsos(msg.logs, msg.log_meta);
    stats.received += stored; stats.batches++; updateStats();
    sendMsg({ type: 'ack_batch', batch_id: msg.batch_id, status: 'ok', stored_count: stored });
    if (msg.is_last) {
      if (msg.count > 0) await updateSyncState(msg.source_device_id, msg.to_seq);
      log('Received total ' + stats.received + ' QSOs from remote');
      pendingReqIds.delete(msg.request_id);
      checkDone();
    }
  }

  function onAckBatch(msg) { log('ACK stored=' + msg.stored_count); }

  function onSyncDone(msg) {
    log('Remote done: received=' + msg.received_count + ' sent=' + msg.sent_count);
    setPhase('done');
    refreshInfo();
  }

  function checkDone() {
    if (helloReceived && pendingReqIds.size === 0 && !doneSent) {
      doneSent = true;
      sendMsg({ type: 'sync_done', device_id: deviceId(),
                received_count: stats.received, sent_count: stats.sent, errors: [] });
      setPhase('done');
      log('Sync complete! Received=' + stats.received + ' Sent=' + stats.sent);
      refreshInfo();
    }
  }

  // ── Export ───────────────────────────────────────────────────────────────────

  async function exportDb() {
    const [cdb, sdb] = await Promise.all([openContestDb(), openSyncDb()]);
    const now = new Date();

    const [logs, qso, settings, syncState, devices] = await Promise.all([
      idbGetAll(cdb, 'logs'),
      idbGetAll(cdb, 'qso'),
      idbGetAll(cdb, 'settings'),
      idbGetAll(sdb, 'sync_state'),
      idbGetAll(sdb, 'devices'),
    ]);

    const backup = {
      export_version: 1,
      exported_at:    now.toISOString(),
      device_id:      deviceId(),
      device_label:   deviceLabel(),
      stores: { logs, qso, settings, sync_state: syncState, devices },
    };

    download(fileTs(now) + '-ic705-backup.json', JSON.stringify(backup, null, 2), 'application/json');
    log('Exported: ' + qso.length + ' QSOs, ' + logs.length + ' logs');
  }

  // ── Import ───────────────────────────────────────────────────────────────────

  async function importDb(file) {
    let backup;
    try {
      backup = JSON.parse(await file.text());
    } catch (e) {
      log('Import error: invalid JSON — ' + e.message);
      return;
    }

    if (!backup.stores) {
      log('Import error: unrecognised backup format (missing "stores")');
      return;
    }

    const [cdb, sdb] = await Promise.all([openContestDb(), openSyncDb()]);
    const counts = {};

    // contest DB stores
    for (const [key, storeName] of [['logs','logs'],['qso','qso'],['settings','settings']]) {
      const records = backup.stores[key] || [];
      if (!records.length || !cdb.objectStoreNames.contains(storeName)) continue;
      counts[storeName] = await putAll(cdb, storeName, records);
    }

    // sync DB stores
    for (const [key, storeName] of [['sync_state','sync_state'],['devices','devices']]) {
      const records = backup.stores[key] || [];
      if (!records.length || !sdb.objectStoreNames.contains(storeName)) continue;
      counts[storeName] = await putAll(sdb, storeName, records);
    }

    const summary = Object.entries(counts).map(([k, v]) => k + '=' + v).join(' ');
    log('Import complete: ' + (summary || 'nothing imported'));
    await refreshInfo();
  }

  function putAll(db, storeName, records) {
    return new Promise((resolve, reject) => {
      const t     = db.transaction(storeName, 'readwrite');
      const store = t.objectStore(storeName);
      let n = 0;
      records.forEach(r => { store.put(r); n++; });
      t.oncomplete = () => resolve(n);
      t.onerror    = e  => reject(e.target.error);
    });
  }

  // ── Download helper ──────────────────────────────────────────────────────────

  function download(fname, content, mime) {
    const blob = new Blob([content], { type: mime });
    const url  = URL.createObjectURL(blob);
    const a    = document.createElement('a');
    a.href = url; a.download = fname; a.click();
    URL.revokeObjectURL(url);
  }

  // ── UI helpers ───────────────────────────────────────────────────────────────

  function setPhase(p) {
    phase = p;
    const labels = {
      idle: 'Idle', 'creating-offer': 'Creating offer…', 'waiting-answer': 'Waiting for answer',
      connecting: 'Connecting…', connected: 'Connected', syncing: 'Syncing…',
      done: 'Done ✓', failed: 'Failed',
    };
    document.getElementById('stPhase').textContent = labels[p] || p;
  }

  function resetStats() {
    stats = { sent: 0, received: 0, batches: 0, errors: 0 };
    updateStats();
  }

  function updateStats() {
    document.getElementById('stSent').textContent     = stats.sent;
    document.getElementById('stReceived').textContent = stats.received;
    document.getElementById('stBatches').textContent  = stats.batches;
    document.getElementById('stErrors').textContent   = stats.errors;
  }

  function log(msg) {
    const el = document.getElementById('syncLog');
    el.textContent += new Date().toISOString().slice(11, 19) + ' ' + msg + '\n';
    el.scrollTop = el.scrollHeight;
  }

  function showPhase(id) {
    ['phaseIdle','phaseOffer','phaseJoin'].forEach(p =>
      document.getElementById(p).classList.toggle('ds-hidden', p !== id)
    );
  }

  function resetPairing() {
    if (pc) { try { pc.close(); } catch (_) {} pc = null; dc = null; }
    stopScan();
    showPhase('phaseIdle');
    document.getElementById('qrAnswerSection').classList.add('ds-hidden');
    document.getElementById('scanAnswerWrap').classList.add('ds-hidden');
    document.getElementById('scanOfferWrap').classList.add('ds-hidden');
    setPhase('idle');
    resetStats();
  }

  async function refreshInfo() {
    document.getElementById('infoDeviceId').textContent    = deviceId().slice(0, 16) + '…';
    document.getElementById('infoDeviceLabel').textContent = deviceLabel();
    try {
      const cdb       = await openContestDb();
      const numRange  = IDBKeyRange.bound(1, Number.MAX_SAFE_INTEGER);
      const localCnt  = await idbCount(cdb, 'qso', numRange);
      const totalCnt  = await idbCount(cdb, 'qso');
      document.getElementById('infoQsoCount').textContent    = localCnt;
      document.getElementById('infoRemoteCount').textContent = totalCnt - localCnt;

      // max local seq
      let maxSeq = 0;
      await new Promise((resolve, reject) => {
        const req = cdb.transaction('qso','readonly').objectStore('qso')
                       .openCursor(numRange, 'prev');
        req.onsuccess = e => { if (e.target.result) maxSeq = e.target.result.key; resolve(); };
        req.onerror   = e => reject(e.target.error);
      });
      document.getElementById('infoLastSeq').textContent = maxSeq || '—';
    } catch (_) {
      ['infoQsoCount','infoRemoteCount','infoLastSeq'].forEach(id =>
        document.getElementById(id).textContent = '?'
      );
    }
    try {
      const sdb  = await openSyncDb();
      const devs = await idbGetAll(sdb, 'devices');
      document.getElementById('infoDeviceCount').textContent = devs.length;
    } catch (_) {
      document.getElementById('infoDeviceCount').textContent = '?';
    }
  }

  // ── Event wiring ─────────────────────────────────────────────────────────────

  function el(id) {
    const e = document.getElementById(id);
    if (!e) console.error('[datasync] missing element:', id);
    return e;
  }

  function wire() {
    function on(id, ev, fn) { const e = el(id); if (e) e.addEventListener(ev, fn); }

    on('btnCreateSession', 'click', async () => { showPhase('phaseOffer'); await createOffer(); });
    on('btnJoinSession',   'click', () => showPhase('phaseJoin'));
    on('btnCancelOffer',   'click', resetPairing);
    on('btnCancelJoin',    'click', resetPairing);

    function copyEl(id) {
      const t = document.getElementById(id);
      if (!t) return;
      t.select();
      t.setSelectionRange(0, 99999);
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(t.value).catch(() => document.execCommand('copy'));
      } else {
        document.execCommand('copy');
      }
    }
    on('btnCopyOffer',  'click', () => copyEl('offerText'));
    on('btnCopyAnswer', 'click', () => copyEl('answerText'));

    on('btnScanOffer', 'click', () =>
      startScan('videoOffer', 'scanOfferWrap', async p => {
        el('offerPasteText').value = p; await processOffer(p);
      })
    );
    on('btnStopScanOffer', 'click', () => {
      stopScan(); el('scanOfferWrap').classList.add('ds-hidden');
    });
    on('btnProcessOffer', 'click', async () => {
      const t = el('offerPasteText').value.trim(); if (t) await processOffer(t);
    });

    on('btnScanAnswer', 'click', () =>
      startScan('videoAnswer', 'scanAnswerWrap', async p => {
        el('answerPasteText').value = p; await processAnswer(p);
      })
    );
    on('btnStopScanAnswer', 'click', () => {
      stopScan(); el('scanAnswerWrap').classList.add('ds-hidden');
    });
    on('btnProcessAnswer', 'click', async () => {
      const t = el('answerPasteText').value.trim(); if (t) await processAnswer(t);
    });

    const labelEditRow   = el('labelEditRow');
    const labelInput     = el('labelInput');
    const btnLabelSave   = el('btnLabelSave');
    const btnLabelCancel = el('btnLabelCancel');

    function openLabelEdit() {
      if (labelInput)   labelInput.value = deviceLabel();
      if (labelEditRow) labelEditRow.classList.remove('ds-hidden');
      if (labelInput)   { labelInput.focus(); labelInput.select(); }
    }
    function closeLabelEdit() {
      if (labelEditRow) labelEditRow.classList.add('ds-hidden');
    }

    on('btnEditLabel', 'click', openLabelEdit);
    if (btnLabelSave) btnLabelSave.addEventListener('click', () => {
      const v = labelInput ? labelInput.value.trim() : '';
      if (v) {
        localStorage.setItem('ds_device_label', v);
        const dd = el('infoDeviceLabel');
        if (dd) dd.textContent = v;
      }
      closeLabelEdit();
    });
    if (btnLabelCancel) btnLabelCancel.addEventListener('click', closeLabelEdit);
    if (labelInput) labelInput.addEventListener('keydown', e => {
      if (e.key === 'Enter')  btnLabelSave && btnLabelSave.click();
      if (e.key === 'Escape') closeLabelEdit();
    });

    on('btnExport', 'click', exportDb);

    document.getElementById('importFile').addEventListener('change', async function () {
      if (!this.files[0]) return;
      const file = this.files[0];
      this.value = '';
      const msg = document.getElementById('importMsg');
      msg.textContent = 'Importing…';
      try {
        await importDb(file);
        msg.textContent = 'Done.';
      } catch (e) {
        msg.textContent = 'Error: ' + e.message;
        log('Import error: ' + e.message);
      }
      setTimeout(() => { msg.textContent = ''; }, 3000);
    });
  }

  // ── Init ─────────────────────────────────────────────────────────────────────

  async function init() {
    try {
      wire();
      setPhase('idle');
      updateStats();
      await refreshInfo();
      log('Ready. Device: ' + deviceLabel() + ' (' + deviceId().slice(0, 8) + '…)');
    } catch (e) {
      log('INIT ERROR: ' + e.name + ': ' + e.message);
    }
  }

  document.addEventListener('DOMContentLoaded', init);

}());
