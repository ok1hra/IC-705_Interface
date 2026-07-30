// Activity log for the WSPR beacon: one record per attempted transmission, and
// the pure aggregation that turns them into the grid. See
// docs/wspr-majak-implementace.md chapter 9.
//
// A deliberately separate IndexedDB database from the QSO log: WSPR
// transmissions are not contacts and have no business in a logbook that gets
// uploaded to LoTW.
//
// The aggregation is a pure function on purpose. Colour precedence is the part
// most likely to be wrong and the part hardest to see in a browser, so it is
// testable in Node without IndexedDB (tools/wspr-log-smoke.js).

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.WsprLog = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  const DB_NAME = "wspr-beacon", DB_VERSION = 1, STORE = "sessions";
  const DAY_MS = 86400000;
  const RETENTION_DAYS = 90;

  // Worst status wins in a cell. Ordered as chapter 9 defines it: a truncated
  // signal on the air outranks a slot that was never keyed, which outranks a
  // transmission whose power could not be confirmed.
  const STATUS_RANK = {idle: 0, sent: 1, suspect: 2, missed: 3, broken: 4};
  const STATUSES = ["idle", "sent", "suspect", "missed", "broken"];

  const dayStart = utcMs => Math.floor(utcMs / DAY_MS) * DAY_MS;

  // ---- pure aggregation -----------------------------------------------------

  // rows = hour of day UTC (24), columns = day, newest last.
  function summarise(records, {days = 28, nowUtcMs = Date.now()} = {}) {
    const lastDay = dayStart(nowUtcMs);
    const firstDay = lastDay - (days - 1) * DAY_MS;
    const cells = [];
    for (let hour = 0; hour < 24; hour++) {
      const row = [];
      for (let index = 0; index < days; index++)
        row.push({status: "idle", counts: {}, records: [],
                  dayUtcMs: firstDay + index * DAY_MS, hour});
      cells.push(row);
    }
    let counted = 0;
    for (const record of records || []) {
      const at = Number(record && record.slotUtcMs);
      if (!Number.isFinite(at)) continue;
      const dayIndex = Math.round((dayStart(at) - firstDay) / DAY_MS);
      if (dayIndex < 0 || dayIndex >= days) continue;
      const hour = new Date(at).getUTCHours();
      const cell = cells[hour][dayIndex];
      const status = STATUSES.includes(record.status) ? record.status : "missed";
      cell.records.push(record);
      cell.counts[status] = (cell.counts[status] || 0) + 1;
      if (STATUS_RANK[status] > STATUS_RANK[cell.status]) cell.status = status;
      counted++;
    }
    return {cells, days, firstDay, lastDay, counted};
  }

  // Short ranges get slot resolution instead: rows = one UTC hour, columns = the
  // thirty two-minute WSPR slots inside it. An hour-per-cell grid cannot show
  // which slots of an hour were lost, and that is exactly what an operator
  // debugging a beacon needs to see.
  const HOUR_MS = 3600000, SLOT_MS = 120000, SLOTS_PER_HOUR = 30;
  const hourStart = utcMs => Math.floor(utcMs / HOUR_MS) * HOUR_MS;

  // aheadHours appends empty rows after the current hour. This file stays about
  // records -- it never predicts anything -- but the caller needs the future rows
  // to exist on the same axis before it can mark what the schedule will key there.
  function summariseSlots(records, {hours = 24, aheadHours = 0, nowUtcMs = Date.now()} = {}) {
    const lastHour = hourStart(nowUtcMs);
    const firstHour = lastHour - (hours - 1) * HOUR_MS;
    const rows = hours + Math.max(0, aheadHours);
    const cells = [];
    for (let row = 0; row < rows; row++) {
      const hourUtcMs = firstHour + row * HOUR_MS, line = [];
      for (let slot = 0; slot < SLOTS_PER_HOUR; slot++)
        line.push({status: "idle", counts: {}, records: [], hourUtcMs,
                   slotUtcMs: hourUtcMs + slot * SLOT_MS + 1000});
      cells.push(line);
    }
    let counted = 0;
    for (const record of records || []) {
      const at = Number(record && record.slotUtcMs);
      if (!Number.isFinite(at)) continue;
      const row = Math.round((hourStart(at) - firstHour) / HOUR_MS);
      if (row < 0 || row >= rows) continue;
      const slot = Math.floor((at - hourStart(at)) / SLOT_MS);
      if (slot < 0 || slot >= SLOTS_PER_HOUR) continue;
      const cell = cells[row][slot];
      const status = STATUSES.includes(record.status) ? record.status : "missed";
      cell.records.push(record);
      cell.counts[status] = (cell.counts[status] || 0) + 1;
      if (STATUS_RANK[status] > STATUS_RANK[cell.status]) cell.status = status;
      counted++;
    }
    return {cells, hours, firstHour, lastHour, counted};
  }

  // Totals for the header line, over the same window the grid shows. `hours`
  // wins when both are given, because that is the finer of the two.
  function totals(records, {days = 28, hours = 0, nowUtcMs = Date.now()} = {}) {
    const from = hours > 0 ? hourStart(nowUtcMs) - (hours - 1) * HOUR_MS
                           : dayStart(nowUtcMs) - (days - 1) * DAY_MS;
    const out = {sent: 0, suspect: 0, missed: 0, broken: 0};
    for (const record of records || []) {
      if (Number(record.slotUtcMs) < from) continue;
      if (out[record.status] !== undefined) out[record.status]++;
    }
    return out;
  }

  // A transmission that completed but whose forward power did not match what
  // TUNE measured is reported, not hidden: the dBm on the air is then a guess,
  // and a wrong power figure quietly corrupts everyone's propagation data.
  function classify({completed, afterKeying, powerMeterRaw, referenceRaw,
                     tolerance = 0.2}) {
    if (!completed) return afterKeying ? "broken" : "missed";
    if (referenceRaw > 0 && powerMeterRaw >= 0) {
      const drift = Math.abs(powerMeterRaw - referenceRaw) / referenceRaw;
      if (drift > tolerance) return "suspect";
    }
    return "sent";
  }

  // ---- storage --------------------------------------------------------------

  let dbPromise = null;

  function open(indexedDBImpl = globalThis.indexedDB) {
    if (dbPromise) return dbPromise;
    if (!indexedDBImpl) return Promise.reject(new Error("IndexedDB is unavailable"));
    dbPromise = new Promise((resolve, reject) => {
      const request = indexedDBImpl.open(DB_NAME, DB_VERSION);
      request.onupgradeneeded = () => {
        const db = request.result;
        if (!db.objectStoreNames.contains(STORE))
          db.createObjectStore(STORE, {keyPath: "slotUtcMs"});
      };
      request.onsuccess = () => resolve(request.result);
      request.onerror = () => reject(request.error);
    });
    return dbPromise;
  }

  function transact(mode, work) {
    return open().then(db => new Promise((resolve, reject) => {
      const tx = db.transaction(STORE, mode);
      const store = tx.objectStore(STORE);
      let result;
      try { result = work(store); } catch (error) { reject(error); return; }
      tx.oncomplete = () => resolve(result && result.__request ? result.__request.result : result);
      tx.onerror = () => reject(tx.error);
      tx.onabort = () => reject(tx.error);
    }));
  }

  // slotUtcMs is the key, so a retried slot overwrites rather than duplicating.
  function record(session) {
    return transact("readwrite", store => { store.put(session); });
  }

  function all() {
    return open().then(db => new Promise((resolve, reject) => {
      const tx = db.transaction(STORE, "readonly");
      const request = tx.objectStore(STORE).getAll();
      request.onsuccess = () => resolve(request.result || []);
      request.onerror = () => reject(request.error);
    }));
  }

  function prune(nowUtcMs = Date.now(), retentionDays = RETENTION_DAYS) {
    const cutoff = dayStart(nowUtcMs) - retentionDays * DAY_MS;
    return transact("readwrite", store => {
      const request = store.openCursor();
      request.onsuccess = () => {
        const cursor = request.result;
        if (!cursor) return;
        if (Number(cursor.key) < cutoff) cursor.delete();
        cursor.continue();
      };
    });
  }

  function clear() { return transact("readwrite", store => { store.clear(); }); }

  return {DB_NAME, STORE, STATUS_RANK, STATUSES, RETENTION_DAYS,
          summarise, summariseSlots, totals, classify, open, record, all, prune, clear};
});
