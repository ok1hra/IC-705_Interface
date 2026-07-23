// Transmit arbiter.
//
// The station has six independent things that may want the radio: the operator
// typing, auto replies, heartbeat, relay forwards, inbox delivery and file
// transfer. Until now there was exactly one (an operator click), so no arbiter
// existed and stage 3 had to refuse automatic answers outright while busy.
//
// Rule E of docs/js8call-neobsluhovany-provoz-plan.md. The important half is not
// the ordering but the EXPIRY: a BIN transfer keys the radio slot after slot for
// minutes, and without expiry the queue would spend that time collecting stale
// answers and then dump ten minutes of them at once. A late SNR report is worth
// nothing, so it is dropped -- visibly, never silently.
//
// Heartbeat never queues. It is periodic by nature, so when it cannot go out now
// the right answer is to try again next interval, not to stack up beacons.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8TxQueue = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  const PRIORITY = {operator: 1, relay: 2, inbox: 2, autoreply: 3, heartbeat: 4};

  // How long an entry stays worth sending. `null` = never expires.
  const TTL_MS = {operator: null, relay: 30 * 60000, inbox: 30 * 60000,
    autoreply: null /* computed from the submode period */, heartbeat: 0};

  // JS8 slot periods per submode, used to express the auto-reply TTL as
  // "two periods" rather than a wall-clock guess.
  const PERIOD_MS = {0: 15000, 1: 10000, 2: 6000, 4: 30000, 8: 4000};
  const AUTOREPLY_PERIODS = 2;

  function autoReplyTtlMs(submode) {
    return (PERIOD_MS[submode] ?? PERIOD_MS[0]) * AUTOREPLY_PERIODS;
  }

  class Js8TxQueue {
    constructor({onEvent = null, maxSize = 32} = {}) {
      this.onEvent = onEvent;
      this.maxSize = maxSize;
      this.items = [];
      this.sequence = 0;
      this.stats = {queued: 0, sent: 0, expired: 0, rejected: 0, replaced: 0};
    }

    _emit(event) { if (this.onEvent) this.onEvent(event); }

    /**
     * @param entry {source, text, to?, nowMs, submode?, ttlMs?, meta?}
     * @returns {queued:true, id} | {queued:false, reason}
     */
    push(entry) {
      const source = entry.source;
      const priority = PRIORITY[source];
      if (!priority || !entry.text) {
        this.stats.rejected += 1;
        return {queued: false, reason: "invalid"};
      }
      // Heartbeat is periodic: never let it pile up behind other traffic.
      if (source === "heartbeat") {
        this.stats.rejected += 1;
        this._emit({type: "deferred", source, detail: "heartbeat reschedules instead of queueing"});
        return {queued: false, reason: "reschedule"};
      }
      if (this.items.length >= this.maxSize) {
        this.stats.rejected += 1;
        this._emit({type: "rejected", source, detail: `queue full (${this.maxSize})`});
        return {queued: false, reason: "full"};
      }

      const ttl = entry.ttlMs !== undefined ? entry.ttlMs
        : source === "autoreply" ? autoReplyTtlMs(entry.submode) : TTL_MS[source];
      const item = {id: ++this.sequence, source, priority, text: entry.text,
        to: entry.to || "", meta: entry.meta || null, queuedAtMs: entry.nowMs,
        expiresAtMs: ttl === null ? null : entry.nowMs + ttl};

      // One pending auto reply per station and command: asking twice should not
      // buy two answers, and the newer one carries the fresher report.
      if (source === "autoreply") {
        const index = this.items.findIndex(existing => existing.source === "autoreply" &&
          existing.to === item.to && existing.meta?.command === item.meta?.command);
        if (index >= 0) {
          this.items.splice(index, 1);
          this.stats.replaced += 1;
          this._emit({type: "replaced", source, to: item.to, detail: "newer answer supersedes"});
        }
      }

      this.items.push(item);
      this.stats.queued += 1;
      this._emit({type: "queued", source, to: item.to, text: item.text, id: item.id, size: this.items.length});
      return {queued: true, id: item.id};
    }

    // Drops everything past its expiry. Called by next() and safe to call alone
    // so the UI can show a truthful queue length.
    prune(nowMs) {
      const dropped = [];
      this.items = this.items.filter(item => {
        if (item.expiresAtMs === null || nowMs < item.expiresAtMs) return true;
        dropped.push(item);
        return false;
      });
      for (const item of dropped) {
        this.stats.expired += 1;
        this._emit({type: "expired", source: item.source, to: item.to, id: item.id,
          detail: `waited ${Math.round((nowMs - item.queuedAtMs) / 1000)} s`});
      }
      return dropped.length;
    }

    // Highest priority first, oldest first within a priority.
    peek(nowMs) {
      this.prune(nowMs);
      if (!this.items.length) return null;
      return this.items.reduce((best, item) =>
        item.priority < best.priority ||
        (item.priority === best.priority && item.id < best.id) ? item : best);
    }

    take(nowMs) {
      const item = this.peek(nowMs);
      if (!item) return null;
      this.items = this.items.filter(existing => existing.id !== item.id);
      this.stats.sent += 1;
      this._emit({type: "sent", source: item.source, to: item.to, id: item.id});
      return item;
    }

    remove(id) { this.items = this.items.filter(item => item.id !== id); }
    clear(reason = "cleared") {
      const count = this.items.length;
      this.items = [];
      if (count) this._emit({type: "cleared", detail: `${reason} (${count} dropped)`});
      return count;
    }

    size(nowMs) { if (nowMs !== undefined) this.prune(nowMs); return this.items.length; }

    snapshot(nowMs) {
      this.prune(nowMs);
      return {...this.stats, size: this.items.length,
        items: this.items.map(item => ({id: item.id, source: item.source, to: item.to,
          text: item.text,
          inMs: item.expiresAtMs === null ? null : item.expiresAtMs - nowMs}))};
    }
  }

  return {Js8TxQueue, PRIORITY, TTL_MS, PERIOD_MS, AUTOREPLY_PERIODS, autoReplyTtlMs};
});
