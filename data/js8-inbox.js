// Inbox: store-and-forward messaging.
//
// Five commands from docs/js8call-komunikacni-funkce.md group E:
//
//   MSG <text>              store for me, acknowledge
//   MSG TO:<CALL> <text>    store at this station for CALL, acknowledge
//   QUERY MSGS              do you hold anything for me?
//   QUERY MSG <id>          deliver message <id>
//   QUERY CALL <call>       can you reach <call>?
//
// The decision layer is here; the actual store lives in the firmware
// (decision 10) so mail survives a reload, a different computer and a cleared
// browser cache, and can be read from a phone. The store is injected, which also
// makes every rule testable without a radio or a filesystem.
//
// Storing is deliberately cheaper than transmitting: accepting a message costs
// us nothing but flash, so MSG is accepted even when unattended mode is off.
// Only the replies need a transmitter, and delivering somebody else's stored
// message is treated like relaying -- it needs arming.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8Inbox = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  const DEFAULTS = {
    maxMessages: 64,
    maxTextLength: 120,
    maxPerSender: 8,   // one station must not be able to fill the whole store
  };

  const CALL_RE = /^[A-Z0-9]{2,6}(?:\/[A-Z0-9]{1,2})?$/;
  const isCallsign = v => CALL_RE.test(String(v || "").toUpperCase().trim());
  const norm = v => String(v || "").toUpperCase().trim();
  const formatSnr = value => {
    const measured = Math.round(Number(value));
    const snr = Number.isFinite(measured)
      ? Math.max(-30, Math.min(31, measured)) : 0;
    return `${snr < 0 ? "-" : "+"}${String(Math.abs(snr)).padStart(2, "0")}`;
  };
  const since = (thenMs, nowMs) => {
    const seconds = Math.max(0, Math.floor((Number(nowMs) - Number(thenMs)) / 1000));
    if (!Number.isFinite(seconds) || seconds < 15) return "now";
    if (seconds >= 86400) return `${Math.floor(seconds / 86400)}d`;
    if (seconds >= 3600) return `${Math.floor(seconds / 3600)}h`;
    if (seconds >= 60) return `${Math.floor(seconds / 60)}m`;
    return `${seconds - (seconds % 15)}s`;
  };

  // "TO:OK1ABC HELLO" -> {to:"OK1ABC", text:"HELLO"}; null when malformed.
  function parseMsgTo(payload) {
    const match = /^TO:\s*([A-Z0-9/]+)\s+([\s\S]+)$/i.exec(String(payload || "").trim());
    if (!match) return null;
    const to = norm(match[1]);
    if (!isCallsign(to)) return null;
    return {to, text: match[2].trim()};
  }

  // A tiny in-memory store used by the tests and as the browser-side mirror.
  class MemoryStore {
    constructor() { this.items = []; this.nextId = 1; }
    add(record) { const item = {...record, id: this.nextId++}; this.items.push(item); return item; }
    forCall(call) { return this.items.filter(i => i.to === norm(call) && !i.delivered); }
    byId(id) { return this.items.find(i => i.id === Number(id)) || null; }
    countFrom(from) { return this.items.filter(i => i.from === norm(from) && !i.delivered).length; }
    markDelivered(id) { const i = this.byId(id); if (i) i.delivered = true; return i; }
    size() { return this.items.length; }
    all() { return this.items.slice(); }
  }

  class Js8Inbox {
    constructor(options = {}) {
      this.config = {...DEFAULTS, ...options};
      this.store = options.store || new MemoryStore();
      this.onEvent = options.onEvent || null;
      this.stats = {stored: 0, delivered: 0, refused: 0};
    }

    _emit(event) { if (this.onEvent) this.onEvent(event); }
    _refuse(reason, detail) {
      this.stats.refused += 1;
      this._emit({type: "refused", reason, detail});
      return {action: "skip", reason, detail};
    }

    /**
     * @param frame {from, to, command, text, complete}
     * @param ctx   {nowMs, myCall, armed, hearing:[calls]}
     */
    handle(frame, ctx) {
      const from = norm(frame && frame.from);
      const to = norm(frame && frame.to);
      const myCall = norm(ctx && ctx.myCall);
      const command = String(frame && frame.command || "").trim().toUpperCase();
      const payload = String(frame && frame.text || "").trim();

      if (!from || !myCall) return this._refuse("invalid", "missing callsign");
      if (to !== myCall) return this._refuse("not-addressed", `for ${to || "nobody"}`);
      // Checksummed commands: acting on half a message would store or transmit
      // corrupted text.
      if (frame.complete === false) return this._refuse("incomplete", `${command} still arriving`);

      switch (command) {
        case "MSG":       return this._store(from, from, payload, ctx);
        case "MSG TO:":   return this._storeFor(from, payload, ctx);
        case "QUERY MSGS":
        case "QUERY MSGS?": return this._queryMsgs(from, ctx);
        case "QUERY MSG": return this._queryMsg(from, payload, ctx);
        case "QUERY CALL": return this._queryCall(from, payload, ctx);
        default: return this._refuse("unsupported", `inbox does not handle ${command}`);
      }
    }

    _store(from, to, text, ctx) {
      if (!text) return this._refuse("empty", "no message text");
      if (text.length > this.config.maxTextLength)
        return this._refuse("too-long", `${text.length} characters, limit ${this.config.maxTextLength}`);
      if (this.store.size() >= this.config.maxMessages)
        return this._refuse("full", `inbox holds ${this.config.maxMessages} messages`);
      if (this.store.countFrom(from) >= this.config.maxPerSender)
        return this._refuse("sender-quota",
          `${from} already has ${this.config.maxPerSender} undelivered messages here`);

      const record = this.store.add({from, to, text, atMs: ctx.nowMs, delivered: false});
      this.stats.stored += 1;
      this._emit({type: "stored", id: record.id, from, to, text});
      // Storing costs nothing to transmit, so it works while disarmed; only the
      // acknowledgement needs the radio, and the caller decides that.
      // The local storage id is advertised by HEARTBEAT SNR / QUERY MSGS. A
      // normal MSG acknowledgement is the single-frame ACK command.
      return {action: "store", record, ack: {to: from, text: "ACK"}};
    }

    _storeFor(from, payload, ctx) {
      const parsed = parseMsgTo(payload);
      if (!parsed) {
        const skip = this._refuse("malformed", "expected MSG TO:CALL text");
        return {...skip, nack: {to: from, text: "NACK"}};
      }
      if (parsed.to === norm(ctx.myCall)) return this._store(from, parsed.to, parsed.text, ctx);
      return this._store(from, parsed.to, parsed.text, ctx);
    }

    _queryMsgs(from, ctx) {
      if (!ctx.armed)
        return this._refuse("not-armed", "QUERY MSGS requires AUTO");
      const waiting = this.store.forCall(from);
      if (!waiting.length)
        return {action: "reply", to: from, text: "NO", detail: "nothing stored"};
      // Upstream answers with the id of the oldest message still to be
      // delivered; the asker then requests it explicitly.
      const oldest = waiting[0];
      const more = waiting.length > 1 ? ` +${waiting.length - 1}` : "";
      return {action: "reply", to: from, text: `YES MSG ID ${oldest.id}${more}`,
        detail: `${waiting.length} waiting`};
    }

    _queryMsg(from, payload, ctx) {
      const id = Number(String(payload || "").trim());
      if (!Number.isInteger(id) || id <= 0) return this._refuse("malformed", "expected a message id");
      const record = this.store.byId(id);
      if (!record) return this._refuse("unknown-id", `no message ${id}`);
      if (record.to !== from)
        return this._refuse("not-yours", `message ${id} is not addressed to ${from}`);
      // Handing over a message we are holding for somebody else is transmitting
      // third-party content, exactly like a relay hop.
      if (!ctx.armed) return this._refuse("not-armed", "delivery requires unattended mode");
      // Do not mark it here. The caller confirms only after the RF transaction
      // reaches "completed"; an aborted or unavailable transmitter must leave
      // the message retrievable.
      const waiting = this.store.forCall(from).filter(item => item.id !== id);
      const next = waiting.length
        ? ` NEXT MSG ID ${waiting[0].id}${waiting.length > 1 ? ` +${waiting.length - 1}` : ""}`
        : "";
      return {action: "deliver", to: from,
        text: `MSG ${record.text} FROM ${record.from}${next}`, record,
        deliveryId: record.id};
    }

    _queryCall(from, payload, ctx) {
      if (!ctx.armed)
        return this._refuse("not-armed", "QUERY CALL requires AUTO");
      const target = norm(payload);
      if (!isCallsign(target)) return this._refuse("malformed", "expected a callsign");
      const heard = (ctx.hearing || []).find(item =>
        norm(typeof item === "object" && item ? item.call : item) === target);
      // Reference JS8Call stays silent when the target is unknown.
      if (!heard) return this._refuse("not-heard", `${target} not heard`);
      const text = typeof heard === "object" && heard
        ? `YES ${formatSnr(heard.snr)} (${since(heard.lastSlotUtcMs, ctx.nowMs)})`
        : "YES";
      return {action: "reply", to: from, text, detail: `we hear ${target}`};
    }

    // Messages this station is holding, for the UI and the remote panel.
    pending(call) { return this.store.forCall(call); }
    confirmDelivered(id) {
      const record = this.store.byId(id);
      if (!record || record.delivered) return false;
      this.store.markDelivered(id);
      this.stats.delivered += 1;
      this._emit({type: "delivered", id: record.id, to: record.to,
        text: record.text});
      return true;
    }
    snapshot() {
      return {...this.stats, size: this.store.size(),
        items: this.store.all().map(i => ({id: i.id, from: i.from, to: i.to,
          text: i.text, atMs: i.atMs, delivered: Boolean(i.delivered)}))};
    }
  }

  return {Js8Inbox, MemoryStore, DEFAULTS, parseMsgTo, isCallsign};
});
