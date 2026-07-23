// Heartbeat scheduling and acknowledgement.
//
// A heartbeat is not a call: it announces the station to the @HB group so other
// operators can see who is reachable, and so relays and stored messages have a
// map to plan against. Upstream is explicit that HBs are not meant to start
// conversations, which shapes every rule below.
//
// Pure decision layer, like the auto-reply engine: it never transmits, and every
// refusal comes back with a reason (decision 13).

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8Heartbeat = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  // Upstream restricts HB to Slow/Normal/Fast for bandwidth efficiency and
  // compatibility across the HB network; the button is not even offered on
  // JS8 40 and JS8 60.
  const HB_SUBMODES = [4, 0, 1];       // Slow, Normal, Fast
  const OFFSET_MIN_HZ = 500;           // HBs sit in 500..1000 Hz so they do not
  const OFFSET_MAX_HZ = 1000;          // scatter across the whole passband
  const DEFAULT_INTERVAL_MS = 15 * 60000;
  const INTERVAL_CHOICES_MS = [5, 10, 15, 30, 60].map(minutes => minutes * 60000);

  const isHbSubmode = submode => HB_SUBMODES.includes(Number(submode));

  class Js8Heartbeat {
    constructor({restrictions = null, onEvent = null, random = Math.random} = {}) {
      this.restrictions = restrictions;
      this.onEvent = onEvent;
      this.random = random;
      this.enabled = false;
      this.ackEnabled = true;
      this.intervalMs = DEFAULT_INTERVAL_MS;
      this.nextDueMs = null;
      this.lastSentMs = null;
      this.stats = {sent: 0, deferred: 0, acked: 0, ackSkipped: 0};
    }

    _emit(event) { if (this.onEvent) this.onEvent(event); }

    configure({enabled, ackEnabled, intervalMs} = {}, nowMs = 0) {
      if (typeof enabled === "boolean") {
        if (enabled && !this.enabled) this.nextDueMs = nowMs + this.intervalMs;
        if (!enabled) this.nextDueMs = null;
        this.enabled = enabled;
      }
      if (typeof ackEnabled === "boolean") this.ackEnabled = ackEnabled;
      if (Number.isFinite(intervalMs) && intervalMs > 0) {
        this.intervalMs = intervalMs;
        if (this.enabled) this.nextDueMs = nowMs + intervalMs;
      }
      return this;
    }

    // Any traffic in our conversation window pushes the beacon back, so the
    // station does not talk over a QSO it is part of or watching.
    noteBandActivity(nowMs) {
      if (!this.enabled || this.nextDueMs === null) return false;
      if (this.nextDueMs - nowMs >= this.intervalMs) return false;
      this.nextDueMs = nowMs + this.intervalMs;
      this._emit({type: "postponed", detail: "band activity reset the heartbeat timer"});
      return true;
    }

    noteSent(nowMs) {
      this.lastSentMs = nowMs;
      this.nextDueMs = nowMs + this.intervalMs;
      this.stats.sent += 1;
    }

    dueInMs(nowMs) {
      return this.enabled && this.nextDueMs !== null ? this.nextDueMs - nowMs : null;
    }

    /**
     * Should a heartbeat go out now?
     * @returns {send:true, offsetHz} | {send:false, reason}
     */
    evaluate({nowMs, submode, txBusy = false, armed = true, myCall = ""}) {
      if (!this.enabled) return {send: false, reason: "disabled"};
      if (!myCall) return {send: false, reason: "no-callsign"};
      if (!armed) return {send: false, reason: "not-armed"};
      if (!isHbSubmode(submode))
        return {send: false, reason: "speed",
          detail: "heartbeat is not offered on JS8 40 or JS8 60"};
      if (this.nextDueMs === null || nowMs < this.nextDueMs)
        return {send: false, reason: "not-due"};
      if (txBusy) {
        // Periodic by nature: try again next interval rather than queueing a
        // beacon that would go out at the wrong time.
        this.stats.deferred += 1;
        this.nextDueMs = nowMs + this.intervalMs;
        this._emit({type: "deferred", detail: "radio busy, rescheduled"});
        return {send: false, reason: "tx-busy"};
      }
      return {send: true, offsetHz: this.pickOffsetHz()};
    }

    // A fixed offset would make every station's beacon collide with its own
    // previous one; upstream randomises inside a narrow band instead.
    pickOffsetHz() {
      const span = OFFSET_MAX_HZ - OFFSET_MIN_HZ;
      return Math.round(OFFSET_MIN_HZ + this.random() * span);
    }

    /**
     * Decide whether to acknowledge somebody else's heartbeat. "HB ACK" names
     * the behaviour; on the wire current JS8Call sends HEARTBEAT SNR, not the
     * ordinary message-flow ACK command. Upstream gates it behind a long
     * per-callsign window so the network does not turn into a storm.
     */
    handleHeartbeat(frame, ctx) {
      const from = String(frame && frame.from || "").toUpperCase().trim();
      if (!this.enabled) { this.stats.ackSkipped += 1; return {action: "skip", reason: "disabled"}; }
      if (!this.ackEnabled) { this.stats.ackSkipped += 1; return {action: "skip", reason: "ack-disabled"}; }
      if (!from || !ctx.myCall) { this.stats.ackSkipped += 1; return {action: "skip", reason: "invalid"}; }
      if (from === String(ctx.myCall).toUpperCase()) {
        this.stats.ackSkipped += 1;
        return {action: "skip", reason: "self"};
      }
      if (!ctx.armed) { this.stats.ackSkipped += 1; return {action: "skip", reason: "not-armed"}; }
      if (!isHbSubmode(ctx.submode)) {
        this.stats.ackSkipped += 1;
        return {action: "skip", reason: "speed",
          detail: "heartbeat acknowledgements are not sent on JS8 40 or JS8 60"};
      }
      if (ctx.messageBusy) {
        this.stats.ackSkipped += 1;
        return {action: "skip", reason: "message-busy",
          detail: "a buffered message is still being received"};
      }

      if (this.restrictions) {
        const verdict = this.restrictions.evaluate({call: from, command: "HEARTBEAT",
          nowMs: ctx.nowMs, isSelectedCall: false});
        if (!verdict.allowed) {
          this.stats.ackSkipped += 1;
          return {action: "skip", reason: verdict.reason, detail: verdict.detail};
        }
      }

      // Acknowledging resets our own beacon: the ACK already announced us, so a
      // separate heartbeat right after would be redundant traffic.
      if (this.enabled) this.nextDueMs = ctx.nowMs + this.intervalMs;
      this.stats.acked += 1;
      const measured = Number.isFinite(Number(frame.snr)) ? Math.round(Number(frame.snr)) : 0;
      const snr = Math.max(-30, Math.min(31, measured));
      const snrText = `${snr < 0 ? "-" : "+"}${String(Math.abs(snr)).padStart(2, "0")}`;
      // If we are holding a stored message for this station, the heartbeat is the
      // moment to tell it so: "HEARTBEAT SNR -12 MSG ID 32". It then fetches the
      // message with QUERY MSG. Upstream calls this a lightweight advertisement.
      const pendingId = typeof ctx.pendingMsgId === "function" ? ctx.pendingMsgId(from) : null;
      if (pendingId) {
        const text = `HEARTBEAT SNR ${snrText} MSG ID ${pendingId}`;
        this._emit({type: "advertise", to: from, text, msgId: pendingId});
        return {action: "ack", to: from, text};
      }
      const text = `HEARTBEAT SNR ${snrText}`;
      this._emit({type: "ack", to: from, text});
      return {action: "ack", to: from, text};
    }

    snapshot(nowMs) {
      return {...this.stats, enabled: this.enabled, ackEnabled: this.ackEnabled,
        intervalMs: this.intervalMs, dueInMs: this.dueInMs(nowMs)};
    }
  }

  return {Js8Heartbeat, HB_SUBMODES, OFFSET_MIN_HZ, OFFSET_MAX_HZ,
    DEFAULT_INTERVAL_MS, INTERVAL_CHOICES_MS, isHbSubmode};
});
