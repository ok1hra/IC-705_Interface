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
  const DEFAULT_INTERVAL_MS = 60 * 60000;
  const INTERVAL_CHOICES_MS = [5, 10, 15, 30, 60].map(minutes => minutes * 60000);
  // Only a beacon that is about to key up needs to yield to band traffic; one JS8
  // frame (~15 s) is the window in which transmitting would actually talk over a
  // decode. Anything earlier in the interval is somebody else's exchange we are
  // not part of, and must not touch our schedule.
  const POSTPONE_GUARD_MS = 15000;
  // Ceiling on how far routine traffic may push a due beacon past its schedule.
  // On a wall-to-wall band the one-frame slip would repeat every decode; this
  // guarantees the beacon still announces the station within a bounded delay --
  // the whole point of an unattended beacon is that it cannot be starved out.
  const MAX_DEFER_MS = 2 * 60000;

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
      this.dueBaseMs = null;   // the un-slipped schedule, so the defer cap has a datum
      this.lastSentMs = null;
      this.stats = {sent: 0, deferred: 0, acked: 0, ackSkipped: 0};
    }

    _emit(event) { if (this.onEvent) this.onEvent(event); }

    // Arm the next beacon. dueBaseMs records the schedule before any slip, so
    // noteBandActivity can cap how far traffic drags it.
    _arm(dueMs) { this.nextDueMs = dueMs; this.dueBaseMs = dueMs; return dueMs; }

    configure({enabled, ackEnabled, intervalMs} = {}, nowMs = 0) {
      if (typeof enabled === "boolean") {
        if (enabled && !this.enabled) this._arm(nowMs + this.intervalMs);
        if (!enabled) { this.nextDueMs = null; this.dueBaseMs = null; }
        this.enabled = enabled;
      }
      if (typeof ackEnabled === "boolean") this.ackEnabled = ackEnabled;
      if (Number.isFinite(intervalMs) && intervalMs > 0) {
        this.intervalMs = intervalMs;
        if (this.enabled) this._arm(nowMs + intervalMs);
      }
      return this;
    }

    // Traffic in the last frame before the beacon fires makes it yield, so it
    // does not transmit on top of a decode. It slips by a single frame, not a
    // whole interval: a busy band would otherwise push the beacon forward on
    // every decode and it would never announce the station at all.
    // Push an imminent beacon back by one frame without ever crossing the defer
    // ceiling (dueBaseMs + MAX_DEFER_MS). Returns true if it actually moved. Only
    // noteSent re-anchors dueBaseMs, so traffic, ACKs and a busy radio can delay the
    // beacon but never starve it: it always fires within one interval + MAX_DEFER_MS.
    _deferBounded(nowMs) {
      if (!this.enabled || this.nextDueMs === null) return false;
      if (this.nextDueMs - nowMs > POSTPONE_GUARD_MS) return false;   // not imminent
      const slipped = nowMs + POSTPONE_GUARD_MS;
      if (slipped > (this.dueBaseMs ?? this.nextDueMs) + MAX_DEFER_MS) return false;  // ceiling
      this.nextDueMs = slipped;
      return true;
    }

    noteBandActivity(nowMs) {
      if (!this._deferBounded(nowMs)) return false;
      this._emit({type: "postponed", detail: "band activity delayed the heartbeat by one frame"});
      return true;
    }

    noteSent(nowMs) {
      this.lastSentMs = nowMs;
      this._arm(nowMs + this.intervalMs);
      this.stats.sent += 1;
    }

    // A beacon that was keyed but faulted (e.g. a missed TX slot) never reached the
    // air, so it must retry in the next quiet frame rather than wait a whole
    // interval -- otherwise one fault hides the station until the next schedule.
    // A subsequent success re-anchors the normal cadence via noteSent.
    noteFault(nowMs) {
      if (!this.enabled) return;
      this._arm(nowMs + POSTPONE_GUARD_MS);
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
        // The radio is mid-transmission. Do not queue a beacon for the wrong slot,
        // but do not push it a whole interval either: bounded-defer it so an active
        // station still announces itself within dueBaseMs + MAX_DEFER_MS and fires
        // as soon as the radio frees, instead of vanishing for a whole interval.
        this.stats.deferred += 1;
        this._deferBounded(nowMs);
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

      // A just-sent ACK already announced us, so postpone an imminent beacon to
      // avoid keying again right behind it -- but only when it is actually imminent,
      // and bounded by the defer ceiling. On an ACK-heavy HB band this no longer
      // pushes our @HB broadcast forward forever (which used to hide the station
      // from the HB map even though it was clearly active).
      this._deferBounded(ctx.nowMs);
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
    DEFAULT_INTERVAL_MS, INTERVAL_CHOICES_MS, POSTPONE_GUARD_MS, isHbSubmode};
});
