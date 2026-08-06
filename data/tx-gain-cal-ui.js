// One calibration tool, hosted by two pages.
//
// The search itself is tx-gain-cal.js. This is everything around it: the carrier,
// the byte accounting, the panel, the verdict and the write into the station's
// table. It lives here because the WSPR page and the JS8Call page must run the
// SAME tool -- the measurement is a property of the radio, not of the mode, and
// two copies of a transmitter-keying panel would be "similar" within a month.
//
// Like lan-gate.js and wake-lock.js this carries its own markup and CSS, so
// putting the tool on a page is one script tag and one create() call.
//
// The carrier is deliberately NOT either page's own tune path. It is a WsprStream
// single tone driven by WsprTx, for one reason: the search has to move the level
// eight or nine times inside one keying, and only that pair generates audio a
// packet at a time. The JS8 page's tune carrier is pre-rendered into one buffer,
// so its level is fixed the moment it is built -- which is why the tool could not
// simply be "the tune button with a loop around it".
//
// Everything a page must supply is in the adapter. Nothing in here reads a global.

(function (root, factory) {
  const value = factory(
    typeof module === "object" && module.exports ? require("./tx-gain-cal.js") : root.TxGainCal,
    typeof module === "object" && module.exports ? require("./wspr-core.js") : root.WsprCore,
    typeof module === "object" && module.exports ? require("./wspr-tx.js") : root.WsprTx);
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.TxGainCalUi = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function (TxGainCal, WsprCore, WsprTx) {

  const CAL_MAX_MS = 20000;             // carrier cap; a cold search needs ~12 s
  const CAL_TARGET_FILL_BYTES = 4000;   // 0.5 s ahead of the radio, half the usual
  const CAL_RAMP_SAMPLES = 5760;        // 120 ms
  const CAL_LEAD_MS = 2500;             // shortest lead the firmware's window allows
  const CAL_SWR_LIMIT = 3.0;            // firmware units, which overstate SWR
  const MOD_LEVEL_TARGET = 0.7;         // where the knee should sit; see decision 11

  const CSS = ""
    + ".cal-field{grid-column:1 / -1;border:1px solid #674a30;padding:8px;display:grid;gap:6px}"
    + ".cal-field[hidden]{display:none}"
    + ".cal-field p{margin:0}"
    + ".cal-field b{color:#fff}"
    + ".cal-intro,.cal-target{color:#8ba59d;font-size:12px;line-height:1.5}"
    + ".cal-live{font-family:ui-monospace,monospace;font-size:12px;color:#e0a020}"
    + ".cal-result{font-size:12px;color:#9fd39f;line-height:1.5}"
    + ".cal-error{font-size:12px;color:#e06c5a;line-height:1.5}"
    + ".cal-field button{cursor:pointer}";

  const MARKUP = ""
    + '<b>Automatic TX gain</b>'
    + '<p class="cal-intro">Keys a carrier for up to 20 s on the band and power the'
    + ' radio is set to <em>now</em>, raises the audio level until ALC just starts'
    + ' to act, and stores the level below that. Nothing else about the radio is'
    + ' changed.</p>'
    // Ids as well as the data attributes: one radio means one instance per page,
    // and the browser harnesses address these by id.
    + '<p class="cal-target" id="calTarget" data-cal="target"></p>'
    + '<p><button type="button" id="calStart" data-cal="start">START CALIBRATION</button></p>'
    + '<p class="cal-live" id="calLive" data-cal="live" hidden></p>'
    + '<p class="cal-result" id="calResult" data-cal="result" hidden></p>'
    + '<p class="cal-error" id="calMessage" role="alert" data-cal="error" hidden></p>';

  let cssInstalled = false;
  function installCss(document) {
    if (cssInstalled || !document) return;
    const style = document.createElement("style");
    style.textContent = CSS;
    document.head.appendChild(style);
    cssInstalled = true;
  }

  // The three causes the operator can actually act on, when the level ran to the
  // ceiling without the radio ever limiting. None of them is readable over CI-V,
  // so listing them is the only defence against a diagnostic dead end.
  const CEILING_ADVICE =
    " — ALC never acted, even at the maximum level: check that the radio's MOD" +
    " input is the LAN one, that its MOD level is not too low, and that RF power" +
    " is not set higher than the audio path can drive";

  class TxGainCalRun {
    // `adapter` is the page. See the bottom of this file for the contract.
    constructor(adapter) {
      this.page = adapter;
      this.store = adapter.store;
      this.cal = null;
      this.armed = false;
      this.running = false;
      this.timer = null;
      this.restoreMode = "";
      this.run = {key: "", band: "", percent: 0, model: "", dbm: null,
                  message: "", result: null, poPeak: 0, swrPeak: 0, sawAlcField: false};
      this.tx = new WsprTx.WsprTx({
        sink: adapter.sink,
        wallNow: adapter.wallNow,
        streamId: adapter.streamId,
        targetFillBytes: CAL_TARGET_FILL_BYTES,
        onEvent: event => this.onTxEvent(event),
      });
      if (adapter.mount) {
        installCss(adapter.mount.ownerDocument);
        adapter.mount.classList.add("cal-field");
        adapter.mount.innerHTML = MARKUP;
        adapter.mount.hidden = true;
        this.dom = {};
        for (const node of adapter.mount.querySelectorAll("[data-cal]"))
          this.dom[node.dataset.cal] = node;
        this.dom.start.addEventListener("click",
          () => (this.running ? this.stop("operator stop") : this.start()));
      }
    }

    // ---- identity ----------------------------------------------------------

    // The key this radio, band and power belong under, or null when any part is
    // unknown. A calibration filed under a guessed model or a power the radio has
    // not confirmed would be worse than none, because it would be applied without
    // anyone being asked.
    identity() {
      const radio = this.page.radio();
      const model = this.page.model();
      const band = TxGainCal.bandOf(radio.frequency);
      if (!model || !band || radio.rfPowerSeen !== true) return null;
      const percent = WsprCore.civPercent(radio.rfPower);
      return {key: TxGainCal.entryKey(model, band, percent), model, band, percent};
    }

    blockingReason() {
      const page = this.page.blockingReason();
      if (page) return page;
      if (!this.identity()) {
        const radio = this.page.radio();
        if (!this.page.model()) return "the radio has not reported its model yet";
        if (!TxGainCal.bandOf(radio.frequency)) return "the radio is not on an amateur band";
        return "the radio has not reported its power setting yet";
      }
      return "";
    }

    // What the table says for the radio as it stands right now.
    resolved() {
      const manual = this.page.manualGain();
      const identity = this.identity();
      if (!identity) return {gain: manual, calibrated: false, key: "",
                             why: "the radio has not reported its model, band or power yet"};
      const entry = this.store.entry(identity.key);
      if (!entry) return {gain: manual, calibrated: false, key: identity.key,
                          band: identity.band, percent: identity.percent,
                          why: `not calibrated for ${identity.band} @ ${identity.percent} %`};
      return {gain: Number(entry.gain), calibrated: true, key: identity.key, entry,
              band: identity.band, percent: identity.percent, why: ""};
    }

    // ---- the run -----------------------------------------------------------

    arm(armed) { this.armed = Boolean(armed); this.render(); }

    async start() {
      const problem = this.blockingReason();
      if (problem) { this.run.message = problem; this.render(); return; }
      const identity = this.identity();
      Object.assign(this.run, identity, {message: "", result: null, dbm: this.page.dbm(),
                                         poPeak: 0, swrPeak: 0, sawAlcField: false});
      try {
        // The only write the calibration makes. Without a data mode the LAN audio
        // never reaches the modulator, the search runs to the ceiling and reports
        // "ALC never acted" -- a true statement pointing at the wrong cause.
        // Snapshot and restore, the way announceIpViaCw does in the firmware.
        this.restoreMode = "";
        const mode = this.page.radio().mode;
        if (mode !== "USB-D") {
          this.restoreMode = mode;
          await this.page.ensureDataMode();
        }
        const stored = this.store.entry(this.run.key);
        this.cal = new TxGainCal.TxGainCal();
        this.cal.begin({knownKnee: stored ? Number(stored.knee) || 0 : 0});
        this.running = true;
        this.endsAtMs = this.page.wallNow() + CAL_LEAD_MS + CAL_MAX_MS;
        this.tx.targetFillBytes = CAL_TARGET_FILL_BYTES;
        this.tx.queue({symbols: new Uint8Array(162).fill(1),
                       slotUtcMs: this.page.wallNow() + CAL_LEAD_MS,
                       baseHz: 1500, amplitude: this.cal.gain,
                       leadMs: CAL_LEAD_MS, alcFast: true});
        this.timer = setTimeout(
          () => this.stop("the carrier ran out before the search finished"),
          CAL_LEAD_MS + CAL_MAX_MS);
        this.page.onRunChange(true);
      } catch (error) {
        this.cal = null;
        this.running = false;
        this.run.message = String(error.message || error);
        await this.restoreModeNow();
        this.page.onRunChange(false);
      }
      this.render();
    }

    // Every control frame from the firmware, for the same reason the beacon needs
    // them: tx-ready and tx-state are what move this driver out of "preparing",
    // and tx-level is both the credit clock and the ALC reading. Feeding only the
    // search left the driver waiting for a PTT confirmation that had already
    // arrived, and the run died with "PTT confirmation did not arrive".
    onControl(message) {
      if (!message) return;
      this.tx.onControl(message);
      if (message.type === "tx-level") this.noteLevel(message);
      else this.render();
    }

    // Also the only clock the search has: the byte counts say what the radio has
    // actually played, which no timer here can know.
    noteLevel(message) {
      if (!this.running || !this.cal) return;
      if (this.cal.state !== "searching" && this.cal.state !== "holding") return;
      if (Object.prototype.hasOwnProperty.call(message, "alcSeq")) this.run.sawAlcField = true;
      this.cal.noteSent(this.tx.sentUlaw);
      this.cal.noteLevel({consumed: message.consumed, alc: message.alc, alcSeq: message.alcSeq});
      if (this.tx.stream && Math.abs(this.tx.stream.amplitude - this.cal.gain) > 1e-6)
        this.tx.stream.setAmplitude(this.cal.gain, CAL_RAMP_SAMPLES);
      if (this.cal.state === "done") this.finish();
      else if (this.cal.state === "failed") this.stop(this.failureText(this.cal.error));
      else this.render();
    }

    // Peaks and the SWR watch, from whatever the page polls. This is the one
    // thing on either page that deliberately drives the level UP, so it is also
    // the one place that must not keep doing so into a mismatched antenna.
    noteMeters({powerMeterRaw, swr}) {
      if (!this.running) return;
      this.run.poPeak = Math.max(this.run.poPeak, Number(powerMeterRaw) || 0);
      this.run.swrPeak = Math.max(this.run.swrPeak, Number(swr) || 0);
      if ((Number(swr) || 0) >= CAL_SWR_LIMIT)
        this.stop(`SWR reached ${Number(swr).toFixed(1)} — check the antenna`);
    }

    // The module reports what it observed; naming the likely cause needs to know
    // whether the field was even present, which only the page's link can say.
    failureText(error) {
      if (error !== "radio does not report ALC") return error;
      return this.run.sawAlcField
        ? "the radio never answered the ALC meter (CI-V 15 13) — this interface asked" +
          " for it, so the radio or its LAN CI-V stream is not returning it"
        : "this interface's firmware does not send ALC readings — flash the firmware," +
          " not just the web assets (check that /txgain.json answers)";
    }

    async finish() {
      const result = this.cal.result;
      this.cal = null;
      this.run.result = result;
      this.run.message = "";
      this.stopCarrier();
      try {
        await this.store.put(this.run.key, {
          knee: Number(result.knee.toFixed(4)), gain: Number(result.gain.toFixed(4)),
          po: this.run.poPeak, swrMax: Number(this.run.swrPeak.toFixed(1)),
          model: this.run.model, band: this.run.band, percent: this.run.percent,
          reachedCeiling: result.reachedCeiling, autoTrimmed: false,
          at: this.page.now ? this.page.now() : Date.now(),
        });
      } catch (error) { this.run.message = String(error.message || error); }
      // The forward power measured during the final hold is a better reference
      // than a tune peak: the level it was taken at is known and steady.
      if (this.run.poPeak > 0 && this.run.dbm !== null && this.page.onReference)
        this.page.onReference(this.run.band, this.run.dbm, this.run.poPeak);
      await this.restoreModeNow();
      this.page.onRunChange(false);
      this.render();
    }

    stop(reason = "operator stop") {
      if (!this.running && !this.cal) return;
      this.cal = null;
      this.run.message = reason;
      this.stopCarrier();
      this.restoreModeNow().then(() => { this.page.onRunChange(false); this.render(); });
      this.render();
    }

    stopCarrier() {
      if (this.timer) { clearTimeout(this.timer); this.timer = null; }
      this.running = false;
      this.endsAtMs = 0;
      if (["preparing", "waiting-slot", "prebuffering", "streaming"].includes(this.tx.state))
        this.tx.fail("calibration finished");
    }

    async restoreModeNow() {
      if (!this.restoreMode || this.restoreMode === "USB-D") { this.restoreMode = ""; return; }
      const wanted = this.restoreMode;
      this.restoreMode = "";
      // Best effort: a failed restore must not turn a successful measurement into
      // an error, and the operator can see the mode on the page.
      try { await this.page.setMode(wanted); } catch (_error) {}
    }

    // The carrier's own driver has to be pumped, and its failures are the
    // calibration's, never the host page's beacon policy.
    tick() { if (this.running) this.tx.tick(); }

    onTxEvent(event) {
      if (event.type !== "failed" || !this.running) return;
      this.stop(event.reason || "the calibration carrier failed");
    }

    // ---- the panel ---------------------------------------------------------

    render() {
      if (!this.dom) return;
      this.page.mount.hidden = !this.armed;
      if (!this.armed) return;
      const identity = this.identity();
      const stored = identity ? this.store.entry(identity.key) : null;
      const problem = this.blockingReason();
      this.dom.target.textContent = identity
        ? `Will measure ${identity.model} on ${identity.band} at ${identity.percent} %` +
          (stored ? ` (previous knee ${stored.knee})` : " (never calibrated)")
        : problem || "";
      this.dom.start.textContent = this.running ? "STOP" : "START CALIBRATION";
      this.dom.start.disabled = !this.running && Boolean(problem);

      this.dom.live.hidden = !this.running;
      if (this.running && this.cal) {
        const left = Math.max(0, Math.round((this.endsAtMs - this.page.wallNow()) / 1000));
        this.dom.live.textContent =
          `${this.cal.phase || this.cal.state} · level ${this.cal.gain.toFixed(4)}` +
          ` · ALC ${this.cal.alcMax} · step ${this.cal.steps}` +
          ` · Po ${this.run.poPeak}/255 · ${left} s left`;
      }

      const result = this.run.result;
      this.dom.result.hidden = !result;
      if (result) this.dom.result.textContent =
        `knee ${result.knee.toFixed(4)}, stored ${result.gain.toFixed(4)}` +
        (result.reachedCeiling ? CEILING_ADVICE : ` — ${modLevelAdvice(result.knee)}`);
      this.dom.error.hidden = !this.run.message;
      this.dom.error.textContent = this.run.message;
    }
  }

  // The measured answer to "what should the MOD level in the radio's menu be",
  // which is the question this whole feature was asked for. A number, not a
  // value copied out of a manual that cannot know this radio, band or power.
  function modLevelAdvice(knee) {
    if (!(knee > 0)) return "";
    const db = 20 * Math.log10(knee / MOD_LEVEL_TARGET);
    if (Math.abs(db) < 3) return "the radio's MOD level is about right for this power";
    return `the radio's MOD level is ${Math.abs(db).toFixed(1)} dB too ` +
           `${db > 0 ? "low" : "high"} for this power (aim for a knee near ${MOD_LEVEL_TARGET})`;
  }

  // adapter: {
  //   mount, store, sink, streamId(), wallNow(), now?(),
  //   radio() -> {connected, mode, frequency, rfPower, rfPowerSeen, radioName},
  //   model() -> string, manualGain() -> number, dbm() -> number|null,
  //   blockingReason() -> string,      // the page's own gates, "" when clear
  //   ensureDataMode() -> Promise, setMode(mode) -> Promise,
  //   onRunChange(running), onReference?(band, dbm, poPeak),
  // }
  const create = adapter => new TxGainCalRun(adapter);

  return {create, TxGainCalRun, modLevelAdvice,
          CAL_MAX_MS, CAL_TARGET_FILL_BYTES, CAL_RAMP_SAMPLES, CAL_LEAD_MS,
          CAL_SWR_LIMIT, MOD_LEVEL_TARGET};
});
