// WSPR beacon page. See docs/wspr-majak-implementace.md.
//
// Owns: settings, the single-operator lock, /state polling, CAT (band, USB-D,
// power with readback), the 48-slot schedule, TUNE, and the beacon state machine
// that hands one frame at a time to WsprTx.
//
// Deliberately does not own: symbol generation and pacing (wspr-core.js,
// wspr-tx.js) or the AUD1 wire protocol (js8-aud1.js, used unchanged).

(function () {
  "use strict";

  // Same seam data.js uses (data.js:11): lets a fixture point the audio socket at
  // an unprivileged port on the same host. Port 83 is privileged, so without this
  // the page could not be exercised outside a real device at all.
  const AUDIO_WS_PORT =
    Number(new URLSearchParams(location.search).get("audioPort")) || 83;
  const STATE_POLL_MS = 1000;
  const SESSION_PING_MS = 5000, SESSION_RETRY_MS = 3000, SESSION_PROBE_MS = 250;
  const SETTINGS_KEY = "ic705.wspr.v1";
  const PREPARE_LEAD_MS = 10000;    // must match WsprTx's window
  // TUNE is a manual action, so it uses the shortest lead the TX path allows
  // (prebuffering starts 1.35 s before the slot) rather than the beacon's ten
  // seconds -- a button that does nothing for ten seconds reads as broken.
  const TUNE_LEAD_MS = 2500;
  const TUNE_MAX_MS = 10000;        // hard cap, same as the JS8 tune carrier
  // LAN is exclusive to one of TRX1-TRX3 and the operator picks which; ?radio=lan
  // makes the firmware answer for that slot instead of the primary radio. Without
  // it the beacon would read and command TRX1 while its audio went to the LAN
  // radio -- silently wrong whenever LAN sits on TRX2 or TRX3. data.js does the
  // same, and lan-gate.js has already proved the slot exists by the time these
  // are used.
  const RADIO_STATE_URL = "/state?radio=lan";
  const RADIO_CMD_URL = "/cmd?radio=lan";

  const $ = id => document.getElementById(id);
  const dom = {};
  for (const id of ["trxFrequencyValue", "trxMode", "radioModel", "linkState", "utcClock",
    "sessionBusy", "sessionBusyWhere", "sessionTakeover",
    "beaconInterface", "callsign", "locator", "locatorTransmitted", "powerDbm", "powerWatts",
    "powerPercent", "powerSet", "stationError", "radioModelOverride", "fullPowerWatts",
    "fullPowerSource", "clockCorrection",
    "txGain", "txGainValue", "txSafety", "tuneButton", "powerMeter", "swr", "powerReference",
    "periodFrames", "periodHint", "randomizeFrame", "scheduleClear", "scheduleGrid",
    "schedulePopover", "startStop", "beaconState", "nextSession", "liveSession",
    "sessionProgress", "ringFill", "packetCount", "pttState", "beaconError",
    "activityDays", "activityTotals", "activityGrid", "activityDetail",
    // shell shared with the JS8Call page
    "waterfall", "waterfallCanvas", "waterfallOverlay", "spectrumSummary", "audioLevel",
    "aud1State", "lanHealth", "trxReconnect", "trxHelpButton", "trxHelpDialog",
    "trxHelpModeWarning", "timingState",
    "freqTimetableButton", "freqTimetableValue", "freqTimetablePanel",
    "txSessionSummary", "activitySummary", "settingsSummary"]) dom[id] = $(id);

  // The WSPR sub-band inside the SSB passband. The offset is randomised in here
  // for every transmission; the waterfall only shows where it landed.
  const WINDOW_LOW_HZ = 1400, WINDOW_HIGH_HZ = 1600;
  const RX_LOW = 500, RX_HIGH = 2700, AUDIO_RATE = 8000;
  // Never key above this, whatever the radio can do. A 110 s carrier is a
  // different proposition from an SSB transmission, and this page is a QRP
  // beacon on someone's desk.
  const POWER_CEILING_W = 10;

  const state = {
    radio: {connected: false, transceiverType: "", radioName: "", mode: "", frequency: 0,
            tx: false, powerMeterRaw: 0, swr: 0, rfPower: 0, supplyVolts: 0,
            lanDrops: 0, lanStalls: 0, lanFilled: 0},
    beacon: "stopped",          // stopped | armed | tuning | transmitting | paused
    consecutiveBroken: 0,
    tunePeakRaw: 0,             // highest forward-power reading seen during TUNE
    lastError: "",
    pendingSlotUtcMs: 0,
    currentSession: null,     // what the in-flight transmission will be logged as
    activity: [],             // records loaded from IndexedDB, newest first
    activityRange: "24h",
    selectedCell: null,
    audioDb: -90,
    lastOffsetHz: 1500,       // where the last (or current) transmission landed
  };

  // ---- settings ------------------------------------------------------------

  // The station locator lives in the shared Js8Settings blob, not here: it is a
  // property of the station, so fixing it on arrival at a portable site should
  // fix it for JS8LAN too.
  function loadShared() { return Js8Settings.load(localStorage).settings; }
  function sharedGrid() {
    const settings = loadShared();
    return (settings.modems && settings.modems.js8call && settings.modems.js8call.grid) || "";
  }
  function sharedCall() {
    const settings = loadShared();
    return (settings.modems && settings.modems.js8call && settings.modems.js8call.myCall) || "";
  }
  function saveShared(patch) {
    const settings = loadShared();
    Object.assign(settings.modems.js8call, patch);
    Js8Settings.save(localStorage, settings);
  }
  // The same pledge JS8LAN requires before it will key, deliberately shared: it
  // is a statement about the antenna and the power, not about a mode. Read fresh
  // on every call, so ticking the box on either page is seen here immediately.
  function txSafetyAccepted() {
    const settings = loadShared();
    return Boolean(settings.modems && settings.modems.js8call &&
                   settings.modems.js8call.txSafetyAccepted);
  }

  // WSPR starts one second past an even UTC minute and the decoder tolerates
  // about a second, so a device whose clock is off transmits into nothing. The
  // correction is shared with JS8Call, which is also the only page that can
  // measure it -- WSPR does not decode, so it cannot check its own clock and
  // deliberately offers no automatic timing.
  function clockCorrectionMs() {
    const settings = loadShared();
    const value = settings.modems && settings.modems.js8call
      ? Number(settings.modems.js8call.clockCorrectionMs) : 0;
    return Number.isFinite(value) ? value : 0;
  }
  const utcNow = () => Date.now() + clockCorrectionMs();

  const settingsDefaults = () => ({
    version: 1, powerDbm: 30, txGain: 0.25, modelOverride: "",
    powerReferenceRaw: 0, periodFrames: 5, randomizeFrame: true, slots: {},
  });

  let settings = settingsDefaults();

  function loadSettings() {
    try {
      const raw = JSON.parse(localStorage.getItem(SETTINGS_KEY) || "null");
      if (raw && typeof raw === "object") settings = {...settingsDefaults(), ...raw};
    } catch (_error) { settings = settingsDefaults(); }
    if (!WsprCore.POWER_LEVELS.includes(settings.powerDbm)) settings.powerDbm = 30;
    settings.txGain = Math.min(0.8, Math.max(0.1, Number(settings.txGain) || 0.25));
    settings.periodFrames = Math.min(15, Math.max(1, Number(settings.periodFrames) || 5));
    if (!settings.slots || typeof settings.slots !== "object") settings.slots = {};
  }
  function saveSettings() {
    try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings)); }
    catch (_error) { /* private mode: the page still works for this session */ }
  }

  // ---- single-operator lock -------------------------------------------------
  //
  // Mirrors the JS8LAN implementation rather than importing it, because that code
  // lives inside data.js rather than in a module. The firmware is the authority;
  // this only renders what it decides. The BroadcastChannel probe exists because
  // a duplicated tab shares sessionStorage, so the firmware cannot tell the two
  // apart and would grant both.

  const SESSION_TOKEN_KEY = "js8lan.session.token.v1";
  let sessionTokenCache = null, sessionHeld = false, sessionConfirmed = false;
  let sessionRetryTimer = null, sessionSince = 0, sessionLocalHolder = null;

  function makeToken() {
    const bytes = new Uint8Array(16);
    if (globalThis.crypto && crypto.getRandomValues) crypto.getRandomValues(bytes);
    else for (let i = 0; i < bytes.length; i++) bytes[i] = Math.floor(Math.random() * 256);
    return Array.from(bytes, byte => byte.toString(16).padStart(2, "0")).join("");
  }
  function sessionToken() {
    if (sessionTokenCache) return sessionTokenCache;
    let token = null;
    try { token = sessionStorage.getItem(SESSION_TOKEN_KEY); } catch (_error) {}
    if (!token) {
      token = makeToken();
      try { sessionStorage.setItem(SESSION_TOKEN_KEY, token); } catch (_error) {}
    }
    sessionTokenCache = token;
    return token;
  }

  const pageId = makeToken();
  const channel = (() => { try { return new BroadcastChannel("js8lan.session"); } catch (_error) { return null; } })();
  if (channel) channel.onmessage = event => {
    const message = event.data || {};
    if (message.id === pageId) return;
    if (message.type === "probe" && sessionHeld)
      channel.postMessage({type: "held", id: pageId, since: sessionSince});
    if (message.type === "held") sessionLocalHolder = {id: message.id, since: Number(message.since) || 0};
    if (message.type === "released" && !sessionHeld) scheduleSessionRetry(200);
    if (message.type === "evict" && sessionHeld) loseSession({});
  };

  function probeLocalHolder() {
    if (!channel) return Promise.resolve(null);
    sessionLocalHolder = null;
    channel.postMessage({type: "probe", id: pageId});
    return new Promise(resolve => setTimeout(() => resolve(sessionLocalHolder), SESSION_PROBE_MS));
  }
  function localHolderOutranks(holder) {
    if (!holder) return false;
    if (holder.since !== sessionSince) return holder.since < sessionSince;
    return holder.id < pageId;
  }

  // Only an explicit 409 is a refusal: a firmware without the lock, or a fetch
  // that simply failed, must not leave the operator staring at a panel.
  async function sessionPost(path, extra) {
    try {
      const response = await fetch(path, {method: "POST", cache: "no-store",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({token: sessionToken(), ...extra})});
      if (response.status !== 409) return {granted: true};
      const info = await response.json().catch(() => ({}));
      return {granted: false, owner: info.owner || "", ageMs: Number(info.ageMs) || 0};
    } catch (_error) { return {granted: true}; }
  }

  function scheduleSessionRetry(delayMs = SESSION_RETRY_MS) {
    if (sessionRetryTimer) clearTimeout(sessionRetryTimer);
    sessionRetryTimer = setTimeout(claimSession, delayMs);
  }

  function markHeld(confirmed) {
    sessionHeld = true; sessionConfirmed = confirmed; sessionSince = Date.now();
    if (sessionRetryTimer) { clearTimeout(sessionRetryTimer); sessionRetryTimer = null; }
    dom.sessionBusy.hidden = true;
    // The audio socket belongs to the page, not to a transmission. Opening it
    // here rather than at START is what keeps `hello` from racing the first
    // tx.prepare, and it is what gives the waterfall something to draw.
    openSession();
    render();
  }

  function loseSession(info) {
    stopBeacon("session lost");
    closeSession();
    sessionHeld = false; sessionConfirmed = false;
    dom.sessionBusy.hidden = false;
    dom.sessionBusyWhere.textContent = info.owner ? `held by ${info.owner}` : "";
    scheduleSessionRetry();
    render();
  }

  async function claimSession(force = false) {
    const holder = await probeLocalHolder();
    if (holder && localHolderOutranks(holder) && !force) {
      loseSession({owner: "another tab in this browser"});
      return;
    }
    const claim = await sessionPost("/js8/session/claim", {force});
    if (!claim.granted) { loseSession(claim); return; }
    markHeld(true);
  }

  setInterval(async () => {
    if (!sessionHeld) return;
    const ping = await sessionPost("/js8/session/ping", {});
    if (!ping.granted) loseSession(ping);
  }, SESSION_PING_MS);

  addEventListener("pagehide", () => {
    if (!sessionHeld) return;
    if (channel) channel.postMessage({type: "released", id: pageId});
    try {
      fetch("/js8/session/release", {method: "POST", keepalive: true,
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({token: sessionToken()})});
    } catch (_error) { /* leaving anyway */ }
  });

  // ---- radio state ----------------------------------------------------------

  async function pollState() {
    try {
      const response = await fetch(RADIO_STATE_URL, {cache: "no-store"});
      if (!response.ok) throw new Error(String(response.status));
      const json = await response.json();
      state.radio = {
        connected: Boolean(json.connected),
        transceiverType: String(json.transceiverType || ""),
        radioName: String(json.radioName || ""),
        mode: String(json.mode || ""),
        frequency: Number(json.frequency) || 0,
        tx: Boolean(json.tx),
        powerMeterRaw: Number(json.powerMeterRaw) || 0,
        swr: Number(json.swr) || 0,
        rfPower: Number(json.rfPower) || 0,
        supplyVolts: Number(json.supplyVolts) || 0,
        lanDrops: Number(json.lanDrops) || 0,
        lanStalls: Number(json.lanStalls) || 0,
        lanFilled: Number(json.lanFilled) || 0,
      };
    } catch (_error) {
      state.radio.connected = false;
    }
    render();
  }

  const isLan = () => state.radio.transceiverType === "IC-705-LAN";

  // Full power of the transmitter on the CI-V percentage scale. Order matters:
  // an explicit override wins, then what the radio itself reported, and an
  // unknown model yields null so the UI refuses to start rather than guessing a
  // curve that could be a factor of ten out.
  function fullPower() {
    if (settings.modelOverride)
      return {watts: WsprCore.fullPowerWatts(settings.modelOverride), source: "manual override"};
    const reported = WsprCore.fullPowerWatts(state.radio.radioName);
    if (reported) return {watts: reported, source: `reported as ${state.radio.radioName}`};
    return {watts: null, source: state.radio.radioName
      ? `unknown model ${state.radio.radioName}` : "radio has not reported a model yet"};
  }

  // ---- CAT ------------------------------------------------------------------

  async function command(payload) {
    const response = await fetch(RADIO_CMD_URL, {method: "POST",
      headers: {"Content-Type": "application/json"}, body: JSON.stringify(payload)});
    if (!response.ok) throw new Error(`${payload.type} failed (${response.status})`);
    return true;
  }

  // setMode cannot express USB-D, so the data-mode bit goes through the generic
  // civ.raw endpoint exactly as data.js does it: 26 00 <USB> <DATA on> <FILx>.
  async function ensureUsbDataMode() {
    if (state.radio.mode === "USB-D") return;
    const filter = [1, 2, 3].includes(Number(state.radio.filter)) ? Number(state.radio.filter) : 1;
    await command({type: "civ.raw", data: "26000101" + String(filter).padStart(2, "0")});
  }

  // Waits for /state to actually reflect a change. Everything on this page that
  // touches the radio verifies rather than assumes, because a transmission sent
  // on the wrong band or at the wrong power is worse than a missed slot.
  function waitForState(test, timeoutMs = 5000) {
    const started = Date.now();
    return new Promise((resolve, reject) => {
      const tick = async () => {
        await pollState();
        if (test(state.radio)) return resolve(true);
        if (Date.now() - started > timeoutMs) return reject(new Error("the radio did not confirm the change"));
        setTimeout(tick, 400);
      };
      tick();
    });
  }

  // Band and mode only. Power is deliberately NOT written here any more: the
  // radio is the authority on it, and what the message reports is derived from
  // what the radio says it is set to. Writing it back before every slot would
  // silently undo the operator's own knob.
  async function tuneRadio(slot) {
    if (state.radio.tx) throw new Error("the radio is transmitting");
    if (state.radio.frequency !== slot.hz) {
      await command({type: "setFrequency", frequency: String(slot.hz)});
      await waitForState(radio => radio.frequency === slot.hz);
    }
    await ensureUsbDataMode();
  }

  // What the radio is actually set to, and the legal WSPR level that reports it
  // most honestly. rfPower is the 0..255 CI-V level, so the model's full-scale
  // figure is what turns it into watts. The rounding error is surfaced rather
  // than hidden: WSPR levels are 3 dB apart and everyone's propagation data
  // depends on this number being close to true.
  function radioPower() {
    const full = fullPower().watts;
    if (!full) return {watts: null, dbm: null, errorDb: 0};
    const watts = full * state.radio.rfPower / 255;
    const exact = WsprCore.wattsToDbm(watts);
    let dbm = WsprCore.POWER_LEVELS[0];
    for (const level of WsprCore.POWER_LEVELS)
      if (Math.abs(level - exact) < Math.abs(dbm - exact)) dbm = level;
    return {watts, dbm, errorDb: dbm - exact};
  }

  // The one place the page writes power: an explicit SET, never a side effect.
  async function setRadioPower() {
    if (!isLan() || !state.radio.connected || !sessionHeld) {
      state.lastError = "the radio is not reachable"; render(); return;
    }
    dom.powerSet.disabled = true;
    try {
      const power = WsprCore.powerCommand(settings.powerDbm, fullPower().watts);
      await command({type: "civ.raw", data: power.data});
      // rfPower is the decoded 0..255 level; allow a little slack for rounding
      // inside the radio rather than demanding an exact echo.
      await waitForState(radio => Math.abs(radio.rfPower - power.level) <= 2);
      state.lastError = "";
    } catch (error) {
      state.lastError = String(error.message || error);
    }
    dom.powerSet.disabled = false;
    render();
  }

  // ---- schedule -------------------------------------------------------------

  const slotIndexNow = () => {
    const now = new Date();
    return now.getUTCHours() * 2 + (now.getUTCMinutes() >= 30 ? 1 : 0);
  };
  const slotLabel = index =>
    `${String(Math.floor(index / 2)).padStart(2, "0")}:${index % 2 ? "30" : "00"}`;

  // Deterministic but different every half hour, so the beacon does not always
  // land on the same two-minute frame while staying predictable enough to show.
  function frameOffset(slotIndex, dayNumber) {
    if (!settings.randomizeFrame) return 0;
    let hash = (dayNumber * 48 + slotIndex) >>> 0;
    hash = (hash ^ (hash >>> 13)) * 0x5bd1e995 >>> 0;
    return (hash ^ (hash >>> 15)) % settings.periodFrames;
  }

  // The next slot this beacon should transmit in, as UTC ms, or 0 when the
  // schedule is silent for the next 24 hours.
  function nextTransmission(fromUtcMs = utcNow()) {
    for (let step = 0; step < 720; step++) {
      const slotUtcMs = WsprCore.nextSlotUtcMs(fromUtcMs + step * 120000 - 1000);
      const at = new Date(slotUtcMs);
      const index = at.getUTCHours() * 2 + (at.getUTCMinutes() >= 30 ? 1 : 0);
      const slot = settings.slots[index];
      if (!slot) continue;
      const dayNumber = Math.floor(slotUtcMs / 86400000);
      const frameIndex = Math.floor((at.getUTCHours() * 60 + at.getUTCMinutes()) / 2);
      if (frameIndex % settings.periodFrames !== frameOffset(index, dayNumber)) continue;
      return {slotUtcMs, slot, index};
    }
    return null;
  }

  // Same markup and classes as the JS8Call frequency timetable, so the shared
  // stylesheet does the work and the two schedules look like one idea.
  function renderSchedule() {
    const now = slotIndexNow();
    let html = "";
    for (let hour = 0; hour < 24; hour++) {
      html += `<div class="tt-row"><span class="tt-hour">${String(hour).padStart(2, "0")}</span>`;
      for (const index of [hour * 2, hour * 2 + 1]) {
        const slot = settings.slots[index];
        html += `<button class="tt-cell${slot ? " filled" : ""}${index === now ? " now" : ""}"` +
                ` type="button" data-slot="${index}" title="${slotLabel(index)} UTC">` +
                `${slot ? slot.band : "·"}</button>`;
      }
      html += "</div>";
    }
    dom.scheduleGrid.innerHTML = html;
  }

  // The topbar button carries the two things worth glancing at: which band the
  // beacon is on, and how long until it speaks again. There is no ON/OFF switch
  // as there is on JS8Call -- here START runs the schedule and the schedule is
  // the whole programme.
  function renderTimetableButton() {
    const slot = settings.slots[slotIndexNow()];
    const next = state.pendingSlotUtcMs ? state.pendingSlotUtcMs
      : (nextTransmission() || {}).slotUtcMs;
    if (!Object.keys(settings.slots).length) {
      dom.freqTimetableValue.textContent = "NO SCHEDULE";
    } else if (next) {
      const seconds = Math.max(0, Math.round((next - utcNow()) / 1000));
      const clock = `${String(Math.floor(seconds / 60)).padStart(2, "0")}:` +
                    `${String(seconds % 60).padStart(2, "0")}`;
      dom.freqTimetableValue.textContent = `${slot ? slot.band : "—"} · ${clock}`;
    } else {
      dom.freqTimetableValue.textContent = slot ? slot.band : "SILENT";
    }
    const active = Boolean(slot) && state.beacon !== "stopped";
    dom.freqTimetableButton.classList.toggle("active", active);
    dom.freqTimetablePanel.classList.toggle("active", active);
  }

  let editingSlot = null;
  function openSlotPopover(index, cell) {
    editingSlot = index;
    const current = settings.slots[index];
    const bands = WsprCore.PRESETS.map(preset =>
      `<button class="tt-band${current && current.hz === preset.hz ? " current" : ""}"` +
      ` type="button" data-band="${preset.band}" data-hz="${preset.hz}">${preset.band}</button>`).join("");
    dom.schedulePopover.innerHTML =
      `<header><strong>${slotLabel(index)} UTC</strong><small>WSPR dial frequency</small></header>` +
      `<div class="tt-bands">${bands}</div>` +
      `<button class="tt-clear-slot" type="button" data-clear-slot>Clear slot</button>`;
    dom.schedulePopover.hidden = false;
    const panelBox = dom.freqTimetablePanel.getBoundingClientRect();
    const cellBox = cell.getBoundingClientRect();
    dom.schedulePopover.style.left =
      `${Math.max(6, Math.min(cellBox.left - panelBox.left, dom.freqTimetablePanel.clientWidth - dom.schedulePopover.offsetWidth - 6))}px`;
    dom.schedulePopover.style.top = `${cellBox.bottom - panelBox.top + 4}px`;
  }
  function closeSlotPopover() { editingSlot = null; dom.schedulePopover.hidden = true; }

  // ---- AUD1 session ---------------------------------------------------------
  //
  // Aud1WebSocketSession swallows tx-level: it only reacts to hello, tx-ready,
  // tx-state, tx-drained and tx-error. tx-level is what paces the credit loop, so
  // it has to be observed -- and a subclass does that without touching the shared
  // file that JS8LAN also depends on.
  class WsprSession extends Js8Aud1Transport.Aud1WebSocketSession {
    constructor(options) { super(options); this.controlCallback = null; }
    onControl(callback) { this.controlCallback = callback; return this; }
    receiveControl(message) {
      super.receiveControl(message);
      if (this.controlCallback) this.controlCallback(message);
    }
    get bufferedAmount() { return this.socket ? this.socket.bufferedAmount : 0; }
  }

  let session = null, tx = null, beaconTimer = null;

  function audioUrl() {
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    return `${scheme}://${location.hostname}:${AUDIO_WS_PORT}/audiows` +
           `?token=${encodeURIComponent(sessionToken())}`;
  }

  function openSession() {
    if (session) return session;
    session = new WsprSession({url: audioUrl(), WebSocketImpl: WebSocket, wallNow: utcNow})
      .onStatus(status => { if (status.type === "closed") render(); })
      .onControl(message => { if (tx) tx.onControl(message); render(); });
    session.onSamples(onSamples);
    session.start();
    // The stream identity comes from the firmware's hello and is re-minted on
    // every reconnect, so WsprTx reads it per transmission rather than being
    // handed a constant it would then send to a radio that rejects it.
    tx = new WsprTx.WsprTx({sink: session, onEvent: onTxEvent, wallNow: utcNow,
                            streamId: () => (session && session.hello
                                             ? session.hello.streamId : 0)});
    return session;
  }

  // ---- waterfall ------------------------------------------------------------
  //
  // Read-only by design. WSPR randomises its offset inside the 200 Hz window for
  // every transmission, so there is nothing here for the operator to choose; the
  // marker reports where the signal went, and the two lines say where the window
  // is. Unlike JS8Call the analyser keeps running through transmission: the radio
  // sends RX audio while keyed, so with the monitor on this is the only sight the
  // operator gets of their own signal.
  const waterfall = new Spectrum.Waterfall({
    canvas: dom.waterfallCanvas, overlay: dom.waterfallOverlay, container: dom.waterfall,
    sampleRate: AUDIO_RATE, lowHz: RX_LOW, highHz: RX_HIGH,
    drawOverlay: (context, view) => drawWindowOverlay(context, view),
  });

  function onSamples(samples) {
    let sum = 0;
    for (const value of samples) sum += value * value;
    state.audioDb = 20 * Math.log10(Math.sqrt(sum / Math.max(1, samples.length)) + 1e-9);
    waterfall.ingest(samples);
  }

  function drawWindowOverlay(context, view) {
    const width = dom.waterfallOverlay.width, height = dom.waterfallOverlay.height;
    const transmitting = Boolean(tx && tx.ptt);

    // The WSPR window: everything outside it is dimmed, so the 200 Hz that
    // matter are obvious inside a 2200 Hz display.
    const left = view.hzToX(WINDOW_LOW_HZ, width), right = view.hzToX(WINDOW_HIGH_HZ, width);
    context.fillStyle = "rgba(2,6,14,.45)";
    context.fillRect(0, 0, left, height);
    context.fillRect(right, 0, width - right, height);
    context.strokeStyle = "rgba(120,220,200,.65)"; context.lineWidth = 1;
    context.setLineDash([3, 3]);
    for (const x of [left, right]) {
      context.beginPath();
      context.moveTo(Math.round(x) + .5, 0); context.lineTo(Math.round(x) + .5, height);
      context.stroke();
    }
    context.setLineDash([]);
    context.fillStyle = "rgba(190,220,214,.72)"; context.font = "bold 9px monospace";
    context.fillText("WSPR 1400–1600", Math.max(3, left - 46), 10);

    // Where this beacon transmits. Dotted while idle because it is then history,
    // solid while keyed because it is then live.
    const marker = view.hzToX(state.lastOffsetHz, width);
    context.strokeStyle = transmitting ? "#ff1838" : "rgba(255,24,56,.55)";
    context.lineWidth = 2;
    context.setLineDash(transmitting ? [] : [4, 4]);
    context.beginPath();
    context.moveTo(marker, 0); context.lineTo(marker, height);
    context.stroke();
    context.setLineDash([]);

    if (transmitting) {
      context.fillStyle = "rgba(255,24,56,.14)";
      context.fillRect(0, 0, width, height);
      context.fillStyle = "#ff8fa3"; context.font = "bold 11px monospace";
      context.fillText(`TX ${state.lastOffsetHz} Hz — not receive audio`, 6, height - 6);
    }
  }

  function closeSession() {
    if (tx) { try { tx.reset(); } catch (_error) {} tx = null; }
    if (session) { session.stop(); session = null; }
  }

  function onTxEvent(event) {
    // stopBeacon() aborts the transmission on purpose, which arrives back here as
    // a failure. Without this guard the handler re-arms the beacon and STOP does
    // not stop -- it just skips one slot.
    if (state.beacon === "stopped") return;
    // A tune carrier ends with a deliberate abort, and stopTune() leaves the
    // tuning state before it fires -- so anything still arriving here while
    // tuning is a genuine fault. It must reach the operator; running it through
    // the beacon's failure policy instead would arm a beacon nobody started.
    if (state.beacon === "tuning") {
      if (event.type !== "failed") return;
      if (tuneTimer) { clearTimeout(tuneTimer); tuneTimer = null; }
      state.lastError = event.reason || "the tune carrier failed";
      state.beacon = "stopped";
      render();
      return;
    }
    if (event.type === "completed") {
      state.consecutiveBroken = 0;
      state.beacon = "armed";
      recordSession({completed: true});
    } else if (event.type === "failed") {
      recordSession({completed: false, afterKeying: event.afterKeying, reason: event.reason});
      // Only a failure after keying means a truncated signal actually went out;
      // a missed slot before keying is ordinary on this link and must not stop a
      // beacon. Three broken ones in a row is systemic.
      if (event.afterKeying) {
        state.consecutiveBroken += 1;
        if (state.consecutiveBroken >= 3) {
          state.beacon = "paused";
          state.lastError = `paused after three broken transmissions (${event.reason})`;
        } else state.beacon = "armed";
      } else state.beacon = "armed";
      state.lastError = state.lastError || event.reason;
    }
    render();
  }

  // ---- activity log ---------------------------------------------------------

  async function refreshActivity() {
    try {
      state.activity = await WsprLog.all();
      renderActivity();
    } catch (_error) { /* private mode or no IndexedDB: the beacon still runs */ }
  }

  // Every attempt is written, including the ones that never keyed. A beacon that
  // only logs its successes cannot tell the operator that half its slots are
  // being lost.
  async function recordSession({completed, afterKeying, reason}) {
    const session = state.currentSession;
    state.currentSession = null;
    if (!session) return;
    const powerMeterRaw = session.powerSamples.length ? Math.max(...session.powerSamples) : 0;
    const record = {
      slotUtcMs: session.slotUtcMs, band: session.band, dialHz: session.dialHz,
      offsetHz: session.offsetHz, callsign: session.callsign, locator: session.locator,
      dbm: session.dbm, powerMeterRaw,
      ulawSent: tx ? Math.floor(tx.sentSamples / 6) : 0,
      status: WsprLog.classify({completed, afterKeying, powerMeterRaw,
                                referenceRaw: settings.powerReferenceRaw}),
      reason: completed ? "" : String(reason || ""),
    };
    try { await WsprLog.record(record); } catch (_error) {}
    await refreshActivity();
  }

  // Two resolutions, because one geometry cannot serve both questions. Up to a
  // day the operator is asking "which slots did I lose", which needs a cell per
  // two-minute slot; over a week they are asking "when does this band work",
  // which only needs an hour. The old 24 x 28 grid answered neither and filled
  // the screen doing it.
  const ACTIVITY_RANGES = {
    "1h": {hours: 1}, "6h": {hours: 6}, "24h": {hours: 24}, "7d": {days: 7},
  };
  const activityRange = () => ACTIVITY_RANGES[state.activityRange] || ACTIVITY_RANGES["24h"];

  function activityCells() {
    const range = activityRange();
    return range.hours
      ? {slotView: true, ...WsprLog.summariseSlots(state.activity, {hours: range.hours, nowUtcMs: utcNow()})}
      : {slotView: false, ...WsprLog.summarise(state.activity, {days: range.days, nowUtcMs: utcNow()})};
  }

  function renderActivity() {
    const range = activityRange();
    const view = activityCells();
    const totals = WsprLog.totals(state.activity,
      range.hours ? {hours: range.hours, nowUtcMs: utcNow()} : {days: range.days, nowUtcMs: utcNow()});
    const summary = `${totals.sent} sent · ${totals.suspect} unconfirmed · ` +
                    `${totals.missed} missed · ${totals.broken} broken`;
    dom.activityTotals.textContent = summary;
    dom.activitySummary.textContent = summary;

    const stamp = utcMs => new Date(utcMs).toISOString().replace("T", " ").slice(0, 16);
    let html = "";
    view.cells.forEach((row, rowIndex) => {
      const label = view.slotView
        ? new Date(row[0].hourUtcMs).toISOString().slice(11, 13)
        : String(rowIndex).padStart(2, "0");
      html += `<div class="activity-row"><span class="activity-hour">${label}</span>`;
      row.forEach((cell, columnIndex) => {
        const when = view.slotView ? stamp(cell.slotUtcMs)
          : `${new Date(cell.dayUtcMs).toISOString().slice(0, 10)} ${label}:00`;
        const detail = cell.records.length
          ? Object.entries(cell.counts).map(([status, count]) => `${count} ${status}`).join(", ")
          : "nothing scheduled";
        html += `<button class="activity-cell ${cell.status}` +
                `${state.selectedCell === `${rowIndex}:${columnIndex}` ? " selected" : ""}"` +
                ` type="button" data-row="${rowIndex}" data-column="${columnIndex}"` +
                ` title="${when} UTC — ${detail}"></button>`;
      });
      html += "</div>";
    });
    dom.activityGrid.classList.toggle("slot-view", view.slotView);
    dom.activityGrid.innerHTML = html;
  }

  function showCellDetail(rowIndex, columnIndex) {
    const view = activityCells();
    const cell = view.cells[rowIndex] && view.cells[rowIndex][columnIndex];
    if (!cell || !cell.records.length) {
      state.selectedCell = null;
      dom.activityDetail.hidden = true;
      renderActivity();
      return;
    }
    state.selectedCell = `${rowIndex}:${columnIndex}`;
    const heading = view.slotView
      ? new Date(cell.slotUtcMs).toISOString().replace("T", " ").slice(0, 16)
      : `${new Date(cell.dayUtcMs).toISOString().slice(0, 10)} ` +
        `${String(rowIndex).padStart(2, "0")}:00`;
    const rows = cell.records
      .slice().sort((a, b) => a.slotUtcMs - b.slotUtcMs)
      .map(record => `<tr>` +
        `<td>${new Date(record.slotUtcMs).toISOString().slice(11, 19)}</td>` +
        `<td>${escapeHtml(record.band || "")}</td>` +
        `<td>${record.dbm} dBm</td>` +
        `<td>${record.offsetHz || "--"} Hz</td>` +
        `<td class="status-${record.status}">${record.status}</td>` +
        `<td class="reason">${escapeHtml(record.reason || "")}</td></tr>`).join("");
    dom.activityDetail.hidden = false;
    dom.activityDetail.innerHTML =
      `<h3>${heading} UTC</h3>` +
      `<table><thead><tr><th>slot</th><th>band</th><th>power</th><th>offset</th>` +
      `<th>status</th><th>reason from the firmware</th></tr></thead><tbody>${rows}</tbody></table>`;
    renderActivity();
  }

  const escapeHtml = text => String(text).replace(/[&<>"]/g,
    c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));

  // ---- beacon ---------------------------------------------------------------

  function startBeacon() {
    const problem = blockingReason();
    if (problem) { state.lastError = problem; render(); return; }
    state.lastError = ""; state.consecutiveBroken = 0;
    state.beacon = "armed";
    render();
  }

  // Stopping the beacon no longer closes the audio socket: it is the page's, and
  // the waterfall keeps running between beacon runs. Only losing the session
  // lease or leaving the page closes it.
  function stopBeacon(reason = "") {
    state.beacon = "stopped";
    state.pendingSlotUtcMs = 0;
    if (tx && ["preparing", "waiting-slot", "prebuffering", "streaming"].includes(tx.state))
      tx.fail(reason || "operator stop");
    render();
  }

  async function beaconTick() {
    if (state.beacon !== "armed" || !tx) return;
    if (["preparing", "waiting-slot", "prebuffering", "streaming"].includes(tx.state)) return;

    const next = nextTransmission();
    if (!next) { state.pendingSlotUtcMs = 0; return; }
    state.pendingSlotUtcMs = next.slotUtcMs;
    const untilSlot = next.slotUtcMs - utcNow();
    if (untilSlot > PREPARE_LEAD_MS + 20000) return;      // still far away
    if (untilSlot < PREPARE_LEAD_MS * 0.5) return;        // too late, wait for the next

    // Re-checked here rather than only at START: the operator can reach over and
    // turn the power up hours into an unattended run, and this slot must not go
    // out over the ceiling just because the beacon was armed when it was legal.
    const blocked = blockingReason();
    const dbm = radioPower().dbm;
    try {
      if (blocked) throw new Error(blocked);
      await tuneRadio(next.slot);
      const frame = WsprCore.encode({
        callsign: dom.callsign.value.trim().toUpperCase(),
        locator: sharedGrid(),
        powerDbm: dbm});
      // Randomised inside the 200 Hz WSPR window, standard practice against
      // collisions with other beacons on the same dial frequency.
      const baseHz = WINDOW_LOW_HZ + Math.floor(Math.random() * (WINDOW_HIGH_HZ - WINDOW_LOW_HZ));
      state.lastOffsetHz = baseHz;
      state.currentSession = {
        slotUtcMs: next.slotUtcMs, band: next.slot.band, dialHz: next.slot.hz,
        offsetHz: baseHz, callsign: frame.message.callsign,
        locator: frame.message.transmittedLocator, dbm,
        powerSamples: [],
      };
      tx.queue({symbols: frame.symbols, slotUtcMs: next.slotUtcMs,
                baseHz, amplitude: settings.txGain});
      state.beacon = "transmitting";
    } catch (error) {
      // Failing to tune means the slot passes with nothing radiated. That is a
      // missed slot, and the grid has to say so rather than showing a gap.
      state.currentSession = {
        slotUtcMs: next.slotUtcMs, band: next.slot.band, dialHz: next.slot.hz,
        offsetHz: 0, callsign: dom.callsign.value.trim().toUpperCase(),
        locator: sharedGrid().slice(0, 4), dbm: dbm || 0, powerSamples: [],
      };
      await recordSession({completed: false, afterKeying: false,
                           reason: String(error.message || error)});
      state.lastError = String(error.message || error);
      state.beacon = "armed";
    }
    render();
  }

  // ---- tune -----------------------------------------------------------------
  //
  // A toggle with a hard cap, mirroring the JS8 tune carrier: five seconds is not
  // enough to turn the MOD level knob and watch the meter settle, and a button
  // that cannot be stopped early is one the operator learns not to press.
  let tuneTimer = null;

  async function startTune() {
    const problem = radioBlockingReason();
    if (problem) { state.lastError = problem; render(); return; }
    state.lastError = ""; state.tunePeakRaw = 0;
    try {
      const next = nextTransmission() || {slot: WsprCore.PRESETS[5]};
      await tuneRadio(next.slot);
      // A steady tone in the middle of the WSPR window. The 162 symbols are a
      // full frame's worth of audio; stopTune aborts long before it runs out.
      const symbols = new Uint8Array(162).fill(1);
      tx.queue({symbols, slotUtcMs: utcNow() + TUNE_LEAD_MS, baseHz: 1500,
                amplitude: settings.txGain, leadMs: TUNE_LEAD_MS});
      state.beacon = "tuning";
      tuneTimer = setTimeout(() => stopTune("tune finished"), TUNE_LEAD_MS + TUNE_MAX_MS);
    } catch (error) {
      state.lastError = String(error.message || error);
      state.beacon = "stopped";
    }
    render();
  }

  function stopTune(reason = "operator stop") {
    if (tuneTimer) { clearTimeout(tuneTimer); tuneTimer = null; }
    // Read the reference before aborting: forward power is only polled while
    // keyed, so after the abort this is the last reading from a dead carrier.
    if (state.tunePeakRaw > 0) {
      settings.powerReferenceRaw = state.tunePeakRaw;
      saveSettings();
    }
    // Leave the tuning state first. The abort below comes back through
    // onTxEvent as a failure, and it is one we caused on purpose -- the
    // "stopped" guard there is what tells the two cases apart.
    state.beacon = "stopped";
    if (tx && ["preparing", "waiting-slot", "prebuffering", "streaming"].includes(tx.state))
      tx.fail(reason);
    render();
  }

  // Everything that must be true before this page may key the transmitter at
  // all, whether for a beacon slot or a tune carrier.
  function radioBlockingReason() {
    if (!isLan()) return "the primary radio is not on the ICOM-LAN transport";
    if (!state.radio.connected) return "the radio is not connected";
    if (!sessionHeld || !sessionConfirmed) return "another page holds the radio";
    // Without a hello there is no stream identity, and the firmware rejects
    // every TX packet that carries the wrong one. Saying so beats keying into a
    // transmission the radio will abort.
    if (!session || !session.hello) return "the audio link is not ready";
    if (!txSafetyAccepted()) return "confirm Enable radio TX";
    return "";
  }

  // Everything that must be true before the beacon may key the transmitter.
  function blockingReason() {
    const radio = radioBlockingReason();
    if (radio) return radio;
    if (!fullPower().watts) return "the radio model is unknown, so its power reading cannot be converted to watts";
    const power = radioPower();
    if (!(power.watts > 0.001)) return "the radio is set to zero power";
    // The ceiling is the operator's rule, not the radio's. Refuse rather than
    // turning the power down: this page does not touch the knob by itself.
    if (power.watts > POWER_CEILING_W * 1.01)
      return `the radio is set to ${power.watts.toFixed(1)} W, above the ${POWER_CEILING_W} W ceiling`;
    try {
      WsprCore.validate({callsign: dom.callsign.value, locator: sharedGrid(),
                         powerDbm: power.dbm});
    } catch (error) { return error.message; }
    if (!Object.keys(settings.slots).length) return "the schedule is empty";
    return "";
  }

  // ---- rendering ------------------------------------------------------------

  function render() {
    // Whether ICOM-LAN is configured at all was settled by lan-gate.js before
    // this page booted, so the only thing left to hide the interface for is
    // another page holding the radio.
    dom.beaconInterface.hidden = !dom.sessionBusy.hidden;

    dom.trxFrequencyValue.textContent = state.radio.frequency
      ? (state.radio.frequency / 1e6).toFixed(6) : "--.---.---";
    dom.trxMode.textContent = state.radio.mode || "---";
    dom.radioModel.textContent = state.radio.radioName || "model unknown";
    dom.linkState.textContent = state.radio.connected ? "● LINKED" : "● OFFLINE";
    dom.linkState.classList.toggle("up", state.radio.connected);

    // The audio socket has its own indicator: LINKED above reports CI-V over
    // LAN, and a beacon can sit there "linked" and still be unable to key.
    const audioUp = Boolean(session && session.hello);
    dom.aud1State.textContent = audioUp ? "AUD1 ●" : "AUD1 —";
    dom.aud1State.classList.toggle("up", audioUp);
    dom.trxReconnect.hidden = state.radio.connected;
    dom.lanHealth.textContent =
      `LAN ${state.radio.lanDrops}·${state.radio.lanStalls}·${state.radio.lanFilled}`;
    dom.lanHealth.classList.toggle("warn",
      state.radio.lanDrops > 0 || state.radio.lanFilled > 0);
    dom.timingState.textContent = `clock ${clockCorrectionMs() >= 0 ? "+" : ""}${clockCorrectionMs()} ms`;

    const scale = fullPower();
    dom.fullPowerWatts.textContent = scale.watts ? `${scale.watts} W` : "unknown";
    dom.fullPowerSource.textContent = scale.source;

    // Power is a readout of the radio, so the rounding to a legal WSPR level is
    // shown rather than hidden -- that number goes on the air and into everyone
    // else's propagation data.
    const power = radioPower();
    if (power.watts === null) {
      dom.powerWatts.textContent = "--";
      dom.powerPercent.textContent = "the radio model is unknown";
    } else {
      dom.powerWatts.textContent = power.watts < 1
        ? `${(power.watts * 1000).toFixed(0)} mW` : `${power.watts.toFixed(2)} W`;
      const error = power.errorDb;
      dom.powerPercent.textContent = `reports ${power.dbm} dBm` +
        (Math.abs(error) >= 0.05 ? ` (${error > 0 ? "+" : ""}${error.toFixed(1)} dB)` : " exactly");
    }
    dom.powerWatts.classList.toggle("over",
      power.watts !== null && power.watts > POWER_CEILING_W * 1.01);

    try { dom.locatorTransmitted.textContent = WsprCore.normalizeLocator(sharedGrid()).transmitted; }
    catch (_error) { dom.locatorTransmitted.textContent = "----"; }

    dom.txGainValue.textContent = Number(settings.txGain).toFixed(2);
    dom.powerMeter.textContent = String(state.radio.powerMeterRaw);
    dom.swr.textContent = state.radio.swr ? state.radio.swr.toFixed(1) : "--";
    dom.powerReference.value = settings.powerReferenceRaw
      ? String(settings.powerReferenceRaw) : "not measured";

    dom.txSafety.checked = txSafetyAccepted();
    if (document.activeElement !== dom.clockCorrection)
      dom.clockCorrection.value = String(clockCorrectionMs());
    // What the collapsed SETTINGS header has to answer at a glance: who is on
    // the air, from where, and at what power.
    dom.settingsSummary.textContent =
      `${dom.callsign.value || "no callsign"} · ${dom.locatorTransmitted.textContent} · ` +
      `${power.dbm === null ? "power unknown" : `${power.dbm} dBm`}` +
      `${txSafetyAccepted() ? "" : " · TX not enabled"}`;
    dom.periodHint.textContent = settings.periodFrames === 1
      ? "Transmits in every two-minute frame of a scheduled slot."
      : `Transmits in one of every ${settings.periodFrames} two-minute frames` +
        `${settings.randomizeFrame ? ", picked differently each half hour" : ", always the first"}.`;
    dom.audioLevel.textContent = `${Math.round(state.audioDb)} dBFS`;
    dom.spectrumSummary.textContent =
      `RX ${RX_LOW}–${RX_HIGH} Hz · WSPR window ${WINDOW_LOW_HZ}–${WINDOW_HIGH_HZ} Hz · TX ${state.lastOffsetHz} Hz`;

    const problem = blockingReason();
    const tuning = state.beacon === "tuning";
    const beaconRunning = state.beacon !== "stopped" && !tuning;
    // The two buttons key the same transmitter, so each one locks the other out
    // rather than letting a tune carrier collide with a scheduled slot.
    dom.startStop.disabled = tuning || (!beaconRunning && Boolean(problem));
    dom.startStop.textContent = beaconRunning ? "STOP" : "START";
    dom.startStop.classList.toggle("running", beaconRunning);
    dom.tuneButton.disabled = !tuning && (beaconRunning || Boolean(radioBlockingReason()));
    dom.tuneButton.textContent = tuning ? "STOP" : "TUNE";
    dom.tuneButton.classList.toggle("running", tuning);
    dom.beaconState.textContent = state.beacon;
    dom.beaconState.className = `beacon-state ${state.beacon === "paused" ? "paused"
      : state.beacon === "stopped" ? "" : "running"}`;

    const message = state.lastError || (state.beacon === "stopped" ? problem : "");
    dom.beaconError.hidden = !message;
    dom.beaconError.textContent = message;

    const next = state.pendingSlotUtcMs ? {slotUtcMs: state.pendingSlotUtcMs} : nextTransmission();
    const countdown = next
      ? Math.max(0, Math.round((next.slotUtcMs - utcNow()) / 1000)) : 0;
    if (next) {
      const at = new Date(next.slotUtcMs);
      dom.nextSession.textContent =
        `next slot ${at.toISOString().slice(11, 19)} UTC, in ${countdown} s`;
    } else {
      dom.nextSession.textContent = "no slot scheduled";
    }

    const live = tx && ["prebuffering", "streaming"].includes(tx.state);
    dom.liveSession.hidden = !live;
    dom.pttState.textContent = tx && tx.ptt ? "ON" : "off";
    dom.pttState.classList.toggle("keyed", Boolean(tx && tx.ptt));
    if (live) {
      const snapshot = tx.snapshot();
      dom.sessionProgress.value = Math.round(snapshot.sentSamples / 48);
      dom.ringFill.textContent = `${Math.round(100 * snapshot.ringEstimate / 12288)} %`;
      dom.packetCount.textContent = String(snapshot.sentPackets);
      dom.txSessionSummary.textContent =
        `transmitting ${Math.round(100 * snapshot.sentSamples / WsprCore.SIGNAL_SAMPLES)} %`;
    } else {
      dom.txSessionSummary.textContent = tuning ? "tune carrier"
        : next ? `next in ${countdown} s` : "no slot scheduled";
    }

    renderTimetableButton();
    waterfall.paintOverlay();
  }

  function renderClock() {
    dom.utcClock.textContent = `UTC ${new Date(utcNow()).toISOString().slice(11, 19)}`;
  }

  // ---- wiring ---------------------------------------------------------------

  function populateSelects() {
    // Only levels this page is willing to key at: SET is a write into the radio,
    // and offering 100 W in a beacon's dropdown invites exactly the mistake the
    // ceiling exists to prevent.
    dom.powerDbm.innerHTML = WsprCore.POWER_LEVELS
      .filter(dbm => WsprCore.dbmToWatts(dbm) <= POWER_CEILING_W * 1.01)
      .map(dbm =>
      `<option value="${dbm}">${dbm} dBm · ${WsprCore.dbmToWatts(dbm) < 1
        ? `${(WsprCore.dbmToWatts(dbm) * 1000).toFixed(0)} mW`
        : `${WsprCore.dbmToWatts(dbm).toFixed(WsprCore.dbmToWatts(dbm) < 10 ? 1 : 0)} W`}</option>`).join("");
    dom.powerDbm.value = String(settings.powerDbm);

    dom.periodFrames.innerHTML = Array.from({length: 15}, (_, index) => index + 1)
      .map(frames => `<option value="${frames}">${frames}${frames === 1 ? "st" : "th"} frame</option>`).join("");
    dom.periodFrames.value = String(settings.periodFrames);

    for (const model of Object.keys(WsprCore.RADIO_FULL_POWER_W)) {
      const option = document.createElement("option");
      option.value = model;
      option.textContent = `${model} (${WsprCore.RADIO_FULL_POWER_W[model]} W)`;
      dom.radioModelOverride.append(option);
    }
    dom.radioModelOverride.value = settings.modelOverride;
  }

  function wire() {
    dom.callsign.addEventListener("change", () => {
      const value = dom.callsign.value.trim().toUpperCase();
      dom.callsign.value = value;
      saveShared({myCall: value});
      render();
    });
    dom.locator.addEventListener("change", () => {
      try {
        const locator = WsprCore.parseLocatorInput(dom.locator.value);
        dom.locator.value = locator;
        saveShared({grid: locator});
        dom.stationError.hidden = true;
      } catch (error) {
        dom.stationError.hidden = false;
        dom.stationError.textContent = error.message;
      }
      render();
    });
    dom.powerDbm.addEventListener("change", () => {
      settings.powerDbm = Number(dom.powerDbm.value); saveSettings(); render();
    });
    dom.powerSet.addEventListener("click", setRadioPower);
    dom.clockCorrection.addEventListener("change", () => {
      const value = Math.max(-1000, Math.min(1000, Number(dom.clockCorrection.value) || 0));
      saveShared({clockCorrectionMs: value});
      render();
    });
    dom.radioModelOverride.addEventListener("change", () => {
      settings.modelOverride = dom.radioModelOverride.value; saveSettings(); render();
    });
    dom.txGain.addEventListener("input", () => {
      settings.txGain = Number(dom.txGain.value); render();
    });
    dom.txGain.addEventListener("change", saveSettings);
    dom.periodFrames.addEventListener("change", () => {
      settings.periodFrames = Number(dom.periodFrames.value); saveSettings(); render();
    });
    dom.randomizeFrame.addEventListener("change", () => {
      settings.randomizeFrame = dom.randomizeFrame.checked; saveSettings(); render();
    });
    dom.scheduleClear.addEventListener("click", () => {
      settings.slots = {}; saveSettings(); renderSchedule(); render();
    });
    dom.scheduleGrid.addEventListener("click", event => {
      const cell = event.target.closest("[data-slot]");
      if (!cell) return;
      const index = Number(cell.dataset.slot);
      if (editingSlot === index) { closeSlotPopover(); return; }
      openSlotPopover(index, cell);
    });
    dom.schedulePopover.addEventListener("click", event => {
      const band = event.target.closest("[data-band]");
      if (band && editingSlot !== null) {
        settings.slots[editingSlot] = {band: band.dataset.band, hz: Number(band.dataset.hz)};
        saveSettings(); closeSlotPopover(); renderSchedule(); render();
        return;
      }
      if (event.target.closest("[data-clear-slot]") && editingSlot !== null) {
        delete settings.slots[editingSlot];
        saveSettings(); closeSlotPopover(); renderSchedule(); render();
      }
    });
    document.addEventListener("click", event => {
      if (dom.schedulePopover.hidden) return;
      if (event.target.closest(".tt-popover") || event.target.closest("[data-slot]")) return;
      closeSlotPopover();
    });
    // The schedule now lives in the topbar, as it does on JS8Call.
    dom.freqTimetableButton.addEventListener("click", () => {
      const opening = dom.freqTimetablePanel.hidden;
      dom.freqTimetablePanel.hidden = !opening;
      dom.freqTimetableButton.setAttribute("aria-expanded", String(opening));
      if (opening) renderSchedule(); else closeSlotPopover();
    });
    document.addEventListener("click", event => {
      if (dom.freqTimetablePanel.hidden) return;
      if (event.target.closest(".tt-control")) return;
      dom.freqTimetablePanel.hidden = true;
      dom.freqTimetableButton.setAttribute("aria-expanded", "false");
      closeSlotPopover();
    });
    addEventListener("keydown", event => {
      if (event.key !== "Escape") return;
      closeSlotPopover();
      dom.freqTimetablePanel.hidden = true;
      dom.freqTimetableButton.setAttribute("aria-expanded", "false");
    });
    dom.trxHelpButton.addEventListener("click", () => {
      if (dom.trxHelpDialog.showModal) dom.trxHelpDialog.showModal();
      else dom.trxHelpDialog.setAttribute("open", "");
    });
    // Same endpoint the JS8Call page uses (data.js reconnectRadio): a POST with
    // no body, answered 202 by whichever slot owns the LAN radio.
    dom.trxReconnect.addEventListener("click", async () => {
      dom.trxReconnect.disabled = true;
      try {
        const response = await fetch("/lan/reconnect", {method: "POST"});
        if (!response.ok) throw new Error(`reconnect failed (HTTP ${response.status})`);
        state.lastError = "";
      } catch (error) {
        state.lastError = String(error.message || error);
      }
      dom.trxReconnect.disabled = false;
      render();
    });
    window.addEventListener("resize", () => waterfall.resize());
    dom.txSafety.addEventListener("change", () => {
      saveShared({txSafetyAccepted: dom.txSafety.checked});
      render();
    });
    dom.startStop.addEventListener("click", () => {
      if (state.beacon === "stopped") startBeacon(); else stopBeacon();
    });
    dom.tuneButton.addEventListener("click", () => {
      if (state.beacon === "tuning") stopTune(); else startTune();
    });
    dom.sessionTakeover.addEventListener("click", () => claimSession(true));
    dom.activityDays.addEventListener("change", () => {
      state.activityRange = ACTIVITY_RANGES[dom.activityDays.value] ? dom.activityDays.value : "24h";
      state.selectedCell = null;
      dom.activityDetail.hidden = true;
      renderActivity();
    });
    dom.activityGrid.addEventListener("click", event => {
      const cell = event.target.closest("[data-row]");
      if (!cell) return;
      const key = `${cell.dataset.row}:${cell.dataset.column}`;
      if (state.selectedCell === key) {
        state.selectedCell = null;
        dom.activityDetail.hidden = true;
        renderActivity();
        return;
      }
      showCellDetail(Number(cell.dataset.row), Number(cell.dataset.column));
    });
  }

  // ---- boot -----------------------------------------------------------------

  // Nothing starts until ICOM-LAN is known to be configured -- not the session
  // claim either. The beacon cannot key a radio it has no audio path to, and
  // taking the single-operator lease in that state would lock a working JS8LAN
  // out of the radio for no reason.
  LanGate.gate().then(ready => {
    if (!ready) return;

    loadSettings();
    populateSelects();
    dom.callsign.value = sharedCall();
    dom.locator.value = sharedGrid();
    dom.txGain.value = String(settings.txGain);
    dom.randomizeFrame.checked = settings.randomizeFrame;
    dom.activityDays.value = state.activityRange;
    wire();
    renderSchedule();
    renderActivity();
    renderClock();
    waterfall.resize();
    render();

    // Ninety days of retention keeps the database small enough to be irrelevant
    // (144 transmissions a day is about 800 kB) while still covering a season.
    WsprLog.prune().then(refreshActivity).catch(() => refreshActivity());

    claimSession();
    pollState();
    setInterval(pollState, STATE_POLL_MS);
    setInterval(renderClock, 1000);
    setInterval(() => { renderSchedule(); }, 60000);
    // The beacon's own heartbeat. Slot planning happens here; once a transmission
    // is queued, WsprTx is driven by inbound tx-level messages instead.
    beaconTimer = setInterval(() => {
      beaconTick();
      if (tx) tx.tick();
      // Forward power is only meaningful while keyed, and /state reports it only
      // then (the poller reads SWR/power during TX and S-meter/supply otherwise).
      if (tx && tx.ptt && state.currentSession)
        state.currentSession.powerSamples.push(state.radio.powerMeterRaw);
      // The tune reference is the peak of a carrier, not whatever happened to be
      // polled when the operator let go of the button.
      if (tx && tx.ptt && state.beacon === "tuning")
        state.tunePeakRaw = Math.max(state.tunePeakRaw, state.radio.powerMeterRaw);
      render();
    }, 500);

    // Exposed for tools/wspr-browser-smoke.js, which drives the real page. Its
    // absence is how the harness tells that the gate closed the page.
    globalThis.__wspr = {state, settings, nextTransmission, blockingReason,
                         radioBlockingReason, fullPower, radioPower, render,
                         recordSession, refreshActivity, renderActivity, onTxEvent,
                         renderSchedule, clockCorrectionMs, waterfall,
                         get tx() { return tx; },
                         get sessionHeld() { return sessionHeld && sessionConfirmed; }};
  });
})();
