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
  // AUD1_TX_RING_SIZE in the sketch: the firmware's TX audio ring, 1.536 s of
  // 8 kHz mu-law. This is the margin the browser keeps ahead of the radio, and
  // the thing that runs out when a transmission dies of "TX buffer underrun".
  const TX_RING_BYTES = 12288;
  const TX_RING_SECONDS = TX_RING_BYTES / 8000;
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
    "fullPowerSource", "clockCorrection", "powerField", "powerMismatch",
    "txGain", "txSafety", "tuneButton", "powerMeter", "swr", "tuneReference",
    "referenceCount", "referenceClear",
    "periodFrames", "periodHint", "randomizeFrame", "scheduleClear", "scheduleUndo",
    "scheduleGrid", "schedulePopover", "previewTitle", "previewCount", "previewGrid",
    "previewNext", "startStop", "beaconState", "nextSession", "liveSession",
    "sessionProgress", "ringFill", "packetCount", "pttState", "beaconError",
    "activityDays", "activityTotals", "activityGrid", "activityDetail", "plannedLegend",
    // shell shared with the JS8Call page
    "waterfall", "waterfallCanvas", "waterfallOverlay", "spectrumSummary", "audioLevel",
    "aud1State", "lanHealth", "trxReconnect", "trxHelpButton", "trxHelpDialog",
    "trxHelpModeWarning", "timingState", "trxFrequency", "frequencyMenu",
    "trxSlotLabel", "slotTimer",
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
  // The densest schedule the page offers: one transmission in three two-minute
  // frames, so a single beacon can never occupy more than a third of a WSPR dial
  // frequency it shares with everyone else.
  const MIN_PERIOD_FRAMES = 3;
  // How far off a WSPR dial frequency the radio may sit before the page decides
  // the operator needs the setup help rather than a silent refusal.
  const DIAL_TOLERANCE_HZ = 500;

  const state = {
    radio: {connected: false, transceiverType: "", radioName: "", mode: "", frequency: 0,
            tx: false, powerMeterRaw: 0, swr: 0, rfPower: 0, supplyVolts: 0,
            lanDrops: 0, lanStalls: 0, lanFilled: 0},
    beacon: "stopped",          // stopped | armed | tuning | transmitting | paused
    consecutiveBroken: 0,
    tunePeakRaw: 0,             // highest forward-power reading seen during TUNE
    tuneBand: "",               // band that peak belongs to, filed with it
    tuneDbm: null,              // and the level the radio was on while measuring
    tuneEndsAtMs: 0,            // when the tune watchdog will cut the carrier
    lastError: "",
    pendingSlotUtcMs: 0,
    currentSession: null,     // what the in-flight transmission will be logged as
    activity: [],             // records loaded from IndexedDB, newest first
    activityRange: "6h",
    selectedCell: null,
    audioDb: -90,
    lastOffsetHz: 1500,       // where the last (or current) transmission landed
    pendingFrequency: 0,      // a hand-tuned band waiting for the radio to confirm
    helpShownForSetup: false, // latch: the setup dialog opens once per bad state
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

  // Shared for the same reason as the pledge: it is one MOD input on one radio.
  // Both modes are constant-envelope -- WSPR is a single tone, JS8 is FSK -- so
  // the same level gives the same ALC reading, and two stored values only meant
  // the drive depended on which page transmitted last, invisibly.
  function txGain() {
    const settings = loadShared();
    const value = settings.modems && settings.modems.js8call
      ? Number(settings.modems.js8call.txGain) : NaN;
    return Number.isFinite(value) ? Math.min(0.8, Math.max(0.1, value)) : 0.25;
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

  // One formatter for every countdown on the page. A sparse schedule can put the
  // next transmission most of a day away, and mm:ss without an hour rollover
  // turned that into "720:00" -- a number that looks like a fault.
  // A running clock rather than a phrase: minutes always padded, hours only when
  // there are any. Used by the timer beside START, where a jumping width would
  // make the button move.
  function formatClock(seconds) {
    const total = Math.max(0, Math.round(seconds));
    const pad = value => String(value).padStart(2, "0");
    const hours = Math.floor(total / 3600);
    return hours ? `${hours}:${pad(Math.floor(total / 60) % 60)}:${pad(total % 60)}`
                 : `${pad(Math.floor(total / 60))}:${pad(total % 60)}`;
  }

  function formatDuration(seconds) {
    const total = Math.max(0, Math.round(seconds));
    if (total < 60) return `${total} s`;
    const pad = value => String(value).padStart(2, "0");
    const minutes = Math.floor(total / 60) % 60, hours = Math.floor(total / 3600);
    return hours ? `${hours}:${pad(minutes)}:${pad(total % 60)}`
                 : `${minutes}:${pad(total % 60)}`;
  }

  const SETTINGS_VERSION = 2;

  // `powerDbm: null` means "the operator has never chosen", which is what lets
  // the 1 % rule apply exactly once. Version 1 wrote 30 into storage as its
  // default, so a saved number there proves nothing -- hence the migration below
  // throws it away rather than mistaking a default for a decision.
  const settingsDefaults = () => ({
    version: SETTINGS_VERSION, powerDbm: null, modelOverride: "",
    powerReferences: {}, periodFrames: 5, randomizeFrame: true, slots: {},
  });

  let settings = settingsDefaults();

  function loadSettings() {
    try {
      const raw = JSON.parse(localStorage.getItem(SETTINGS_KEY) || "null");
      if (raw && typeof raw === "object") settings = {...settingsDefaults(), ...raw};
    } catch (_error) { settings = settingsDefaults(); }
    migrateSettings();
    if (settings.powerDbm !== null && !WsprCore.POWER_LEVELS.includes(settings.powerDbm))
      settings.powerDbm = null;
    // Never below MIN_PERIOD_FRAMES, including for settings saved by an older
    // build: one beacon taking a third of the frames on a WSPR dial frequency is
    // already a lot, and every frame would simply sit on top of everyone else.
    settings.periodFrames =
      Math.min(15, Math.max(MIN_PERIOD_FRAMES, Number(settings.periodFrames) || 5));
    if (!settings.slots || typeof settings.slots !== "object") settings.slots = {};
    if (!settings.powerReferences || typeof settings.powerReferences !== "object")
      settings.powerReferences = {};
  }

  // v1 -> v2. All three dropped values are dropped for the same reason: they
  // cannot be converted honestly. `txGain` now lives in the shared blob, the
  // scalar reference has no band and so cannot be filed under one, and a saved
  // `powerDbm` cannot be told apart from v1's own default.
  function migrateSettings() {
    if (Number(settings.version) >= SETTINGS_VERSION) return;
    delete settings.txGain;
    delete settings.powerReferenceRaw;
    settings.powerDbm = null;
    settings.powerReferences = {};
    settings.version = SETTINGS_VERSION;
    saveSettings();
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

  // A beacon's opening bid is one percent of the transmitter, not a fixed dBm:
  // 30 dBm is a tenth of an IC-705 but a hundredth of an IC-7610, and only one
  // of those is a sensible place to start. It lands on a legal WSPR level
  // exactly for every model in the table (10 W -> 20, 100 W -> 30, 200 W -> 33),
  // so nothing is rounded away.
  const DEFAULT_POWER_FRACTION = 0.01;

  function defaultPowerDbm(fullWatts) {
    if (!(fullWatts > 0)) return null;
    const wanted = WsprCore.wattsToDbm(fullWatts * DEFAULT_POWER_FRACTION);
    let best = null;
    for (const level of WsprCore.POWER_LEVELS) {
      if (WsprCore.dbmToWatts(level) > POWER_CEILING_W * 1.01) continue;
      if (best === null || Math.abs(level - wanted) < Math.abs(best - wanted)) best = level;
    }
    return best;
  }

  // The target for SET. Never written to the radio by itself -- the whole point
  // of the invariant is that power only changes when the button is pressed --
  // so an unchosen target is just a proposal until the operator agrees with it.
  function targetDbm() {
    if (settings.powerDbm !== null) return settings.powerDbm;
    return defaultPowerDbm(fullPower().watts);
  }

  // Do the radio and the target agree? Compared on the raw 0..255 CI-V level,
  // the same tolerance setRadioPower() uses to confirm a write landed: the
  // rounded dBm would call it a match with the radio half a decibel away.
  function powerMismatch() {
    const full = fullPower().watts;
    const target = targetDbm();
    if (!full || target === null || !isLan() || !state.radio.connected) return null;
    let level;
    try { level = WsprCore.powerCommand(target, full).level; }
    catch (_error) { return null; }
    return Math.abs(state.radio.rfPower - level) <= 2 ? null : {target, level};
  }

  // ---- TUNE power references ------------------------------------------------
  //
  // One reference per band and target level, not one for the whole beacon. The
  // forward-power reading depends on the band, the antenna and the tuner, and a
  // schedule spanning three bands would otherwise mark two of them unconfirmed
  // for the rest of the night on the strength of a tune on the third.
  const referenceKey = (band, dbm) => `${band || "?"}|${dbm}`;

  function referenceFor(band, dbm) {
    const entry = settings.powerReferences[referenceKey(band, dbm)];
    return entry && entry.raw > 0 ? entry.raw : 0;
  }

  function storeReference(band, dbm, raw) {
    settings.powerReferences[referenceKey(band, dbm)] =
      {raw, dbm, at: Date.now(), band: band || ""};
    saveSettings();
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

  // ---- manual band choice ---------------------------------------------------
  //
  // The dial menu from the JS8Call page, with WSPR frequencies. It opens at any
  // time, but the presets only act while the beacon is stopped: during a run the
  // schedule tunes the radio a few seconds before every slot, so a manual choice
  // would be silently undone and the operator left wondering.
  const beaconStopped = () => state.beacon === "stopped";

  function renderFrequencyMenu() {
    const selected = state.pendingFrequency || state.radio.frequency;
    const locked = !beaconStopped();
    dom.frequencyMenu.innerHTML =
      `<header><strong>WSPR dial frequencies</strong><small>${locked
        ? "the schedule is driving the radio" : "choose a band to tune the TRX"}</small></header>` +
      `<div class="frequency-presets">${WsprCore.PRESETS.map(preset =>
        `<button class="frequency-preset${preset.hz === selected ? " current" : ""}"` +
        ` type="button" data-frequency="${preset.hz}"${locked ? " disabled" : ""}>` +
        `<strong>${preset.band}</strong><span>${(preset.hz / 1e6).toFixed(4)} MHz</span></button>`).join("")}</div>` +
      `<footer>${locked
        ? "Stop the beacon to tune by hand; while it runs the schedule sets the band before each slot."
        : "Standard WSPR dial frequencies. The beacon sets USB-D itself."}</footer>`;
  }

  async function requestFrequency(hz) {
    dom.frequencyMenu.hidden = true;
    dom.trxFrequency.setAttribute("aria-expanded", "false");
    state.pendingFrequency = hz;
    render();
    try {
      await command({type: "setFrequency", frequency: String(hz)});
      await waitForState(radio => radio.frequency === hz);
      // Tuning by hand also prepares the radio, exactly as the JS8Call page does:
      // a WSPR dial frequency in the wrong mode is not a usable state.
      await ensureUsbDataMode();
      state.lastError = "";
    } catch (error) {
      state.lastError = String(error.message || error);
    }
    state.pendingFrequency = 0;
    render();
  }

  // ---- setup help -----------------------------------------------------------
  //
  // A radio that is neither in a data mode nor anywhere near a WSPR dial
  // frequency has not been set up for this page at all -- most likely it is
  // still where the operator left it after voice or JS8. Rather than refuse
  // quietly at the first slot, open the setup card that says what to change.
  //
  // Both conditions have to hold: USB-D on a random frequency is a legitimate
  // mid-tune state, and so is sitting on a WSPR frequency in USB while the
  // beacon is about to set the mode itself.
  function offDialFrequency() {
    if (!state.radio.frequency) return false;
    return !WsprCore.PRESETS.some(preset =>
      Math.abs(preset.hz - state.radio.frequency) <= DIAL_TOLERANCE_HZ);
  }

  function maybeOfferSetupHelp() {
    const unusable = state.radio.connected && Boolean(state.radio.mode) &&
                     state.radio.mode !== "USB-D" && offDialFrequency();
    // Latched: it opens on the transition into that state, not on every poll,
    // so dismissing it does not start a fight with the one-second poller.
    if (unusable && !state.helpShownForSetup) openSetupHelp("mode");
    state.helpShownForSetup = unusable;
  }

  function openSetupHelp(reason = "manual") {
    dom.trxHelpModeWarning.hidden = reason !== "mode";
    if (dom.trxHelpDialog.open) return;
    if (typeof dom.trxHelpDialog.showModal === "function") dom.trxHelpDialog.showModal();
    else dom.trxHelpDialog.setAttribute("open", "");
  }

  // The one place the page writes power: an explicit SET, never a side effect.
  async function setRadioPower() {
    if (!isLan() || !state.radio.connected || !sessionHeld) {
      state.lastError = "the radio is not reachable"; render(); return;
    }
    const target = targetDbm();
    if (target === null) {
      state.lastError = "the radio model is unknown"; render(); return;
    }
    dom.powerSet.disabled = true;
    try {
      const power = WsprCore.powerCommand(target, fullPower().watts);
      await command({type: "civ.raw", data: power.data});
      // rfPower is the decoded 0..255 level; allow a little slack for rounding
      // inside the radio rather than demanding an exact echo.
      await waitForState(radio => Math.abs(radio.rfPower - power.level) <= 2);
      // Pressing SET is the decision the 1 % proposal was waiting for, so the
      // level stops being a suggestion and starts being the operator's choice.
      settings.powerDbm = target; saveSettings();
      state.lastError = "";
    } catch (error) {
      state.lastError = String(error.message || error);
    }
    dom.powerSet.disabled = false;
    render();
  }

  // ---- schedule -------------------------------------------------------------

  const slotIndexNow = () => WsprCore.slotIndexAt(utcNow());
  const slotLabel = WsprCore.slotLabel;

  // Which frame the beacon keys in is arithmetic three views have to agree on, so
  // it lives in wspr-core and nothing here re-derives it: the countdown below, the
  // preview strip in the panel and the activity block's future all ask the same
  // function. A preview with its own copy of the rule is a preview that can lie.
  const nextTransmission = (fromUtcMs = utcNow()) =>
    WsprCore.nextTransmission(fromUtcMs, settings);

  // Editing state for the panel: which slot's popover is open, which span the
  // next write covers, the slots that span is about to touch, and where the click
  // being handled started (see the capture-phase listener that fills it in).
  let editingSlot = null, spanChoice = "slot", pendingSlots = new Set();
  let clickOrigin = {timetable: false, popover: false};

  // Every write wider than one slot is undoable in one step -- including Clear,
  // which until now wiped all 48 slots with nothing to fall back on. The
  // snapshot is a JSON string and lives only while the panel is open.
  let scheduleUndo = null;

  function writeSlots(targets, slot) {
    if (targets.length > 1) scheduleUndo = JSON.stringify(settings.slots);
    for (const index of targets) {
      if (slot) settings.slots[index] = {band: slot.band, hz: slot.hz};
      else delete settings.slots[index];
    }
    saveSettings();
  }

  function undoSchedule() {
    if (scheduleUndo === null) return;
    try { settings.slots = JSON.parse(scheduleUndo); } catch (_error) { return; }
    scheduleUndo = null;
    saveSettings(); closeSlotPopover(); renderSchedule(); render();
  }

  // Same markup and classes as the JS8Call frequency timetable, so the shared
  // stylesheet does the work and the two schedules look like one idea. The four
  // columns of six hours and the top-to-bottom order are the .compact modifier in
  // wspr.css doing it in CSS -- the rows here stay in plain time order.
  function renderSchedule() {
    const now = slotIndexNow();
    let html = "";
    for (let hour = 0; hour < 24; hour++) {
      html += `<div class="tt-row"><span class="tt-hour">${String(hour).padStart(2, "0")}</span>`;
      for (const index of [hour * 2, hour * 2 + 1]) {
        const slot = settings.slots[index];
        html += `<button class="tt-cell${slot ? " filled" : ""}${index === now ? " now" : ""}` +
                `${pendingSlots.has(index) ? " pending" : ""}"` +
                ` type="button" data-slot="${index}" title="${slotLabel(index)} UTC">` +
                `${slot ? slot.band : "·"}</button>`;
      }
      html += "</div>";
    }
    dom.scheduleGrid.innerHTML = html;
    renderPreview();
    // The activity block continues this same axis into the future, so its hollow
    // tail is a function of the schedule and of the time -- and both of those
    // change here, on every edit and on the minute tick. Redrawing it anywhere
    // else meant the prediction only appeared when something unrelated (a range
    // switch) happened to rebuild the grid, which is what made a freshly filled
    // schedule look like it had planned nothing at all.
    renderActivity();
  }

  // ---- what the schedule is going to do -------------------------------------
  //
  // The grid says which band a half hour uses; it cannot say which two-minute
  // frame inside it will key, because that comes out of the period and the
  // randomised offset. That missing minute is the whole reason the schedule feels
  // unpredictable, so it gets drawn: the activity block's raster, one row per
  // hour and thirty frames across, hollow because it has not happened yet.

  const PREVIEW_HOURS = 6, FRAMES_PER_HOUR = 30, HOUR_MS = 3600000;

  function renderPreview() {
    if (dom.freqTimetablePanel.hidden) return;
    const now = utcNow();
    const firstHour = Math.floor(now / HOUR_MS) * HOUR_MS;
    const until = firstHour + PREVIEW_HOURS * HOUR_MS;
    const planned = WsprCore.plannedFrames(now, (until - now) / HOUR_MS, settings);

    let html = "", edgeDrawn = false, marked = 0;
    for (let row = 0; row < PREVIEW_HOURS; row++) {
      const rowStart = firstHour + row * HOUR_MS;
      html += `<div class="tt-preview-row">` +
              `<b>${new Date(rowStart).toISOString().slice(11, 13)}</b>`;
      for (let frame = 0; frame < FRAMES_PER_HOUR; frame++) {
        const frameUtcMs = rowStart + frame * WsprCore.FRAME_MS + 1000;
        const past = frameUtcMs + WsprCore.FRAME_MS <= now;
        const edge = !past && !edgeDrawn;
        if (edge) edgeDrawn = true;
        const keys = past ? null : WsprCore.frameTransmission(frameUtcMs, settings);
        if (keys) marked++;
        html += `<i class="tt-frame${keys ? " planned" : ""}${past ? " past" : ""}` +
                `${edge ? " edge" : ""}" title="${new Date(frameUtcMs).toISOString().slice(11, 16)}` +
                ` UTC${keys ? ` — ${keys.slot.band}` : ""}"></i>`;
      }
      html += "</div>";
    }
    dom.previewGrid.innerHTML = html;

    // A plan the beacon cannot currently carry out is still worth showing -- it is
    // usually being built while the beacon is stopped -- but it must not be dressed
    // up as what is going to happen.
    const blocked = state.beacon === "stopped" ? "beacon stopped" : blockingReason();
    dom.previewTitle.textContent = blocked
      ? `would transmit · next ${PREVIEW_HOURS} h — ${blocked}`
      : `planned · next ${PREVIEW_HOURS} h`;
    dom.previewTitle.classList.toggle("hypothetical", Boolean(blocked));
    // Counted from the cells that were actually drawn, not from `planned`. A frame
    // already running is marked in the strip but is no longer "next", so the two
    // differ by one exactly when the beacon is mid-transmission -- and a head that
    // says 34 above 35 hollow cells is the kind of detail that costs trust.
    dom.previewCount.textContent = `${marked} TX`;
    dom.previewNext.textContent = planned.length
      ? `next ${planned.slice(0, 3).map(frame =>
          `${new Date(frame.slotUtcMs).toISOString().slice(11, 16)} ${frame.slot.band}`)
          .join(" · ")}`
      : Object.keys(settings.slots).length
        ? `nothing in the next ${PREVIEW_HOURS} hours`
        : "the schedule is empty — click a slot to give it a band";
  }

  // The topbar button carries the two things worth glancing at: which band the
  // beacon is on, and how long until it speaks again. There is no ON/OFF switch
  // as there is on JS8Call -- here START runs the schedule and the schedule is
  // the whole programme.
  function renderTimetableButton() {
    // The band shown is the one the countdown belongs to -- the NEXT
    // transmission's. Showing the current half hour's band next to a countdown
    // aimed at a different slot said "20m" while it was about to key on 40.
    const upcoming = nextTransmission();
    if (!Object.keys(settings.slots).length) {
      dom.freqTimetableValue.textContent = "NO SCHEDULE";
    } else if (upcoming) {
      const at = state.pendingSlotUtcMs || upcoming.slotUtcMs;
      dom.freqTimetableValue.textContent =
        `${upcoming.slot.band} · ${formatDuration((at - utcNow()) / 1000)}`;
    } else {
      dom.freqTimetableValue.textContent = "SILENT";
    }
    const active = Boolean(upcoming) && state.beacon !== "stopped";
    dom.freqTimetableButton.classList.toggle("active", active);
    dom.freqTimetablePanel.classList.toggle("active", active);
  }

  // The slots the next click will write. Only marked when the span is wider than
  // the clicked slot: highlighting the one cell already under the cursor says
  // nothing, while a "+6h" from 21:00 reaching into tomorrow morning has to be
  // visible before it overwrites it.
  const spanTargets = () => WsprCore.spanSlots(editingSlot, spanChoice);
  // Toggled in place rather than through renderSchedule(). Rebuilding the grid
  // detaches the cell the operator just clicked, and the document-level "clicked
  // outside the panel" guard then cannot find .tt-control above a detached node --
  // so every click on a slot closed the whole panel.
  function markPending(targets) {
    pendingSlots = new Set(targets.length > 1 ? targets : []);
    for (const cell of dom.scheduleGrid.querySelectorAll("[data-slot]"))
      cell.classList.toggle("pending", pendingSlots.has(Number(cell.dataset.slot)));
  }

  function openSlotPopover(index) {
    editingSlot = index;
    renderSlotPopover();
    dom.schedulePopover.hidden = false;
    positionSlotPopover();
  }

  function renderSlotPopover() {
    if (editingSlot === null) return;
    const current = settings.slots[editingSlot];
    const targets = spanTargets();
    const spans = WsprCore.SPANS.map(span =>
      `<button class="tt-span${span.id === spanChoice ? " current" : ""}"` +
      ` type="button" data-span="${span.id}">${span.label}</button>`).join("");
    const bands = WsprCore.PRESETS.map(preset =>
      `<button class="tt-band${current && current.hz === preset.hz ? " current" : ""}"` +
      ` type="button" data-band="${preset.band}" data-hz="${preset.hz}">${preset.band}</button>`).join("");
    // Spelling the reach out in clock times is what makes the wrap past midnight
    // honest: "+6h" from 21:00 reads 21:00-02:30, not "twelve slots".
    const reach = targets.length > 1
      ? `<small class="tt-span-reach">writes ${targets.length} slots, ` +
        `${slotLabel(targets[0])}–${slotLabel(targets[targets.length - 1])} UTC</small>`
      : "";
    dom.schedulePopover.innerHTML =
      `<header><strong>${slotLabel(editingSlot)} UTC</strong>` +
      `<small>WSPR dial frequency</small></header>` +
      `<div class="tt-span-row"><span>apply to</span>${spans}</div>${reach}` +
      `<div class="tt-bands">${bands}</div>` +
      `<button class="tt-clear-slot" type="button" data-clear-slot>` +
      `${targets.length > 1 ? `Clear ${targets.length} slots` : "Clear slot"}</button>`;
    markPending(targets);
  }

  // Re-queried rather than remembered: renderSchedule() rewrites the grid, so the
  // element a caller could hand over here is detached by the time it is measured.
  function positionSlotPopover() {
    const cell = dom.scheduleGrid.querySelector(`[data-slot="${editingSlot}"]`);
    if (!cell) return;
    const panelBox = dom.freqTimetablePanel.getBoundingClientRect();
    const cellBox = cell.getBoundingClientRect();
    dom.schedulePopover.style.left =
      `${Math.max(6, Math.min(cellBox.left - panelBox.left, dom.freqTimetablePanel.clientWidth - dom.schedulePopover.offsetWidth - 6))}px`;
    dom.schedulePopover.style.top = `${cellBox.bottom - panelBox.top + 4}px`;
  }

  function closeSlotPopover() {
    dom.schedulePopover.hidden = true;
    if (pendingSlots.size) markPending([]);
    editingSlot = null;
  }

  // Closing the panel drops the undo snapshot with it. An UNDO that outlives the
  // editing session it belongs to offers to restore a schedule the operator has
  // stopped thinking about, which is a worse surprise than the one it prevents.
  function closeTimetablePanel() {
    dom.freqTimetablePanel.hidden = true;
    dom.freqTimetableButton.setAttribute("aria-expanded", "false");
    closeSlotPopover();
    scheduleUndo = null;
    dom.scheduleUndo.hidden = true;
  }

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
      // A transmission that worked is the answer to whatever the last one
      // complained about. The banner describes the beacon NOW; the failure
      // itself is not lost -- ACTIVITY keeps it, with its reason, for 90 days.
      // Leaving it up meant an underrun from hours ago still read as a fault.
      state.lastError = "";
      recordSession({completed: true});
    } else if (event.type === "failed") {
      recordSession({completed: false, afterKeying: event.afterKeying, reason: event.reason});
      // Only a failure after keying means a truncated signal actually went out;
      // a missed slot before keying is ordinary on this link and must not stop a
      // beacon. Three broken ones in a row is systemic.
      // The newest event wins. The old `lastError || reason` kept whichever
      // message arrived first, which -- together with never clearing on success
      // -- froze the banner on the first fault the beacon ever hit.
      state.lastError = event.reason;
      if (event.afterKeying) {
        state.consecutiveBroken += 1;
        if (state.consecutiveBroken >= 3) {
          state.beacon = "paused";
          state.lastError = `paused after three broken transmissions (${event.reason})`;
        } else state.beacon = "armed";
      } else state.beacon = "armed";
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
      // A missing reference yields 0, which classify() already reads as "do not
      // check". Not checking is honest; accusing a band that was never tuned is
      // not, and that is exactly what one global reference used to do.
      status: WsprLog.classify({completed, afterKeying, powerMeterRaw,
                                referenceRaw: referenceFor(session.band, session.dbm)}),
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
  const activityRange = () => ACTIVITY_RANGES[state.activityRange] || ACTIVITY_RANGES["6h"];

  // The slot views continue past "now" with what the schedule is going to key, so
  // the same axis answers "what did it do" and "what will it do". The 7-day view
  // does not: with a repeating 24-hour schedule every future day is identical, so
  // it would add area and no information.
  const ACTIVITY_AHEAD_HOURS = 6;

  function activityCells() {
    const range = activityRange();
    return range.hours
      ? {slotView: true, ...WsprLog.summariseSlots(state.activity,
          {hours: range.hours, aheadHours: ACTIVITY_AHEAD_HOURS, nowUtcMs: utcNow()})}
      : {slotView: false, ...WsprLog.summarise(state.activity, {days: range.days, nowUtcMs: utcNow()})};
  }

  function renderActivity() {
    const range = activityRange();
    const view = activityCells();
    const now = utcNow();
    const totals = WsprLog.totals(state.activity,
      range.hours ? {hours: range.hours, nowUtcMs: now} : {days: range.days, nowUtcMs: now});

    const stamp = utcMs => new Date(utcMs).toISOString().replace("T", " ").slice(0, 16);
    let html = "", planned = 0, edgeDrawn = false;
    view.cells.forEach((row, rowIndex) => {
      const label = view.slotView
        ? new Date(row[0].hourUtcMs).toISOString().slice(11, 13)
        : String(rowIndex).padStart(2, "0");
      html += `<div class="activity-row"><span class="activity-hour">${label}</span>`;
      row.forEach((cell, columnIndex) => {
        const when = view.slotView ? stamp(cell.slotUtcMs)
          : `${new Date(cell.dayUtcMs).toISOString().slice(0, 10)} ${label}:00`;
        // A frame still running counts as present, not past, so the "now" mark
        // lands on the frame after it rather than inside it.
        const future = view.slotView && cell.slotUtcMs + WsprCore.FRAME_MS > now;
        const keys = future ? WsprCore.frameTransmission(cell.slotUtcMs, settings) : null;
        const edge = future && !edgeDrawn;
        if (edge) edgeDrawn = true;
        if (keys) planned++;
        const detail = keys ? `planned ${keys.slot.band}`
          : cell.records.length
            ? Object.entries(cell.counts).map(([status, count]) => `${count} ${status}`).join(", ")
            : "nothing scheduled";
        html += `<button class="activity-cell ${cell.status}${keys ? " planned" : ""}` +
                `${edge ? " edge" : ""}` +
                `${state.selectedCell === `${rowIndex}:${columnIndex}` ? " selected" : ""}"` +
                ` type="button" data-row="${rowIndex}" data-column="${columnIndex}"` +
                ` title="${when} UTC — ${detail}"></button>`;
      });
      html += "</div>";
    });
    dom.activityGrid.classList.toggle("slot-view", view.slotView);
    dom.activityGrid.innerHTML = html;

    const summary = `${totals.sent} sent · ${totals.suspect} unconfirmed · ` +
                    `${totals.missed} missed · ${totals.broken} broken` +
                    (view.slotView ? ` · ${planned} planned` : "");
    dom.activityTotals.textContent = summary;
    dom.activitySummary.textContent = summary;
    // Nothing is drawn hollow in the 7-day view, so the legend must not claim it.
    dom.plannedLegend.hidden = !view.slotView;
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
                baseHz, amplitude: txGain()});
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
      // Which band and which level this carrier is measuring. Filed under the
      // radio's own dBm, not the target, because that is the figure a logged
      // transmission carries -- the two have to be looked up the same way.
      state.tuneBand = next.slot.band;
      state.tuneDbm = radioPower().dbm;
      // A steady tone in the middle of the WSPR window. The 162 symbols are a
      // full frame's worth of audio; stopTune aborts long before it runs out.
      const symbols = new Uint8Array(162).fill(1);
      tx.queue({symbols, slotUtcMs: utcNow() + TUNE_LEAD_MS, baseHz: 1500,
                amplitude: txGain(), leadMs: TUNE_LEAD_MS});
      state.beacon = "tuning";
      // Armed on the same line as the deadline the timer counts down, so the
      // two cannot drift apart. Anywhere earlier and the awaited retune would
      // be spent off the clock the operator is watching.
      tuneTimer = setTimeout(() => stopTune("tune finished"), TUNE_LEAD_MS + TUNE_MAX_MS);
      state.tuneEndsAtMs = utcNow() + TUNE_LEAD_MS + TUNE_MAX_MS;
    } catch (error) {
      state.lastError = String(error.message || error);
      state.beacon = "stopped";
    }
    render();
  }

  function stopTune(reason = "operator stop") {
    if (tuneTimer) { clearTimeout(tuneTimer); tuneTimer = null; }
    state.tuneEndsAtMs = 0;
    // Read the reference before aborting: forward power is only polled while
    // keyed, so after the abort this is the last reading from a dead carrier.
    if (state.tunePeakRaw > 0 && state.tuneDbm !== null)
      storeReference(state.tuneBand, state.tuneDbm, state.tunePeakRaw);
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

    // LAN is exclusive to one of TRX1-3 and the operator chooses which, so the
    // button names it rather than showing an anonymous "TRX" -- same as JS8Call.
    const lanSlot = LanGate.slot ? LanGate.slot() : 0;
    dom.trxSlotLabel.textContent = lanSlot ? `TRX ${lanSlot}` : "TRX";
    dom.trxSlotLabel.title = lanSlot ? `TRX${lanSlot} is the LAN radio` : "TRX";

    const shownHz = state.pendingFrequency || state.radio.frequency;
    dom.trxFrequencyValue.textContent =
      shownHz ? Js8TrxPresets.formatFrequency(shownHz) : "--.---.---";
    dom.trxFrequency.classList.toggle("pending", Boolean(state.pendingFrequency));
    if (!dom.frequencyMenu.hidden) renderFrequencyMenu();
    maybeOfferSetupHelp();
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

    // The target half of the row. Amber says "not written", nothing more: what
    // goes on the air is the readout to its left, so a mismatch never stops a
    // transmission -- one turn of the knob would otherwise silence the beacon
    // for the night. Only the 10 W ceiling refuses to key, and it wears red.
    const target = targetDbm();
    if (target !== null && document.activeElement !== dom.powerDbm)
      dom.powerDbm.value = String(target);
    const mismatch = powerMismatch();
    dom.powerField.classList.toggle("mismatch", Boolean(mismatch));
    dom.powerMismatch.hidden = !mismatch;
    if (mismatch)
      dom.powerMismatch.textContent = power.watts === null
        ? `the radio is not on ${mismatch.target} dBm — press SET`
        : `the radio is on ${power.watts < 1 ? `${(power.watts * 1000).toFixed(0)} mW`
            : `${power.watts.toFixed(2)} W`} — press SET to write ${mismatch.target} dBm`;

    try { dom.locatorTransmitted.textContent = WsprCore.normalizeLocator(sharedGrid()).transmitted; }
    catch (_error) { dom.locatorTransmitted.textContent = "----"; }

    if (document.activeElement !== dom.txGain) dom.txGain.value = String(txGain());
    // Both meters are only polled while the radio is keyed: the firmware asks for
    // the S-meter and the supply voltage instead during RX (the aux poll's
    // `stateTx ? 0x11 : 0x02` and `stateTx ? 0x12 : 0x15` in the sketch), and it
    // never zeroes what it read last. So off key these fields hold the PREVIOUS
    // transmission's numbers, frozen -- and the live session row appears while the
    // page is still prebuffering, before the radio keys, which is exactly when
    // that stale SWR was on screen looking like a measurement.
    // "118" alone reads like watts. "118/255" reads like a position on a dial,
    // which is what it is -- and it makes the reference below it comparable at a
    // glance rather than by arithmetic.
    const metering = state.radio.tx;
    dom.powerMeter.textContent = metering ? `${state.radio.powerMeterRaw}/255` : "--";
    dom.swr.textContent = metering && state.radio.swr ? state.radio.swr.toFixed(1) : "--";
    // While tuning this counts the peak up live; afterwards it shows what is on
    // file for the band the next transmission will use, so the operator can see
    // whether the meter beside it is about to agree.
    const tuneActive = state.beacon === "tuning";
    const referenceBand = tuneActive ? state.tuneBand
      : ((nextTransmission() || {}).slot || {}).band;
    const stored = referenceFor(referenceBand, power.dbm);
    const referenceRaw = tuneActive && state.tunePeakRaw > 0 ? state.tunePeakRaw : stored;
    dom.tuneReference.textContent = referenceRaw ? `${referenceRaw}/255` : "--";

    const bandCount = Object.keys(settings.powerReferences).length;
    dom.referenceCount.textContent = bandCount
      ? `${bandCount} band${bandCount === 1 ? "" : "s"} measured` : "no bands measured";
    dom.referenceClear.disabled = !bandCount;

    dom.txSafety.checked = txSafetyAccepted();
    if (document.activeElement !== dom.clockCorrection)
      dom.clockCorrection.value = String(clockCorrectionMs());
    // What the collapsed SETTINGS header has to answer at a glance: who is on
    // the air, from where, and at what power. The mismatch belongs here too --
    // the section opens collapsed, and an amber row nobody can see is not a
    // warning. The dBm on the left stays the radio's, because that is the truth
    // that goes on the air whether or not the target was ever written.
    dom.settingsSummary.textContent =
      `${dom.callsign.value || "no callsign"} · ${dom.locatorTransmitted.textContent} · ` +
      `${power.dbm === null ? "power unknown" : `${power.dbm} dBm`}` +
      `${mismatch ? ` · target ${mismatch.target} dBm not applied` : ""}` +
      `${txSafetyAccepted() ? "" : " · TX not enabled"}`;
    dom.settingsSummary.classList.toggle("mismatch", Boolean(mismatch));
    // Say what the setting produces, not what it is called. "every 5th frame" is
    // meaningless until it reads as "one transmission per 10 minutes, 20 % of
    // the time" -- which is the number an operator actually reasons about.
    // The second sentence explains randomise in place rather than in yet another
    // pop-up: the answer an operator wants is why the frame moves and whether
    // the countdown can still be trusted, and both fit on one line.
    const perFrames = settings.periodFrames;
    dom.periodHint.textContent =
      `One transmission every ${perFrames * 2} minutes — ` +
      `${(100 / perFrames).toFixed(0)} % of the time — ` +
      `${settings.randomizeFrame ? "in a frame picked differently each half hour"
                                 : "always in the first frame of the period"}. ` +
      (settings.randomizeFrame
        ? "randomise derives that frame from the date and the half-hour slot, so it is "
          + "known in advance and the countdown stays exact, but it is never the same "
          + "frame for long — which keeps the beacon off a permanent collision with "
          + "another station on the same frame."
        : "Without randomise the period is counted from 00:00 UTC, so the beacon keeps "
          + "the same frames day after day — simple to predict, but it will clash with "
          + "any other beacon that picked the same ones.");
    dom.scheduleUndo.hidden = scheduleUndo === null;
    // Cheap while the panel is closed -- renderPreview returns immediately -- and
    // this is what keeps the strip's "would transmit" head honest when the beacon
    // starts or a blocker clears while the panel is open.
    renderPreview();
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
    // No countdown here any more: the timer beside START carries it, and two
    // clocks for one event disagree the moment one of them lags a render.
    dom.nextSession.textContent = next
      ? `next slot ${new Date(next.slotUtcMs).toISOString().slice(11, 19)} UTC`
      : "no slot scheduled";

    const live = tx && ["prebuffering", "streaming"].includes(tx.state);
    dom.liveSession.hidden = !live;
    dom.pttState.textContent = tx && tx.ptt ? "ON" : "off";
    dom.pttState.classList.toggle("keyed", Boolean(tx && tx.ptt));
    if (live) {
      const snapshot = tx.snapshot();
      // Radiated, not sent. The browser runs more than a second ahead of the
      // radio -- it has to, that is what the ring is for -- so a bar fed from
      // sent samples was already part full the instant it appeared. `consumed`
      // is what the firmware has actually clocked out, at 8 mu-law bytes per ms.
      const radiatedMs = Math.min(WsprCore.DURATION_S * 1000, snapshot.consumedUlaw / 8);
      // Tied to PTT, not to the state machine. The radio drops out of TX as soon
      // as it has clocked the last sample, but `tx-drained` only arrives after
      // that -- so a bar driven by state alone sat there full while the operator
      // was already back on receive.
      dom.sessionProgress.value = tx.ptt ? Math.round(radiatedMs) : 0;
      // Seconds as well as percent: the margin before the tone breaks is what
      // the operator is actually judging, and "18 %" does not say whether that
      // is a quarter of a second or two.
      dom.ringFill.textContent =
        `${Math.round(100 * snapshot.ringEstimate / TX_RING_BYTES)} % ` +
        `· ${(TX_RING_SECONDS * snapshot.ringEstimate / TX_RING_BYTES).toFixed(1)} s`;
      dom.packetCount.textContent = String(snapshot.sentPackets);
      dom.txSessionSummary.textContent =
        `transmitting ${Math.round(100 * radiatedMs / (WsprCore.DURATION_S * 1000))} %`;
    } else {
      // Emptied on the way out, so the bar can never reappear part-full at the
      // start of the next transmission.
      dom.sessionProgress.value = 0;
      dom.txSessionSummary.textContent = tuning ? "tune carrier"
        : next ? `next slot ${new Date(next.slotUtcMs).toISOString().slice(11, 19)} UTC`
               : "no slot scheduled";
    }

    renderSlotTimer(next, countdown);

    renderTimetableButton();
    waterfall.paintOverlay();
  }

  // Green while waiting, red while radiating. Two states of one clock rather
  // than two indicators, because at any moment exactly one of them is the answer
  // to "how long until something changes".
  function renderSlotTimer(next, countdown) {
    const keyed = Boolean(tx && tx.ptt);
    if (state.beacon === "stopped") { dom.slotTimer.hidden = true; return; }
    // TUNE is not a slot. It is queued as a full 162-symbol frame so the tone
    // cannot run dry, but the watchdog cuts it after TUNE_MAX_MS -- so counting
    // the frame out here promised a minute and a half of carrier that was never
    // going to be sent. What ends the tune is the only honest number.
    if (state.beacon === "tuning") {
      dom.slotTimer.hidden = !state.tuneEndsAtMs;
      if (!state.tuneEndsAtMs) return;
      dom.slotTimer.textContent = formatClock((state.tuneEndsAtMs - utcNow()) / 1000);
      // Red only once the radio is actually radiating; the lead-in is still a
      // wait, and the colours mean the same thing here as everywhere else.
      dom.slotTimer.className = `slot-timer ${keyed ? "transmitting" : "waiting"}`;
      dom.slotTimer.title = "time until TUNE stops itself";
      return;
    }
    if (keyed && tx.slotUtcMs) {
      const endsAt = tx.slotUtcMs + WsprCore.DURATION_S * 1000;
      dom.slotTimer.hidden = false;
      dom.slotTimer.textContent = formatClock((endsAt - utcNow()) / 1000);
      dom.slotTimer.className = "slot-timer transmitting";
      dom.slotTimer.title = "time left of this transmission";
      return;
    }
    if (!next) { dom.slotTimer.hidden = true; return; }
    dom.slotTimer.hidden = false;
    dom.slotTimer.textContent = formatClock(countdown);
    dom.slotTimer.className = "slot-timer waiting";
    dom.slotTimer.title = "time until the next transmission starts";
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
    // Left blank when the model is still unknown: 1 % of an unknown radio is not
    // a number, and render() fills it in the moment the radio says what it is.
    if (settings.powerDbm !== null) dom.powerDbm.value = String(settings.powerDbm);

    // This is a period, not a position: "every 5th frame" means one transmission
    // in every five two-minute frames. The old label read "2th frame", which was
    // both ungrammatical and describing the wrong thing.
    const ordinal = value => {
      const tens = value % 100, units = value % 10;
      if (tens >= 11 && tens <= 13) return `${value}th`;
      return `${value}${units === 1 ? "st" : units === 2 ? "nd" : units === 3 ? "rd" : "th"}`;
    };
    dom.periodFrames.innerHTML =
      Array.from({length: 16 - MIN_PERIOD_FRAMES}, (_, index) => index + MIN_PERIOD_FRAMES)
      .map(frames => `<option value="${frames}">every ${ordinal(frames)} frame</option>`).join("");
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
    // Written to the shared blob on `change` only: `input` fires on every
    // keystroke, and a half-typed "0." saved as 0 would silently mute JS8Call's
    // drive too. Clamped here rather than trusting the number field, because
    // typing into it can produce anything.
    dom.txGain.addEventListener("change", () => {
      const value = Number(dom.txGain.value);
      saveShared({txGain: Number.isFinite(value)
        ? Math.min(0.8, Math.max(0.1, value)) : 0.25});
      render();
    });
    dom.referenceClear.addEventListener("click", () => {
      settings.powerReferences = {}; saveSettings(); render();
    });
    dom.periodFrames.addEventListener("change", () => {
      settings.periodFrames = Number(dom.periodFrames.value); saveSettings(); render();
    });
    dom.randomizeFrame.addEventListener("change", () => {
      settings.randomizeFrame = dom.randomizeFrame.checked; saveSettings(); render();
    });
    dom.scheduleClear.addEventListener("click", () => {
      writeSlots(Array.from({length: WsprCore.SLOTS_PER_DAY}, (_, slot) => slot), null);
      closeSlotPopover(); renderSchedule(); render();
    });
    dom.scheduleUndo.addEventListener("click", undoSchedule);
    dom.scheduleGrid.addEventListener("click", event => {
      const cell = event.target.closest("[data-slot]");
      if (!cell) return;
      const index = Number(cell.dataset.slot);
      if (editingSlot === index) { closeSlotPopover(); return; }
      openSlotPopover(index);
    });
    dom.schedulePopover.addEventListener("click", event => {
      if (editingSlot === null) return;
      // Picking a span leaves the popover open on purpose: it is the choice that
      // governs the next click, so the operator has to see the marked range and
      // then pick the band, in that order.
      const span = event.target.closest("[data-span]");
      if (span) {
        spanChoice = span.dataset.span;
        renderSlotPopover(); positionSlotPopover();
        return;
      }
      const band = event.target.closest("[data-band]");
      if (band) {
        writeSlots(spanTargets(), {band: band.dataset.band, hz: Number(band.dataset.hz)});
        closeSlotPopover(); renderSchedule(); render();
        return;
      }
      if (event.target.closest("[data-clear-slot]")) {
        writeSlots(spanTargets(), null);
        closeSlotPopover(); renderSchedule(); render();
      }
    });
    document.addEventListener("click", () => {
      if (dom.schedulePopover.hidden) return;
      if (clickOrigin.popover) return;
      closeSlotPopover();
    });
    dom.trxFrequency.addEventListener("click", () => {
      const opening = dom.frequencyMenu.hidden;
      if (opening) renderFrequencyMenu();
      dom.frequencyMenu.hidden = !opening;
      dom.trxFrequency.setAttribute("aria-expanded", String(opening));
    });
    dom.frequencyMenu.addEventListener("click", event => {
      const button = event.target.closest("[data-frequency]");
      if (button && !button.disabled) requestFrequency(Number(button.dataset.frequency));
    });
    // The schedule now lives in the topbar, as it does on JS8Call.
    dom.freqTimetableButton.addEventListener("click", () => {
      const opening = dom.freqTimetablePanel.hidden;
      dom.freqTimetablePanel.hidden = !opening;
      dom.freqTimetableButton.setAttribute("aria-expanded", String(opening));
      if (opening) renderSchedule(); else closeTimetablePanel();
    });
    // Where a click came from has to be decided in the capture phase, before the
    // handlers below rebuild the grid or the popover. Once innerHTML has been
    // replaced the event target is detached, closest() can no longer see the panel
    // above it, and the "clicked outside" logic then closed the panel on every
    // single edit.
    document.addEventListener("click", event => {
      clickOrigin = {
        timetable: Boolean(event.target.closest(".tt-control")),
        popover: Boolean(event.target.closest(".tt-popover") ||
                         event.target.closest("[data-slot]")),
      };
    }, true);
    document.addEventListener("click", () => {
      if (dom.freqTimetablePanel.hidden) return;
      if (clickOrigin.timetable) return;
      closeTimetablePanel();
    });
    document.addEventListener("click", event => {
      if (dom.frequencyMenu.hidden) return;
      if (event.target.closest("#frequencyMenu") || event.target.closest("#trxFrequency")) return;
      dom.frequencyMenu.hidden = true;
      dom.trxFrequency.setAttribute("aria-expanded", "false");
    });
    addEventListener("keydown", event => {
      if (event.key !== "Escape") return;
      closeTimetablePanel();
      dom.frequencyMenu.hidden = true;
      dom.trxFrequency.setAttribute("aria-expanded", "false");
    });
    dom.trxHelpButton.addEventListener("click", () => openSetupHelp("manual"));
    dom.trxHelpDialog.addEventListener("click", event => {
      if (event.target === dom.trxHelpDialog) dom.trxHelpDialog.close();
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
      state.activityRange = ACTIVITY_RANGES[dom.activityDays.value] ? dom.activityDays.value : "6h";
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
    dom.txGain.value = String(txGain());
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
                         targetDbm, defaultPowerDbm, powerMismatch,
                         referenceFor, storeReference, txGain,
                         get tx() { return tx; },
                         get sessionHeld() { return sessionHeld && sessionConfirmed; }};
  });
})();
