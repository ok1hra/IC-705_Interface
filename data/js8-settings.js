// PROTOTYPE — versioned, validated settings store for the compact JS8 UI.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8Settings = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  const STORAGE_KEY = "ic705.data.js8-settings";
  const SCHEMA_VERSION = 7;
  const MODEMS = ["js8call", "rtty45", "psk31", "ft8", "ft4", "cw"];
  const SPEEDS = ["AUTO", "A", "B", "C", "E", "I"];
  const DISCLOSURES = ["spectrum", "reply", "traffic", "stations", "inbox",
    "settings", "timing"];
  // Must match UNATTENDED_ARM_CHOICES_H in unattended_guard.h.
  const ARM_HOURS = [1, 6, 12, 24, 168];
  // Must match INTERVAL_CHOICES_MS in js8-heartbeat.js.
  const HB_MINUTES = [5, 10, 15, 30, 60];
  // @ALLCALL and @HB are always joined: the first is how anyone addresses
  // everybody, the second is what makes the heartbeat network work at all.
  const ALWAYS_GROUPS = ["@ALLCALL", "@HB"];
  const GROUP_RE = /^@[A-Z0-9/]{1,8}$/;
  // Minutes between repeated CQ calls; 0 means the repeat is off.
  const CQ_REPEAT_MIN = [0, 2, 5, 10, 15];

  const clone = value => JSON.parse(JSON.stringify(value));
  const clamp = (value, low, high, fallback) => {
    const number = Number(value);
    return Number.isFinite(number) ? Math.max(low, Math.min(high, number)) : fallback;
  };

  function defaults() {
    return {schemaVersion: SCHEMA_VERSION, activeModem: "js8call",
      modems: {js8call: {myCall: "", grid: "", speed: "AUTO",
        txOffsetHz: 1500, clockCorrectionMs: 0, autoTiming: true,
        followSpeed:true, txGain:0.25, txSafetyAccepted:false,
        // Unattended operation. `auto` is the operator's switch; `armHours` is
        // how long it stays on before the firmware lets it lapse by itself.
        auto:false, armHours:1, infoText:"", statusText:"",
        hb:false, hbAck:true, hbMinutes:15, groups:[], cqRepeatMin:0}},
      ui: {disclosures: {spectrum: true, reply: true, traffic: false,
        stations: false, inbox: false, settings: false, timing: false}}};
  }

  function normalize(input) {
    const fallback = defaults();
    const source = input && typeof input === "object" ? input : {};
    const js8 = source.modems && source.modems.js8call || {};
    const call = String(js8.myCall ?? fallback.modems.js8call.myCall)
      .toUpperCase().replace(/[^A-Z0-9/]/g, "").slice(0, 12);
    const gridCandidate = String(js8.grid ?? fallback.modems.js8call.grid)
      .toUpperCase().trim();
    const grid = /^[A-R]{2}[0-9]{2}(?:[A-X]{2})?$/.test(gridCandidate)
      ? gridCandidate : fallback.modems.js8call.grid;
    const speed = SPEEDS.includes(js8.speed) ? js8.speed : "AUTO";
    const disclosures = {};
    for (const key of DISCLOSURES)
      disclosures[key] = typeof source.ui?.disclosures?.[key] === "boolean"
        ? source.ui.disclosures[key] : fallback.ui.disclosures[key];
    return {schemaVersion: SCHEMA_VERSION,
      activeModem: MODEMS.includes(source.activeModem)
        ? source.activeModem : "js8call",
      modems: {js8call: {myCall: call, grid,
        speed, txOffsetHz: Math.round(clamp(js8.txOffsetHz, 500, 2700, 1500)),
        clockCorrectionMs: Math.round(clamp(js8.clockCorrectionMs,
          -1000, 1000, 0)), autoTiming: js8.autoTiming !== false,
        followSpeed:js8.followSpeed !== false,
        txGain:clamp(js8.txGain, 0.1, 0.8, 0.25),
        txSafetyAccepted:js8.txSafetyAccepted === true,
        auto:js8.auto === true,
        armHours:ARM_HOURS.includes(Number(js8.armHours)) ? Number(js8.armHours) : 1,
        // Free text answered to INFO?/STATUS?; kept short so the reply fits
        // one frame, and stripped of characters the protocol cannot pack.
        infoText:String(js8.infoText ?? "").toUpperCase()
          .replace(/[^A-Z0-9 ./?+-]/g, "").slice(0, 40).trim(),
        statusText:String(js8.statusText ?? "").toUpperCase()
          .replace(/[^A-Z0-9 ./?+-]/g, "").slice(0, 40).trim(),
        hb:js8.hb === true, hbAck:js8.hbAck !== false,
        hbMinutes:HB_MINUTES.includes(Number(js8.hbMinutes)) ? Number(js8.hbMinutes) : 15,
        // Custom groups only; the always-joined ones are added at use time so a
        // stored profile can never accidentally drop them.
        groups:Array.isArray(js8.groups)
          ? [...new Set(js8.groups
              .map(g => String(g || "").toUpperCase().trim())
              .map(g => g.startsWith("@") ? g : `@${g}`)
              .filter(g => GROUP_RE.test(g) && !ALWAYS_GROUPS.includes(g)))].slice(0, 8)
          : [],
        cqRepeatMin:CQ_REPEAT_MIN.includes(Number(js8.cqRepeatMin)) ? Number(js8.cqRepeatMin) : 0}},
      ui: {disclosures}};
  }

  function migrate(input) {
    if (!input || typeof input !== "object")
      return {settings: defaults(), status: "invalid"};
    if (input.schemaVersion === SCHEMA_VERSION)
      return {settings: normalize(input), status: "loaded"};
    if (input.schemaVersion === 1 || input.version === 1) {
      return {settings: normalize({activeModem: input.activeModem,
        modems: {js8call: {myCall: input.myCall, grid: input.grid,
          speed: input.speed, txOffsetHz: input.txOffsetHz,
          clockCorrectionMs: input.clockCorrectionMs,
          autoTiming: input.autoTiming}},
        ui: {disclosures: input.disclosures}}), status: "migrated-v1"};
    }
    if (input.schemaVersion === 2)
      return {settings:normalize(input), status:"migrated-v2"};
    if (input.schemaVersion === 3)
      return {settings:normalize(input), status:"migrated-v3"};
    if (input.schemaVersion === 4)
      return {settings:normalize(input), status:"migrated-v4"};
    if (input.schemaVersion === 5)
      return {settings:normalize(input), status:"migrated-v5"};
    if (input.schemaVersion === 6)
      return {settings:normalize(input), status:"migrated-v6"};
    if (Number(input.schemaVersion) > SCHEMA_VERSION)
      return {settings: defaults(), status: "unsupported-future"};
    return {settings: defaults(), status: "invalid"};
  }

  function label(status) {
    return ({default: "Defaults · not saved", loaded: "Saved locally",
      saved: "Saved locally", reset: "Defaults restored",
      "migrated-v1": "Migrated from schema v1",
      "migrated-v2": "Migrated from schema v2",
      "migrated-v3": "Migrated from schema v3",
      "migrated-v4": "Migrated from schema v4",
      "migrated-v5": "Migrated from schema v5",
      "migrated-v6": "Migrated from schema v6",
      corrupt: "Invalid saved data · defaults used",
      invalid: "Invalid saved data · defaults used",
      "unsupported-future": "Newer schema ignored",
      "storage-unavailable": "Storage unavailable"})[status] || status;
  }

  function load(storage) {
    try {
      const raw = storage && storage.getItem(STORAGE_KEY);
      if (!raw) return {settings: defaults(), status: "default",
        label: label("default")};
      let parsed;
      try { parsed = JSON.parse(raw); }
      catch (_error) {
        return {settings: defaults(), status: "corrupt", label: label("corrupt")};
      }
      const result = migrate(parsed);
      if (result.status.startsWith("migrated-"))
        storage.setItem(STORAGE_KEY, JSON.stringify(result.settings));
      return {...result, label: label(result.status)};
    } catch (_error) {
      return {settings: defaults(), status: "storage-unavailable",
        label: label("storage-unavailable")};
    }
  }

  function save(storage, input) {
    const settings = normalize(input);
    try {
      storage.setItem(STORAGE_KEY, JSON.stringify(settings));
      return {settings, status: "saved", label: label("saved")};
    } catch (_error) {
      return {settings, status: "storage-unavailable",
        label: label("storage-unavailable")};
    }
  }

  function reset(storage) {
    try { storage.removeItem(STORAGE_KEY); }
    catch (_error) {
      return {settings: defaults(), status: "storage-unavailable",
        label: label("storage-unavailable")};
    }
    return {settings: defaults(), status: "reset", label: label("reset")};
  }

  return {STORAGE_KEY, SCHEMA_VERSION, ARM_HOURS, HB_MINUTES, CQ_REPEAT_MIN, ALWAYS_GROUPS, defaults, normalize, migrate, load,
    save, reset, clone};
});
