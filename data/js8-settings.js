// PROTOTYPE — versioned, validated settings store for the compact JS8 UI.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8Settings = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  const STORAGE_KEY = "ic705.data.js8-settings";
  const SCHEMA_VERSION = 3;
  const MODEMS = ["js8call", "rtty45", "psk31", "ft8", "ft4", "cw"];
  const SPEEDS = ["AUTO", "A", "B", "C", "E", "I"];
  const DISCLOSURES = ["spectrum", "reply", "traffic", "stations", "settings",
    "timing"];

  const clone = value => JSON.parse(JSON.stringify(value));
  const clamp = (value, low, high, fallback) => {
    const number = Number(value);
    return Number.isFinite(number) ? Math.max(low, Math.min(high, number)) : fallback;
  };

  function defaults() {
    return {schemaVersion: SCHEMA_VERSION, activeModem: "js8call",
      modems: {js8call: {myCall: "", grid: "", speed: "AUTO",
        txOffsetHz: 1500, clockCorrectionMs: 0, autoTiming: true,
        followSpeed:true, txGain:0.25, txSafetyAccepted:false}},
      ui: {disclosures: {spectrum: true, reply: true, traffic: false,
        stations: false, settings: false, timing: false}}};
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
        txSafetyAccepted:js8.txSafetyAccepted === true}},
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
    if (Number(input.schemaVersion) > SCHEMA_VERSION)
      return {settings: defaults(), status: "unsupported-future"};
    return {settings: defaults(), status: "invalid"};
  }

  function label(status) {
    return ({default: "Defaults · not saved", loaded: "Saved locally",
      saved: "Saved locally", reset: "Defaults restored",
      "migrated-v1": "Migrated from schema v1",
      "migrated-v2": "Migrated from schema v2",
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
      if (result.status === "migrated-v1" || result.status === "migrated-v2")
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

  return {STORAGE_KEY, SCHEMA_VERSION, defaults, normalize, migrate, load,
    save, reset, clone};
});
