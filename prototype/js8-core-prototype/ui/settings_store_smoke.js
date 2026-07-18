#!/usr/bin/env node
// PROTOTYPE — deterministic state story for settings schema and migration.

const Settings = require("./settings_store.js");

class MemoryStorage {
  constructor() { this.values = new Map(); }
  getItem(key) { return this.values.has(key) ? this.values.get(key) : null; }
  setItem(key, value) { this.values.set(key, String(value)); }
  removeItem(key) { this.values.delete(key); }
}

const storage = new MemoryStorage();
const initial = Settings.load(storage);
const saved = Settings.save(storage, {schemaVersion: 2, activeModem: "js8call",
  modems: {js8call: {myCall: " ok1abc ", grid: "jo70aa", speed: "E",
    txOffsetHz: 9999, clockCorrectionMs: -1200, autoTiming: false}},
  ui: {disclosures: {settings: true}}});
const reloaded = Settings.load(storage);
storage.setItem(Settings.STORAGE_KEY, JSON.stringify({version: 1,
  activeModem: "ft8", myCall: "K0OG", grid: "FN31", speed: "A",
  txOffsetHz: 703, clockCorrectionMs: 25, autoTiming: true,
  disclosures: {stations: true}}));
const migrated = Settings.load(storage);
const migrationPersistedV2 = JSON.parse(storage.getItem(Settings.STORAGE_KEY) || "{}")
  .schemaVersion === 2;
storage.setItem(Settings.STORAGE_KEY, "{broken");
const corrupt = Settings.load(storage);
storage.setItem(Settings.STORAGE_KEY, JSON.stringify({schemaVersion: 99}));
const future = Settings.load(storage);
const checks = {
  defaultSchema: initial.settings.schemaVersion === 2,
  emptyIdentityDefaults: initial.settings.modems.js8call.myCall === "" &&
    initial.settings.modems.js8call.grid === "",
  normalizedCall: saved.settings.modems.js8call.myCall === "OK1ABC",
  clampedValues: saved.settings.modems.js8call.txOffsetHz === 2700 &&
    saved.settings.modems.js8call.clockCorrectionMs === -1000,
  reloadStable: reloaded.settings.modems.js8call.speed === "E" &&
    reloaded.settings.modems.js8call.autoTiming === false,
  v1Migrated: migrated.status === "migrated-v1" &&
    migrated.settings.activeModem === "ft8" &&
    migrated.settings.modems.js8call.txOffsetHz === 703,
  migrationPersistedV2,
  corruptSafe: corrupt.status === "corrupt" &&
    corrupt.settings.activeModem === "js8call",
  futureSafe: future.status === "unsupported-future" &&
    future.settings.schemaVersion === 2,
};
const pass = Object.values(checks).every(Boolean);
console.log(`SETTINGS STORE ${pass ? "PASS" : "FAIL"} ` +
  JSON.stringify({schemaVersion: Settings.SCHEMA_VERSION, checks}));
if (!pass) process.exitCode = 1;
