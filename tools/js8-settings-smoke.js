#!/usr/bin/env node
"use strict";

const Js8Settings = require("../data/js8-settings.js");

const saved = Js8Settings.defaults();
saved.modems.js8call.txGain = 0.55;

const storage = {
  value: JSON.stringify(saved),
  getItem() { return this.value; },
  setItem(_key, value) { this.value = value; },
  removeItem() { this.value = null; }
};

// A v8 profile with the old fifteen-minute heartbeat, which is what every
// browser that has ever opened the page is holding.
const v8Profile = Js8Settings.defaults();
v8Profile.schemaVersion = 8;
v8Profile.modems.js8call.hbMinutes = 15;
v8Profile.modems.js8call.rfPercent = undefined;
const v8Storage = {
  value: JSON.stringify(v8Profile),
  getItem() { return this.value; },
  setItem(_key, value) { this.value = value; },
  removeItem() { this.value = null; }
};
const migrated = Js8Settings.load(v8Storage);

const checks = {
  defaultGain: Js8Settings.defaults().modems.js8call.txGain === 0.25,
  missingGainFallback: Js8Settings.normalize({}).modems.js8call.txGain === 0.25,
  savedGainPreserved: Js8Settings.load(storage).settings.modems.js8call.txGain === 0.55,
  resetGain: Js8Settings.reset(storage).settings.modems.js8call.txGain === 0.25,

  defaultHbMinutes: Js8Settings.defaults().modems.js8call.hbMinutes === 60,
  missingHbFallback: Js8Settings.normalize({}).modems.js8call.hbMinutes === 60,
  // The whole point of the v9 migration: a stored 15 cannot be told apart from
  // v8's own default, so leaving it alone would mean the new interval never
  // reached anyone who had already opened the page.
  hbMigratedFromV8: migrated.status === "migrated-v8" &&
                    migrated.settings.modems.js8call.hbMinutes === 60,
  // Power starts unchosen, and only 1..100 is settable at all: the radio's step
  // is one percent, so anything below it would be written as a different level.
  defaultRfPercent: Js8Settings.defaults().modems.js8call.rfPercent === null,
  rfPercentUnchosenAfterMigration:
    migrated.settings.modems.js8call.rfPercent === null,
  rfPercentRejectsBelowOneStep:
    Js8Settings.normalize({modems: {js8call: {rfPercent: 0.4}}})
      .modems.js8call.rfPercent === null,
  rfPercentClampedAndRounded:
    Js8Settings.normalize({modems: {js8call: {rfPercent: 130}}})
      .modems.js8call.rfPercent === 100 &&
    Js8Settings.normalize({modems: {js8call: {rfPercent: 29.6}}})
      .modems.js8call.rfPercent === 30
};

const pass = Object.values(checks).every(Boolean);
console.log(`JS8 SETTINGS ${pass ? "PASS" : "FAIL"} ${JSON.stringify(checks)}`);
if (!pass) process.exitCode = 1;
