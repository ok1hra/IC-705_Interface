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

const checks = {
  defaultGain: Js8Settings.defaults().modems.js8call.txGain === 0.25,
  missingGainFallback: Js8Settings.normalize({}).modems.js8call.txGain === 0.25,
  savedGainPreserved: Js8Settings.load(storage).settings.modems.js8call.txGain === 0.55,
  resetGain: Js8Settings.reset(storage).settings.modems.js8call.txGain === 0.25
};

const pass = Object.values(checks).every(Boolean);
console.log(`JS8 SETTINGS ${pass ? "PASS" : "FAIL"} ${JSON.stringify(checks)}`);
if (!pass) process.exitCode = 1;
