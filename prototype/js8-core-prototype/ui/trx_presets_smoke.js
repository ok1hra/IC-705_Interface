#!/usr/bin/env node
// PROTOTYPE — deterministic check of the future TRX tune request boundary.

const Trx = require("./trx_presets.js");
const request = Trx.createFrequencyRequest("40m", "trx-7");
const checks = {
  completeCatalogue: Trx.PRESETS.length === 12,
  upstreamEdges: Trx.PRESETS[0].frequencyHz === 1843500 &&
    Trx.PRESETS[11].frequencyHz === 144178000,
  displayFormat: Trx.formatFrequency(14078000) === "14.078.000",
  productionContract: request.type === "setFrequency" &&
    request.frequency === 7078000 && request.reqId === "trx-7",
  invalidRejected: false,
};
try { Trx.createFrequencyRequest("bogus", "trx-8"); }
catch (_error) { checks.invalidRejected = true; }
const pass = Object.values(checks).every(Boolean);
console.log(`TRX PRESETS ${pass ? "PASS" : "FAIL"} ${JSON.stringify({checks})}`);
if (!pass) process.exitCode = 1;
