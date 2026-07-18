#!/usr/bin/env node
// PROTOTYPE — interactive shell and deterministic stress story for Js8Timebase.

const readline = require("readline");
const {Js8Timebase} = require("./js8_timebase.js");

function createSimulation() {
  return {wall0: 1700000000000, mono0: 1000, wallJumpMs: 0,
    streamId: 1, sequence: 0, nextSample: 0, streamStartMono: 1000,
    currentMono: 1000, lastPacket: null, decodeSlot: 0};
}

function sendPacket(timebase, simulation, delayMs, options = {}) {
  simulation.nextSample += options.skipSamples || 0;
  simulation.sequence += options.skipSequences || 0;
  const sampleCount = 160;
  const arrivalMonotonicMs = simulation.streamStartMono +
    (simulation.nextSample + sampleCount) * 1000 / 8000 + delayMs;
  const packet = {streamId: simulation.streamId,
    sequence: simulation.sequence, firstSample: simulation.nextSample,
    sampleCount, arrivalMonotonicMs,
    arrivalWallMs: simulation.wall0 +
      (arrivalMonotonicMs - simulation.mono0) + simulation.wallJumpMs,
    discontinuity: Boolean(options.discontinuity)};
  const result = timebase.observePacket(packet);
  simulation.lastPacket = packet;
  simulation.currentMono = arrivalMonotonicMs;
  simulation.sequence += 1;
  simulation.nextSample += sampleCount;
  return result;
}

function bootstrap(timebase, simulation, delays = [38, 12, 55, 24, 31]) {
  for (const delay of delays) sendPacket(timebase, simulation, delay);
}

function duplicateLast(timebase, simulation) {
  const packet = {...simulation.lastPacket,
    arrivalMonotonicMs: simulation.lastPacket.arrivalMonotonicMs + 80,
    arrivalWallMs: simulation.lastPacket.arrivalWallMs + 80};
  return timebase.observePacket(packet);
}

function reconnect(timebase, simulation) {
  simulation.streamId += 1;
  simulation.sequence = 0;
  simulation.nextSample = 0;
  simulation.streamStartMono = simulation.currentMono + 100;
  bootstrap(timebase, simulation, [26, 14, 42, 19, 30]);
}

function clockJump(timebase, simulation, jumpMs = 600) {
  simulation.wallJumpMs += jumpMs;
  bootstrap(timebase, simulation, [20, 10, 35, 18, 22]);
}

function timingGroup(timebase, simulation, centerMs, call = "KN4CRD") {
  const values = [centerMs + 5, centerMs - 5, centerMs];
  return values.map(dtMs => timebase.observeDecode({submode: 0, dtMs, call,
    slotUtcMs: simulation.decodeSlot++ * 15000}));
}

function runScenario() {
  const timebase = new Js8Timebase();
  const simulation = createSimulation();
  bootstrap(timebase, simulation);
  const bootstrapAnchorMs = timebase.snapshot().media.anchorUtcSample0Ms;
  const oneSecondSampleUtcMs = timebase.mediaUtcMs(8000);
  duplicateLast(timebase, simulation);
  sendPacket(timebase, simulation, 100, {skipSamples: 160, skipSequences: 1});
  const anchorAfterJitterAndGapMs = timebase.snapshot().media.anchorUtcSample0Ms;
  timebase.confirmClock();
  timebase.setManualCorrection(25);
  timebase.observeDecode({submode: 0, dtMs: 3000, call: "BADCLK",
    slotUtcMs: simulation.decodeSlot++ * 15000});
  for (const dtMs of [500, -300, 100])
    timebase.observeDecode({submode: 0, dtMs, call: "UNSTABLE",
      slotUtcMs: simulation.decodeSlot++ * 15000});
  const correctionAfterInconsistentMs = timebase.autoCorrectionMs;
  for (const residual of [245, 195, 145, 95, 45])
    timingGroup(timebase, simulation, residual);
  const afterTiming = timebase.snapshot();
  reconnect(timebase, simulation);
  const afterReconnect = timebase.snapshot();
  clockJump(timebase, simulation);
  const afterClockJumpUnchecked = timebase.snapshot();
  timebase.confirmClock();
  const final = timebase.snapshot();

  const checks = {
    minimumDelayAnchor: bootstrapAnchorMs === simulation.wall0 + 12,
    jitterDidNotMoveAnchor: anchorAfterJitterAndGapMs === bootstrapAnchorMs,
    sampleIndexedUtc: oneSecondSampleUtcMs === bootstrapAnchorMs + 1000,
    duplicateRejected: final.transport.duplicatePackets === 1,
    gapPreserved: final.transport.gapSamples === 160 &&
      final.transport.sequenceGaps === 1,
    timingConverged: afterTiming.correction.autoMs === 245 &&
      afterTiming.correction.updates === 5,
    inconsistentTimingIgnored: correctionAfterInconsistentMs === 0,
    reconnectReanchored: afterReconnect.media.epoch === 2 &&
      afterReconnect.transport.reconnects === 1 &&
      afterReconnect.correction.autoMs === 245,
    clockJumpReanchored: final.clock.jumps === 1 && final.media.epoch === 3 &&
      final.media.status === "locked" && final.media.slotFlushes === 2,
    clockRequiresReconfirmation: afterClockJumpUnchecked.clock.status ===
      "unchecked" && final.clock.status === "confirmed",
    clockJumpResetAutoOnly: final.correction.autoMs === 0 &&
      final.correction.manualMs === 25,
    outlierIgnored: final.correction.outliers === 1,
  };
  const result = {pass: Object.values(checks).every(Boolean), checks, final,
    checkpoints: {bootstrapAnchorMs, anchorAfterJitterAndGapMs,
      oneSecondSampleUtcMs,
      afterTimingCorrectionMs: afterTiming.correction.totalMs,
      reconnectAnchorMs: afterReconnect.media.anchorUtcSample0Ms,
      finalAnchorMs: final.media.anchorUtcSample0Ms}};
  Object.defineProperty(result, "runtime",
    {value: {timebase, simulation}, enumerable: false});
  return result;
}

function render(timebase, note) {
  console.clear();
  console.log("\x1b[1mPROTOTYPE — JS8 browser/sample timebase\x1b[0m");
  console.log("\x1b[2mQuestion: do jitter, gaps, reconnect and wall-clock jumps preserve slot time?\x1b[0m\n");
  console.log(JSON.stringify(timebase.snapshot(), null, 2));
  if (note) console.log(`\n\x1b[2m${note}\x1b[0m`);
  console.log("\n[p] packet  [g] gap  [u] duplicate  [r] reconnect  [j] +600ms clock jump");
  console.log("[d] stable dt trio  [o] outlier  [m] manual +25ms  [c] confirm clock");
  console.log("[x] reset timing  [s] full stress scenario  [q] quit");
}

if (process.argv.includes("--snapshot")) {
  const result = runScenario();
  console.log(`TIMEBASE ${result.pass ? "PASS" : "FAIL"}`);
  console.log(JSON.stringify(result, null, 2));
  process.exitCode = result.pass ? 0 : 1;
} else if (!process.stdin.isTTY) {
  console.error("interactive timebase prototype requires a TTY; use --snapshot");
  process.exitCode = 2;
} else {
  let timebase = new Js8Timebase();
  let simulation = createSimulation();
  bootstrap(timebase, simulation);
  render(timebase, "five packets selected the minimum-delay anchor");
  readline.emitKeypressEvents(process.stdin);
  process.stdin.setRawMode(true);
  process.stdin.on("keypress", (_text, key) => {
    let note = "";
    if (key.name === "q" || (key.ctrl && key.name === "c")) {
      process.stdin.setRawMode(false);
      process.exit(0);
    } else if (key.name === "p") {
      sendPacket(timebase, simulation, 75); note = "late packet; anchor unchanged";
    } else if (key.name === "g") {
      sendPacket(timebase, simulation, 30,
        {skipSamples: 160, skipSequences: 1}); note = "one missing packet";
    } else if (key.name === "u") {
      duplicateLast(timebase, simulation); note = "duplicate rejected";
    } else if (key.name === "r") {
      reconnect(timebase, simulation); note = "new stream epoch anchored";
    } else if (key.name === "j") {
      clockJump(timebase, simulation); note = "wall-clock jump created new epoch";
    } else if (key.name === "d") {
      timingGroup(timebase, simulation, 120 - timebase.autoCorrectionMs);
      note = "three consistent decode offsets applied one bounded step";
    } else if (key.name === "o") {
      timebase.observeDecode({submode: 0, dtMs: 3000, call: "BADCLK",
        slotUtcMs: simulation.decodeSlot++ * 15000}); note = "outlier ignored";
    } else if (key.name === "m") {
      timebase.setManualCorrection(timebase.manualCorrectionMs + 25);
      note = "manual correction changed";
    } else if (key.name === "c") {
      timebase.confirmClock(); note = "operator confirmed system clock";
    } else if (key.name === "x") {
      timebase.resetTiming(); note = "auto and station timing reset";
    } else if (key.name === "s") {
      const result = runScenario();
      timebase = result.runtime.timebase;
      simulation = result.runtime.simulation;
      note = `stress scenario ${result.pass ? "PASS" : "FAIL"}`;
    }
    render(timebase, note);
  });
}

module.exports = {runScenario};
