#!/usr/bin/env node
// PROTOTYPE — manifest-driven upstream WAV capture replay in strict Worker mode.

const fs = require("fs");
const path = require("path");
const {Worker} = require("worker_threads");

function readPackets(file) {
  const dump = fs.readFileSync(file);
  const packets = [];
  for (let offset = 0; offset < dump.length;) {
    const length = dump.readUInt32BE(offset);
    offset += 4;
    const wire = Uint8Array.from(dump.subarray(offset, offset + length));
    offset += length;
    const view = new DataView(wire.buffer, wire.byteOffset, wire.byteLength);
    packets.push({wire, arrivalMs: Number(view.getBigUint64(24, false)) / 8});
  }
  return packets;
}

function streamIdOf(packets) {
  if (!packets.length) throw new Error("AUD1 replay is empty");
  const wire = packets[0].wire;
  return new DataView(wire.buffer, wire.byteOffset, wire.byteLength)
    .getUint32(12, false);
}

function request(worker, message) {
  return new Promise((resolve, reject) => {
    const cleanup = () => {
      worker.off("message", receive);
      worker.off("error", fail);
    };
    const receive = response => {
      cleanup();
      if (response.type === "error") reject(new Error(response.message));
      else resolve(response);
    };
    const fail = error => { cleanup(); reject(error); };
    worker.on("message", receive);
    worker.on("error", fail);
    worker.postMessage(message);
  });
}

function matchFrames(expected, actual, submode, slotUtcMs) {
  const candidates = actual.filter(frame => frame.submode === submode &&
    (slotUtcMs === null || frame.slotUtcMs === slotUtcMs));
  const used = new Set();
  const matches = expected.map(reference => {
    let found = -1;
    let errorHz = Infinity;
    candidates.forEach((frame, index) => {
      if (used.has(index) || frame.raw !== reference.raw ||
          frame.frameType !== reference.frameType) return;
      const error = Math.abs(frame.offsetHz - reference.frequencyHz);
      if (error < errorHz) { found = index; errorHz = error; }
    });
    const toleranceHz = submode === 4 ? 2 : 4;
    const pass = found >= 0 && errorHz <= toleranceHz;
    if (pass) used.add(found);
    return {raw: reference.raw, expectedHz: reference.frequencyHz,
      actualHz: found >= 0 ? Number(candidates[found].offsetHz.toFixed(3)) : null,
      errorHz: found >= 0 ? Number(errorHz.toFixed(3)) : null, pass};
  });
  return {expected: expected.length, observed: candidates.length,
    matched: matches.filter(value => value.pass).length, matches,
    pass: matches.every(value => value.pass)};
}

async function main() {
  const prototypeDir = path.resolve(process.argv[2]);
  const fixtureDir = path.resolve(process.argv[3]);
  const manifest = JSON.parse(fs.readFileSync(
    path.join(fixtureDir, "manifest.json")));
  const worker = new Worker(path.join(prototypeDir, "wasm/worker_entry.js"));
  await request(worker, {type: "init",
    runtimeJs: path.join(prototypeDir, "wasm/worker_runtime.js"),
    portableJs: path.join(prototypeDir, "build-wasm/js8-prototype.js"),
    decoderJs: path.join(prototypeDir, "build-decoder-wasm/js8-decoder.js"),
    protocolJs: path.join(prototypeDir, "protocol/protocol_runtime.js"),
    jscPath: path.join(prototypeDir, "build-protocol/jsc-map.bin"),
    anchorUtcMs: 0, strictEpochAnchoring: true});

  const vectors = [];
  for (let index = 0; index < manifest.vectors.length; index += 1) {
    const vector = manifest.vectors[index];
    const packets = readPackets(path.join(fixtureDir, vector.dump));
    await request(worker, {type: "epoch", streamId: streamIdOf(packets),
      anchorUtcMs: 0});
    const started = process.hrtime.bigint();
    await request(worker, {type: "audioBatch", packets});
    const state = (await request(worker, {type: "finish"})).state;
    const elapsedMs = Number(process.hrtime.bigint() - started) / 1e6;
    const frames = matchFrames(vector.frames, state.frames, vector.submode, 0);
    const samplePass = state.totalSamples12k === vector.samples12k;
    vectors.push({id: vector.id, submode: vector.submode,
      upstreamExpected: vector.upstreamExpected, nativeCount: vector.nativeCount,
      nativeMatchesUpstream: vector.nativeMatchesUpstream,
      packets: packets.length, samples12k: state.totalSamples12k,
      elapsedMs: Number(elapsedMs.toFixed(1)), frames, samplePass,
      pass: frames.pass && samplePass});
  }

  const longPackets = readPackets(path.join(fixtureDir,
    manifest.longSequence.dump));
  await request(worker, {type: "epoch", streamId: streamIdOf(longPackets),
    anchorUtcMs: 0});
  const longStarted = process.hrtime.bigint();
  await request(worker, {type: "audioBatch", packets: longPackets});
  const longState = (await request(worker, {type: "finish"})).state;
  const longElapsedMs = Number(process.hrtime.bigint() - longStarted) / 1e6;
  const segmentResults = manifest.vectors.map(vector => ({id: vector.id,
    submode: vector.submode, slotUtcMs: vector.startUtcMs,
    ...matchFrames(vector.frames, longState.frames, vector.submode,
      vector.startUtcMs)}));
  const longResult = {audioSeconds: manifest.longSequence.audioSeconds,
    packets: longPackets.length, samples12k: longState.totalSamples12k,
    frames: longState.frames.length, elapsedMs: Number(longElapsedMs.toFixed(1)),
    segments: segmentResults,
    samplePass: longState.totalSamples12k === manifest.longSequence.samples12k,
    framePass: segmentResults.every(segment => segment.pass)};
  longResult.pass = longResult.samplePass && longResult.framePass;

  await request(worker, {type: "close"});
  await worker.terminate();
  const pass = manifest.nativePass && vectors.every(vector => vector.pass) &&
    longResult.pass;
  const result = {sourceClass: manifest.sourceClass,
    missingReferenceSubmodes: manifest.missingReferenceSubmodes,
    nativeTotal: manifest.nativeTotal,
    upstreamExpectedTotal: manifest.upstreamExpectedTotal,
    upstreamParity: manifest.nativeTotal === manifest.upstreamExpectedTotal,
    vectors, long: longResult, pass};
  fs.writeFileSync(path.join(fixtureDir, "result.json"),
    `${JSON.stringify(result, null, 2)}\n`);
  console.log(`CAPTURE WORKER ${pass ? "PASS" : "FAIL"} ${JSON.stringify(result)}`);
  if (!pass) process.exitCode = 1;
}

main().catch(error => { console.error(error); process.exitCode = 1; });
