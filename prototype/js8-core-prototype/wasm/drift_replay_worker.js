#!/usr/bin/env node
// PROTOTYPE — replay continuous drifting frames through the auto-speed Worker.

const fs = require("fs");
const path = require("path");
const {Worker} = require("worker_threads");

const JITTER_MS = [0, 7, -5, 12, -9, 3];

function readPackets(file) {
  const dump = fs.readFileSync(file);
  const packets = [];
  for (let offset = 0; offset < dump.length;) {
    const length = dump.readUInt32BE(offset);
    offset += 4;
    const wire = Uint8Array.from(dump.subarray(offset, offset + length));
    offset += length;
    const view = new DataView(wire.buffer, wire.byteOffset, wire.byteLength);
    packets.push({wire,
      arrivalMs: Number(view.getBigUint64(24, false)) / 8 +
        JITTER_MS[packets.length % JITTER_MS.length]});
  }
  return packets;
}

function addDuplicates(packets) {
  const delivered = [];
  let duplicates = 0;
  for (let index = 0; index < packets.length; index += 1) {
    delivered.push(packets[index]);
    if (index > 0 && index % 97 === 0) {
      delivered.push({wire: packets[index].wire,
        arrivalMs: packets[index].arrivalMs + 1});
      duplicates += 1;
    }
  }
  return {delivered, duplicates};
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
    anchorUtcMs: 0});

  const vectors = [];
  for (const vector of manifest.vectors) {
    await request(worker, {type: "reset"});
    const packets = readPackets(path.join(fixtureDir, vector.dump));
    const impaired = addDuplicates(packets);
    const started = process.hrtime.bigint();
    await request(worker, {type: "audioBatch", packets: impaired.delivered});
    const finished = await request(worker, {type: "finish"});
    const elapsedMs = Number(process.hrtime.bigint() - started) / 1e6;
    const state = finished.state;
    const targetFrames = state.frames.filter(frame =>
      frame.raw === manifest.raw && frame.submode === vector.submode);
    const observed = vector.frames.map(expected => {
      const candidates = targetFrames.filter(frame =>
        frame.slotUtcMs === expected.slotUtcMs);
      const frame = candidates.sort((left, right) =>
        Math.abs(left.offsetHz - expected.nativeFrequencyHz) -
        Math.abs(right.offsetHz - expected.nativeFrequencyHz))[0];
      const frequencyErrorHz = frame
        ? Math.abs(frame.offsetHz - expected.nativeFrequencyHz) : null;
      const toleranceHz = Math.max(1, vector.toneSpacingHz * 0.51);
      return {slotUtcMs: expected.slotUtcMs, expectedHz: expected.baseHz,
        decodedHz: frame ? Number(frame.offsetHz.toFixed(3)) : null,
        frequencyErrorHz: frequencyErrorHz === null
          ? null : Number(frequencyErrorHz.toFixed(3)),
        pass: Boolean(frame) && frequencyErrorHz <= toleranceHz};
    });
    const decodedOffsets = observed.map(frame => frame.decodedHz)
      .filter(value => value !== null);
    const driftPass = decodedOffsets.length === manifest.frameCount &&
      decodedOffsets.at(-1) > decodedOffsets[0] &&
      decodedOffsets.at(-1) - decodedOffsets[0] >=
        vector.toneSpacingHz * 0.5;
    const framePass = targetFrames.length === manifest.frameCount &&
      observed.every(frame => frame.pass);
    const messagePass = state.activity.messages.filter(message =>
      message.text === manifest.text &&
      message.submode === vector.submode).length === manifest.frameCount;
    const samplePass = state.totalSamples12k === vector.samples12k;
    const duplicatePass = state.audio.duplicatePackets === impaired.duplicates;
    const jitterPass = state.audio.maxArrivalJitterMs >= 10 &&
      state.audio.maxArrivalJitterMs <= 25;
    const continuityPass = state.audio.sequenceGaps === 0 &&
      state.audio.insertedGapSamples8k === 0;
    const pass = framePass && messagePass && samplePass && duplicatePass &&
      jitterPass && continuityPass && driftPass;
    vectors.push({letter: vector.letter, submode: vector.submode,
      audioSeconds: vector.samples12k / 12000, packets: packets.length,
      deliveredPackets: impaired.delivered.length,
      duplicatePackets: state.audio.duplicatePackets,
      maxArrivalJitterMs: Number(state.audio.maxArrivalJitterMs.toFixed(1)),
      samples12k: state.totalSamples12k, frames: targetFrames.length,
      elapsedMs: Number(elapsedMs.toFixed(1)), observed, framePass, messagePass,
      samplePass, duplicatePass, jitterPass, continuityPass, driftPass, pass});
  }
  await request(worker, {type: "close"});
  await worker.terminate();
  const pass = manifest.nativePass && vectors.every(vector => vector.pass);
  const result = {nativePass: manifest.nativePass, vectors, pass};
  fs.writeFileSync(path.join(fixtureDir, "result.json"),
    `${JSON.stringify(result, null, 2)}\n`);
  console.log(`DRIFT WORKER ${pass ? "PASS" : "FAIL"} ${JSON.stringify(result)}`);
  if (!pass) process.exitCode = 1;
}

main().catch(error => { console.error(error); process.exitCode = 1; });
