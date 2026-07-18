#!/usr/bin/env node
// PROTOTYPE — auto-speed Worker check at each native uLaw sensitivity floor.

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

function request(worker, message) {
  return new Promise((resolve, reject) => {
    const cleanup = () => { worker.off("message", receive); worker.off("error", fail); };
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
  const weakDir = path.resolve(process.argv[3]);
  const manifest = JSON.parse(fs.readFileSync(path.join(weakDir, "manifest.json")));
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
    const packets = readPackets(path.join(weakDir,
      `${vector.letter}_weak_selected.aud1.bin`));
    const started = process.hrtime.bigint();
    await request(worker, {type: "audioBatch", packets});
    const finished = await request(worker, {type: "finish"});
    const elapsedMs = Number(process.hrtime.bigint() - started) / 1e6;
    const state = finished.state;
    const framePass = state.frames.some(frame =>
      frame.raw === manifest.raw && frame.submode === vector.submode);
    const messagePass = state.activity.messages.some(message =>
      message.text === manifest.text && message.submode === vector.submode);
    const samplePass = state.totalSamples12k === vector.samples;
    const pass = framePass && messagePass && samplePass;
    vectors.push({letter: vector.letter, submode: vector.submode,
      injectedSnrDb: vector.ulawFloorDb, packets: packets.length,
      samples12k: state.totalSamples12k, frames: state.frames.length,
      elapsedMs: Number(elapsedMs.toFixed(1)), framePass, messagePass,
      samplePass, pass});
  }
  await request(worker, {type: "close"});
  await worker.terminate();
  const pass = manifest.nativePass && vectors.every(vector => vector.pass);
  console.log(`WEAK WORKER ${pass ? "PASS" : "FAIL"} ` +
    JSON.stringify({nativePass: manifest.nativePass, vectors}));
  if (!pass) process.exitCode = 1;
}

main().catch(error => { console.error(error); process.exitCode = 1; });
