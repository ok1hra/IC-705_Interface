// PROTOTYPE — drives the classic worker message protocol via worker_threads.

const fs = require("fs");
const path = require("path");
const {Worker} = require("worker_threads");

function readPackets(file) {
  const dump = fs.readFileSync(file);
  const packets = [];
  let offset = 0;
  while (offset < dump.length) {
    const length = dump.readUInt32BE(offset);
    offset += 4;
    const wire = Uint8Array.from(dump.subarray(offset, offset + length));
    offset += length;
    const view = new DataView(wire.buffer);
    packets.push({wire, arrivalMs: Number(view.getBigUint64(24, false)) / 8});
  }
  return packets;
}

function request(worker, message) {
  return new Promise((resolve, reject) => {
    const receive = (response) => {
      worker.off("error", fail);
      if (response.type === "error") reject(new Error(response.message));
      else resolve(response);
    };
    const fail = (error) => {
      worker.off("message", receive);
      reject(error);
    };
    worker.once("message", receive);
    worker.once("error", fail);
    worker.postMessage(message);
  });
}

async function main() {
  const prototypeDir = path.resolve(process.argv[2]);
  const dumpPath = path.resolve(process.argv[3]);
  const worker = new Worker(path.join(prototypeDir, "wasm/worker_entry.js"));
  const ready = await request(worker, {
    type: "init",
    runtimeJs: path.join(prototypeDir, "wasm/worker_runtime.js"),
    portableJs: path.join(prototypeDir, "build-wasm/js8-prototype.js"),
    decoderJs: path.join(prototypeDir, "build-decoder-wasm/js8-decoder.js"),
    protocolJs: path.join(prototypeDir, "protocol/protocol_runtime.js"),
    jscPath: path.join(prototypeDir, "build-protocol/jsc-map.bin"),
    anchorUtcMs: 0,
  });
  if (ready.type !== "ready") throw new Error("worker did not become ready");

  const packets = readPackets(dumpPath);
  const started = process.hrtime.bigint();
  await request(worker, {type: "audioBatch", packets});
  const finished = await request(worker, {type: "finish"});
  const elapsedMs = Number(process.hrtime.bigint() - started) / 1e6;
  const state = finished.state;
  const expectedRaw = process.argv[4];
  const expectedMode = process.argv[5] === undefined ? null : Number(process.argv[5]);
  if (expectedRaw && !state.frames.some(frame => frame.raw === expectedRaw &&
      (expectedMode === null || frame.submode === expectedMode)))
    throw new Error(`missing expected Worker frame ${expectedRaw} mode=${expectedMode}`);
  console.log(`WORKER packets=${packets.length} samples12k=${state.totalSamples12k} ` +
              `windows=${state.windows} by_mode=${JSON.stringify(state.windowsByMode)} ` +
              `frames=${state.frames.length} elapsed_ms=${elapsedMs.toFixed(1)} ` +
              `decoder_memory=${state.decoderMemoryBytes}`);
  for (const frame of state.frames)
    console.log(`WORKER FRAME mode=${frame.submode} snr=${frame.snr} ` +
                `dt_ms=${frame.dtMs} freq=${frame.offsetHz} raw="${frame.raw}"`);
  if (state.activity) {
    console.log(`WORKER ACTIVITY calls=${state.activity.calls.map(v => v.call).join(",")} ` +
                `messages=${state.activity.messages.length}`);
    for (const message of state.activity.messages)
      console.log(`WORKER MESSAGE mode=${message.submode} freq=${message.offsetHz} ` +
                  `text="${message.text}"`);
  }
  if (expectedRaw) console.log(`WORKER EXPECTED mode=${expectedMode} raw="${expectedRaw}" PASS`);
  await request(worker, {type: "close"});
  await worker.terminate();
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
