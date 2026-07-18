#!/usr/bin/env node
// PROTOTYPE — strict browser-time/media epoch handshake across reconnect.
// Question: can a new AUD1 stream be decoded on its new UTC slots without
// accepting unanchored or stale-stream audio?

const fs = require("fs");
const path = require("path");
const {Worker} = require("worker_threads");
const {Js8Timebase} = require("../timebase/js8_timebase.js");

const PERIOD_MS = 15000;
const PACKETS_PER_FRAME = PERIOD_MS / 20;
const FRAMES_PER_EPOCH = 2;
const DELAYS_MS = [12, 0, 25, 7, 16, 4, 19];

function readWires(file) {
  const dump = fs.readFileSync(file);
  const wires = [];
  for (let offset = 0; offset < dump.length;) {
    const length = dump.readUInt32BE(offset);
    offset += 4;
    wires.push(Uint8Array.from(dump.subarray(offset, offset + length)));
    offset += length;
  }
  return wires;
}

function rewriteEpoch(source, firstPacket, streamId) {
  const count = PACKETS_PER_FRAME * FRAMES_PER_EPOCH;
  let firstSample = 0n;
  return source.slice(firstPacket, firstPacket + count).map((input, index) => {
    const wire = Uint8Array.from(input);
    const view = new DataView(wire.buffer, wire.byteOffset, wire.byteLength);
    const payloadBytes = view.getUint32(36, false);
    let flags = view.getUint16(6, false) & ~3;
    if (index === 0) flags |= 1;
    if (index + 1 === count) flags |= 2;
    view.setUint16(6, flags, false);
    view.setUint32(12, streamId, false);
    view.setUint32(16, index, false);
    view.setBigUint64(24, firstSample, false);
    firstSample += BigInt(payloadBytes);
    return {wire};
  });
}

function observeEpoch(timebase, packets, streamId, actualAnchorUtcMs,
                      monotonicStartMs) {
  let lockedAnchorUtcMs = null;
  packets.forEach((packet, index) => {
    const view = new DataView(packet.wire.buffer, packet.wire.byteOffset,
                              packet.wire.byteLength);
    const firstSample = Number(view.getBigUint64(24, false));
    const sampleCount = view.getUint32(36, false);
    const mediaEndMs = (firstSample + sampleCount) / 8;
    const delayMs = DELAYS_MS[index % DELAYS_MS.length];
    packet.arrivalMs = monotonicStartMs + mediaEndMs + delayMs;
    timebase.observePacket({streamId, sequence: index, firstSample, sampleCount,
      arrivalMonotonicMs: packet.arrivalMs,
      arrivalWallMs: actualAnchorUtcMs + mediaEndMs + delayMs,
      discontinuity: index === 0});
    if (index === 4)
      lockedAnchorUtcMs = timebase.snapshot().media.anchorUtcSample0Ms;
  });
  if (lockedAnchorUtcMs === null)
    throw new Error("timebase did not lock after five packets");
  return lockedAnchorUtcMs;
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

async function expectRejected(worker, packet, pattern) {
  try {
    await request(worker, {type: "audio", ...packet});
    return false;
  } catch (error) {
    return pattern.test(String(error.message));
  }
}

function epochPass(state, streamId, anchorUtcMs, raw, text) {
  const frames = state.frames.filter(frame =>
    frame.submode === 0 && frame.raw === raw);
  const expectedSlots = [anchorUtcMs, anchorUtcMs + PERIOD_MS];
  const slots = frames.map(frame => frame.slotUtcMs).sort((a, b) => a - b);
  const allMessages = state.activity.messages.filter(message =>
    message.submode === 0 && message.text === text);
  const messages = allMessages.filter(message =>
    message.firstSlotUtcMs >= anchorUtcMs &&
    message.firstSlotUtcMs < anchorUtcMs + FRAMES_PER_EPOCH * PERIOD_MS);
  return {frames: frames.length, messages: messages.length,
    totalMessages: allMessages.length, slots,
    streamPass: state.streamId === streamId && state.epoch.streamId === streamId,
    anchorPass: state.epoch.strict && state.epoch.ready &&
      state.epoch.anchorUtcMs === anchorUtcMs,
    slotPass: JSON.stringify(slots) === JSON.stringify(expectedSlots),
    samplePass: state.totalSamples12k === FRAMES_PER_EPOCH * PERIOD_MS * 12,
    payloadPass: frames.length === FRAMES_PER_EPOCH &&
      messages.length === FRAMES_PER_EPOCH};
}

async function main() {
  const prototypeDir = path.resolve(process.argv[2]);
  const fixtureDir = path.resolve(process.argv[3]);
  const manifest = JSON.parse(fs.readFileSync(
    path.join(fixtureDir, "manifest.json")));
  const source = readWires(path.join(fixtureDir, "A_drift.aud1.bin"));
  const timebase = new Js8Timebase();
  const stream1 = 101;
  const stream2 = 202;
  const actualAnchor1 = 1699999980000;
  const actualAnchor2 = actualAnchor1 + 60000;
  const wallMinusMonotonic = actualAnchor1 - 1000;
  const packets1 = rewriteEpoch(source, 0, stream1);
  const packets2 = rewriteEpoch(source, PACKETS_PER_FRAME * 2, stream2);
  const anchor1 = observeEpoch(timebase, packets1, stream1,
    actualAnchor1, actualAnchor1 - wallMinusMonotonic);
  const anchor2 = observeEpoch(timebase, packets2, stream2,
    actualAnchor2, actualAnchor2 - wallMinusMonotonic);

  const worker = new Worker(path.join(prototypeDir, "wasm/worker_entry.js"));
  const ready = await request(worker, {type: "init",
    runtimeJs: path.join(prototypeDir, "wasm/worker_runtime.js"),
    portableJs: path.join(prototypeDir, "build-wasm/js8-prototype.js"),
    decoderJs: path.join(prototypeDir, "build-decoder-wasm/js8-decoder.js"),
    protocolJs: path.join(prototypeDir, "protocol/protocol_runtime.js"),
    jscPath: path.join(prototypeDir, "build-protocol/jsc-map.bin"),
    anchorUtcMs: 0, strictEpochAnchoring: true});
  const unanchoredRejected = ready.state.epoch.strict &&
    !ready.state.epoch.ready && await expectRejected(worker, packets1[0],
      /not anchored/);

  await request(worker, {type: "epoch", streamId: stream1,
    anchorUtcMs: anchor1});
  await request(worker, {type: "audioBatch", packets: packets1});
  const first = (await request(worker, {type: "finish"})).state;
  const firstResult = epochPass(first, stream1, anchor1,
    manifest.raw, manifest.text);

  await request(worker, {type: "epoch", streamId: stream2,
    anchorUtcMs: anchor2});
  const staleRejected = await expectRejected(worker, packets1.at(-1),
    /does not match anchored epoch/);
  await request(worker, {type: "audioBatch", packets: packets2});
  const second = (await request(worker, {type: "finish"})).state;
  const secondResult = epochPass(second, stream2, anchor2,
    manifest.raw, manifest.text);
  await request(worker, {type: "close"});
  await worker.terminate();

  const timebaseState = timebase.snapshot();
  const timebasePass = anchor1 === actualAnchor1 && anchor2 === actualAnchor2 &&
    timebaseState.media.epoch === 2 &&
    timebaseState.transport.reconnects === 1 &&
    timebaseState.media.streamId === stream2;
  const epochResultsPass = [firstResult, secondResult].every(result =>
    result.streamPass && result.anchorPass && result.slotPass &&
    result.samplePass && result.payloadPass) &&
    firstResult.totalMessages === FRAMES_PER_EPOCH &&
    secondResult.totalMessages === FRAMES_PER_EPOCH * 2;
  const pass = unanchoredRejected && staleRejected && timebasePass &&
    epochResultsPass;
  const result = {unanchoredRejected, staleRejected, anchor1, anchor2,
    first: firstResult, second: secondResult,
    timebase: timebaseState, timebasePass, pass};
  fs.writeFileSync(path.join(fixtureDir, "reconnect-result.json"),
    `${JSON.stringify(result, null, 2)}\n`);
  console.log(`RECONNECT EPOCH ${pass ? "PASS" : "FAIL"} ${JSON.stringify(result)}`);
  if (!pass) process.exitCode = 1;
}

main().catch(error => { console.error(error); process.exitCode = 1; });
