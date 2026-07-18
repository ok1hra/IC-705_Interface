#!/usr/bin/env node
// PROTOTYPE — deterministic AudioSource anchoring/reconnect/continuity story.

const {WsAudioSource} = require("./ws_audio_source.js");

class FakeWebSocket {
  static OPEN = 1;
  static CONNECTING = 0;
  static instances = [];
  constructor() {
    this.readyState = FakeWebSocket.OPEN;
    this.sent = [];
    FakeWebSocket.instances.push(this);
  }
  send(value) { this.sent.push(value); }
  close() {
    if (this.readyState === 3) return;
    this.readyState = 3;
    if (this.onclose) this.onclose();
  }
  async message(data) { return this.onmessage({data}); }
}

function packet(streamId, sequence, firstSample, flags = 0) {
  const wire = new Uint8Array(200);
  const view = new DataView(wire.buffer);
  wire.set([0x41, 0x55, 0x44, 0x31, 1, 1], 0);
  view.setUint16(6, flags, false);
  view.setUint16(8, 40, false);
  view.setUint32(12, streamId, false);
  view.setUint32(16, sequence, false);
  view.setUint32(20, 8000, false);
  view.setBigUint64(24, BigInt(firstSample), false);
  view.setUint32(36, 160, false);
  wire.fill(0xff, 40);
  return wire.buffer;
}

const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

async function feedEpoch(socket, streamId, descriptors) {
  await socket.message(JSON.stringify({type:"hello", protocol:"AUD1",
    version:1, streamId, rx:[{kind:"RX_ULAW", sampleRate:8000}]}));
  for (const descriptor of descriptors)
    await socket.message(packet(streamId, ...descriptor));
}

async function main() {
  let monotonic = 1000; let wallJump = 0;
  const delivered = []; const epochs = []; const statuses = [];
  const source = new WsAudioSource(8000, {url:"ws://test/audiows",
    WebSocketImpl:FakeWebSocket, reconnectMs:0,
    monotonicNow:() => (monotonic += 20),
    wallNow:() => 1700000000000 + monotonic + 7 + wallJump})
    .onSamples((samples, rate, metadata) => delivered.push({samples, rate, metadata}))
    .onEpoch(epoch => epochs.push(epoch))
    .onStatus(status => statuses.push(status));
  source.start();
  let socket = FakeWebSocket.instances.at(-1);
  if (socket.onopen) socket.onopen();
  await feedEpoch(socket, 101, [[0, 0, 1], [1, 160], [1, 160],
    [3, 480, 4], [4, 640], [5, 800]]);
  socket.close();
  await delay(2);
  socket = FakeWebSocket.instances.at(-1);
  if (socket.onopen) socket.onopen();
  await feedEpoch(socket, 202, [[0, 0, 1], [1, 160], [2, 320],
    [3, 480], [4, 640]]);
  wallJump = 600;
  for (const descriptor of [[5, 800], [6, 960], [7, 1120],
    [8, 1280], [9, 1440]])
    await socket.message(packet(202, ...descriptor));

  const snapshot = source.state().timebase;
  const checks = {
    threeMediaEpochs: epochs.length === 3 && epochs[0].streamId === 101 &&
      epochs[1].streamId === 202 && epochs[2].streamId === 202 &&
      epochs[2].reason === "clock-jump",
    anchoredBeforeDelivery: delivered.length === 15 &&
      delivered.every(item => Number.isFinite(item.metadata.anchorUtcMs)),
    metadataPreserved: delivered.every(item => item.rate === 8000 &&
      item.samples.length === 160 && item.metadata.aud1Wire.byteLength === 200),
    duplicateDropped: snapshot.transport.duplicatePackets === 1 &&
      statuses.some(status => status.type === "packet-dropped" && status.duplicate),
    gapObserved: snapshot.transport.sequenceGaps === 1 &&
      snapshot.transport.gapSamples === 160,
    reconnectObserved: snapshot.transport.reconnects === 1 &&
      snapshot.media.streamId === 202 && snapshot.media.status === "locked",
    clockJumpReanchored: snapshot.clock.jumps === 1 &&
      snapshot.clock.status === "unchecked" && snapshot.media.epoch === 3 &&
      snapshot.media.reason === "clock-jump" && snapshot.media.slotFlushes === 2,
  };
  source.stop();
  const pass = Object.values(checks).every(Boolean);
  console.log(`WS AUDIO SOURCE ${pass ? "PASS" : "FAIL"} ` +
    JSON.stringify({checks, epochs:epochs.length, delivered:delivered.length,
      transport:snapshot.transport}));
  if (!pass) process.exitCode = 1;
}

main().catch(error => { console.error(error.stack || error); process.exitCode = 1; });
