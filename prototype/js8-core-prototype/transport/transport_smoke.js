#!/usr/bin/env node
// PROTOTYPE — fake socket exercises metadata preservation and async TX control.

const {Aud1WebSocketSession} = require("./aud1_websocket_session.js");
const {packetizeTxPcm48k} = require("../tx/tx_runtime.js");

class FakeWebSocket {
  static OPEN = 1;
  constructor() { this.readyState = FakeWebSocket.OPEN; this.sent = []; FakeWebSocket.last = this; }
  send(value) { this.sent.push(value); }
  close() { this.readyState = 3; if (this.onclose) this.onclose(); }
}

function rxPacket() {
  const wire = new Uint8Array(43); const view = new DataView(wire.buffer);
  wire.set([0x41,0x55,0x44,0x31,1,1], 0); view.setUint16(6,1,false);
  view.setUint16(8,40,false); view.setUint32(12,0x01020304,false);
  view.setUint32(16,7,false); view.setUint32(20,8000,false);
  view.setBigUint64(24,160n,false); view.setUint32(36,3,false);
  wire.set([0xff,0x7f,0x00],40); return wire;
}

async function main() {
  let sampleEvent; const statuses = [];
  const session = new Aud1WebSocketSession({url:"ws://test/audiows",
    WebSocketImpl:FakeWebSocket, readyTimeoutMs:1000, now:()=>123.5})
    .onSamples((samples, rate, metadata) => { sampleEvent = {samples,rate,metadata}; })
    .onStatus(status => statuses.push(status));
  session.start(); const socket = FakeWebSocket.last; socket.onopen();
  session.receiveControl({type:"hello", protocol:"AUD1", version:1,
    streamId:0x01020304, tx:[{kind:"TX_PCM16",sampleRate:48000}]});
  await session.receive(rxPacket().buffer);
  if (!sampleEvent || sampleEvent.rate !== 8000 || sampleEvent.metadata.sequence !== 7 ||
      sampleEvent.metadata.firstSample !== 160n || sampleEvent.metadata.aud1Wire.length !== 43)
    throw new Error("RX metadata was not preserved");

  const ready = session.prepare(9, {samples:960,packets:1,mode:1,toneHz:1500,
    slotUtcMs:1700000010000,prebufferSamples:12000,packetMs:20});
  session.receiveControl({type:"tx-ready",txId:9}); await ready;
  session.begin(9);
  const packet = packetizeTxPcm48k(new Int16Array(960),
    {streamId:0x01020304,txId:9})[0];
  session.write(packet); session.end(9);
  session.receiveControl({type:"tx-state",txId:9,ptt:true});
  session.receiveControl({type:"tx-drained",txId:9});
  if (!session.isDrained(9) || session.ptt) throw new Error("TX drain/PTT state failed");
  session.complete(9); session.stop();
  const controls = socket.sent.filter(value => typeof value === "string").map(JSON.parse);
  if (controls[0].type !== "tx.prepare" || !socket.sent.some(value => value instanceof Uint8Array))
    throw new Error("TX WebSocket sequence missing");

  const remoteFault = new Aud1WebSocketSession({url:"ws://test/audiows",
    WebSocketImpl:FakeWebSocket, readyTimeoutMs:1000});
  remoteFault.start(); const faultSocket = FakeWebSocket.last; faultSocket.onopen();
  remoteFault.receiveControl({type:"hello", protocol:"AUD1", version:1,
    streamId:0x01020304, tx:[{kind:"TX_PCM16",sampleRate:48000}]});
  const faultReady = remoteFault.prepare(10, {samples:960,packets:1,mode:1,toneHz:1500,
    slotUtcMs:1700000010000,prebufferSamples:12000,packetMs:20});
  remoteFault.receiveControl({type:"tx-ready",txId:10}); await faultReady;
  remoteFault.begin(10);
  remoteFault.receiveControl({type:"tx-error",txId:10,reason:"TX prebuffer missed slot"});
  let remoteFaultStoppedWrite = false;
  try { remoteFault.write(packet); }
  catch (error) { remoteFaultStoppedWrite = error.message.includes("prebuffer missed slot"); }
  if (!remoteFaultStoppedWrite || faultSocket.sent.some(value => value instanceof Uint8Array))
    throw new Error("remote TX fault did not stop queued binary audio");
  remoteFault.stop();

  const strict = new Aud1WebSocketSession({url:"ws://test/audiows",
    WebSocketImpl:FakeWebSocket});
  let beforeHelloRejected = false; let mismatchedStreamRejected = false;
  try { await strict.receive(rxPacket().buffer); }
  catch (error) { beforeHelloRejected = error.message.includes("before hello"); }
  strict.receiveControl({type:"hello", protocol:"AUD1", version:1,
    streamId:99, rx:[{kind:"RX_ULAW", sampleRate:8000}]});
  try { await strict.receive(rxPacket().buffer); }
  catch (error) { mismatchedStreamRejected = error.message.includes("does not match"); }
  if (!beforeHelloRejected || !mismatchedStreamRejected)
    throw new Error("strict AUD1 hello/stream validation failed");
  console.log(`TRANSPORT PASS rx_samples=${sampleEvent.samples.length} controls=${controls.length} ` +
              `binary=${socket.sent.filter(value => value instanceof Uint8Array).length} ` +
              `strict=true ptt=${session.ptt}`);
}

main().catch(error => { console.error(error.stack || error); process.exitCode = 1; });
