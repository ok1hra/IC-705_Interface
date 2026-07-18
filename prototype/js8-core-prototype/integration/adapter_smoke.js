#!/usr/bin/env node
// PROTOTYPE — proves the candidate adapter shape without touching DATA globals.

const {createJs8ModemAdapter} = require("./js8_modem_adapter.js");

class DecoderBase {
  constructor(sampleRate) { this.sampleRate = sampleRate; this.text = []; }
  onText(callback) { this.callback = callback; return this; }
  _emit(text) { this.text.push(text); if (this.callback) this.callback(text); }
}
class EncoderBase {
  constructor(sampleRate) { this.sampleRate = sampleRate; this.toneHz = 1500; }
  setToneOffset(hz) { this.toneHz = hz; }
}
class FakeWorker {
  constructor() { this.sent = []; }
  postMessage(message) { this.sent.push(message); }
  terminate() { this.terminated = true; }
}
const fakeController = {
  queue(request) { this.request = request; return {status:"queued"}; },
  prepare() { return {status:"waiting-slot"}; }, tick() { return {status:"completed", ptt:false}; },
  abort() { return {status:"aborted", ptt:false}; },
  disconnect() { return {status:"aborted", ptt:false}; },
};
let worker;
const adapter = createJs8ModemAdapter({DecoderBase, EncoderBase,
  createWorker: () => (worker = new FakeWorker()),
  createTxController: () => fakeController});
const decoder = new adapter.Decoder(8000).onText(() => {});
worker.onmessage({data:{type:"ready", state:{activity:{messages:[{text:"KN4CRD: TEST"}]}}}});
let rejectedRawSamples = false;
try { decoder.pushSamples(new Float32Array(16)); } catch { rejectedRawSamples = true; }
decoder.pushSamples(new Float32Array(0), {aud1Wire:new Uint8Array([1]),
  arrivalMs:10, streamId:7, mediaEpoch:1, anchorUtcMs:1700000000000});
decoder.pushSamples(new Float32Array(0), {aud1Wire:new Uint8Array([2]),
  arrivalMs:20, streamId:7, mediaEpoch:1, anchorUtcMs:1700000000000});
const encoder = new adapter.Encoder(48000).configure({myCall:"OK1HRA", toCall:"K0OG", mode:1});
encoder.setToneOffset(1600); encoder.encode("HELLO"); encoder.abort();
const epochMessages = worker.sent.filter(message => message.type === "epoch");
const audioMessages = worker.sent.filter(message => message.type === "audio");
if (!rejectedRawSamples || decoder.text[0] !== "KN4CRD: TEST\n" ||
    epochMessages.length !== 1 || audioMessages.length !== 2 ||
    worker.sent.indexOf(epochMessages[0]) > worker.sent.indexOf(audioMessages[0]) ||
    fakeController.request.toneHz !== 1600 || adapter.id !== "js8call")
  throw new Error("candidate adapter contract failed");
console.log(`ADAPTER PASS id=${adapter.id} raw_samples_rejected=${rejectedRawSamples} ` +
            `epochs=${epochMessages.length} audio=${audioMessages.length} ` +
            `tx_tone=${fakeController.request.toneHz}`);
