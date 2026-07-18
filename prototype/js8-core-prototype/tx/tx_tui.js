#!/usr/bin/env node
// PROTOTYPE — manually drive queue/slot/PTT/abort transitions with no radio.

const readline = require("readline");
const protocol = require("../protocol/protocol_runtime.js");
const {CaptureTxSink, TxController} = require("./tx_runtime.js");

let clock = 1000;
let sink = new CaptureTxSink();
let controller = create();
function create() {
  return new TxController({buildFrames: protocol.buildReplyFrames, sink,
    encoder: (_frame, mode) => {
      const duration = {0:13140, 1:8100, 2:4050, 4:25780, 8:2628}[mode];
      return new Int16Array(duration * 48);
    }});
}
function paceTo(targetUtcMs) {
  const state = controller.snapshot();
  if (state.status === "waiting-slot" && clock < state.prebufferStartUtcMs) {
    clock = state.prebufferStartUtcMs;
    controller.tick(clock);
  }
  while (clock < targetUtcMs && !["fault", "aborted", "completed"].includes(controller.snapshot().status)) {
    clock = Math.min(targetUtcMs, clock + 20);
    controller.tick(clock);
  }
}
function render() {
  console.clear();
  console.log("\x1b[1mPROTOTYPE — safe JS8 TX state\x1b[0m");
  console.log(JSON.stringify({clock, ...controller.snapshot(), sinkEvents: sink.events}, null, 2));
  console.log("\n\x1b[1mq\x1b[0m queue  \x1b[1ma\x1b[0m arm  \x1b[1ms\x1b[0m slot  " +
              "\x1b[1mf\x1b[0m finish  \x1b[1mx\x1b[0m abort  \x1b[1md\x1b[0m disconnect  \x1b[1mr\x1b[0m reset  \x1b[1mEsc\x1b[0m quit");
}
readline.emitKeypressEvents(process.stdin);
if (process.stdin.isTTY) process.stdin.setRawMode(true);
render();
process.stdin.on("keypress", (_text, key) => {
  try {
    if (key.name === "escape" || (key.ctrl && key.name === "c")) process.exit(0);
    if (key.name === "q") controller.queue({myCall:"OK1HRA", toCall:"K0OG",
      text:"HELLO WORLD", mode:1, toneHz:1500}, clock);
    if (key.name === "a") controller.prepare(clock);
    if (key.name === "s") paceTo(controller.snapshot().nextSlotUtcMs);
    if (key.name === "f") paceTo(controller.snapshot().endUtcMs + 1);
    if (key.name === "x") controller.abort("operator", clock);
    if (key.name === "d") controller.disconnect(clock);
    if (key.name === "r") { sink = new CaptureTxSink(); controller = create(); clock = 1000; }
  } catch (error) { console.error(error.message); }
  render();
});
