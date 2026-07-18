#!/usr/bin/env node
// PROTOTYPE — drive decoded JS8 activity state without a browser or radio.

const fs = require("fs");
const readline = require("readline");
const {ActivityStore, JscDictionary} = require("./protocol_runtime.js");

const dictionary = new JscDictionary(fs.readFileSync(process.argv[2]));
const store = new ActivityStore(dictionary);
const fixtures = {
  h: {raw: "2Y-wUW3FOjFp", frameType: 3, submode: 0, slotUtcMs: 0,
      snr: -27, offsetHz: 1866.125},
  d: {raw: "Vk4xfHSNwzaX", frameType: 3, submode: 0, slotUtcMs: 15000,
      snr: -22, offsetHz: 702.625},
  e: {raw: "TrMcT8++++++", frameType: 7, submode: 4, slotUtcMs: 30000,
      snr: 33, offsetHz: 1291.125},
};

function render() {
  const state = store.snapshot();
  console.clear();
  console.log("\x1b[1mPROTOTYPE — JS8 protocol/activity state\x1b[0m");
  console.log(JSON.stringify(state, null, 2));
  console.log("\n\x1b[1mh\x1b[0m heartbeat  \x1b[1md\x1b[0m directed  " +
              "\x1b[1me\x1b[0m JSC data  \x1b[1mq\x1b[0m quit");
}

if (process.argv.includes("--snapshot")) {
  for (const frame of Object.values(fixtures)) store.push(frame);
  const state = store.snapshot();
  console.log(JSON.stringify({calls: state.calls.map(item => item.call),
    messages: state.messages.map(item => item.text),
    kinds: state.frames.map(item => item.kind)}, null, 2));
  process.exit(0);
}

readline.emitKeypressEvents(process.stdin);
if (process.stdin.isTTY) process.stdin.setRawMode(true);
render();
process.stdin.on("keypress", (_text, key) => {
  if (key.name === "q" || (key.ctrl && key.name === "c")) process.exit(0);
  if (fixtures[key.name]) store.push({...fixtures[key.name]});
  render();
});
