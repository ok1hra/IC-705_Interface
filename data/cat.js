const frequencyReadout = document.getElementById("frequencyReadout");
const modeSelect = document.getElementById("modeSelect");
const ritReadout = document.getElementById("ritReadout");
const clearRitButton = document.getElementById("clearRit");
const meterBar = document.getElementById("meterBar");
const meterLabel = document.getElementById("meterLabel");
const meterValue = document.getElementById("meterValue");
const subValueLabel = document.getElementById("subValueLabel");
const subValueReadout = document.getElementById("subValueReadout");
const afVolume = document.getElementById("afVolume");
const afVolumeValue = document.getElementById("afVolumeValue");
const keySpeed = document.getElementById("keySpeed");
const keySpeedValue = document.getElementById("keySpeedValue");
const rfPower = document.getElementById("rfPower");
const rfPowerValue = document.getElementById("rfPowerValue");
const preampButton = document.getElementById("preampButton");
const voxButton = document.getElementById("voxButton");
const filterButton = document.getElementById("filterButton");
const pttButton = document.getElementById("pttButton");
const cwInput = document.getElementById("cwInput");
const clearCwButton = document.getElementById("clearCw");
const statusAddress = document.getElementById("statusAddress");
const statusConnection = document.getElementById("statusConnection");
const statusRadio = document.getElementById("statusRadio");

let socket;
let reqId = 1;
let pendingReads = 0;
let cwSentLength = 0;
let tuneTimer = null;
let ritTimer = null;

const state = {
  wsConnected: false,
  radioConnected: false,
  btStatus: "BT idle",
  wifiStatus: "WiFi down",
  wifiRssi: -999,
  power: false,
  frequency: 0,
  mode: "USB",
  filter: 1,
  radioAddress: "0x00",
  tx: false,
  ritRaw: 0,
  meterRaw: 0,
  meterKind: "sMeter",
  meterText: "S0",
  subValue: "--.- V",
  subLabel: "Supply",
  preamp: "OFF",
  vox: "none"
};

const pollPlan = [
  () => send({ type: "readFrequency", reqId: nextReqId() }, true),
  () => send({ type: "readMode", reqId: nextReqId() }, true),
  () => send({ type: "readRit", reqId: nextReqId() }, true),
  () => sendRaw("1401", true),
  () => sendRaw("140C", true),
  () => sendRaw("140A", true),
  () => sendRaw("1C00", true),
  () => state.tx ? send({ type: "readPower", reqId: nextReqId() }, true) : send({ type: "readSmeter", reqId: nextReqId() }, true),
  () => state.tx ? send({ type: "readSwr", reqId: nextReqId() }, true) : sendRaw("1515", true)
];

let pollIndex = 0;

for (let i = 0; i < 24; i++) {
  const segment = document.createElement("span");
  segment.className = "meter-segment";
  if (i >= 18) {
    segment.classList.add("over");
  }
  meterBar.appendChild(segment);
}

function nextReqId() {
  return String(reqId++);
}

function send(message, isRead = false) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    return false;
  }
  socket.send(JSON.stringify(message));
  if (isRead) {
    pendingReads++;
  }
  return true;
}

function sendRaw(data, expectReply = false, expectAck = false) {
  return send({
    type: "civ.raw",
    reqId: nextReqId(),
    data,
    expectReply,
    expectAck
  }, expectReply);
}

function encodeBcdValue(value) {
  const safe = Math.max(0, Math.min(255, Math.round(Number(value) || 0)));
  return safe.toString().padStart(4, "0");
}

function encodeBcdFixed(value, digits) {
  const text = String(Math.max(0, Math.round(Number(value) || 0))).padStart(digits, "0");
  const pairs = [];
  for (let index = 0; index < digits; index += 2) {
    pairs.push(text.slice(index, index + 2));
  }
  return pairs.join("");
}

function buildFrequencySlots(value) {
  const hz = Math.max(0, Number(value) || 0);
  const shown = String(Math.floor(hz / 10)).padStart(8, " ");
  const steps = [100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10];
  return [
    { char: shown[0], step: steps[0], blank: shown[0] === " " },
    { char: shown[1], step: steps[1], blank: shown[1] === " " },
    { char: shown[2], step: steps[2], blank: shown[2] === " " },
    { char: ".", dot: true, step: null },
    { char: shown[3], step: steps[3] },
    { char: shown[4], step: steps[4] },
    { char: shown[5], step: steps[5] },
    { char: ".", dot: true, step: null },
    { char: shown[6], step: steps[6] },
    { char: shown[7], step: steps[7] }
  ];
}

function renderFrequency() {
  const slots = buildFrequencySlots(state.frequency);
  frequencyReadout.textContent = "";
  for (const slot of slots) {
    const span = document.createElement("span");
    span.className = `frequency-slot${slot.dot ? " dot" : ""}${slot.blank ? " blank" : ""}`;
    span.textContent = slot.char;
    if (slot.step) {
      span.dataset.step = String(slot.step);
    }
    frequencyReadout.appendChild(span);
  }
}

function renderRit() {
  const raw = Math.max(0, Number(state.ritRaw) || 0);
  const text = (raw / 100).toFixed(2);
  ritReadout.textContent = "";
  for (const char of text) {
    const span = document.createElement("span");
    span.className = `rit-slot${char === "." ? " dot" : ""}`;
    span.textContent = char;
    if (char >= "0" && char <= "9") {
      span.dataset.step = "10";
    }
    ritReadout.appendChild(span);
  }
  const suffix = document.createElement("span");
  suffix.className = "rit-slot";
  suffix.style.width = "auto";
  suffix.textContent = " kHz";
  ritReadout.appendChild(suffix);
}

function readWheelStep(target, fallbackStep) {
  const slot = target.closest("[data-step]");
  return slot ? Number(slot.dataset.step) || fallbackStep : fallbackStep;
}

function scheduleFrequencyWrite() {
  window.clearTimeout(tuneTimer);
  tuneTimer = window.setTimeout(() => {
    send({
      type: "setFrequency",
      reqId: nextReqId(),
      frequency: String(state.frequency)
    });
  }, 70);
}

function scheduleRitWrite() {
  window.clearTimeout(ritTimer);
  ritTimer = window.setTimeout(() => {
    sendRaw(`2100${encodeBcdFixed(state.ritRaw, 6)}`, true, true);
  }, 70);
}

function tuneFrequency(step, direction) {
  const next = Math.max(0, state.frequency + (step * direction));
  state.frequency = next;
  render();
  scheduleFrequencyWrite();
}

function tuneRit(direction) {
  const next = Math.max(0, state.ritRaw + (10 * direction));
  state.ritRaw = Math.min(999999, next);
  render();
  scheduleRitWrite();
}

function formatSMeter(raw) {
  const numeric = Number(raw) || 0;
  if (numeric <= 0) {
    return { text: "S0", segments: 0 };
  }
  if (numeric <= 120) {
    const sValue = Math.max(1, Math.round((numeric / 120) * 9));
    const segments = Math.min(18, Math.round((numeric / 120) * 18));
    return { text: `S${sValue}`, segments };
  }
  const over = Math.round(((numeric - 120) * 60) / 121);
  const segments = Math.min(24, 18 + Math.round(((numeric - 120) / 121) * 6));
  return { text: `S9+${Math.max(5, over)}dB`, segments };
}

function formatPowerMeter(raw) {
  const numeric = Number(raw) || 0;
  const percent = Math.min(100, Math.round((numeric * 100) / 213));
  return { text: `${percent}%`, segments: Math.round((percent / 100) * 24) };
}

function formatSwr(raw) {
  const numeric = Number(raw) || 0;
  let ratio = 1;
  if (numeric <= 48) {
    ratio = 1 + (numeric * 0.5 / 48);
  } else if (numeric <= 80) {
    ratio = 1.5 + ((numeric - 48) * 0.5 / 32);
  } else if (numeric <= 120) {
    ratio = 2 + ((numeric - 80) * 1.0 / 40);
  } else {
    ratio = 3 + ((numeric - 120) / 40);
  }
  return ratio.toFixed(2);
}

function refreshSupplyPlaceholder() {
  state.subLabel = "Supply";
  state.subValue = "--.- V";
  render();
}

function resetRxIndicatorsForPowerOff() {
  state.tx = false;
  state.meterKind = "sMeter";
  state.meterRaw = 0;
  state.meterText = "S0";
  state.subLabel = "Supply";
  state.subValue = "--.- V";
  setMeterSegments(0);
}

function applyState(message) {
  if (!message) {
    return;
  }
  const wasPowerOn = state.power;
  state.radioConnected = Boolean(message.connected);
  state.btStatus = message.btStatus || state.btStatus;
  state.wifiStatus = message.wifiStatus || state.wifiStatus;
  state.wifiRssi = Number(message.wifiRssi ?? state.wifiRssi);
  state.power = Boolean(message.power);
  state.frequency = Number(message.frequency) || 0;
  state.mode = message.mode || state.mode;
  state.radioAddress = message.radioAddress || state.radioAddress;
  if (!state.power) {
    resetRxIndicatorsForPowerOff();
  } else if (!wasPowerOn) {
    state.subLabel = state.tx ? "SWR" : "Supply";
  }
  render();
}

function applyDecoded(decoded) {
  if (!decoded || typeof decoded !== "object") {
    return;
  }

  if (decoded.kind === "mode" && decoded.mode) {
    state.mode = decoded.mode;
    if (decoded.filter) {
      state.filter = Number(decoded.filter);
    }
  }

  if (decoded.kind === "rit" && typeof decoded.raw !== "undefined") {
    state.ritRaw = Number(decoded.raw) || 0;
  }

  if (decoded.kind === "control" && decoded.name && typeof decoded.raw !== "undefined") {
    const value = Math.max(0, Math.min(255, Number(decoded.raw) || 0));
    if (decoded.name === "afGain") {
      afVolume.value = String(value);
    } else if (decoded.name === "keySpeed") {
      keySpeed.value = String(value);
    } else if (decoded.name === "rfPower") {
      rfPower.value = String(value);
    }
  }

  if (decoded.kind === "sMeter") {
    const meter = formatSMeter(decoded.raw);
    state.meterKind = "sMeter";
    state.meterRaw = decoded.raw;
    state.meterText = meter.text;
    setMeterSegments(meter.segments);
  }

  if (decoded.kind === "powerMeter") {
    const meter = formatPowerMeter(decoded.raw);
    state.meterKind = "powerMeter";
    state.meterRaw = decoded.raw;
    state.meterText = meter.text;
    setMeterSegments(meter.segments);
  }

  if (decoded.kind === "swrMeter") {
    state.subLabel = "SWR";
    state.subValue = formatSwr(decoded.raw);
  }

  if (decoded.kind === "vdMeter") {
    state.subLabel = "Supply";
    state.subValue = `${Number(decoded.voltsApprox ?? 0).toFixed(2)} V`;
  }

  if (decoded.kind === "txState") {
    state.tx = Boolean(decoded.tx);
  }

  render();
}

function setMeterSegments(activeCount) {
  [...meterBar.children].forEach((segment, index) => {
    segment.classList.toggle("active", index < activeCount);
  });
}

function render() {
  renderFrequency();
  frequencyReadout.classList.toggle("tx", state.tx);
  frequencyReadout.classList.toggle("off", !state.power);
  if (modeSelect.value !== state.mode) {
    modeSelect.value = state.mode;
  }
  renderRit();
  filterButton.textContent = `FIL${state.filter}`;
  meterLabel.textContent = state.tx ? "Power" : "S meter";
  meterValue.textContent = state.meterText;
  subValueLabel.textContent = state.subLabel;
  subValueReadout.textContent = state.subValue;
  afVolumeValue.textContent = afVolume.value;
  keySpeedValue.textContent = keySpeed.value;
  rfPowerValue.textContent = rfPower.value;
  preampButton.textContent = `P.AMP/ATT ${state.preamp}`;
  voxButton.textContent = `VOX ${state.vox}`;
  pttButton.classList.toggle("tx-active", state.tx);
  pttButton.textContent = state.tx ? "PTT TX" : "PTT RX";
  statusAddress.textContent = `CI-V ${state.radioAddress}`;
  const wifiText = state.wifiRssi > -999 ? `${state.wifiStatus} ${state.wifiRssi} dBm` : state.wifiStatus;
  statusConnection.textContent = `${state.btStatus} | ${wifiText}`;
  statusRadio.textContent = state.power ? (state.tx ? "TX active" : "RX active") : "Radio OFF";
}

function handleMessage(msg) {
  if (msg.type === "hello" && msg.state) {
    applyState(msg.state);
    return;
  }

  if (msg.type === "state" || msg.type === "reply") {
    applyState(msg.state ?? msg);
  }

  if (msg.decoded) {
    applyDecoded(msg.decoded);
  }

  if (msg.type === "reply" || msg.type === "error") {
    pendingReads = Math.max(0, pendingReads - 1);
  }
}

function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);

  socket.addEventListener("open", () => {
    state.wsConnected = true;
    render();
  });

  socket.addEventListener("close", () => {
    state.wsConnected = false;
    state.radioConnected = false;
    state.btStatus = "BT idle";
    state.wifiStatus = "WiFi down";
    state.wifiRssi = -999;
    render();
    setTimeout(connect, 2000);
  });

  socket.addEventListener("message", (event) => {
    let message;
    try {
      message = JSON.parse(event.data);
    } catch (error) {
      return;
    }
    handleMessage(message);
  });
}

setInterval(() => {
  if (!socket || socket.readyState !== WebSocket.OPEN || pendingReads > 1) {
    return;
  }
  pollPlan[pollIndex % pollPlan.length]();
  pollIndex++;
}, 140);

modeSelect.addEventListener("change", () => {
  state.mode = modeSelect.value;
  send({
    type: "setMode",
    reqId: nextReqId(),
    mode: state.mode,
    filter: `FIL${state.filter}`
  });
  render();
});

clearRitButton.addEventListener("click", () => {
  send({ type: "setRitClear", reqId: nextReqId() });
  state.ritRaw = 0;
  render();
});

function bindSlider(input, subCommand) {
  input.addEventListener("input", () => {
    render();
  });
  input.addEventListener("change", () => {
    sendRaw(`14${subCommand}${encodeBcdValue(input.value)}`, true, true);
    render();
  });
}

bindSlider(afVolume, "01");
bindSlider(keySpeed, "0C");
bindSlider(rfPower, "0A");

frequencyReadout.addEventListener("wheel", (event) => {
  event.preventDefault();
  const step = readWheelStep(event.target, 10);
  tuneFrequency(step, event.deltaY < 0 ? 1 : -1);
}, { passive: false });

ritReadout.addEventListener("wheel", (event) => {
  event.preventDefault();
  tuneRit(event.deltaY < 0 ? 1 : -1);
}, { passive: false });

preampButton.addEventListener("click", () => {
  if (state.preamp === "OFF") {
    state.preamp = "P.AMP";
    sendRaw("160201", true, true);
    sendRaw("1100", true, true);
  } else if (state.preamp === "P.AMP") {
    state.preamp = "ATT";
    sendRaw("160200", true, true);
    sendRaw("1101", true, true);
  } else {
    state.preamp = "OFF";
    sendRaw("1100", true, true);
    sendRaw("160200", true, true);
  }
  render();
});

voxButton.addEventListener("click", () => {
  if (state.vox === "none") {
    state.vox = "BKIN";
    sendRaw("164701", true, true);
  } else if (state.vox === "BKIN") {
    state.vox = "F-BKIN";
    sendRaw("164702", true, true);
  } else {
    state.vox = "none";
    sendRaw("164700", true, true);
  }
  render();
});

filterButton.addEventListener("click", () => {
  state.filter = state.filter >= 3 ? 1 : state.filter + 1;
  send({
    type: "setMode",
    reqId: nextReqId(),
    mode: state.mode,
    filter: `FIL${state.filter}`
  });
  render();
});

pttButton.addEventListener("click", () => {
  state.tx = !state.tx;
  sendRaw(state.tx ? "1C0001" : "1C0000", true, true);
  render();
});

cwInput.addEventListener("input", () => {
  if (cwInput.value.length < cwSentLength) {
    cwSentLength = cwInput.value.length;
    return;
  }
  const nextChunk = cwInput.value.slice(cwSentLength);
  if (nextChunk.length > 0) {
    send({ type: "sendCw", reqId: nextReqId(), text: nextChunk });
    cwSentLength = cwInput.value.length;
  }
});

clearCwButton.addEventListener("click", () => {
  cwInput.value = "";
  cwSentLength = 0;
});

connect();
render();
