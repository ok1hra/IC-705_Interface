const frequencyReadout = document.getElementById("frequencyReadout");
const modeSelect = document.getElementById("modeSelect");
const ritReadout = document.getElementById("ritReadout");
const clearRitButton = document.getElementById("clearRit");
const freqMemorySelect = document.getElementById("freqMemorySelect");
const freqInput = document.getElementById("freqInput");
const meterBar = document.getElementById("meterBar");
const subBar = document.getElementById("subBar");
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
const cwMemoryButtons = [
  document.getElementById("cwMemory1"),
  document.getElementById("cwMemory2"),
  document.getElementById("cwMemory3"),
  document.getElementById("cwMemory4")
];
const statusAddress = document.getElementById("statusAddress");
const statusConnection = document.getElementById("statusConnection");
const statusRadio = document.getElementById("statusRadio");
const statusFw = document.getElementById("statusFw");

let cwSentLength = 0;
let tuneTimer = null;
let ritTimer = null;
let pauseFrequencyReadUntil = 0;
let pauseRitReadUntil = 0;
let pauseTxReadUntil = 0;
let pausePreampReadUntil = 0;
let pauseVoxReadUntil = 0;
let pauseFilterReadUntil = 0;
let pollConnected = false;

const cwMemories = [
  document.getElementById("cwMem1Data").value || "",
  document.getElementById("cwMem2Data").value || "",
  document.getElementById("cwMem3Data").value || "",
  document.getElementById("cwMem4Data").value || ""
];

const freqMemories = [
  document.getElementById("freqMem1Data").value || "",
  document.getElementById("freqMem2Data").value || "",
  document.getElementById("freqMem3Data").value || "",
  document.getElementById("freqMem4Data").value || "",
  document.getElementById("freqMem5Data").value || "",
  document.getElementById("freqMem6Data").value || "",
  document.getElementById("freqMem7Data").value || "",
  document.getElementById("freqMem8Data").value || "",
  document.getElementById("freqMem9Data").value || "",
  document.getElementById("freqMem10Data").value || ""
];

const state = {
  radioConnected: false,
  btStatus: "BT idle",
  wifiStatus: "WiFi down",
  wifiRssi: -999,
  fwRev: "----",
  power: false,
  frequency: 0,
  mode: "USB",
  filter: 1,
  radioAddress: "0x00",
  transceiverType: "",
  tx: false,
  ritRaw: 0,
  meterRaw: 0,
  meterText: "S0",
  subValue: "--.- V",
  subLabel: "Supply",
  subBarSegments: 0,
  subBarLowSeg: 0,
  subBarHighSeg: 24,
  preamp: "OFF",
  vox: "none"
};

for (let i = 0; i < 24; i++) {
  const segment = document.createElement("span");
  segment.className = "meter-segment";
  if (i >= 18) {
    segment.classList.add("over");
  }
  meterBar.appendChild(segment);
}

for (let i = 0; i < 24; i++) {
  const segment = document.createElement("span");
  segment.className = "meter-segment";
  subBar.appendChild(segment);
}

// ---------- command transport ----------

function postCmd(body) {
  return fetch("/cmd", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body)
  }).catch(() => {});
}

function sendRaw(data) {
  postCmd({ type: "civ.raw", data });
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

// ---------- frequency display ----------

function buildFrequencySlots(value) {
  const hz = Math.max(0, Number(value) || 0);
  const shown = String(Math.floor(hz / 10)).padStart(8, " ");
  const hzLast = String(hz % 10);
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
    { char: shown[7], step: steps[7] },
    { char: hzLast, step: 1, smallHz: true }
  ];
}

function renderFrequency() {
  const slots = buildFrequencySlots(state.frequency);
  frequencyReadout.textContent = "";
  for (const slot of slots) {
    const span = document.createElement("span");
    span.className = `frequency-slot${slot.dot ? " dot" : ""}${slot.blank ? " blank" : ""}${slot.smallHz ? " hz-last" : ""}`;
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

// ---------- meter formatting ----------

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

function formatKeySpeed(raw) {
  const numeric = Math.max(0, Math.min(255, Number(raw) || 0));
  return `${6 + Math.floor((numeric * 42) / 255)} WPM`;
}

function formatAfVolume(raw) {
  const numeric = Math.max(0, Math.min(255, Number(raw) || 0));
  return `${Math.round((numeric * 100) / 255)} %`;
}

function formatRfPower(raw) {
  const numeric = Math.max(0, Math.min(255, Number(raw) || 0));
  return `${Math.round((numeric * 100) / 255)} %`;
}

// ---------- state reset helpers ----------

function resetRxIndicatorsForPowerOff() {
  state.tx = false;
  state.meterRaw = 0;
  state.meterText = "S0";
  state.subLabel = "Supply";
  state.subValue = "--.- V";
  state.subBarSegments = 0;
  state.subBarLowSeg = 0;
  state.subBarHighSeg = 24;
  setMeterSegments(0);
  setSubBarSegments(0, 0, 24);
}

function resetControlSliders() {
  afVolume.value = "0";
  keySpeed.value = "0";
  rfPower.value = "0";
}

// ---------- main render ----------

function setMeterSegments(activeCount) {
  [...meterBar.children].forEach((segment, index) => {
    segment.classList.toggle("active", index < activeCount);
  });
}

// lowSeg: segments below this index are red; highSeg: segments at or above this are red
function setSubBarSegments(activeCount, lowSeg, highSeg) {
  [...subBar.children].forEach((segment, index) => {
    const active = index < activeCount;
    const red = active && (index < lowSeg || index >= highSeg);
    segment.classList.toggle("active", active);
    segment.classList.toggle("aux-active", active && !red);
    segment.classList.toggle("aux-over", red);
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
  setSubBarSegments(state.subBarSegments, state.subBarLowSeg, state.subBarHighSeg);
  const controlsEnabled = state.radioConnected && state.power;
  afVolume.disabled = !controlsEnabled;
  keySpeed.disabled = !controlsEnabled;
  rfPower.disabled = !controlsEnabled;
  afVolumeValue.textContent = formatAfVolume(afVolume.value);
  keySpeedValue.textContent = formatKeySpeed(keySpeed.value);
  rfPowerValue.textContent = formatRfPower(rfPower.value);
  preampButton.textContent = `P.AMP/ATT ${state.preamp}`;
  voxButton.textContent = `VOX ${state.vox}`;
  pttButton.classList.toggle("tx-active", state.tx);
  pttButton.textContent = state.tx ? "PTT TX" : "PTT RX";
  statusAddress.textContent = state.transceiverType
    ? `${state.transceiverType} | ${state.radioAddress}`
    : `CI-V ${state.radioAddress}`;
  const wifiText = state.wifiRssi > -999 ? `${state.wifiStatus} ${state.wifiRssi} dBm` : state.wifiStatus;
  statusConnection.textContent = pollConnected
    ? `${state.btStatus} | ${wifiText}`
    : "Disconnected";
  statusRadio.textContent = state.power ? (state.tx ? "TX active" : "RX active") : "Radio OFF";
  statusFw.textContent = `FW ${state.fwRev}`;
}

// ---------- polling ----------

function applyServerState(s) {
  if (!s) return;
  const wasPowerOn = state.power;

  state.radioConnected = Boolean(s.connected);
  state.btStatus  = s.btStatus  || state.btStatus;
  state.wifiStatus = s.wifiStatus || state.wifiStatus;
  state.wifiRssi  = Number(s.wifiRssi ?? state.wifiRssi);
  state.fwRev     = s.fwRev ? String(s.fwRev) : state.fwRev;
  state.power     = Boolean(s.power);
  state.frequency = Number(s.frequency) || state.frequency;
  state.mode      = s.mode || state.mode;
  state.radioAddress = s.radioAddress || state.radioAddress;
  state.transceiverType = s.transceiverType || state.transceiverType;

  if (!state.power) {
    resetRxIndicatorsForPowerOff();
    resetControlSliders();
  } else {
    // Update TX state — skip if user just clicked PTT
    if (Date.now() >= pauseTxReadUntil) {
      state.tx = Boolean(s.tx);
    }

    if (Date.now() >= pauseRitReadUntil) {
      state.ritRaw = Number(s.ritRaw) || 0;
    }

    if (Date.now() >= pauseFilterReadUntil) {
      state.filter = Number(s.filter) || state.filter;
    }

    // Meter
    if (state.tx) {
      const praw = Number(s.powerMeterRaw) || 0;
      const pm = formatPowerMeter(praw);
      state.meterRaw = praw;
      state.meterText = pm.text;
      setMeterSegments(pm.segments);
      // SWR sub-bar: green below SWR 3.0, red above
      const swr = Number(s.swr) || 1;
      state.subLabel = "SWR";
      state.subValue = swr.toFixed(2);
      state.subBarSegments = Math.min(24, Math.round((Math.min(swr, 4) / 4) * 24));
      state.subBarLowSeg = 0;
      state.subBarHighSeg = Math.round((3 / 4) * 24); // 18 — red above SWR 3.0
    } else {
      const sraw = Number(s.smeterRaw) || 0;
      const sm = formatSMeter(sraw);
      state.meterRaw = sraw;
      state.meterText = sm.text;
      setMeterSegments(sm.segments);
      // Supply sub-bar: seg 0 = red (<7 V), segs 1-21 = green (7-16 V), segs 22-23 = red (>16 V)
      const volts = Number(s.supplyVolts) || 0;
      state.subLabel = "Supply";
      state.subValue = `${volts.toFixed(2)} V`;
      if (volts <= 0) {
        state.subBarSegments = 0;
      } else if (volts < 7) {
        state.subBarSegments = 1;
      } else if (volts <= 16) {
        state.subBarSegments = 1 + Math.round(((volts - 7) / 9) * 21);
      } else {
        state.subBarSegments = Math.min(24, 22 + Math.round(((volts - 16) / 2) * 2));
      }
      state.subBarLowSeg  = 1;   // seg 0 red (<7 V warning)
      state.subBarHighSeg = 22;  // segs 22-23 red (>16 V)
    }

    // Sliders — only update when power on to avoid fighting user
    const afRaw = Number(s.afGain);
    if (!isNaN(afRaw) && !afVolume.matches(":active")) afVolume.value = String(afRaw);
    const ksRaw = Number(s.keySpeed);
    if (!isNaN(ksRaw) && !keySpeed.matches(":active")) keySpeed.value = String(ksRaw);
    const rfRaw = Number(s.rfPower);
    if (!isNaN(rfRaw) && !rfPower.matches(":active")) rfPower.value = String(rfRaw);

    // Preamp (0=OFF, 1=P.AMP1, 2=P.AMP2, 3=ATT)
    if (Date.now() >= pausePreampReadUntil) {
      const preampVal = Number(s.preamp);
      if (preampVal === 1) state.preamp = "P.AMP1";
      else if (preampVal === 2) state.preamp = "P.AMP2";
      else if (preampVal === 3) state.preamp = "ATT";
      else state.preamp = "OFF";
    }

    // VOX (0=none, 1=BKIN, 2=F-BKIN)
    if (Date.now() >= pauseVoxReadUntil) {
      const voxVal = Number(s.vox);
      if (voxVal === 1) state.vox = "BKIN";
      else if (voxVal === 2) state.vox = "F-BKIN";
      else state.vox = "none";
    }
  }

  render();
}

function startPolling() {
  setInterval(() => {
    // Skip frequency read while user is tuning
    if (Date.now() < pauseFrequencyReadUntil) return;

    fetch("/state")
      .then(r => r.json())
      .then(data => {
        pollConnected = true;
        applyServerState(data);
      })
      .catch(() => {
        pollConnected = false;
        state.radioConnected = false;
        state.btStatus = "BT idle";
        state.wifiStatus = "WiFi down";
        state.wifiRssi = -999;
        state.fwRev = "----";
        resetControlSliders();
        render();
      });
  }, 200);
}

// ---------- wheel / click helpers ----------

function pauseFrequencyRead(ms = 1200) {
  pauseFrequencyReadUntil = Date.now() + ms;
}

function pauseRitRead(ms = 1200) {
  pauseRitReadUntil = Date.now() + ms;
}

function readWheelStep(target, fallbackStep) {
  const slot = target.closest("[data-step]");
  return slot ? Number(slot.dataset.step) || fallbackStep : fallbackStep;
}

function scheduleFrequencyWrite() {
  window.clearTimeout(tuneTimer);
  pauseFrequencyRead(1400);
  tuneTimer = window.setTimeout(() => {
    postCmd({ type: "setFrequency", frequency: String(state.frequency) });
  }, 70);
}

function scheduleRitWrite() {
  window.clearTimeout(ritTimer);
  pauseRitRead(5000);
  ritTimer = window.setTimeout(() => {
    sendRaw(`2100${encodeBcdFixed(state.ritRaw, 6)}00`); // 3 BCD bytes + sign 00=positive
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

// ---------- UI event handlers ----------

modeSelect.addEventListener("change", () => {
  state.mode = modeSelect.value;
  postCmd({ type: "setMode", mode: state.mode, filter: `FIL${state.filter}` });
  render();
});

clearRitButton.addEventListener("click", () => {
  postCmd({ type: "setRitClear" });
  state.ritRaw = 0;
  render();
});

function bindSlider(input, subCommand) {
  input.addEventListener("input", () => { render(); });
  input.addEventListener("change", () => {
    sendRaw(`14${subCommand}${encodeBcdValue(input.value)}`);
    render();
  });
}

bindSlider(afVolume, "01");
bindSlider(keySpeed, "0C");
bindSlider(rfPower, "0A");

frequencyReadout.addEventListener("wheel", (event) => {
  event.preventDefault();
  pauseFrequencyRead(1600);
  const step = readWheelStep(event.target, 10);
  tuneFrequency(step, event.deltaY < 0 ? 1 : -1);
}, { passive: false });

frequencyReadout.addEventListener("click", (event) => {
  const step = readWheelStep(event.target, 10);
  pauseFrequencyRead(1600);
  tuneFrequency(step, 1);
});

frequencyReadout.addEventListener("contextmenu", (event) => {
  event.preventDefault();
  const step = readWheelStep(event.target, 10);
  pauseFrequencyRead(1600);
  tuneFrequency(step, -1);
});

ritReadout.addEventListener("wheel", (event) => {
  event.preventDefault();
  pauseRitRead(1600);
  tuneRit(event.deltaY < 0 ? 1 : -1);
}, { passive: false });

frequencyReadout.addEventListener("mouseenter", () => pauseFrequencyRead(5000));
frequencyReadout.addEventListener("mousemove",  () => pauseFrequencyRead(1200));

ritReadout.addEventListener("mouseenter", () => pauseRitRead(5000));
ritReadout.addEventListener("mousemove",  () => pauseRitRead(1200));

preampButton.addEventListener("click", () => {
  pausePreampReadUntil = Date.now() + 2000;
  if (state.preamp === "OFF") {
    state.preamp = "P.AMP1";
    sendRaw("160201"); // PREAMP1 ON
    sendRaw("1100");   // ATT OFF
  } else if (state.preamp === "P.AMP1") {
    state.preamp = "P.AMP2";
    sendRaw("160202"); // PREAMP2 ON
    sendRaw("1100");   // ATT OFF
  } else if (state.preamp === "P.AMP2") {
    state.preamp = "ATT";
    sendRaw("160200"); // PREAMP OFF
    sendRaw("1120");   // ATT 20 dB ON (0x20 = 20 dB, per CI-V cmd 0x11)
  } else {
    state.preamp = "OFF";
    sendRaw("1100");   // ATT OFF
    sendRaw("160200"); // PREAMP OFF
  }
  render();
});

voxButton.addEventListener("click", () => {
  pauseVoxReadUntil = Date.now() + 2000;
  if (state.vox === "none") {
    state.vox = "BKIN";
    sendRaw("164701");
  } else if (state.vox === "BKIN") {
    state.vox = "F-BKIN";
    sendRaw("164702");
  } else {
    state.vox = "none";
    sendRaw("164700");
  }
  render();
});

filterButton.addEventListener("click", () => {
  pauseFilterReadUntil = Date.now() + 2000;
  state.filter = state.filter >= 3 ? 1 : state.filter + 1;
  postCmd({ type: "setMode", mode: state.mode, filter: `FIL${state.filter}` });
  render();
});

pttButton.addEventListener("click", () => {
  state.tx = !state.tx;
  pauseTxReadUntil = Date.now() + 1500;
  sendRaw(state.tx ? "1C0001" : "1C0000");
  render();
});

cwInput.addEventListener("input", () => {
  if (cwInput.value.length < cwSentLength) {
    cwSentLength = cwInput.value.length;
    return;
  }
  const nextChunk = cwInput.value.slice(cwSentLength);
  if (nextChunk.length > 0) {
    postCmd({ type: "sendCw", text: nextChunk });
    cwSentLength = cwInput.value.length;
  }
});

freqInput.addEventListener("keydown", (event) => {
  if (event.key !== "Enter") return;
  event.preventDefault();
  const frequency = Number(freqInput.value.trim());
  if (!Number.isFinite(frequency) || frequency <= 0) {
    freqInput.select();
    return;
  }
  pauseFrequencyRead(1800);
  postCmd({ type: "setFrequency", frequency: String(Math.round(frequency)) });
  freqInput.value = "";
});

clearCwButton.addEventListener("click", () => {
  cwInput.value = "";
  cwSentLength = 0;
});

function parseFrequencyMemory(value) {
  const trimmed = value.trim();
  if (!trimmed) return null;
  const parts = trimmed.split(/\s+/);
  const frequency = Number(parts[0]);
  const mode = parts[1] ? parts[1].toUpperCase() : "";
  if (!Number.isFinite(frequency) || frequency <= 0) return null;
  return { frequency, mode };
}

freqMemorySelect.addEventListener("change", () => {
  const memory = parseFrequencyMemory(freqMemorySelect.value);
  if (!memory) return;
  pauseFrequencyRead(1800);
  postCmd({ type: "setFrequency", frequency: String(memory.frequency) });
  if (memory.mode) {
    postCmd({ type: "setMode", mode: memory.mode, filter: `FIL${state.filter}` });
  }
  freqMemorySelect.value = "";
});

cwMemoryButtons.forEach((button, index) => {
  button.addEventListener("click", () => {
    const text = cwMemories[index].trim();
    if (!text) return;
    postCmd({ type: "sendCw", text });
  });
});

// ---------- memory UI init ----------

function renderCwMemoryButtons() {
  cwMemoryButtons.forEach((button, index) => {
    const text = cwMemories[index].trim();
    button.disabled = text.length === 0;
    button.textContent = text.length > 0 ? text.slice(0, 10) : `CW ${index + 1}`;
  });
}

function renderFrequencyMemories() {
  for (let index = 0; index < freqMemories.length; index++) {
    const value = freqMemories[index].trim();
    if (!value) continue;
    const option = document.createElement("option");
    option.value = value;
    option.textContent = `${index + 1}: ${value.slice(0, 18)}`;
    freqMemorySelect.appendChild(option);
  }
}

// ---------- startup ----------

renderCwMemoryButtons();
renderFrequencyMemories();
startPolling();
render();
