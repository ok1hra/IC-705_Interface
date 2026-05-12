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

let smeterPeak = 0;
let powerPeak  = 0;
let swrPeak    = 0;
const PEAK_DECAY = 0.4; // segments per poll (~200 ms) → ~12 s full-scale fall

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
  subBarPeak: -1,
  preamp: "OFF",
  vox: "none",
  activeTrx: 0,
  trx: []
};

for (let i = 0; i < 24; i++) {
  const segment = document.createElement("span");
  segment.className = "meter-segment";
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

function encodeBcdFixedLsb(value, digits) {
  const text = String(Math.max(0, Math.round(Number(value) || 0))).padStart(digits, "0");
  const pairs = [];
  for (let index = 0; index < digits; index += 2) {
    pairs.push(text.slice(index, index + 2));
  }
  pairs.reverse();
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
  const text = (raw / 1000).toFixed(2);
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
  state.subBarPeak = -1;
  smeterPeak = 0;
  powerPeak  = 0;
  swrPeak    = 0;
  setMeterSegments(0);
  setSubBarSegments(0, 0, 24);
}

function resetControlSliders() {
  afVolume.value = "0";
  keySpeed.value = "0";
  rfPower.value = "0";
}

// ---------- main render ----------

// overThreshold: segments at or above this index get the "over" (red) class
// peakSeg: index of the peak marker segment (-1 = none)
function setMeterSegments(activeCount, overThreshold = 18, peakSeg = -1) {
  const peak = Math.round(peakSeg);
  [...meterBar.children].forEach((segment, index) => {
    const active = index < activeCount;
    segment.classList.toggle("active", active);
    segment.classList.toggle("over", index >= overThreshold);
    segment.classList.toggle("peak", !active && index === peak);
  });
}

// lowSeg: segments below this index are red; highSeg: segments at or above this are red
// peakSeg: index of peak marker (-1 = none)
function setSubBarSegments(activeCount, lowSeg, highSeg, peakSeg = -1) {
  const peak = Math.round(peakSeg);
  [...subBar.children].forEach((segment, index) => {
    const active = index < activeCount;
    const red = active && (index < lowSeg || index >= highSeg);
    const isPeak = !active && index === peak;
    const peakRed = isPeak && (index < lowSeg || index >= highSeg);
    segment.classList.toggle("active", active);
    segment.classList.toggle("aux-active", active && !red);
    segment.classList.toggle("aux-over", red);
    segment.classList.toggle("peak", isPeak);
    segment.classList.toggle("peak-green", isPeak && !peakRed);
    segment.classList.toggle("peak-red", isPeak && peakRed);
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
  const isRtty = state.mode === "RTTY";
  cwInput.placeholder = isRtty ? "Send RTTY text…" : "Send CW text…";
  meterLabel.textContent = state.tx ? "Power" : "S meter";
  meterValue.textContent = state.meterText;
  subValueLabel.textContent = state.subLabel;
  subValueReadout.textContent = state.subValue;
  setSubBarSegments(state.subBarSegments, state.subBarLowSeg, state.subBarHighSeg, state.subBarPeak);
  const controlsEnabled = state.radioConnected && state.power;
  afVolume.disabled = !controlsEnabled;
  keySpeed.disabled = !controlsEnabled;
  rfPower.disabled = !controlsEnabled;
  afVolumeValue.textContent = formatAfVolume(afVolume.value);
  keySpeedValue.textContent = formatKeySpeed(keySpeed.value);
  rfPowerValue.textContent = formatRfPower(rfPower.value);
  preampButton.textContent = state.preamp === "OFF" ? "P.AMP OFF" : state.preamp;
  preampButton.dataset.preamp = state.preamp;
  voxButton.textContent = state.vox === "none" ? "VOX OFF" : state.vox;
  voxButton.dataset.vox = state.vox;
  pttButton.classList.toggle("tx-active", state.tx);
  pttButton.textContent = "PTT";
  renderCwMemoryButtons();
  statusAddress.textContent = state.transceiverType
    ? `${state.transceiverType} | ${state.radioAddress}`
    : `CI-V ${state.radioAddress}`;
  const wifiText = state.wifiRssi > -999 ? `${state.wifiStatus} ${state.wifiRssi} dBm` : state.wifiStatus;
  statusConnection.textContent = pollConnected
    ? `${state.btStatus} | ${wifiText}`
    : "Disconnected";
  statusRadio.textContent = state.power ? (state.tx ? "TX active" : "RX active") : "Radio OFF";
  statusFw.textContent = `FW ${state.fwRev}`;
  renderTrxChips();
}

// ---------- TRX chips ----------

const trxChipsEl = document.getElementById("trxChips");

function fmtFreq(hz) {
  if (!hz) return "—";
  const mhz = hz / 1e6;
  return mhz % 1 === 0 ? mhz.toFixed(0) + " MHz" : mhz.toFixed(3) + " MHz";
}

function renderTrxChips() {
  if (!trxChipsEl) return;
  const trxList = state.trx;
  if (!trxList || trxList.length === 0) { trxChipsEl.innerHTML = ""; return; }
  const enabled = trxList.filter(t => t.mode !== "disabled" && t.mode !== undefined || t.freq > 0 || t.label);
  if (enabled.length <= 1) { trxChipsEl.innerHTML = ""; return; }

  let html = "";
  trxList.forEach((t, i) => {
    if (!t.label) return;
    const active = i === state.activeTrx;
    const conn   = Boolean(t.conn);
    html += `<button class="trx-chip${active ? " active" : ""}" data-trx="${i}" type="button" title="Switch to ${t.label}">` +
            `<span class="trx-chip-dot${conn ? " conn" : ""}"></span>` +
            `<span class="trx-chip-label">${t.label}</span>` +
            `<span class="trx-chip-freq">${fmtFreq(t.freq)}</span>` +
            `</button>`;
  });
  trxChipsEl.innerHTML = html;

  trxChipsEl.querySelectorAll(".trx-chip").forEach(btn => {
    btn.addEventListener("click", () => {
      const idx = parseInt(btn.dataset.trx, 10);
      if (idx === state.activeTrx) return;
      fetch("/trx", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ idx })
      }).then(() => {
        state.activeTrx = idx;
        localStorage.setItem("ic705_activeTrx", String(idx));
        render();
      }).catch(() => {});
    });
  });
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
  if (s.activeTrx !== undefined) state.activeTrx = Number(s.activeTrx);
  if (Array.isArray(s.trx) && s.trx.length) state.trx = s.trx;

  if (!state.power) {
    resetRxIndicatorsForPowerOff();
    resetControlSliders();
  } else {
    // Update TX state — skip if user just clicked PTT
    if (Date.now() >= pauseTxReadUntil) {
      const newTx = Boolean(s.tx);
      if (newTx && !state.tx) { smeterPeak = 0; }          // switched to TX: clear S-meter peak
      if (!newTx && state.tx) { powerPeak = 0; swrPeak = 0; state.subBarPeak = -1; } // switched to RX: clear power/SWR peaks
      state.tx = newTx;
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
      if (pm.segments > powerPeak) powerPeak = pm.segments;
      else powerPeak = Math.max(0, powerPeak - PEAK_DECAY);
      setMeterSegments(pm.segments, 12, powerPeak); // blue below 50%, red above 50%
      // SWR sub-bar: reflection coefficient mapping — SWR1=left, SWR3=center, ∞=right
      const swr = Number(s.swr) || 1;
      state.subLabel = "SWR";
      state.subValue = swr.toFixed(2);
      const rho = (swr - 1) / (swr + 1);
      state.subBarSegments = Math.min(24, Math.round(rho * 24));
      state.subBarLowSeg = 0;
      state.subBarHighSeg = 12; // red above SWR 3.0 (center)
      if (state.subBarSegments > swrPeak) swrPeak = state.subBarSegments;
      else swrPeak = Math.max(0, swrPeak - PEAK_DECAY);
      state.subBarPeak = swrPeak;
    } else {
      const sraw = Number(s.smeterRaw) || 0;
      const sm = formatSMeter(sraw);
      state.meterRaw = sraw;
      state.meterText = sm.text;
      if (sm.segments > smeterPeak) smeterPeak = sm.segments;
      else smeterPeak = Math.max(0, smeterPeak - PEAK_DECAY);
      setMeterSegments(sm.segments, 18, smeterPeak);
      state.subBarPeak = -1; // no peak for supply bar
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
    sendRaw(`2100${encodeBcdFixedLsb(state.ritRaw, 6)}`); // LSB-first 3 BCD bytes, no sign byte
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
  state.ritRaw = 0;
  pauseRitRead(2000);
  sendRaw("2100000000"); // CI-V 0x21 sub 0x00: set RIT freq = 0 Hz
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

frequencyReadout.addEventListener("mouseenter", () => pauseFrequencyRead(1200));

ritReadout.addEventListener("wheel", (event) => {
  event.preventDefault();
  pauseRitRead(1600);
  tuneRit(event.deltaY < 0 ? 1 : -1);
}, { passive: false });

ritReadout.addEventListener("mouseenter", () => pauseRitRead(5000));
ritReadout.addEventListener("mousemove",  () => pauseRitRead(1200));

// ---------- tuning slider ----------

const tuneSlider     = document.getElementById("tuneSlider");
const tuneStepSelect = document.getElementById("tuneStep");

let _tuneBase      = 0;   // frekvence při začátku tažení
let _tuneSendTimer = null;

function _captureBase() {
  _tuneBase = state.frequency;
  pauseFrequencyRead(60000);
}

function _tuneApply() {
  const v    = Number(tuneSlider.value);        // -100..+100
  const step = Number(tuneStepSelect.value);    // Hz / jednotku
  const newFreq = Math.max(0, _tuneBase + v * step);
  if (newFreq === state.frequency) return;
  state.frequency = newFreq;
  render();
  if (_tuneSendTimer) return;
  _tuneSendTimer = setTimeout(() => {
    _tuneSendTimer = null;
    postCmd({ type: "setFrequency", frequency: String(state.frequency) });
  }, 50);
}

function _stopTune() {
  clearTimeout(_tuneSendTimer);
  _tuneSendTimer = null;
  tuneSlider.value = "0";
  _tuneBase = state.frequency;   // připravit základ pro příští tažení
  postCmd({ type: "setFrequency", frequency: String(state.frequency) });
  pauseFrequencyRead(600);
}

// pointerdown zachytí myš i dotyk — slouží k uložení výchozí frekvence
tuneSlider.addEventListener("pointerdown", _captureBase);
// focus: zachytit základ i při ovládání klávesnicí (Tab + šipky)
tuneSlider.addEventListener("focus",       _captureBase);
// input: průběžná aktualizace při tažení
tuneSlider.addEventListener("input",       _tuneApply);
// change: HTML spec garantuje spuštění při uvolnění (mouseup / touchend / keyup)
// — spolehlivější než pointerup na native <input type="range">
tuneSlider.addEventListener("change",      _stopTune);

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
  const active = state.mode === "CW" || state.mode === "RTTY";
  cwMemoryButtons.forEach((button, index) => {
    const text = cwMemories[index].trim();
    button.disabled = !active || text.length === 0;
    button.classList.toggle("mode-inactive", !active);
    button.textContent = text.length > 0 ? text.slice(0, 10) : `MEM ${index + 1}`;
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

// Restore activeTrx from localStorage and sync to server
(function () {
  const stored = parseInt(localStorage.getItem("ic705_activeTrx") || "0", 10);
  if (stored > 0) {
    fetch("/trx", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ idx: stored })
    }).catch(() => {});
    state.activeTrx = stored;
  }
})();

startPolling();

// ── Waterfall ────────────────────────────────────────────────────────────────

(function () {
  const wfSection  = document.getElementById("wfSection");
  const wfCanvas   = document.getElementById("wfCanvas");
  const wfAxisEl   = document.getElementById("wfAxis");
  const wfRangeEl  = document.getElementById("wfRange");
  const wfStatusEl = document.getElementById("wfWsStatus");
  if (!wfCanvas || !wfAxisEl) return;

  const wfCtx   = wfCanvas.getContext("2d");
  const axisCtx = wfAxisEl.getContext("2d");

  // Stav
  const wfState = { startHz: 0, endHz: 0, vfoHz: 0 };

  // Paleta: 0x00 → černá, 0xA0(160) → žlutá  (přes modrou a červenou)
  const palette = new Uint32Array(256);
  (function buildPalette() {
    for (let v = 0; v < 256; v++) {
      const n = Math.min(v, 160);              // clamp na 0xA0
      const r = Math.min(255, Math.round(n * 2));
      const g = Math.max(0,  Math.round(n * 2 - 200));
      const b = Math.max(0,  Math.round(140 - n * 2));
      // ImageData je RGBA (little-endian): byte0=R, byte1=G, byte2=B, byte3=A
      palette[v] = (255 << 24) | (b << 16) | (g << 8) | r;
    }
  })();

  function drawWaterfallLine(pixels) {
    const w = wfCanvas.width;
    const h = wfCanvas.height;

    // Posunout obsah o 1 řádek dolů
    const img = wfCtx.getImageData(0, 0, w, h - 1);
    wfCtx.putImageData(img, 0, 1);

    // Kreslit nový řádek nahoře
    const row = wfCtx.createImageData(w, 1);
    const buf = new Uint32Array(row.data.buffer);
    const scale = pixels.length / w;
    for (let x = 0; x < w; x++) {
      const idx = Math.min(Math.floor(x * scale), pixels.length - 1);
      buf[x] = palette[pixels[idx] & 0xFF];
    }
    wfCtx.putImageData(row, 0, 0);
  }

  function drawVfoMarker() {
    const { startHz, endHz, vfoHz } = wfState;
    if (endHz <= startHz) return;
    const x = Math.round((vfoHz - startHz) / (endHz - startHz) * wfCanvas.width);
    wfCtx.save();
    wfCtx.strokeStyle = "rgba(255,230,0,0.9)";
    wfCtx.lineWidth   = 1;
    wfCtx.beginPath();
    wfCtx.moveTo(x + 0.5, 0);
    wfCtx.lineTo(x + 0.5, wfCanvas.height);
    wfCtx.stroke();
    wfCtx.restore();
  }

  function drawFreqAxis() {
    const { startHz, endHz } = wfState;
    if (endHz <= startHz) return;
    const w = wfAxisEl.width;
    const h = wfAxisEl.height;
    axisCtx.clearRect(0, 0, w, h);
    axisCtx.fillStyle = "#667788";
    axisCtx.font      = "10px monospace";

    const spanHz = endHz - startHz;
    // Zvol krok: ~5 popisků
    const rawStep = spanHz / 5;
    const mag = Math.pow(10, Math.floor(Math.log10(rawStep)));
    const step = Math.round(rawStep / mag) * mag;
    const first = Math.ceil(startHz / step) * step;

    for (let hz = first; hz < endHz; hz += step) {
      const x = Math.round((hz - startHz) / spanHz * w);
      const kHz = (hz / 1000).toFixed(hz % 1000 === 0 ? 0 : 1);
      axisCtx.textAlign = "center";
      axisCtx.fillText(kHz, x, h - 2);
      axisCtx.fillStyle = "#334455";
      axisCtx.fillRect(x, 0, 1, 4);
      axisCtx.fillStyle = "#667788";
    }

    // Zobrazit rozsah v hlavičce
    if (wfRangeEl) {
      wfRangeEl.textContent =
        `${(startHz / 1000).toFixed(1)} – ${(endHz / 1000).toFixed(1)} kHz`;
    }
  }

  // WebSocket připojení k portu 82
  let wfSocket = null;
  let wfRetryTimer = null;

  function wfConnect() {
    if (wfSocket && wfSocket.readyState <= WebSocket.OPEN) return;
    wfSocket = new WebSocket(`ws://${location.hostname}:82/ws`);

    wfSocket.addEventListener("open", () => {
      if (wfStatusEl) wfStatusEl.textContent = "WS ✓";
    });

    wfSocket.addEventListener("close", () => {
      if (wfStatusEl) wfStatusEl.textContent = "WS –";
      clearTimeout(wfRetryTimer);
      wfRetryTimer = setTimeout(wfConnect, 3000);
    });

    wfSocket.addEventListener("error", () => {
      wfSocket.close();
    });

    wfSocket.addEventListener("message", (event) => {
      let msg;
      try { msg = JSON.parse(event.data); } catch (_) { return; }
      if (!msg || msg.t !== "wf") return;

      wfState.startHz = Number(msg.s) || 0;
      wfState.endHz   = Number(msg.e) || 0;
      wfState.vfoHz   = Number(msg.v) || 0;

      if (Array.isArray(msg.d) && msg.d.length > 0) {
        drawWaterfallLine(msg.d);
        drawVfoMarker();
        drawFreqAxis();
      }
    });
  }

  // Click-to-tune: klik na canvas → přeladit na danou frekvenci
  wfCanvas.addEventListener("click", (e) => {
    const { startHz, endHz } = wfState;
    if (endHz <= startHz) return;
    const rect   = wfCanvas.getBoundingClientRect();
    const x      = e.clientX - rect.left;
    const ratio  = x / rect.width;
    const target = Math.round(startHz + ratio * (endHz - startHz));
    postCmd({ type: "setFrequency", frequency: String(target) });
    pauseFrequencyRead(1400);
  });

  // Zobrazit/skrýt sekci podle transceiverType
  function updateWfVisibility() {
    const isLan = state.transceiverType && state.transceiverType.includes("-LAN");
    if (isLan) {
      wfSection.hidden = false;
      wfConnect();
    } else {
      wfSection.hidden = true;
    }
  }

  // Hook do render() — sledovat změnu transceiverType
  const _origRender = render;
  window._wfRenderHooked = true;
  // Pravidelně kontrolovat viditelnost (každé 2 s)
  setInterval(updateWfVisibility, 2000);
  updateWfVisibility();
})();
render();
