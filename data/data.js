"use strict";
// DATA page — audio waterfall for digital modes.
//
// Milestone 1 (this file): full front-end. The waterfall is fed by a synthetic
// audio source so the whole pipeline (source -> FFT -> waterfall -> decoder) can be
// exercised without a radio. Header freq/mode/HF-PWR come from the real GET /state.
//
// Milestone 2 will add WsAudioSource (binary WebSocket carrying real RX audio from
// the IC-705 LAN audio channel) and drop it in place of the synthetic source — the
// rest of the page stays unchanged.

// ---------- DSP / display constants ----------

const HOP_SIZE  = 256;             // samples advanced per waterfall column
const FFT_SIZE  = 1024;            // FFT window (freq resolution = sampleRate/FFT_SIZE)
const BIN_COUNT = FFT_SIZE >> 1;   // displayed bins (0 .. sampleRate/2)
const ROWS      = 260;             // waterfall height in pixels

let sampleRate = 8000;             // default; a real source may re-announce its rate

// Bench-test without a radio: open /data?test=1 to feed the synthetic source.
const TEST_MODE = new URLSearchParams(location.search).has("test");

// ---------- DOM ----------

const modeSelect    = document.getElementById("modeSelect");
const hdrFreq       = document.getElementById("hdrFreq");
const hdrMode       = document.getElementById("hdrMode");
const hdrPwr        = document.getElementById("hdrPwr");
const wfWrap        = document.querySelector(".wf-wrap");
const wfCanvas      = document.getElementById("waterfall");
const wfOverlay     = document.getElementById("wfOverlay");
const wfAxis        = document.getElementById("wfAxis");
const wfBanner      = document.getElementById("wfBanner");
const decoderOut    = document.getElementById("decoderOut");
const decoderStatus = document.getElementById("decoderStatus");
const clearDecoder  = document.getElementById("clearDecoder");
const encoderIn     = document.getElementById("encoderIn");
const encoderStatus = document.getElementById("encoderStatus");
const toneReadout   = document.getElementById("toneReadout");
const txButton      = document.getElementById("txButton");
const statusLink    = document.getElementById("statusLink");
const statusSample  = document.getElementById("statusSample");

const wfCtx = wfCanvas.getContext("2d");
const ovCtx = wfOverlay.getContext("2d");
const axCtx = wfAxis.getContext("2d");

wfCanvas.width = BIN_COUNT;
wfCanvas.height = ROWS;

// ---------- page state ----------

const state = {
  connected: false,
  transceiverType: "",
  power: false,
  frequency: 0,
  mode: "",
  rfPower: 0,
  tx: false,
  txToneHz: 1500,
};

// ============================================================================
//  Modem interface — contracts for future decoders (RX) and encoders (TX).
//  Milestone 1 ships the registry and base classes only; real modems plug in
//  later by subclassing Decoder / Encoder and calling registerModem().
// ============================================================================

// Decoder (RX): consumes audio samples, emits decoded text.
class Decoder {
  constructor(sr) { this.sampleRate = sr; this._onText = null; }
  pushSamples(_float32) {}          // called with every incoming audio block
  onText(cb) { this._onText = cb; return this; }
  _emit(text) { if (this._onText) this._onText(text); }
  reset() {}
}

// Encoder (TX): turns text into audio. In M3 the produced audio is streamed to
// the radio's LAN TX audio channel and PTT is keyed.
class Encoder {
  constructor(sr) { this.sampleRate = sr; this._onAudio = null; this.toneHz = 1500; }
  setToneOffset(hz) { this.toneHz = hz; }
  encode(_text) {}                  // -> should call this._emit(Float32Array)
  onAudio(cb) { this._onAudio = cb; return this; }
  _emit(float32) { if (this._onAudio) this._onAudio(float32); }
}

// id -> { label, Decoder?, Encoder?, minTone?, maxTone? }
const Modems = {};
function registerModem(id, def) { Modems[id] = def; }

// Placeholder catalogue — real decoders/encoders are wired in later milestones.
// (Decoder/Encoder left null => the panels show "not implemented".)
// Test-tone encoder: a steady 3 s tone at the TX offset — validates the TX audio
// path end-to-end (browser -> ESP32 -> radio) without needing a real modem.
class TestToneEncoder extends Encoder {
  encode(_text) {
    const sr = this.sampleRate, dur = 3;
    const out = new Float32Array(Math.floor(sr * dur));
    const dphi = 2 * Math.PI * this.toneHz / sr;
    for (let i = 0; i < out.length; i++) out[i] = 0.5 * Math.sin(i * dphi);
    this._emit(out);
  }
}

registerModem("testtone", { label: "Test tón (TX)", Decoder: null, Encoder: TestToneEncoder });
registerModem("rtty45", { label: "RTTY 45",  Decoder: null, Encoder: null });
registerModem("psk31",  { label: "PSK31",    Decoder: null, Encoder: null });
registerModem("ft8",    { label: "FT8",      Decoder: null, Encoder: null });
registerModem("ft4",    { label: "FT4",      Decoder: null, Encoder: null });
registerModem("cw",     { label: "CW skimmer", Decoder: null, Encoder: null });

let activeMode = "psk31";
let activeDecoder = null;
let activeEncoder = null;

// ============================================================================
//  FFT — iterative radix-2 Cooley-Tukey (in-place), precomputed twiddles.
// ============================================================================

function makeFFT(n) {
  const cos = new Float32Array(n >> 1), sin = new Float32Array(n >> 1);
  for (let i = 0; i < (n >> 1); i++) {
    cos[i] = Math.cos(-2 * Math.PI * i / n);
    sin[i] = Math.sin(-2 * Math.PI * i / n);
  }
  const rev = new Uint32Array(n);
  const bits = Math.round(Math.log2(n));
  for (let i = 0; i < n; i++) {
    let x = i, r = 0;
    for (let j = 0; j < bits; j++) { r = (r << 1) | (x & 1); x >>= 1; }
    rev[i] = r;
  }
  return function (re, im) {
    for (let i = 0; i < n; i++) {
      const j = rev[i];
      if (j > i) {
        let t = re[i]; re[i] = re[j]; re[j] = t;
        t = im[i]; im[i] = im[j]; im[j] = t;
      }
    }
    for (let size = 2; size <= n; size <<= 1) {
      const half = size >> 1, step = n / size;
      for (let i = 0; i < n; i += size) {
        for (let k = 0; k < half; k++) {
          const ci = k * step, c = cos[ci], s = sin[ci];
          const a = i + k, b = a + half;
          const tr = re[b] * c - im[b] * s;
          const ti = re[b] * s + im[b] * c;
          re[b] = re[a] - tr; im[b] = im[a] - ti;
          re[a] += tr;        im[a] += ti;
        }
      }
    }
  };
}

const fft  = makeFFT(FFT_SIZE);
const re    = new Float32Array(FFT_SIZE);
const im    = new Float32Array(FFT_SIZE);
const mags  = new Float32Array(BIN_COUNT);  // magnitudes (dB)
const hann  = new Float32Array(FFT_SIZE);
for (let i = 0; i < FFT_SIZE; i++) hann[i] = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / (FFT_SIZE - 1));

// Circular sample buffer: incoming blocks vary in size (256 synthetic, ~1364 from
// the radio), so feed sample-by-sample and run one FFT every HOP_SIZE samples.
const ring = new Float32Array(FFT_SIZE);
let ringPos = 0;      // next write index
let hopCount = 0;     // samples since last FFT

// ============================================================================
//  Waterfall rendering
// ============================================================================

// black -> blue -> cyan -> green -> yellow -> red -> white
const CMAP_STOPS = [
  [0.00, 0, 0, 0], [0.20, 0, 0, 90], [0.40, 0, 120, 190], [0.55, 0, 185, 120],
  [0.70, 190, 205, 0], [0.85, 235, 95, 0], [1.00, 255, 255, 255],
];
function colormap(v) {
  v = v < 0 ? 0 : v > 1 ? 1 : v;
  for (let i = 1; i < CMAP_STOPS.length; i++) {
    if (v <= CMAP_STOPS[i][0]) {
      const a = CMAP_STOPS[i - 1], b = CMAP_STOPS[i];
      const t = (v - a[0]) / (b[0] - a[0]);
      return [a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t, a[3] + (b[3] - a[3]) * t];
    }
  }
  return [255, 255, 255];
}

// Adaptive intensity scaling (AGC) so the image stays readable without tuning.
let agcMin = -90, agcMax = -30;

function processWindow() {
  // unwrap the circular buffer oldest-first into the windowed FFT input
  for (let i = 0; i < FFT_SIZE; i++) { re[i] = ring[(ringPos + i) % FFT_SIZE] * hann[i]; im[i] = 0; }
  fft(re, im);
  let cmin = 1e9, cmax = -1e9;
  for (let i = 0; i < BIN_COUNT; i++) {
    const m = Math.hypot(re[i], im[i]) / BIN_COUNT;
    const db = 20 * Math.log10(m + 1e-9);
    mags[i] = db;
    if (db < cmin) cmin = db;
    if (db > cmax) cmax = db;
  }
  agcMin += (cmin - agcMin) * 0.05;
  agcMax += (cmax - agcMax) * 0.05;
  if (agcMax - agcMin < 12) agcMax = agcMin + 12;
  drawColumn();
}

function drawColumn() {
  // scroll existing image down 1px, then paint the new spectrum row at the top
  wfCtx.drawImage(wfCanvas, 0, 0, BIN_COUNT, ROWS - 1, 0, 1, BIN_COUNT, ROWS - 1);
  const row = wfCtx.createImageData(BIN_COUNT, 1);
  const span = agcMax - agcMin;
  for (let i = 0; i < BIN_COUNT; i++) {
    const c = colormap((mags[i] - agcMin) / span);
    const o = i << 2;
    row.data[o] = c[0]; row.data[o + 1] = c[1]; row.data[o + 2] = c[2]; row.data[o + 3] = 255;
  }
  wfCtx.putImageData(row, 0, 0);
}

// ---------- overlay (markers) + frequency axis ----------

function freqToX(hz, width) { return (hz / (sampleRate / 2)) * width; }

function drawOverlay() {
  const w = wfOverlay.width, h = wfOverlay.height;
  ovCtx.clearRect(0, 0, w, h);
  // TX tone marker (green) — click the waterfall to move it
  const x = freqToX(state.txToneHz, w);
  ovCtx.strokeStyle = "rgba(0, 220, 120, 0.9)";
  ovCtx.lineWidth = 1;
  ovCtx.beginPath(); ovCtx.moveTo(x + 0.5, 0); ovCtx.lineTo(x + 0.5, h); ovCtx.stroke();
  ovCtx.fillStyle = "rgba(0, 220, 120, 0.95)";
  ovCtx.font = "11px Tahoma, sans-serif";
  ovCtx.fillText("TX " + state.txToneHz + " Hz", Math.min(x + 4, w - 70), 12);
}

function drawAxis() {
  const w = wfAxis.width, h = wfAxis.height;
  axCtx.clearRect(0, 0, w, h);
  axCtx.fillStyle = "#91a2b9";
  axCtx.strokeStyle = "#1f2731";
  axCtx.font = "10px Tahoma, sans-serif";
  const nyquist = sampleRate / 2;
  const stepHz = nyquist > 6000 ? 1000 : 500;
  for (let f = 0; f <= nyquist; f += stepHz) {
    const x = freqToX(f, w);
    axCtx.beginPath(); axCtx.moveTo(x + 0.5, 0); axCtx.lineTo(x + 0.5, 5); axCtx.stroke();
    const label = f >= 1000 ? (f / 1000) + "k" : String(f);
    const tx = Math.max(1, Math.min(w - 22, x - 8));
    axCtx.fillText(label, tx, 16);
  }
}

// Match overlay/axis pixel width to their on-screen size for crisp lines/text.
function resizeCanvases() {
  const w = Math.max(1, Math.round(wfWrap.clientWidth));
  wfOverlay.width = w; wfOverlay.height = ROWS;
  wfAxis.width = w;    wfAxis.height = 22;
  drawOverlay();
  drawAxis();
}

// ============================================================================
//  Audio source interface  (M1 = synthetic, M2 = WsAudioSource)
// ============================================================================

class AudioSource {
  constructor(sr) { this.sampleRate = sr; this._cb = null; }
  onSamples(cb) { this._cb = cb; return this; }
  start() {}
  stop() {}
}

// Synthetic stand-in: noise floor + a few drifting tones, emitted in real time.
class SyntheticAudioSource extends AudioSource {
  constructor(sr) { super(sr); this._timer = null; this._ph = [0, 0, 0]; }
  start() {
    if (this._timer) return;
    const hopMs = (HOP_SIZE / this.sampleRate) * 1000;
    this._timer = setInterval(() => this._tick(), hopMs);
  }
  stop() { clearInterval(this._timer); this._timer = null; }
  _tick() {
    const n = HOP_SIZE, sr = this.sampleRate, out = new Float32Array(n);
    const now = performance.now() / 1000;
    const f1 = 1000 + 400 * Math.sin(now * 0.15);   // sweeping tone
    const f2 = 1500;                                 // steady carrier
    const f3 = 2200 + 300 * Math.sin(now * 0.4);     // second sweep
    for (let i = 0; i < n; i++) {
      this._ph[0] += 2 * Math.PI * f1 / sr;
      this._ph[1] += 2 * Math.PI * f2 / sr;
      this._ph[2] += 2 * Math.PI * f3 / sr;
      out[i] = 0.05 * (Math.random() * 2 - 1)
             + 0.35 * Math.sin(this._ph[0])
             + 0.25 * Math.sin(this._ph[1])
             + 0.20 * Math.sin(this._ph[2]);
    }
    if (this._cb) this._cb(out, sr);
  }
}

// G.711 uLaw byte -> Float32 sample (must match firmware AUDIO_RX_CODEC = 0x01).
const ULAW = (() => {
  const t = new Float32Array(256);
  for (let i = 0; i < 256; i++) {
    const u = ~i & 0xff;
    const mant = u & 0x0f, exp = (u >> 4) & 0x07;
    let s = ((mant << 3) + 0x84) << exp;
    s -= 0x84;
    t[i] = ((u & 0x80) ? -s : s) / 32768;
  }
  return t;
})();

// Float32 sample (-1..1) -> G.711 uLaw byte, for TX (matches firmware AUDIO_TX_CODEC).
function linToUlaw(sample) {
  const BIAS = 0x84, CLIP = 32635;
  let s = Math.max(-1, Math.min(1, sample)) * 32768;
  let sign = s < 0 ? 0x80 : 0;
  if (s < 0) s = -s;
  if (s > CLIP) s = CLIP;
  s += BIAS;
  let exp = 7;
  for (let mask = 0x4000; (s & mask) === 0 && exp > 0; exp--, mask >>= 1) {}
  const mant = (s >> (exp + 3)) & 0x0f;
  return (~(sign | (exp << 4) | mant)) & 0xff;
}

// Real RX audio: binary WebSocket to the ESP32 (port 83, /audiows) carrying uLaw
// bytes. Decodes to Float32 and feeds the same onSamples pipeline as the synthetic
// source. Sample rate must match the firmware's AUDIO_RX_SAMPLE (8000 Hz).
class WsAudioSource extends AudioSource {
  constructor(sr) { super(sr); this._ws = null; }
  start() {
    if (this._ws && (this._ws.readyState === WebSocket.OPEN || this._ws.readyState === WebSocket.CONNECTING)) return;
    const proto = location.protocol === "https:" ? "wss://" : "ws://";
    const ws = new WebSocket(proto + location.hostname + ":83/audiows");
    ws.binaryType = "arraybuffer";
    ws.onmessage = (ev) => {
      const bytes = new Uint8Array(ev.data);
      const out = new Float32Array(bytes.length);
      for (let i = 0; i < bytes.length; i++) out[i] = ULAW[bytes[i]];
      if (this._cb) this._cb(out, this.sampleRate);
    };
    ws.onclose = () => { if (this._ws === ws) this._ws = null; };
    ws.onerror = () => {};
    this._ws = ws;
  }
  stop() { if (this._ws) { try { this._ws.close(); } catch (e) {} this._ws = null; } }
  // TX path: text control ("TX1"/"TX0") and binary uLaw audio to the ESP32.
  sendText(s)   { if (this._ws && this._ws.readyState === WebSocket.OPEN) this._ws.send(s); }
  sendBinary(u8){ if (this._ws && this._ws.readyState === WebSocket.OPEN) this._ws.send(u8); }
}

let audioSource = null;
let audioSourceKind = null;   // 'ws' (real) | 'synth' (test)
let lastWsAudioMs = 0;        // last time real audio arrived (proof the LAN link is up)

// Single entry point for every audio block, whatever the source.
function onAudioSamples(block, sr) {
  if (audioSourceKind === "ws") lastWsAudioMs = performance.now();
  if (sr !== sampleRate) { sampleRate = sr; drawAxis(); drawOverlay(); statusSample.textContent = sr + " Hz"; }
  // feed the circular buffer; one FFT column per HOP_SIZE samples
  for (let i = 0; i < block.length; i++) {
    ring[ringPos] = block[i];
    ringPos = (ringPos + 1) % FFT_SIZE;
    if (++hopCount >= HOP_SIZE) { hopCount = 0; processWindow(); }
  }
  // feed the active decoder (raw, un-windowed samples)
  if (activeDecoder) activeDecoder.pushSamples(block);
}

// Start (or keep running) the requested source kind; switch cleanly if it changed.
function ensureAudio(kind) {
  if (audioSourceKind === kind && audioSource) { audioSource.start(); return; }
  if (audioSource) audioSource.stop();
  sampleRate = 8000;
  audioSource = (kind === "ws")
    ? new WsAudioSource(sampleRate).onSamples(onAudioSamples)
    : new SyntheticAudioSource(sampleRate).onSamples(onAudioSamples);
  audioSourceKind = kind;
  statusSample.textContent = sampleRate + " Hz";
  audioSource.start();
}

function stopAudio() {
  if (audioSource) audioSource.stop();
  audioSource = null;
  audioSourceKind = null;
}

// ---------- TX (M3): pace encoder audio to the radio over the audio WS ----------
let txActive = false, txTimer = null, txCapTimer = null;

function wsSendText(s)    { if (audioSourceKind === "ws" && audioSource) audioSource.sendText(s); }
function wsSendBinary(u8) { if (audioSourceKind === "ws" && audioSource) audioSource.sendBinary(u8); }

// Called from the txButton: ask the active encoder to produce audio.
function startTransmit() {
  if (txActive || !activeEncoder) return;
  if (audioSourceKind !== "ws") { encoderStatus.textContent = "TX jen v LAN režimu"; return; }
  activeEncoder.setToneOffset(state.txToneHz);
  activeEncoder.encode(encoderIn.value);   // -> onAudio -> txStream()
}

// Pace the encoder's Float32 audio out in real time: 160-sample (20 ms) uLaw chunks.
function txStream(samples) {
  if (txActive || !samples || !samples.length) return;
  txActive = true;
  txButton.disabled = true; txButton.textContent = "Vysílá…";
  encoderStatus.textContent = "TX";
  renderHeader();                             // header red immediately (we are TX)
  wsSendText("TX1");                          // key PTT
  const CHUNK = 160;
  const startAt = performance.now() + 200;    // PTT lead
  let pos = 0;
  txTimer = setInterval(() => {
    if (performance.now() < startAt) return;
    if (pos >= samples.length) { txEnd(); return; }
    const end = Math.min(pos + CHUNK, samples.length);
    const buf = new Uint8Array(end - pos);
    for (let i = pos; i < end; i++) buf[i - pos] = linToUlaw(samples[i]);
    wsSendBinary(buf);
    pos = end;
  }, 20);
  txCapTimer = setTimeout(txEnd, 60000);      // hard safety cap
}

function txEnd() {
  if (txTimer) { clearInterval(txTimer); txTimer = null; }
  if (txCapTimer) { clearTimeout(txCapTimer); txCapTimer = null; }
  wsSendText("TX0");                           // un-key PTT
  txActive = false;
  txButton.disabled = false; txButton.textContent = "Vysílat";
  encoderStatus.textContent = "aktivní";
  renderHeader();                              // clear TX red
}

// ============================================================================
//  Mode selection + decoder/encoder wiring
// ============================================================================

function populateModes() {
  modeSelect.innerHTML = "";
  for (const id in Modems) {
    const opt = document.createElement("option");
    opt.value = id;
    opt.textContent = Modems[id].label;
    modeSelect.appendChild(opt);
  }
  const saved = localStorage.getItem("data.mode");
  if (saved && Modems[saved]) activeMode = saved;
  modeSelect.value = activeMode;
}

function selectMode(id) {
  if (!Modems[id]) return;
  activeMode = id;
  localStorage.setItem("data.mode", id);
  const def = Modems[id];

  // (Re)build decoder
  activeDecoder = null;
  if (def.Decoder) {
    activeDecoder = new def.Decoder(sampleRate).onText(appendDecoded);
    decoderStatus.textContent = "aktivní";
  } else {
    decoderStatus.textContent = "dekodér pro „" + def.label + "“ zatím není implementován";
  }

  // (Re)build encoder
  activeEncoder = null;
  if (def.Encoder) {
    activeEncoder = new def.Encoder(sampleRate);
    activeEncoder.onAudio((f) => txStream(f));
    activeEncoder.setToneOffset(state.txToneHz);
    encoderStatus.textContent = "aktivní";
    txButton.disabled = false;
  } else {
    encoderStatus.textContent = "kodér pro „" + def.label + "“ zatím není implementován";
    txButton.disabled = true;
  }
}

function appendDecoded(text) {
  decoderOut.textContent += text;
  decoderOut.scrollTop = decoderOut.scrollHeight;
}

function setTxTone(hz) {
  hz = Math.max(0, Math.min(Math.round(sampleRate / 2), Math.round(hz)));
  state.txToneHz = hz;
  toneReadout.textContent = String(hz);
  if (activeEncoder) activeEncoder.setToneOffset(hz);
  drawOverlay();
}

// ============================================================================
//  Header + gating (driven by GET /state)
// ============================================================================

function formatFreq(hz) {
  hz = Math.max(0, Math.floor(hz));
  const mhz = Math.floor(hz / 1e6);
  const rest = hz % 1e6;
  return `${mhz}.${String(Math.floor(rest / 1000)).padStart(3, "0")}.${String(rest % 1000).padStart(3, "0")}`;
}

function formatRfPower(raw) {
  const n = Math.max(0, Math.min(255, Number(raw) || 0));
  return `${Math.round((n * 100) / 255)} %`;
}

function renderHeader() {
  hdrFreq.textContent = formatFreq(state.frequency);
  hdrFreq.classList.toggle("tx", state.tx || txActive);   // txActive = we are transmitting now
  hdrFreq.classList.toggle("off", !state.power);
  hdrMode.textContent = state.mode || "---";
  hdrPwr.textContent = state.power ? formatRfPower(state.rfPower) : "-- %";
}

function updateGating() {
  const lanConnected = state.transceiverType === "IC-705-LAN" && state.connected;
  // Audio actively arriving is itself proof the LAN link is up — keep the waterfall
  // active even if a /state poll momentarily reports otherwise.
  const audioAlive = audioSourceKind === "ws" && (performance.now() - lastWsAudioMs) < 2000;
  if (lanConnected || audioAlive) {
    wfBanner.hidden = true; statusLink.textContent = lanConnected ? "Link LAN" : "Link LAN*"; ensureAudio("ws");
  } else if (TEST_MODE) {
    wfBanner.hidden = true; statusLink.textContent = "Link TEST"; ensureAudio("synth");
  } else {
    wfBanner.hidden = false; statusLink.textContent = "Link no-LAN"; stopAudio();
  }
}

function applyServerState(s) {
  if (!s) return;
  state.connected = Boolean(s.connected);
  state.transceiverType = s.transceiverType || state.transceiverType;
  state.power = Boolean(s.power);
  state.frequency = Number(s.frequency) || 0;
  state.mode = s.mode || state.mode;
  state.rfPower = Number(s.rfPower);
  state.tx = Boolean(s.tx);
  renderHeader();
  updateGating();
}

function startPolling() {
  const poll = () => {
    fetch("/state")
      .then(r => r.json())
      .then(applyServerState)      // only a SUCCESSFUL read drives gating/audio
      .catch(() => {});            // a dropped poll is transient — never tear audio down
  };
  poll();
  setInterval(poll, 400);
}

// ============================================================================
//  Init
// ============================================================================

function init() {
  populateModes();
  selectMode(activeMode);
  setTxTone(state.txToneHz);
  resizeCanvases();

  modeSelect.addEventListener("change", () => selectMode(modeSelect.value));
  clearDecoder.addEventListener("click", () => { decoderOut.textContent = ""; });
  wfWrap.addEventListener("click", (e) => {
    const rect = wfWrap.getBoundingClientRect();
    setTxTone(((e.clientX - rect.left) / rect.width) * (sampleRate / 2));
  });
  txButton.addEventListener("click", startTransmit);
  window.addEventListener("resize", resizeCanvases);
  // safety: never leave the radio transmitting if the page is hidden/closed
  window.addEventListener("pagehide", () => { if (txActive) txEnd(); });
  document.addEventListener("visibilitychange", () => { if (document.hidden && txActive) txEnd(); });

  startPolling();
}

init();
