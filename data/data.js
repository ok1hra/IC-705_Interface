"use strict";

// JS8Call page controller. Modem DSP lives in a Worker; this file owns only the
// public modem contracts, radio/audio adapters and DOM projection.

const PAGE_PARAMS = new URLSearchParams(location.search);
const TEST_MODE = PAGE_PARAMS.has("test");
const ASSET_REV = "20260719d";
const assetUrl = path => `${path}?v=${ASSET_REV}`;
const TRX_HELP_SEEN_KEY = "ic705.data.trx-help-seen.v1";
const AUDIO_WS_PORT = Number(new URLSearchParams(location.search).get("audioPort")) || 83;
const RX_LOW = 500, RX_HIGH = 2700, HB_HIGH = 1000, AUDIO_RATE = 8000;
const FFT_SIZE = 4096, HOP_SIZE = 2048;
const SPEED_TO_MODE = {A:0, B:1, C:2, E:4, I:8};
const MODE_TO_SPEED = {0:"A", 1:"B", 2:"C", 4:"E", 8:"I"};
const MODE_PERIOD_SECONDS = {0:15, 1:10, 2:6, 4:30, 8:4};
const ACTIVITY_FREQUENCY_TOLERANCE_HZ = 2000;

function emptyActivity() {
  return {messages:[], calls:[], timing:[], frames:[], channels:[]};
}

class AudioSource {
  constructor(sampleRate) { this.sampleRate = sampleRate; this._cb = null; }
  onSamples(callback) { this._cb = callback; return this; }
  start() {}
  stop() {}
}

class Decoder {
  constructor(sampleRate) { this.sampleRate = sampleRate; this._onText = null; this._onEvent = null; }
  pushSamples(_samples, _metadata) {}
  onText(callback) { this._onText = callback; return this; }
  onEvent(callback) { this._onEvent = callback; return this; }
  configure(_options) { return this; }
  _emit(text) { if (this._onText) this._onText(text); }
  reset(_reason) {}
}

class Encoder {
  constructor(sampleRate) { this.sampleRate = sampleRate; this.toneHz = 1500; this._onAudio = null; this._onEvent = null; }
  setToneOffset(hz) { this.toneHz = hz; return this; }
  configure(_options) { return this; }
  encode(_text, _context) {}
  onAudio(callback) { this._onAudio = callback; return this; }
  onEvent(callback) { this._onEvent = callback; return this; }
  _emit(samples, rate, metadata) { if (this._onAudio) this._onAudio(samples, rate, metadata); }
  abort() {}
}

const Modems = {};
function registerModem(id, definition) {
  if (!id || !definition || !definition.label) throw new Error("Invalid modem registration");
  Modems[id] = definition;
}

// Keep the documented modem extension contract reachable after the release
// build mangles private top-level names for the tight SPIFFS image.
Object.assign(globalThis, {AudioSource, Modems, registerModem, Decoder, Encoder});

const $ = id => document.getElementById(id);
const dom = {
  radioBar:document.querySelector(".radio-bar"), trxFrequency:$("trxFrequency"),
  trxFrequencyValue:$("trxFrequencyValue"), trxMode:$("trxMode"), trxDot:$("trxDot"),
  trxHelpButton:$("trxHelpButton"), trxHelpDialog:$("trxHelpDialog"),
  trxHelpModeWarning:$("trxHelpModeWarning"),
  frequencyMenu:$("frequencyMenu"), linkState:$("linkState"), operatorState:$("operatorState"),
  trxReconnect:$("trxReconnect"),
  utcClock:$("utcClock"), timingState:$("timingState"), modeSelect:$("modeSelect"),
  modemState:$("modemState"), js8:$("js8Interface"),
  spectrumSummary:$("spectrumSummary"), waterfall:$("waterfall"), canvas:$("waterfallCanvas"),
  overlay:$("waterfallOverlay"), recipient:$("recipient"), txSpeed:$("txSpeed"),
  slotMeter:$("slotMeter"), slotLabel:$("slotLabel"), slotFill:$("slotFill"),
  txSpeedResolved:$("txSpeedResolved"), recipientClear:$("recipientClear"),
  txOffset:$("txOffset"), audioLevel:$("audioLevel"), txSummary:$("txSummary"),
  heartbeat:$("heartbeatButton"), heartbeatOffset:$("heartbeatOffset"),
  tune:$("tuneButton"), tuneLabel:$("tuneLabel"), tuneOffset:$("tuneOffset"),
  sessionCall:$("sessionCall"), sessionMeta:$("sessionMeta"), abort:$("abortButton"),
  txSessionMode:$("txSessionMode"), txSessionModeHint:$("txSessionModeHint"),
  txPayload:$("txPayload"),
  chatSession:$("chatSession"), emailSession:$("emailSession"), binSession:$("binSession"),
  chat:$("chatThread"), composer:$("composer"), message:$("messageInput"), send:$("sendButton"),
  emailComposer:$("emailComposer"), emailAddress:$("emailAddress"),
  emailGateway:$("emailGateway"), emailGatewayAdd:$("emailGatewayAdd"),
  emailGatewayEdit:$("emailGatewayEdit"), emailGatewayDelete:$("emailGatewayDelete"),
  emailGatewayDetails:$("emailGatewayDetails"), emailMessage:$("emailMessage"),
  emailBudget:$("emailBudget"), emailPreview:$("emailPreview"), emailError:$("emailError"),
  emailStatus:$("emailStatus"), emailSend:$("emailSend"),
  emailGatewayDialog:$("emailGatewayDialog"), emailGatewayForm:$("emailGatewayForm"),
  emailGatewayDialogTitle:$("emailGatewayDialogTitle"), emailGatewayName:$("emailGatewayName"),
  emailGatewayTarget:$("emailGatewayTarget"), emailGatewayDial:$("emailGatewayDial"),
  emailGatewayOffset:$("emailGatewayOffset"), emailGatewayFormat:$("emailGatewayFormat"),
  emailGatewayTemplateRow:$("emailGatewayTemplateRow"), emailGatewayTemplate:$("emailGatewayTemplate"),
  emailGatewayMaxBody:$("emailGatewayMaxBody"), emailGatewayPolicy:$("emailGatewayPolicy"),
  emailGatewayError:$("emailGatewayError"), emailConfirmDialog:$("emailConfirmDialog"),
  emailConfirmGateway:$("emailConfirmGateway"), emailConfirmFrequency:$("emailConfirmFrequency"),
  emailConfirmOffset:$("emailConfirmOffset"), emailConfirmFrames:$("emailConfirmFrames"),
  emailConfirmPayload:$("emailConfirmPayload"),
  binComposer:$("binComposer"), binRecipient:$("binRecipient"), binFile:$("binFile"),
  binFileDetails:$("binFileDetails"), binPeerExpected:$("binPeerExpected"),
  binError:$("binError"), binDraftStatus:$("binDraftStatus"), binOffer:$("binOffer"),
  binTransferPanel:$("binTransferPanel"), binTransferTitle:$("binTransferTitle"),
  binTransferPeer:$("binTransferPeer"), binTransferState:$("binTransferState"),
  binProgress:$("binProgress"), binProgressText:$("binProgressText"),
  binTransferRate:$("binTransferRate"), binLastActivity:$("binLastActivity"),
  binTransferId:$("binTransferId"), binTransferHash:$("binTransferHash"),
  binProtocolMessage:$("binProtocolMessage"), binTransferLog:$("binTransferLog"),
  binPause:$("binPause"), binResume:$("binResume"), binStop:$("binStop"),
  binDownload:$("binDownload"), binConfirmDialog:$("binConfirmDialog"),
  binConfirmPeer:$("binConfirmPeer"), binConfirmFile:$("binConfirmFile"),
  binConfirmProfile:$("binConfirmProfile"), binConfirmPlan:$("binConfirmPlan"),
  binConfirmHash:$("binConfirmHash"), binCopyHash:$("binCopyHash"), binIncomingDialog:$("binIncomingDialog"),
  binIncomingPeer:$("binIncomingPeer"), binIncomingFile:$("binIncomingFile"),
  binIncomingSize:$("binIncomingSize"), binIncomingHash:$("binIncomingHash"),
  messagePresetsButton:$("messagePresetsButton"), messagePresetsMenu:$("messagePresetsMenu"),
  traffic:$("traffic"), trafficSummary:$("trafficSummary"), stationRows:$("stationRows"),
  trafficSection:document.querySelector('[data-section="traffic"]'),
  trafficFilter:document.querySelector(".traffic-filter"),
  stationsSection:document.querySelector('[data-section="stations"]'),
  stationHead:document.querySelector(".traffic-table thead"), reply:document.querySelector('[data-section="reply"]'),
  stationSummary:$("stationSummary"), myCall:$("myCall"), myGrid:$("myGrid"),
  followSpeed:$("followSpeed"), clockCorrection:$("clockCorrection"), autoTiming:$("autoTiming"),
  txGain:$("txGain"), txSafety:$("txSafety"), storageState:$("storageState"),
  resetSettings:$("resetSettings"), settingsSummary:$("settingsSummary"),
  diagnosticSummary:$("diagnosticSummary"), diagnostics:$("diagnostics"),
  lanRequired:$("lanRequired"), lanRequiredDetail:$("lanRequiredDetail"),
  startup:$("startupLoader"), startupProgress:$("startupProgress"),
  startupPercent:$("startupPercent"), startupLabel:$("startupLabel"),
  startupDetail:$("startupDetail"), startupRetry:$("startupRetry")
};

const loaded = Js8Settings.load(localStorage);
let settings = loaded.settings;
const emailState = {gateways:Js8Email.load(localStorage),selectedId:"",editingId:"",
  pendingDraft:null,activeOutgoing:null,status:"Draft is not stored in message history."};
if(emailState.gateways.length)emailState.selectedId=emailState.gateways[0].id;
const transferStore=new Js8FileTransfer.TransferStore();
const binState={sessions:[],active:null,prepared:null,preparing:false,peerDraft:"",
  txQueue:[],txCurrent:null,responseTimer:null,incomingOffer:null,nackParts:new Map(),
  lastProtocol:"",storageError:"",restored:false};
const state = {
  radio:{connected:false, lanStatus:"connecting", transceiverType:"", power:false, frequency:0, mode:"", tx:false, rfPower:0},
  activeMode:settings.activeModem, selectedCall:"", activity:emptyActivity(),
  activityFrequency:0, activitySessions:[],
  conversations:{}, audioStatus:"stopped", decoderStatus:"loading", txStatus:"idle",
  txState:null, txWasmReady:false, pendingFrequency:null, lastAudioMs:0,
  startup:{ready:false, failed:false, progress:0, label:"Loading JS8Call modem",
    detail:"Preparing modem components…"},
  stationSort:{key:"lastSlotUtcMs", direction:"desc"}, trafficFilter:"all", testActivityLocked:false,
  txSessionMode:"CHAT", audioDb:-90, tuneActive:false, spectrumWasTransmitting:false,
  help:{incompatibleActive:false},
  lanConfig:{checked:false, ready:false, detail:""},
  ownCallAttention:{call:"", messages:new Set(), stations:new Set()},
  activeOutgoing:null, lastOutgoing:null,
  settingsDraft:{myCall:null,grid:null,txGain:null}, reconnectPending:false,
};
let audioSource = null, activeDecoder = null, activeEncoder = null, txTick = null;
let radioPollInFlight = false;
let frequencyMenuKey = "";
const decoderActivitySeen = {messages:new Set(), frames:new Set(), calls:new Map()};

function esc(value) {
  return String(value == null ? "" : value).replace(/[&<>\"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"})[c]);
}
function signed(value) { const n = Math.round(Number(value) || 0); return `${n >= 0 ? "+" : ""}${n}`; }
function formatJs8Snr(value) {
  const n=Math.max(-60,Math.min(60,Math.round(Number(value)||0)));
  return `${n>=0?"+":"-"}${String(Math.abs(n)).padStart(2,"0")}`;
}
function cqType(text) {
  const normalized=String(text||"").trim().toUpperCase();
  return ["CQ CQ CQ","CQ DX","CQ QRP","CQ CONTEST","CQ FIELD","CQ FD","CQ CQ","CQ"].includes(normalized) ? normalized : "";
}
function formatFrequency(hz) { return Js8TrxPresets.formatFrequency(hz || 0); }
function speedDetail(mode) {
  const number=Number(mode), speed=MODE_TO_SPEED[number] || "?", seconds=MODE_PERIOD_SECONDS[number];
  return seconds ? `${speed} · ${seconds} s` : speed;
}
function callOf(message) { return (message.callsigns || []).find(call => call && !call.startsWith("@") && call !== currentJs8().myCall) || ""; }
function currentJs8() { return settings.modems.js8call; }
function sameCall(left,right) {
  return Boolean(right) && String(left||"").toUpperCase()===String(right).toUpperCase();
}
function messageMentionsCall(message,call) {
  return Boolean(call) && (message.callsigns||[]).some(value=>sameCall(value,call));
}
function ownCallText(text,call) {
  const html=esc(text);
  if(!call)return html;
  const escaped=String(call).replace(/[.*+?^${}()|[\]\\]/g,"\\$&");
  return html.replace(new RegExp(`(^|[^A-Z0-9/])(${escaped})(?=$|[^A-Z0-9/])`,"gi"),
    '$1<span class="own-callsign" data-own-call="true">$2</span>');
}

function activityMessageKey(item) {
  return `${item.firstSlotUtcMs || 0}|${item.lastSlotUtcMs || 0}|${item.submode}|${item.offsetHz}|${item.text}|${(item.raw || []).join("")}`;
}
function activityFrameKey(item) {
  return `${item.slotUtcMs || 0}|${item.submode}|${item.offsetHz}|${item.raw}`;
}
function activityCallSignature(item) {
  return `${item.lastSlotUtcMs || 0}|${item.snr}|${item.offsetHz}|${item.submode}|${item.dtMs}|${item.quality}|${item.grid || ""}`;
}
function activitySessionFor(frequency, create=true) {
  const hz=Number(frequency)||0;
  if(hz<=0)return null;
  let session=state.activitySessions
    .filter(item=>Math.abs(item.frequencyHz-hz)<=ACTIVITY_FREQUENCY_TOLERANCE_HZ)
    .sort((a,b)=>Math.abs(a.frequencyHz-hz)-Math.abs(b.frequencyHz-hz))[0];
  if(!session && create){
    session={frequencyHz:hz,activity:emptyActivity()};
    state.activitySessions.push(session);
  }
  return session || null;
}
function selectActivityFrequency(frequency) {
  const hz=Number(frequency)||0;
  if(hz<=0)return false;
  if(state.activityFrequency && Math.abs(state.activityFrequency-hz)<=ACTIVITY_FREQUENCY_TOLERANCE_HZ)return false;
  const session=activitySessionFor(hz);
  state.activityFrequency=session.frequencyHz;
  state.activity=session.activity;
  return true;
}
function applyDecoderActivity(snapshot) {
  if(!snapshot)return;
  if(!state.activityFrequency)selectActivityFrequency(state.radio.frequency);
  const session=activitySessionFor(state.activityFrequency,false);
  if(!session)return;
  const activity=session.activity;
  for(const item of snapshot.messages || []){
    const key=activityMessageKey(item);
    if(decoderActivitySeen.messages.has(key))continue;
    decoderActivitySeen.messages.add(key);
    activity.messages.push({...item});
    Promise.resolve(handleFileActivityMessage(item)).catch(error=>{
      binState.storageError=error.message; renderControls();
    });
  }
  if(activity.messages.length>200)activity.messages.splice(0,activity.messages.length-200);
  const calls=new Map(activity.calls.map(item=>[item.call,item]));
  for(const item of snapshot.calls || []){
    const signature=activityCallSignature(item);
    if(decoderActivitySeen.calls.get(item.call)===signature)continue;
    decoderActivitySeen.calls.set(item.call,signature);
    calls.set(item.call,{...item});
  }
  activity.calls=[...calls.values()].sort((a,b)=>String(a.call).localeCompare(String(b.call)));
  for(const item of snapshot.frames || []){
    const key=activityFrameKey(item);
    if(decoderActivitySeen.frames.has(key))continue;
    decoderActivitySeen.frames.add(key);
    activity.frames.push({...item});
  }
  if(activity.frames.length>500)activity.frames.splice(0,activity.frames.length-500);
  activity.timing=(snapshot.timing || []).map(item=>({...item}));
  activity.channels=(snapshot.channels || []).map(item=>({...item}));
  state.activity=activity;
}
function selectedMode() {
  const speed = currentJs8().speed;
  if (speed !== "AUTO") return SPEED_TO_MODE[speed];
  const station = state.activity.calls.find(item => item.call === state.selectedCall);
  return station ? Number(station.submode) : 0;
}

function settingsSnapshot() { return settings; }
function persistSettings(label = true) {
  settings.activeModem = state.activeMode;
  const saved = Js8Settings.save(localStorage, settingsSnapshot());
  settings = saved.settings;
  if (label) dom.storageState.textContent = saved.label;
  applySettingsToRuntime();
}

function applySettingsToRuntime() {
  const js8 = currentJs8();
  if (audioSource && audioSource.configure)
    audioSource.configure({clockCorrectionMs:js8.clockCorrectionMs, autoTiming:js8.autoTiming});
  if (activeEncoder) activeEncoder.configure({myCall:js8.myCall, toCall:state.selectedCall,
    mode:selectedMode(), clockCorrectionMs:js8.clockCorrectionMs});
  renderControls();
}

// ---- WASM modem registration ------------------------------------------------

const workerInit = {
  runtimeJs:assetUrl("/js8-worker-runtime.js"),
  portableJs:assetUrl("/js8-core.js"), portableWasm:assetUrl("/js8-core.wasm"),
  decoderJs:assetUrl("/js8-decoder.js"),
  decoderWasmBr:assetUrl("/js8-decoder.wasm.br"), decoderWasmSize:895356,
  protocolJs:assetUrl("/js8-protocol.js"),
  jscUrlBr:assetUrl("/js8-jsc.bin.br"), jscSize:1913889,
  brotliJs:assetUrl("/js8-brotli.js"), brotliWasm:assetUrl("/js8-brotli.wasm"),
  strictEpochAnchoring:true,
};

let txWasm = null, txModulePromise = null;
function loadTxModule() {
  if (txModulePromise) return txModulePromise;
  txModulePromise = self.createJs8Prototype({locateFile:path =>
    path.endsWith(".wasm") ? assetUrl("/js8-core.wasm") : path})
    .then(module => { txWasm = module; state.txWasmReady = true; renderControls(); return module; })
    .catch(error => { state.decoderStatus = `TX core error: ${error.message}`; renderControls(); });
  return txModulePromise;
}

function modulateFrame(frame, mode, toneHz) {
  if(frame.role==="tune"){
    const count=48000*Math.max(1,Math.min(10,Number(frame.durationSeconds)||10));
    const pcm=new Int16Array(count), amplitude=Math.round(currentJs8().txGain*32767);
    for(let i=0;i<count;i++)pcm[i]=Math.round(amplitude*Math.sin(2*Math.PI*toneHz*i/48000));
    return pcm;
  }
  if (!txWasm) throw new Error("JS8 TX core is not ready");
  const framePtr = txWasm._malloc(12);
  for (let i = 0; i < 12; i++) txWasm.HEAPU8[framePtr + i] = frame.raw.charCodeAt(i);
  const gain = currentJs8().txGain;
  const count = txWasm._js8_proto_modulate_frame48k(framePtr, frame.frameType, mode, toneHz, gain, 0, 0);
  if (count <= 0) { txWasm._free(framePtr); throw new Error("JS8 modulator rejected frame"); }
  const outputPtr = txWasm._malloc(count * 2);
  const written = txWasm._js8_proto_modulate_frame48k(framePtr, frame.frameType, mode, toneHz, gain, outputPtr, count);
  const pcm = txWasm.HEAP16.slice(outputPtr >> 1, (outputPtr >> 1) + written);
  txWasm._free(outputPtr); txWasm._free(framePtr);
  if (written !== count) throw new Error("JS8 modulator length mismatch");
  return pcm;
}

const sinkProxy = {
  prepare:(...args) => requireAudio().prepare(...args), begin:(...args) => requireAudio().begin(...args),
  write:(...args) => requireAudio().write(...args), end:(...args) => requireAudio().end(...args),
  isDrained:(...args) => audioSource ? audioSource.isDrained(...args) : false,
  complete:(...args) => requireAudio().complete(...args), abort:(...args) => audioSource && audioSource.abort(...args),
  get ptt() { return Boolean(audioSource && audioSource.ptt); }
};
function requireAudio() { if (!audioSource) throw new Error("Audio link is not connected"); return audioSource; }

const adapter = createJs8ModemAdapter({
  DecoderBase:Decoder, EncoderBase:Encoder, workerInit,
  createWorker:() => new Worker(assetUrl("/js8-worker.js")),
  getStreamId:() => audioSource ? audioSource.state().readyStreamId : 0,
  createTxController:() => new Js8Tx.TxController({buildFrames:request=>request.kind==="tune"
    ? [{raw:"",frameType:0,role:"tune",durationSeconds:TEST_MODE?2:10}]
    : Js8Protocol.buildTxFrames(request),
    encoder:modulateFrame, sink:sinkProxy, clockCorrectionMs:currentJs8().clockCorrectionMs,
    prebufferMs:1000, maxCatchupPackets:25}) // Tolerate a 500 ms mobile-browser pause before the TX slot.
});
registerModem(adapter.id, adapter.definition);

// ---- modem lifecycle --------------------------------------------------------

function populateModes() {
  const entries=Object.entries(Modems).sort(([a],[b])=>a==="js8call"?-1:b==="js8call"?1:0);
  dom.modeSelect.innerHTML = entries.map(([id, modem]) =>
    `<option value="${esc(id)}">${esc(modem.label)}</option>`).join("");
  if (!Modems[state.activeMode]) state.activeMode = "js8call";
  dom.modeSelect.value = state.activeMode;
}

function closeActiveModem() {
  if (txTick) { clearInterval(txTick); txTick = null; }
  if (activeEncoder && activeEncoder.disconnect) activeEncoder.disconnect();
  if (activeDecoder && activeDecoder.close) activeDecoder.close();
  activeDecoder = null; activeEncoder = null;
}

function selectMode(id) {
  if (!Modems[id]) return;
  closeActiveModem(); stopAudio(); state.activeMode = id; persistSettings(false);
  const modem = Modems[id];
  dom.js8.hidden = id !== "js8call";
  if (!modem.Decoder && !modem.Encoder) {
    state.startup.ready = true; state.startup.failed = false;
    dom.modemState.textContent = "Not installed"; dom.modemState.className = "modem-state unavailable";
    renderStartup(); return;
  }
  state.decoderStatus = "loading";
  state.startup = {ready:false, failed:false, progress:0,
    label:`Loading ${modem.label} modem`, detail:"Preparing modem components…"};
  renderStartup();
  activeDecoder = new modem.Decoder(AUDIO_RATE).onText(() => {}).onEvent(handleDecoderEvent);
  activeEncoder = new modem.Encoder(AUDIO_RATE).onEvent(handleEncoderEvent);
  activeEncoder.setToneOffset(currentJs8().txOffsetHz);
  applySettingsToRuntime();
  dom.modemState.textContent = "Loading decoder…"; dom.modemState.className = "modem-state";
}

function handleDecoderEvent(event) {
  let activityChanged=false;
  if (event.type === "loading") {
    state.decoderStatus = "loading";
    state.startup.progress = Number(event.progress) || 0;
    state.startup.label = event.label || "Loading JS8Call modem";
    state.startup.detail = event.total > 0
      ? `${Math.round(event.loaded / 1024)} / ${Math.round(event.total / 1024)} KiB`
      : "Initializing modem components…";
  }
  if (event.type === "status") {
    state.decoderStatus = event.status;
    if (event.status === "ready") {
      state.startup.ready = true; state.startup.failed = false;
      state.startup.progress = 100; state.startup.label = "JS8Call modem ready";
      ensureAudio();
    }
  }
  if (event.type === "error") {
    state.decoderStatus = event.message;
    state.startup.failed = true; state.startup.ready = false;
    state.startup.label = "Modem loading failed";
    state.startup.detail = event.message;
    stopAudio();
  }
  if (event.type === "activity" && !state.testActivityLocked) {
    applyDecoderActivity(event.activity); activityChanged=true;
  }
  if (event.type === "frame" && audioSource) {
    const decoded = state.activity.frames.find(item => item.raw === event.frame.raw && item.slotUtcMs === event.frame.slotUtcMs);
    const call = decoded && decoded.callsigns ? decoded.callsigns[0] : "";
    try { audioSource.observeDecode(event.frame, call); } catch (_error) {}
  }
  renderStartup();
  if (activityChanged) renderActivity();
  if (["loading","status","error"].includes(event.type)) renderControls();
  if (["status","error"].includes(event.type)) renderDiagnostics();
}

function handleEncoderEvent(event) {
  if (event.type !== "tx") return;
  state.txState = event.state; state.txStatus = event.state.status;
  updateOutgoingTxProgress(event.state);
  const running = !["idle","completed","aborted","fault"].includes(state.txStatus);
  state.tuneActive=running && Boolean(event.state.frames?.some(frame=>frame.role==="tune"));
  dom.abort.hidden = !running;
  if (!running && txTick) { clearInterval(txTick); txTick = null; }
  renderControls();
  if(!running&&binState.txCurrent&&["completed","aborted","fault"].includes(state.txStatus))
    queueMicrotask(()=>finishFileProtocolTx(state.txStatus));
}

// ---- AUD1 audio and waterfall ----------------------------------------------

function audioUrl() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  return `${scheme}://${location.hostname}:${AUDIO_WS_PORT}/audiows`;
}

function ensureAudio() {
  const lan = state.radio.connected && state.radio.transceiverType === "IC-705-LAN";
  if (!lan || state.decoderStatus !== "ready") { stopAudio(); return; }
  if (audioSource) return;
  audioSource = new Js8WsAudioSource.WsAudioSource(AUDIO_RATE, {url:audioUrl()})
    .onSamples(onSamples).onStatus(onAudioStatus).onEpoch(() => renderDiagnostics());
  audioSource.configure({clockCorrectionMs:currentJs8().clockCorrectionMs, autoTiming:currentJs8().autoTiming});
  audioSource.start();
}

function stopAudio() {
  if (!audioSource) return;
  if (activeEncoder && activeEncoder.disconnect) activeEncoder.disconnect();
  audioSource.stop(); audioSource = null; state.audioStatus = "stopped"; state.lastAudioMs=0;
}

function onAudioStatus(status) {
  state.audioStatus = status.message ? `${status.type}: ${status.message}` : status.type;
  if (status.type === "closed") {
    state.lastAudioMs=0;
    if (activeEncoder && activeEncoder.disconnect) activeEncoder.disconnect();
  }
  renderHeader(); renderControls(); renderDiagnostics();
}

const wfCtx = dom.canvas.getContext("2d"), overlayCtx = dom.overlay.getContext("2d");
const fftRe = new Float32Array(FFT_SIZE), fftIm = new Float32Array(FFT_SIZE), ring = new Float32Array(FFT_SIZE);
const hann = Float32Array.from({length:FFT_SIZE}, (_, i) => .5 - .5 * Math.cos(2 * Math.PI * i / (FFT_SIZE - 1)));
let ringPos = 0, hop = 0, spectrumFill = 0, spectrumRows = 0;
let lastSlotIndex = null, lastSlotPeriod = 0;
let agcLow = -85, agcHigh = -35, agcReady = false;

function fft(re, im) {
  for (let i = 1, j = 0; i < FFT_SIZE; i++) {
    let bit = FFT_SIZE >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit;
    if (i < j) { let t=re[i]; re[i]=re[j]; re[j]=t; t=im[i]; im[i]=im[j]; im[j]=t; }
  }
  for (let len = 2; len <= FFT_SIZE; len <<= 1) {
    const angle = -2 * Math.PI / len, wr0 = Math.cos(angle), wi0 = Math.sin(angle);
    for (let start = 0; start < FFT_SIZE; start += len) {
      let wr=1, wi=0;
      for (let j=0; j<(len>>1); j++) {
        const a=start+j, b=a+(len>>1), tr=wr*re[b]-wi*im[b], ti=wr*im[b]+wi*re[b];
        re[b]=re[a]-tr; im[b]=im[a]-ti; re[a]+=tr; im[a]+=ti;
        const nextWr=wr*wr0-wi*wi0; wi=wr*wi0+wi*wr0; wr=nextWr;
      }
    }
  }
}

function radioTransmitting() { return Boolean(state.radio.tx || sinkProxy.ptt); }

function resetSpectrumAnalyzer() {
  ring.fill(0); ringPos=0; hop=0; spectrumFill=0; agcReady=false;
  agcLow=-85; agcHigh=-35; lastSlotIndex=null;
}

function ingestSpectrum(samples) {
  if(radioTransmitting())return;
  for (const value of samples) {
    ring[ringPos] = value; ringPos = (ringPos + 1) % FFT_SIZE;
    spectrumFill=Math.min(FFT_SIZE,spectrumFill+1);
    if (++hop >= HOP_SIZE) { hop=0; if(spectrumFill>=FFT_SIZE)drawSpectrum(); }
  }
}

function onSamples(samples, rate, metadata) {
  state.lastAudioMs = performance.now();
  let sum=0;
  for (const value of samples) sum += value * value;
  ingestSpectrum(samples);
  const rms = Math.sqrt(sum / Math.max(1, samples.length));
  state.audioDb=20*Math.log10(rms + 1e-9);
  dom.audioLevel.textContent = `${Math.round(state.audioDb)} dBFS`;
  if (activeDecoder) activeDecoder.pushSamples(samples, metadata);
}

function color(value) {
  const v=Math.pow(Math.max(0,Math.min(1,value)),1.8);
  const mix=(a,b,t)=>a.map((channel,index)=>Math.round(channel+(b[index]-channel)*t));
  if (v<.55) return mix([0,2,20],[0,42,145],v/.55);
  if (v<.92) return mix([0,42,145],[0,180,205],(v-.55)/.37);
  return mix([0,180,205],[255,215,55],(v-.92)/.08);
}

function drawSpectrum() {
  for (let i=0;i<FFT_SIZE;i++) { fftRe[i]=ring[(ringPos+i)%FFT_SIZE]*hann[i]; fftIm[i]=0; }
  fft(fftRe,fftIm);
  const first=Math.floor(RX_LOW*FFT_SIZE/AUDIO_RATE), last=Math.ceil(RX_HIGH*FFT_SIZE/AUDIO_RATE);
  const values=new Float32Array(last-first+1);
  for (let bin=first;bin<=last;bin++) values[bin-first]=20*Math.log10(Math.hypot(fftRe[bin],fftIm[bin])/FFT_SIZE+1e-9);
  const sorted=Array.from(values).sort((a,b)=>a-b);
  const targetLow=sorted[Math.floor((sorted.length-1)*.18)]-3;
  const observedHigh=sorted[Math.floor((sorted.length-1)*.985)];
  const targetHigh=Math.max(observedHigh,targetLow+22);
  if(observedHigh>-140){
    if(!agcReady){agcLow=targetLow;agcHigh=targetHigh;agcReady=true;}
    else {agcLow+=(targetLow-agcLow)*.10;agcHigh+=(targetHigh-agcHigh)*.10;}
  }
  wfCtx.drawImage(dom.canvas,0,0,dom.canvas.width,dom.canvas.height-1,0,1,dom.canvas.width,dom.canvas.height-1);
  const row=wfCtx.createImageData(dom.canvas.width,1);
  for (let x=0;x<dom.canvas.width;x++) { const bin=Math.min(values.length-1,Math.floor(x*values.length/dom.canvas.width)); const c=color((values[bin]-agcLow)/(agcHigh-agcLow)); const at=x*4; row.data[at]=c[0];row.data[at+1]=c[1];row.data[at+2]=c[2];row.data[at+3]=255; }
  wfCtx.putImageData(row,0,0);
  // JS8 time-slot ruler: burn a faint horizontal line into the newest row whenever a
  // UTC slot boundary passes, so it scrolls down with the history. Same clock/period as
  // the slot meter (renderRhythm) — the ruler and the slot-fill bar stay in lockstep.
  const slotPeriodMs=(MODE_PERIOD_SECONDS[selectedMode()] || 15)*1000;
  const slotCorrection=audioSource ? Number(audioSource.state().timebase?.correction?.totalMs || 0) : 0;
  const slotIndex=Math.floor((Date.now()+slotCorrection)/slotPeriodMs);
  if(lastSlotPeriod===slotPeriodMs && lastSlotIndex!==null && slotIndex!==lastSlotIndex){
    wfCtx.fillStyle="rgba(235,240,250,0.6)"; wfCtx.fillRect(0,0,dom.canvas.width,2);
  }
  lastSlotIndex=slotIndex; lastSlotPeriod=slotPeriodMs;
  spectrumRows++;
}

function resizeWaterfall() {
  const width=Math.max(320,Math.round(dom.waterfall.clientWidth));
  if (dom.canvas.width !== width) { dom.canvas.width=width; dom.canvas.height=64; }
  dom.overlay.width=width; dom.overlay.height=64; drawTxMarker();
}

function drawTxMarker() {
  overlayCtx.clearRect(0,0,dom.overlay.width,dom.overlay.height);
  const hzToX=hz=>(hz-RX_LOW)/(RX_HIGH-RX_LOW)*dom.overlay.width;
  const heartbeatRight=hzToX(HB_HIGH);
  overlayCtx.strokeStyle="rgba(185,195,191,.52)"; overlayCtx.lineWidth=1; overlayCtx.setLineDash([3,3]);
  overlayCtx.beginPath(); overlayCtx.moveTo(Math.round(heartbeatRight)+.5,0); overlayCtx.lineTo(Math.round(heartbeatRight)+.5,dom.overlay.height); overlayCtx.stroke();
  overlayCtx.setLineDash([]); overlayCtx.fillStyle="rgba(210,220,216,.68)"; overlayCtx.font="bold 9px monospace"; overlayCtx.fillText("HB 500–1000",5,12);
  const mode=selectedMode(), widths={0:50,1:80,2:160,4:25,8:250};
  const start=hzToX(currentJs8().txOffsetHz);
  const width=(widths[mode] || 50)/(RX_HIGH-RX_LOW)*dom.overlay.width;
  overlayCtx.fillStyle="rgba(255,0,36,.28)"; overlayCtx.fillRect(start,0,Math.max(3,width),dom.overlay.height);
  overlayCtx.strokeStyle="#ff1838"; overlayCtx.lineWidth=2; overlayCtx.beginPath(); overlayCtx.moveTo(start+1,0); overlayCtx.lineTo(start+1,dom.overlay.height); overlayCtx.stroke();
  const label=`TX ${currentJs8().txOffsetHz} Hz`, labelX=Math.min(start+5,dom.overlay.width-96);
  overlayCtx.fillStyle="#fff"; overlayCtx.font="bold 11px monospace";
  overlayCtx.shadowColor="#000"; overlayCtx.shadowBlur=3; overlayCtx.fillText(label,labelX,14);
  overlayCtx.shadowBlur=0;
  overlayCtx.lineWidth=1;
}

function renderRhythm() {
  const mode=selectedMode(), period=MODE_PERIOD_SECONDS[mode] || 15;
  const correction=audioSource ? Number(audioSource.state().timebase?.correction?.totalMs || 0) : 0;
  const within=((Date.now()+correction)%(period*1000)+period*1000)%(period*1000);
  dom.slotFill.style.width=`${(within/(period*1000)*100).toFixed(2)}%`;
  dom.slotLabel.textContent=`${MODE_TO_SPEED[mode] || "?"} ${period} s`;
}

// ---- UI projection ----------------------------------------------------------

function renderFrequencyMenu() {
  const selected=state.pendingFrequency || state.radio.frequency;
  dom.frequencyMenu.innerHTML = `<header><strong>JS8 dial frequencies</strong><small>Choose a band to tune the TRX</small></header><div class="frequency-presets">${Js8TrxPresets.PRESETS.map(item =>
    `<button class="frequency-preset${item.frequencyHz===selected?" current":""}" data-frequency="${item.frequencyHz}" type="button"><strong>${item.band}</strong><span>${formatFrequency(item.frequencyHz)}</span></button>`).join("")}</div><footer>Dial frequencies from the bundled JS8Call source</footer>`;
  frequencyMenuKey=String(selected);
}

function hasSeenTrxHelp() {
  try { return localStorage.getItem(TRX_HELP_SEEN_KEY) === "1"; }
  catch (_error) { return false; }
}

function openTrxHelp(reason = "manual") {
  dom.trxHelpModeWarning.hidden=reason!=="mode";
  try { localStorage.setItem(TRX_HELP_SEEN_KEY,"1"); } catch (_error) {}
  if(dom.trxHelpDialog.open)return;
  if(typeof dom.trxHelpDialog.showModal==="function")dom.trxHelpDialog.showModal();
  else dom.trxHelpDialog.setAttribute("open","");
}

function renderHeader() {
  const connected=state.radio.connected && state.radio.transceiverType === "IC-705-LAN";
  const transmitting=radioTransmitting();
  const receiving=connected && state.lastAudioMs>0 && performance.now()-state.lastAudioMs<1500;
  if(state.spectrumWasTransmitting && !transmitting)resetSpectrumAnalyzer();
  state.spectrumWasTransmitting=transmitting;
  const modeCompatible=["USB","USB-D"].includes(state.radio.mode);
  dom.trxFrequencyValue.textContent=formatFrequency(state.pendingFrequency || state.radio.frequency);
  dom.trxFrequency.classList.toggle("pending",Boolean(state.pendingFrequency));
  dom.trxMode.textContent=state.radio.mode || "---";
  dom.trxMode.classList.toggle("incompatible",connected && !modeCompatible);
  dom.trxMode.title=connected && !modeCompatible ? "JS8Call requires USB or USB-D" : "TRX mode";
  const incompatible=connected && Boolean(state.radio.mode) && !modeCompatible;
  if(incompatible && !state.help.incompatibleActive)openTrxHelp("mode");
  state.help.incompatibleActive=incompatible;
  dom.radioBar.classList.toggle("tx",transmitting);
  document.body.classList.toggle("radio-transmitting",transmitting);
  const starting=!state.startup.ready;
  dom.linkState.textContent=starting ? (state.startup.failed ? "● LOAD ERROR" : "● LOADING")
    : connected ? (transmitting ? "● TX" : receiving ? "● RX LIVE" : "● RX WAIT") : "● OFFLINE";
  dom.linkState.classList.toggle("error",state.startup.failed || (!starting && !connected));
  dom.linkState.classList.toggle("warning",!starting && connected && !transmitting && !receiving);
  const reconnectVisible=state.lanConfig.ready && !connected && state.radio.lanStatus==="disconnected";
  dom.trxReconnect.hidden=!reconnectVisible;
  dom.trxReconnect.disabled=state.reconnectPending;
  dom.trxReconnect.textContent=state.reconnectPending ? "Connecting…" : "Reconnect";
  dom.operatorState.textContent=`${currentJs8().myCall} · ${currentJs8().grid}`;
  const tb=audioSource ? audioSource.state().timebase : null;
  dom.timingState.textContent=tb ? `${tb.clock.status} · ${signed(tb.correction.totalMs)} ms` : "clock unchecked";
  if (frequencyMenuKey !== String(state.pendingFrequency || state.radio.frequency)) renderFrequencyMenu();
}

function renderStartup() {
  const pending=!state.startup.ready;
  document.body.classList.toggle("startup-pending",pending);
  dom.startup.hidden=!pending;
  const progress=Math.max(0,Math.min(100,state.startup.progress));
  dom.startupProgress.value=progress;
  dom.startupProgress.textContent=`${Math.round(progress)}%`;
  dom.startupPercent.textContent=`${Math.round(progress)}%`;
  dom.startupLabel.textContent=state.startup.label;
  dom.startupDetail.textContent=state.startup.detail;
  dom.startupRetry.hidden=!state.startup.failed;
  if(!pending)requestAnimationFrame(resizeWaterfall);
}

function txBlockReasons(needsRecipient,allowFileTransfer=false) {
  const js8=currentJs8(), connected=state.radio.connected && state.radio.transceiverType === "IC-705-LAN";
  const busy=!['idle','completed','aborted','fault'].includes(state.txStatus);
  const mediaLocked=Boolean(audioSource && audioSource.state().timebase.media.status==="locked");
  const reasons=[];
  if(busy)reasons.push("TX is busy"); if(!connected)reasons.push("IC-705 LAN is offline");
  if(state.radio.tx&&!busy)reasons.push("TRX PTT is active");
  if(!["USB","USB-D"].includes(state.radio.mode))reasons.push("TRX mode must be USB or USB-D");
  if(!state.txWasmReady)reasons.push("TX core is loading");
  if(state.decoderStatus!=="ready")reasons.push("decoder is loading");
  if(!mediaLocked)reasons.push("audio timebase is not locked");
  if(needsRecipient && !state.selectedCall)reasons.push("select a recipient");
  if(!js8.myCall)reasons.push("set My callsign");
  if(!js8.txSafetyAccepted)reasons.push("confirm Enable radio TX");
  if(!allowFileTransfer&&binState.active&&!terminalTransferState(binState.active.state))reasons.push("a file-transfer session is active");
  return reasons;
}

function selectedEmailGateway() {
  return emailState.gateways.find(item=>item.id===emailState.selectedId) || null;
}

function emailFrameEstimate(draft) {
  const transport=Js8Email.transportParts(draft.payload,draft.gateway.target);
  return Js8Protocol.buildReplyFrames({myCall:currentJs8().myCall,
    toCall:transport.toCall,text:transport.text}).length;
}

function emailTxMode() {
  return currentJs8().speed==="AUTO"?0:SPEED_TO_MODE[currentJs8().speed];
}

function emailDraftResult() {
  const gateway=selectedEmailGateway();
  if(!gateway)return {gateway:null,draft:null,error:"Select or add a gateway."};
  try {
    const draft=Js8Email.buildDraft(gateway,dom.emailAddress.value,dom.emailMessage.value);
    emailFrameEstimate(draft);
    return {gateway,draft,error:""};
  } catch(error) { return {gateway,draft:null,error:error.message}; }
}

function renderEmailControls() {
  const selected=selectedEmailGateway();
  const optionKey=`${emailState.selectedId}|${emailState.gateways.map(item=>`${item.id}:${item.name}`).join("|")}`;
  if(dom.emailGateway.dataset.options!==optionKey){
    dom.emailGateway.innerHTML='<option value="">Add or select a gateway</option>'+emailState.gateways
      .filter(item=>item.enabled).map(item=>`<option value="${esc(item.id)}">${esc(item.name)}</option>`).join("");
    dom.emailGateway.value=emailState.selectedId;
    dom.emailGateway.dataset.options=optionKey;
  }
  dom.emailGatewayEdit.disabled=!selected; dom.emailGatewayDelete.disabled=!selected;
  const values=selected?[selected.target,`${(selected.dialFrequencyHz/1e6).toFixed(6)} MHz`,
    `${selected.offsetHz} Hz`,selected.format]:["—","—","—","—"];
  dom.emailGatewayDetails.innerHTML=`<span>Target</span><code>${esc(values[0])}</code><span>Dial</span><code>${esc(values[1])}</code><span>Offset</span><code>${esc(values[2])}</code><span>Format</span><code>${esc(values[3])}</code>`;
  const email=dom.emailAddress.value.trim();
  const budget=selected?Js8Email.getBodyBudget(selected,email):0;
  const normalizedLength=dom.emailMessage.value.replace(/[\r\n\t]+/g," ").replace(/\s+/g," ").trim().length;
  dom.emailBudget.textContent=selected
    ? `${Math.max(0,budget-normalizedLength)} of ${budget} characters remaining${selected.format==="aprs-email2"?" (email address included in APRS limit)":""}.`
    : "Select a gateway to see the message limit.";
  dom.emailBudget.classList.toggle("invalid",Boolean(selected&&normalizedLength>budget));
  const result=emailDraftResult();
  dom.emailPreview.textContent=result.draft?result.draft.payload:"Complete the form to preview the exact radio payload.";
  const touched=email.length>0||dom.emailMessage.value.trim().length>0;
  dom.emailError.textContent=touched?result.error:"";
  const blocks=txBlockReasons(false);
  if(!result.draft)blocks.push(result.error);
  dom.emailSend.disabled=blocks.filter(Boolean).length>0;
  dom.emailSend.title=blocks.filter(Boolean).join("; ");
  const outgoing=emailState.activeOutgoing;
  if(outgoing){
    if(outgoing.status==="completed")emailState.status="RF transmission completed. Gateway reception and email delivery are unconfirmed.";
    else if(outgoing.status==="fault"||outgoing.status==="aborted")emailState.status=`RF transmission ${outgoing.status}. Email was not confirmed.`;
    else emailState.status=`RF transmission ${outgoing.status}. Gateway reception is not yet confirmed.`;
  }
  dom.emailStatus.textContent=emailState.status;
}

function currentBinProfile() {
  const js8=currentJs8();
  let submode=SPEED_TO_MODE[js8.speed];
  if(js8.speed==="AUTO"){
    const station=state.activity.calls.find(item=>item.call===state.selectedCall);
    submode=station?Number(station.submode):0;
  }
  return Js8FileTransfer.profileForSubmode(submode);
}

function formatBytes(value) {
  const bytes=Number(value)||0;
  if(bytes<1024)return `${bytes} B`;
  const kib=bytes/1024;
  return `${Number.isInteger(kib)?kib:kib.toFixed(1)} KiB`;
}

function formatMinutes(value) {
  const minutes=Math.max(0,Math.ceil(Number(value)||0));
  return minutes<60?`${minutes} min`:`${Math.floor(minutes/60)} h ${minutes%60} min`;
}

function terminalTransferState(value) {
  return ["complete","cancelled","rejected","failed"].includes(value);
}

function binSessionCounts(record) {
  if(!record)return {valid:0,total:0,bytes:0};
  const total=Number(record.blockCount)||0;
  if(record.direction==="tx"){
    const acknowledged=new Set(record.acknowledged||[]);
    let bytes=0;for(const sequence of acknowledged)if(sequence>0&&record.blocks?.[sequence])bytes+=record.blocks[sequence].length;
    return {valid:[...acknowledged].filter(value=>value>0).length,total,bytes};
  }
  let valid=0,bytes=0;for(let sequence=1;sequence<=total;sequence+=1)if(record.blocks?.[sequence]){valid+=1;bytes+=record.blocks[sequence].length;}
  return {valid,total,bytes};
}

function renderBinControls() {
  if(!binState.peerDraft&&state.selectedCall)binState.peerDraft=state.selectedCall;
  if(document.activeElement!==dom.binRecipient)dom.binRecipient.value=binState.peerDraft;
  const profile=currentBinProfile(),prepared=binState.prepared;
  const estimate=prepared?Js8FileTransfer.estimateDuration(prepared.manifest.originalSize,profile):null;
  const details=[
    ["Size",prepared?formatBytes(prepared.manifest.originalSize):"—"],
    ["Profile",`${profile.label} · ${profile.periodSeconds} s`],
    ["Hard limit",profile.hardLimit?formatBytes(profile.hardLimit):"Disabled"],
    ["Recommended",profile.warningSize?`≤ ${formatBytes(profile.warningSize)}`:"Disabled"],
    ["Blocks",prepared?`${prepared.manifest.blockCount} + manifest`:"—"],
    ["Estimate",estimate?`${formatMinutes(estimate.optimisticMinutes)}–${formatMinutes(estimate.plannedMinutes)}`:"—"]
  ];
  dom.binFileDetails.innerHTML=details.map(([label,value])=>`<span>${esc(label)}</span><code>${esc(value)}</code>`).join("");
  let error="";
  try{Js8FileTransfer.normalizeCallsign(binState.peerDraft);if(prepared)Js8FileTransfer.enforceFileLimit(prepared.manifest.originalSize,profile);}
  catch(reason){error=reason.message;}
  if(!prepared&&!binState.preparing)error=error||"Select a file.";
  if(binState.preparing)error="Preparing SHA-256 and blocks…";
  if(binState.storageError)error=binState.storageError;
  if(sameCall(binState.peerDraft,currentJs8().myCall))error="Nelze poslat soubor vlastní značce";
  dom.binError.textContent=error;
  const blocks=txBlockReasons(false);
  if(error)blocks.push(error);
  if(!dom.binPeerExpected.checked)blocks.push("confirm that the peer expects the transfer");
  if(binState.active&&!terminalTransferState(binState.active.state))blocks.push("another file-transfer session is active");
  dom.binOffer.disabled=blocks.length>0;
  dom.binOffer.title=blocks.join("; ");
  dom.binOffer.textContent=binState.preparing?"PREPARING…":"PREPARE OFFER";
  dom.binDraftStatus.textContent=prepared
    ? `${prepared.manifest.fileName} · SHA-256 ${prepared.manifest.sha256Hex.slice(0,16)}… · maximum ${formatBytes(profile.hardLimit)}`
    : profile.hardLimit?`${profile.label} accepts up to ${formatBytes(profile.hardLimit)}. The file is checked before loading.`:`${profile.label} file transfer is disabled.`;
  const record=binState.active;
  dom.binTransferPanel.hidden=!record;
  if(!record)return;
  const counts=binSessionCounts(record),elapsedMinutes=Math.max(1/60,(Date.now()-(record.startedAt||record.createdAt||Date.now()))/60000);
  dom.binTransferTitle.textContent=record.fileName;
  dom.binTransferPeer.textContent=`${record.direction==="tx"?"To":"From"} ${record.peerCallsign}`;
  dom.binTransferState.textContent=String(record.state||"idle").toUpperCase().replaceAll("-"," ");
  dom.binProgress.max=Math.max(1,counts.total);dom.binProgress.value=counts.valid;
  dom.binProgress.textContent=`${Math.round(counts.valid/Math.max(1,counts.total)*100)}%`;
  dom.binProgressText.textContent=`${counts.valid} / ${counts.total} data blocks${record.retransmittedBlocks?` · ${record.retransmittedBlocks} repaired`:""}`;
  const measuredRate=Math.round(counts.bytes/elapsedMinutes),remainingBytes=Math.max(0,record.originalSize-counts.bytes);
  dom.binTransferRate.textContent=`${measuredRate} B/min${measuredRate>0&&remainingBytes?` · ETA ${formatMinutes(remainingBytes/measuredRate)}`:""}`;
  dom.binLastActivity.textContent=record.lastActivityAt?`${age(record.lastActivityAt)} ago${record.lastSnr!=null?` · ${signed(record.lastSnr)} dB`:""}`:"No activity";
  dom.binTransferId.textContent=record.id;
  dom.binTransferHash.textContent=record.sha256Hex||record.hash12||"Pending manifest";
  dom.binProtocolMessage.textContent=binState.lastProtocol||record.lastProtocol||"—";
  dom.binTransferLog.innerHTML=(record.log||[]).slice(-30).map(item=>`<div><span>${esc(new Date(item.at).toISOString().slice(11,19))}</span><code>${esc(item.text)}</code></div>`).join("");
  dom.binPause.hidden=record.state==="paused"||terminalTransferState(record.state);
  dom.binPause.disabled=false;
  dom.binResume.hidden=record.state!=="paused";
  dom.binStop.disabled=terminalTransferState(record.state);
  dom.binDownload.hidden=!(record.direction==="rx"&&record.state==="complete"&&record.fileBytes);
}

function renderControls() {
  const js8=currentJs8(), mode=selectedMode();
  dom.recipient.value=state.selectedCall; dom.txSpeed.value=js8.speed;
  dom.txSpeedResolved.textContent=js8.speed==="AUTO" ? `→ ${speedDetail(mode)}` : `${MODE_PERIOD_SECONDS[mode]} s`;
  dom.txOffset.value=js8.txOffsetHz; dom.spectrumSummary.textContent=`RX ${RX_LOW}–${RX_HIGH} Hz · TX ${js8.txOffsetHz} Hz · ${speedDetail(mode)}`;
  dom.myCall.value=state.settingsDraft.myCall===null?js8.myCall:state.settingsDraft.myCall;
  dom.myGrid.value=state.settingsDraft.grid===null?js8.grid:state.settingsDraft.grid;
  dom.followSpeed.checked=js8.followSpeed;
  dom.clockCorrection.value=js8.clockCorrectionMs; dom.autoTiming.checked=js8.autoTiming;
  dom.txGain.value=state.settingsDraft.txGain===null?js8.txGain:state.settingsDraft.txGain; dom.txSafety.checked=js8.txSafetyAccepted;
  dom.settingsSummary.textContent=`${js8.myCall} · ${js8.grid} · ${js8.speed}`;
  const busy=!["idle","completed","aborted","fault"].includes(state.txStatus);
  const txBlocks=txBlockReasons(!cqType(dom.message.value)), heartbeatBlocks=txBlockReasons(false), tuneBlocks=txBlockReasons(false);
  if(state.txSessionMode!=="CHAT")txBlocks.push(`${state.txSessionMode} uses its own form`);
  dom.send.disabled=txBlocks.length>0; dom.send.title=txBlocks.join("; ");
  dom.heartbeat.disabled=heartbeatBlocks.length>0; dom.heartbeat.title=heartbeatBlocks.join("; ");
  dom.heartbeatOffset.textContent=`${js8.txOffsetHz} Hz`;
  dom.tune.disabled=!state.tuneActive && tuneBlocks.length>0;
  dom.tune.title=state.tuneActive ? "Stop tuning carrier" : tuneBlocks.join("; ");
  dom.tune.classList.toggle("active",state.tuneActive);
  dom.tuneLabel.textContent=state.tuneActive?"STOP":"TUNE";
  dom.tuneOffset.textContent=`${js8.txOffsetHz} Hz`;
  const snrPreset=dom.messagePresetsMenu.querySelector('[data-message-preset="snr"]');
  const snrStation=state.activity.calls.find(item=>item.call===state.selectedCall);
  snrPreset.disabled=!snrStation;
  snrPreset.title=snrStation ? `Insert SNR ${formatJs8Snr(snrStation.snr)}` : "Select a heard station first";
  dom.txSessionMode.value=state.txSessionMode;
  dom.chatSession.hidden=state.txSessionMode!=="CHAT";
  dom.emailSession.hidden=state.txSessionMode!=="EMAIL";
  dom.binSession.hidden=state.txSessionMode!=="BIN";
  dom.txSessionModeHint.textContent=({CHAT:"Keyboard-to-keyboard messages",EMAIL:"Short radio email via a configured JS8 gateway",BIN:"Reliable store-and-resume transfer for small files"})[state.txSessionMode];
  dom.send.textContent=busy ? "QUEUED" : "SEND";
  dom.txSummary.textContent=state.txState ? `${state.txState.status}${state.txState.frameCount ? ` · frame ${Math.min(state.txState.frameIndex+1,state.txState.frameCount)}/${state.txState.frameCount}` : ""}${state.txState.error ? ` · ${state.txState.error}` : ""}` : "Idle";
  dom.modemState.textContent=state.decoderStatus === "ready" ? "JS8Call ready · auto speed RX" : state.decoderStatus;
  dom.modemState.className=`modem-state ${state.decoderStatus === "ready" ? "available" : state.decoderStatus.includes("error") ? "error" : ""}`;
  renderEmailControls(); renderBinControls(); renderTxPayload(); drawTxMarker(); renderHeader();
}

function chooseCall(call) {
  if (!call) return clearRecipient();
  if (call.startsWith("@")) return;
  if (sameCall(call,currentJs8().myCall)) return rejectOwnCall();
  state.selectedCall=call;binState.peerDraft=call;
  state.txSessionMode="CHAT";
  const station=state.activity.calls.find(item=>item.call===call);
  if (station && currentJs8().followSpeed && currentJs8().speed!=="AUTO") currentJs8().speed=MODE_TO_SPEED[station.submode] || currentJs8().speed;
  persistSettings(false); renderActivity(); renderControls();
  dom.reply.open=true;
  dom.message.focus({preventScroll:true});
}

function clearRecipient() {
  state.selectedCall="";
  renderActivity(); renderControls();
  dom.recipient.focus({preventScroll:true});
}

// You can't work yourself: refuse your own callsign as recipient, revert the field to the
// current selection and explain why. Covers both a table-row click and a typed callsign.
function rejectOwnCall() {
  dom.recipient.value=state.selectedCall;
  dom.sessionMeta.textContent="Nelze volat vlastní značku";
}

function stationDirection(station) {
  if(!self.DXCC || !station)return null;
  const own=DXCC.locatorToLatLon(currentJs8().grid);
  let remote=station.grid ? DXCC.locatorToLatLon(station.grid) : null;
  let source=station.grid ? station.grid : "";
  if(!remote){
    const entity=DXCC.lookupDxcc(station.call);
    if(entity){remote={lat:entity.latitude,lon:entity.longitude};source=`DXCC estimate · ${entity.country}`;}
  }
  if(!own || !remote)return null;
  return {...DXCC.calculateQrbAzimuth(own.lat,own.lon,remote.lat,remote.lon),source};
}

function sortedStations(calls) {
  const {key,direction}=state.stationSort, factor=direction==="asc" ? 1 : -1;
  return [...calls].sort((a,b)=>{
    if(key==="distance"){
      const av=stationDirection(a)?.qrbKm, bv=stationDirection(b)?.qrbKm;
      if(av==null && bv==null)return String(a.call).localeCompare(String(b.call));
      if(av==null)return 1; if(bv==null)return -1;
      return (av-bv)*factor || String(a.call).localeCompare(String(b.call));
    }
    const av=a[key], bv=b[key];
    const result=key==="call" ? String(av).localeCompare(String(bv)) : Number(av||0)-Number(bv||0);
    return result*factor || String(a.call).localeCompare(String(b.call));
  });
}

function renderStationSort() {
  dom.stationHead.querySelectorAll("[data-station-sort]").forEach(button=>{
    const active=button.dataset.stationSort===state.stationSort.key;
    button.classList.toggle("active",active);
    button.querySelector(".sort-arrow").textContent=active ? (state.stationSort.direction==="asc" ? "↑" : "↓") : "";
    button.closest("th").setAttribute("aria-sort",active ? (state.stationSort.direction==="asc" ? "ascending" : "descending") : "none");
  });
}

function openSectionsForNewOwnCall(messages,calls) {
  const own=currentJs8().myCall;
  const previous=state.ownCallAttention;
  const messageKeys=new Set(messages.filter(item=>messageMentionsCall(item,own)).map(activityMessageKey));
  const stationKeys=new Set(calls.filter(item=>sameCall(item.call,own))
    .map(item=>`${item.call}|${activityCallSignature(item)}`));
  const sameOperator=previous.call===own;
  if(messageKeys.size && (!sameOperator || [...messageKeys].some(key=>!previous.messages.has(key))))
    dom.trafficSection.open=true;
  if(stationKeys.size && (!sameOperator || [...stationKeys].some(key=>!previous.stations.has(key))))
    dom.stationsSection.open=true;
  state.ownCallAttention={call:own,messages:messageKeys,stations:stationKeys};
}

const TRAFFIC_WINDOWS={"30m":30*60*1000, "2h":2*60*60*1000};
function messageTimeMs(message){return Number(message.lastSlotUtcMs || message.firstSlotUtcMs || 0);}
// Recent-traffic filter: one active mode at a time. Time windows are rolling (recomputed
// each render against Date.now()); MYCALL keeps only frames mentioning the operator's call.
function filterTraffic(messages,own){
  const filter=state.trafficFilter;
  if(filter==="mycall")return own ? messages.filter(message=>messageMentionsCall(message,own)) : messages;
  const windowMs=TRAFFIC_WINDOWS[filter];
  if(!windowMs)return messages;
  const cutoff=Date.now()-windowMs;
  return messages.filter(message=>messageTimeMs(message)>=cutoff);
}
function renderTrafficFilterButtons(own){
  if(state.trafficFilter==="mycall" && !own)state.trafficFilter="all";
  for(const button of dom.trafficFilter.querySelectorAll("[data-traffic-filter]")){
    const value=button.dataset.trafficFilter, active=value===state.trafficFilter;
    button.classList.toggle("active",active);
    button.setAttribute("aria-pressed",String(active));
    if(value==="mycall")button.disabled=!own;
  }
}

function renderActivity() {
  const messages=state.activity.messages || [], calls=state.activity.calls || [];
  const own=currentJs8().myCall;
  renderTrafficFilterButtons(own);
  const filtered=filterTraffic(messages,own);
  dom.trafficSummary.textContent=filtered.length===messages.length
    ? `${messages.length} message${messages.length===1?"":"s"}`
    : `${filtered.length} / ${messages.length} messages`;
  dom.stationSummary.textContent=`${calls.length} active`;
  const recent=[...filtered].sort((a,b)=>Number(b.lastSlotUtcMs||b.firstSlotUtcMs||0)-Number(a.lastSlotUtcMs||a.firstSlotUtcMs||0)).slice(0,100);
  dom.traffic.innerHTML=recent.length ? recent.map(message => {
    const call=callOf(message), when=new Date(message.lastSlotUtcMs || message.firstSlotUtcMs || 0).toISOString().slice(11,19);
    const operational=Array.isArray(message.kinds) && !message.kinds.includes("data");
    const ownCall=sameCall(call,currentJs8().myCall);
    return `<article class="message${operational?" operational":""}"><span class="message-meta"><span>${when}</span><span>${MODE_TO_SPEED[message.submode]||"?"}</span><span>${Math.round(message.offsetHz)} Hz</span></span><strong data-call="${esc(call)}"${ownCall?' class="own-callsign" data-own-call="true"':""}>${esc(call || "JS8")}</strong><span class="message-text">${ownCallText(message.text,currentJs8().myCall)}</span></article>`;
  }).join("") : '<div class="empty-row">Waiting for JS8 activity…</div>';
  dom.stationRows.innerHTML=sortedStations(calls).map(item=>{
    const direction=stationDirection(item);
    const directionHtml=direction ? `<span title="${esc(direction.source)} · ${direction.qrbKm} km · ${direction.azimuthDeg}°"><span class="station-bearing" style="transform:rotate(${direction.azimuthDeg}deg)">↑</span><span class="station-distance">${(direction.qrbKm/1000).toFixed(1)}</span></span>` : "—";
    const ownCall=sameCall(item.call,currentJs8().myCall);
    return `<tr data-call="${esc(item.call)}" class="${item.call===state.selectedCall?"selected":""}"><td class="call${ownCall?" own-callsign":""}"${ownCall?' data-own-call="true"':""}>${esc(item.call)}</td><td>${signed(item.snr)}</td><td>${Math.round(item.offsetHz)}</td><td>${speedDetail(item.submode)}</td><td class="station-direction">${directionHtml}</td><td>${age(item.lastSlotUtcMs)}</td></tr>`;
  }).join("");
  openSectionsForNewOwnCall(recent,calls);
  renderStationSort();
  renderConversation();
}

function age(utcMs) { const seconds=Math.max(0,Math.round((Date.now()-Number(utcMs||0))/1000)); return seconds<60?`${seconds}s`:`${Math.floor(seconds/60)}m`; }
function messageBelongsToConversation(message) {
  const calls=message.callsigns||[];
  if(!sameCall(calls[0],state.selectedCall))return false;
  const directed=Array.isArray(message.kinds)&&message.kinds.includes("directed");
  return !directed || !calls[1] || sameCall(calls[1],currentJs8().myCall);
}
function conversationItems() {
  const received=(state.activity.messages||[]).filter(messageBelongsToConversation).map(message=>({direction:"incoming",time:new Date(message.lastSlotUtcMs||0).toISOString().slice(11,19),text:message.text,status:"received"}));
  return [...received,...(state.conversations[state.selectedCall]||[])].sort((a,b)=>a.time.localeCompare(b.time));
}
function renderOutgoingText(item) {
  const length=item.text.length;
  const sent=Math.max(0,Math.min(length,Number(item.sentChars)||0));
  const failed=["aborted","fault"].includes(item.status);
  const active=!failed&&sent<length&&Number(item.activeFraction)>0;
  const pendingStart=Math.min(length,sent+(active?1:0));
  let html="";
  if(sent>0)html+=`<span class="tx-copy tx-copy-sent">${esc(item.text.slice(0,sent))}</span>`;
  if(active)html+=`<span class="tx-copy tx-copy-active" style="--tx-character-progress:${Math.round(item.activeFraction*100)}%">${esc(item.text.slice(sent,sent+1))}</span>`;
  if(pendingStart<length)html+=`<span class="tx-copy ${failed?"tx-copy-failed":"tx-copy-pending"}">${esc(item.text.slice(pendingStart))}</span>`;
  if(!html)html=`<span class="tx-copy ${failed?"tx-copy-failed":"tx-copy-pending"}">${esc(item.text)}</span>`;
  if(item.status==="completed")html+='<span class="tx-eot" title="End of transmission">♢</span>';
  return html;
}
function renderTxPayload() {
  const item=state.lastOutgoing;
  dom.txPayload.hidden=!item;
  if(!item){dom.txPayload.textContent="";return;}
  dom.txPayload.innerHTML=`<strong>LAST TX</strong><span class="tx-payload-copy">${renderOutgoingText(item)}</span><small>${esc(item.status)}</small>`;
}
function renderConversation() {
  dom.sessionCall.textContent=state.selectedCall || "No station selected";
  const station=state.activity.calls.find(item=>item.call===state.selectedCall);
  dom.sessionMeta.textContent=station ? `${signed(station.snr)} dB · ${Math.round(station.offsetHz)} Hz · speed ${speedDetail(station.submode)}` : "Choose a callsign from traffic or stations";
  const items=state.selectedCall ? conversationItems() : [];
  dom.chat.innerHTML=items.length ? items.map(item=>`<div class="chat-row ${item.direction}"><article class="chat-bubble" data-message-status="${esc(item.status)}"><header><strong>${item.direction==="incoming"?esc(state.selectedCall):esc(currentJs8().myCall)}</strong><time>${esc(item.time)}</time></header><div class="chat-message">${item.direction==="outgoing"?renderOutgoingText(item):esc(item.text)}</div><footer>${esc(item.status)}</footer></article></div>`).join("") : '<div class="chat-empty">No messages in this session.</div>';
  dom.chat.scrollTop=dom.chat.scrollHeight;
}

function renderDiagnostics() {
  const tb=audioSource ? audioSource.state().timebase : null;
  if (!tb) { dom.diagnosticSummary.textContent="Audio link unavailable"; dom.diagnostics.innerHTML="<span>Transport</span><code>Waiting for IC-705 LAN audio</code>"; return; }
  dom.diagnosticSummary.textContent=`${tb.clock.status} · ${tb.media.status} · gaps ${tb.transport.sequenceGaps}`;
  dom.diagnostics.innerHTML=`<span>Audio WebSocket</span><code>${esc(state.audioStatus)}</code><span>Browser/system clock</span><code>${esc(tb.clock.status)} · epoch ${tb.clock.epoch} · jumps ${tb.clock.jumps} <button id="confirmClock" type="button">Confirm synchronized</button></code><span>Media epoch</span><code>${tb.media.epoch} · ${esc(tb.media.reason)} · ${esc(tb.media.status)}</code><span>Packets</span><code>${tb.transport.acceptedPackets} accepted · ${tb.transport.duplicatePackets} duplicate · ${tb.transport.sequenceGaps} gaps</code><span>Timing correction</span><code>manual ${signed(tb.correction.manualMs)} ms · auto ${signed(tb.correction.autoMs)} ms · ${esc(tb.correction.status)} <button id="resetTiming" type="button">Reset</button></code>`;
  $("confirmClock").addEventListener("click",()=>{audioSource.confirmClock();renderDiagnostics();renderHeader();});
  $("resetTiming").addEventListener("click",()=>{audioSource.resetTiming();renderDiagnostics();renderHeader();});
}

function openEmailGatewayDialog(gateway=null) {
  emailState.editingId=gateway?.id||"";
  dom.emailGatewayDialogTitle.textContent=gateway?"Edit email gateway":"Add email gateway";
  dom.emailGatewayName.value=gateway?.name||"";
  dom.emailGatewayTarget.value=gateway?.target||"";
  dom.emailGatewayDial.value=gateway?.dialFrequencyHz||state.radio.frequency||"";
  dom.emailGatewayOffset.value=gateway?.offsetHz||currentJs8().txOffsetHz;
  dom.emailGatewayFormat.value=gateway?.format||"direct";
  dom.emailGatewayTemplate.value=gateway?.template||"{TARGET} MSG EMAIL {EMAIL} {BODY}";
  dom.emailGatewayMaxBody.value=gateway?.maxBodyLength||60;
  dom.emailGatewayPolicy.value=gateway?.characterPolicy||"js8";
  dom.emailGatewayTemplateRow.hidden=dom.emailGatewayFormat.value!=="template";
  dom.emailGatewayError.textContent="";
  dom.emailGatewayDialog.showModal();
  dom.emailGatewayName.focus();
}

function gatewayFromDialog() {
  return Js8Email.normalizeGateway({id:emailState.editingId||undefined,
    name:dom.emailGatewayName.value,target:dom.emailGatewayTarget.value,
    dialFrequencyHz:Number(dom.emailGatewayDial.value),offsetHz:Number(dom.emailGatewayOffset.value),
    format:dom.emailGatewayFormat.value,template:dom.emailGatewayTemplate.value,
    maxBodyLength:Number(dom.emailGatewayMaxBody.value),characterPolicy:dom.emailGatewayPolicy.value});
}

function saveEmailGateway(event) {
  event.preventDefault();
  try {
    const gateway=gatewayFromDialog();
    const index=emailState.gateways.findIndex(item=>item.id===gateway.id);
    if(index>=0)emailState.gateways.splice(index,1,gateway);else emailState.gateways.push(gateway);
    emailState.gateways=Js8Email.save(localStorage,emailState.gateways);
    emailState.selectedId=gateway.id; emailState.status="Gateway profile saved locally.";
    dom.emailGatewayDialog.close(); renderControls();
  } catch(error) { dom.emailGatewayError.textContent=error.message; }
}

function deleteSelectedEmailGateway() {
  const gateway=selectedEmailGateway();
  if(!gateway||!confirm(`Delete gateway profile “${gateway.name}”?`))return;
  emailState.gateways=emailState.gateways.filter(item=>item.id!==gateway.id);
  emailState.gateways=Js8Email.save(localStorage,emailState.gateways);
  emailState.selectedId=emailState.gateways[0]?.id||"";
  emailState.status="Gateway profile deleted."; renderControls();
}

function openEmailConfirmation() {
  const result=emailDraftResult();
  if(!result.draft){dom.emailError.textContent=result.error;return;}
  const draft=result.draft;
  emailState.pendingDraft=draft;
  dom.emailConfirmGateway.textContent=`${draft.gateway.name} · ${draft.gateway.target}`;
  dom.emailConfirmFrequency.textContent=`${(draft.gateway.dialFrequencyHz/1e6).toFixed(6)} MHz`;
  dom.emailConfirmOffset.textContent=`${draft.gateway.offsetHz} Hz`;
  dom.emailConfirmFrames.textContent=String(emailFrameEstimate(draft));
  dom.emailConfirmPayload.textContent=draft.payload;
  dom.emailConfirmDialog.returnValue="";
  dom.emailConfirmDialog.showModal();
}

function waitForRadioFrequency(frequency,timeoutMs=12000) {
  return new Promise((resolve,reject)=>{
    const started=Date.now();
    const timer=setInterval(()=>{
      if(state.radio.frequency===frequency){clearInterval(timer);resolve();}
      else if(Date.now()-started>=timeoutMs){clearInterval(timer);reject(new Error("TRX did not confirm the gateway dial frequency."));}
    },100);
  });
}

function startEmailTx(draft) {
  const js8=currentJs8(),transport=Js8Email.transportParts(draft.payload,draft.gateway.target);
  activeEncoder.setToneOffset(draft.gateway.offsetHz).configure({myCall:js8.myCall,
    toCall:transport.toCall,mode:emailTxMode(),clockCorrectionMs:js8.clockCorrectionMs});
  const item=queueOutgoing(Js8Protocol.formatDirectedMessage({myCall:js8.myCall,
    toCall:transport.toCall,text:transport.text}));
  item.email=true; emailState.activeOutgoing=item;
  driveEncoder(activeEncoder.encode(transport.text),error=>failOutgoing(item,error));
}

async function transmitPendingEmail() {
  const draft=emailState.pendingDraft; emailState.pendingDraft=null;
  if(!draft)return;
  try {
    const current=Js8Email.buildDraft(draft.gateway,draft.recipientEmail,draft.body);
    if(current.payload!==draft.payload)throw new Error("Email draft changed before transmission.");
    const blocks=txBlockReasons(false);
    if(blocks.length)throw new Error(blocks.join("; "));
    emailState.status="Tuning the TRX to the gateway…"; renderControls();
    if(currentJs8().txOffsetHz!==draft.gateway.offsetHz)setJs8Setting("txOffsetHz",draft.gateway.offsetHz);
    if(state.radio.frequency!==draft.gateway.dialFrequencyHz){
      await requestFrequency(draft.gateway.dialFrequencyHz);
      await waitForRadioFrequency(draft.gateway.dialFrequencyHz);
    }
    const readyBlocks=txBlockReasons(false);
    if(readyBlocks.length)throw new Error(readyBlocks.join("; "));
    startEmailTx(draft);
    dom.emailMessage.value="";
    emailState.status="Queued for RF transmission. Gateway reception and email delivery are unconfirmed.";
    renderControls();
  } catch(error) {
    if(state.pendingFrequency===draft.gateway.dialFrequencyHz)state.pendingFrequency=null;
    emailState.status=`Email TX failed: ${error.message}`;
    dom.emailError.textContent=error.message; renderControls();
  }
}

function addTransferLog(record,text) {
  if(!record)return;
  if(!Array.isArray(record.log))record.log=[];
  record.log.push({at:Date.now(),text:String(text)});
  if(record.log.length>100)record.log.splice(0,record.log.length-100);
  record.lastActivityAt=Date.now();record.lastProtocol=String(text);binState.lastProtocol=String(text);
}

async function saveTransfer(record) {
  if(!record)return false;
  record.updatedAt=Date.now();
  const index=binState.sessions.findIndex(item=>item.id===record.id);
  if(index>=0)binState.sessions[index]=record;else binState.sessions.push(record);
  try{await transferStore.save(record);binState.storageError="";renderControls();return true;}
  catch(error){binState.storageError=`Transfer storage failed: ${error.message}`;renderControls();return false;}
}

function transferRecordFromPrepared(prepared,peer,profile) {
  const now=Date.now();
  return {id:prepared.manifest.transferId,direction:"tx",peerCallsign:peer,
    fileName:prepared.manifest.fileName,mimeType:prepared.manifest.mimeType,
    originalSize:prepared.manifest.originalSize,compression:"none",
    blockSize:prepared.manifest.blockSize,blockCount:prepared.manifest.blockCount,
    sha256Hex:prepared.manifest.sha256Hex,hash12:prepared.manifest.hash12,
    blocks:prepared.blocks.map(item=>item.bytes),profileKey:profile.key,submode:profile.submode,
    windowSize:profile.windowSize,state:"offered",acknowledged:[],retransmitQueue:[],
    retransmittedBlocks:0,lastWindow:[],offerAttempts:0,statusAttempts:0,
    createdAt:now,startedAt:now,updatedAt:now,lastActivityAt:now,log:[]};
}

function encodedTransferBlock(record,sequence) {
  const bytes=record.blocks?.[sequence];
  if(!bytes)throw new Error(`Transfer block ${sequence} is unavailable.`);
  return {sequence,binaryLength:bytes.length,crc16:Js8FileTransfer.crc16Ccitt(bytes),
    payloadBase32:Js8FileTransfer.base32Encode(bytes),bytes};
}

function clearTransferTimer() {
  if(binState.responseTimer)clearTimeout(binState.responseTimer);
  binState.responseTimer=null;
}

function queueFileProtocol(record,peer,messages,onDone=null,force=false) {
  const list=Array.isArray(messages)?messages:[messages];
  list.forEach((text,index)=>binState.txQueue.push({record,peer,text,
    onSent:index===list.length-1?onDone:null,force}));
  pumpFileProtocolTx();
}

function pumpFileProtocolTx() {
  if(binState.txCurrent||!binState.txQueue.length)return;
  const task=binState.txQueue[0];
  if(task.record?.state==="paused"&&!task.force)return;
  if(!["idle","completed","aborted","fault"].includes(state.txStatus))return;
  const blocks=txBlockReasons(false,true);
  if(blocks.length){
    if(task.record&&!terminalTransferState(task.record.state)){task.record.state="paused";addTransferLog(task.record,`PAUSED ${blocks.join("; ")}`);saveTransfer(task.record);}
    return;
  }
  binState.txQueue.shift();binState.txCurrent=task;
  addTransferLog(task.record,`TX ${task.text}`);
  const js8=currentJs8(),profile=task.record?Js8FileTransfer.PROFILES[task.record.profileKey]:currentBinProfile();
  activeEncoder.setToneOffset(js8.txOffsetHz).configure({myCall:js8.myCall,toCall:task.peer,
    mode:profile.submode,clockCorrectionMs:js8.clockCorrectionMs});
  const item=queueOutgoing(Js8Protocol.formatDirectedMessage({myCall:js8.myCall,toCall:task.peer,text:task.text}));
  item.fileTransfer=true;task.outgoing=item;
  driveEncoder(activeEncoder.encode(task.text),error=>failOutgoing(item,error));
}

function finishFileProtocolTx(status) {
  const task=binState.txCurrent;if(!task)return;
  binState.txCurrent=null;
  if(status==="completed"){
    Promise.resolve(task.onSent&&task.onSent()).catch(error=>failTransfer(task.record,error));
    pumpFileProtocolTx();
    return;
  }
  if(task.record&&!terminalTransferState(task.record.state)&&task.record.state!=="paused"){
    task.record.state="paused";addTransferLog(task.record,`TX ${status}; session paused`);saveTransfer(task.record);
  }
  binState.txQueue=[];renderControls();
}

function transferTimeoutMs(record,kind) {
  if(TEST_MODE)return 3000;
  const profile=Js8FileTransfer.PROFILES[record.profileKey]||Js8FileTransfer.PROFILES.NORMAL;
  return 1000*(kind==="offer"?profile.offerTimeoutSeconds:profile.statusTimeoutSeconds);
}

function armTransferTimeout(record,kind) {
  clearTransferTimer();
  binState.responseTimer=setTimeout(()=>handleTransferTimeout(record,kind),transferTimeoutMs(record,kind));
}

function handleTransferTimeout(record,kind) {
  if(binState.active!==record||terminalTransferState(record.state)||record.state==="paused")return;
  if(kind==="offer"&&record.offerAttempts<=Js8FileTransfer.DEFAULTS.offerRetries){
    addTransferLog(record,"ACCEPT timeout; repeating OFFER");sendFileOffer(record);return;
  }
  record.statusAttempts=(record.statusAttempts||0)+1;
  if(record.statusAttempts<=Js8FileTransfer.DEFAULTS.statusRetries){
    addTransferLog(record,"Status timeout; sending QUERY");
    queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeQuery(record.id),()=>armTransferTimeout(record,"status"));
    saveTransfer(record);return;
  }
  record.state="paused";addTransferLog(record,"Timeout retry limit reached; session paused");saveTransfer(record);
}

function sendFileOffer(record) {
  record.state="waiting-accept";record.offerAttempts=(record.offerAttempts||0)+1;
  const text=Js8FileTransfer.encodeOffer({transferId:record.id,originalSize:record.originalSize,
    blockCount:record.blockCount,blockSize:record.blockSize,compression:record.compression,
    hash12:record.hash12,fileName:record.fileName});
  queueFileProtocol(record,record.peerCallsign,text,()=>{saveTransfer(record);armTransferTimeout(record,"offer");});
  saveTransfer(record);
}

function transferFrameCount(peer,text) {
  return Js8Protocol.buildReplyFrames({myCall:currentJs8().myCall,toCall:peer,text}).length;
}

function sendNextFileWindow(record) {
  if(record.state==="paused"||terminalTransferState(record.state))return;
  const acknowledged=new Set(record.acknowledged||[]),all=Array.from({length:record.blockCount+1},(_,index)=>index);
  let sequences=[];
  if(record.retransmitQueue?.length){sequences=record.retransmitQueue.splice(0,record.windowSize);record.retransmittedBlocks=(record.retransmittedBlocks||0)+sequences.length;}
  else sequences=all.filter(sequence=>!acknowledged.has(sequence)).slice(0,record.windowSize);
  if(!sequences.length){record.state="waiting-complete";addTransferLog(record,"All blocks acknowledged; waiting for COMPLETE");saveTransfer(record);armTransferTimeout(record,"status");return;}
  const profile=Js8FileTransfer.PROFILES[record.profileKey],dutySequences=[];let seconds=0;
  for(const sequence of sequences){
    const text=Js8FileTransfer.encodeData(record.id,encodedTransferBlock(record,sequence));
    const duration=transferFrameCount(record.peerCallsign,text)*profile.periodSeconds;
    if(dutySequences.length&&seconds+duration>Js8FileTransfer.DEFAULTS.maxContinuousTxSeconds)break;
    dutySequences.push(sequence);seconds+=duration;
  }
  sequences=dutySequences;record.lastWindow=sequences.slice();record.statusScope=Math.max(...sequences);record.state="sending";
  const messages=sequences.map(sequence=>Js8FileTransfer.encodeData(record.id,encodedTransferBlock(record,sequence)));
  messages.push(Js8FileTransfer.encodeEnd(record.id,Math.max(...sequences)));
  queueFileProtocol(record,record.peerCallsign,messages,()=>{record.state="waiting-status";record.statusAttempts=0;addTransferLog(record,"RX status window");saveTransfer(record);armTransferTimeout(record,"status");});
  saveTransfer(record);
}

function acknowledgeThrough(record,through,missing=[]) {
  const missingSet=new Set(missing),acknowledged=new Set(record.acknowledged||[]);
  for(let sequence=0;sequence<=Math.min(Number(through)||0,record.blockCount);sequence+=1)
    if(!missingSet.has(sequence))acknowledged.add(sequence);
  record.acknowledged=[...acknowledged].sort((a,b)=>a-b);
}

function failTransfer(record,error) {
  clearTransferTimer();
  if(record){record.state="failed";addTransferLog(record,`FAILED ${error.message||error}`);saveTransfer(record);}
  binState.storageError=error.message||String(error);renderControls();
}

async function prepareSelectedFile() {
  const file=dom.binFile.files&&dom.binFile.files[0];
  binState.prepared=null;binState.storageError="";
  if(!file){renderControls();return;}
  binState.preparing=true;renderControls();
  try{
    const profile=currentBinProfile();Js8FileTransfer.enforceFileLimit(file.size,profile);
    const bytes=new Uint8Array(await file.arrayBuffer());
    binState.prepared=await Js8FileTransfer.prepareBytes(bytes,{fileName:file.name,mimeType:file.type});
  }catch(error){binState.storageError=error.message;}
  finally{binState.preparing=false;renderControls();}
}

function openBinConfirmation() {
  if(!binState.prepared)return;
  let peer;try{peer=Js8FileTransfer.normalizeCallsign(binState.peerDraft);}catch(error){binState.storageError=error.message;renderControls();return;}
  const profile=currentBinProfile(),manifest=binState.prepared.manifest,estimate=Js8FileTransfer.estimateDuration(manifest.originalSize,profile);
  dom.binConfirmPeer.textContent=peer;dom.binConfirmFile.textContent=`${manifest.fileName} · ${formatBytes(manifest.originalSize)}`;
  dom.binConfirmProfile.textContent=`${profile.label} · ${profile.periodSeconds} s frames · ${profile.windowSize}-block negotiated window`;
  dom.binConfirmPlan.textContent=estimate?`${formatMinutes(estimate.optimisticMinutes)}–${formatMinutes(estimate.plannedMinutes)}, including 30% repair reserve${manifest.originalSize>=profile.warningSize?" · ABOVE RECOMMENDED SIZE; operators must remain present":""}`:"Unavailable";
  dom.binConfirmHash.textContent=manifest.sha256Hex;
  dom.binConfirmDialog.returnValue="";dom.binConfirmDialog.showModal();
}

async function copyPreparedFileHash() {
  const hash=binState.prepared?.manifest.sha256Hex;if(!hash)return;
  try{
    if(navigator.clipboard?.writeText)await navigator.clipboard.writeText(hash);
    else{const input=document.createElement("textarea");input.value=hash;document.body.append(input);input.select();document.execCommand("copy");input.remove();}
    dom.binCopyHash.textContent="COPIED";setTimeout(()=>dom.binCopyHash.textContent="COPY HASH",1200);
  }catch(error){binState.storageError=`Unable to copy hash: ${error.message}`;renderControls();}
}

async function beginPreparedTransfer() {
  const prepared=binState.prepared;if(!prepared)return;
  try{
    const peer=Js8FileTransfer.normalizeCallsign(binState.peerDraft),profile=currentBinProfile();
    Js8FileTransfer.enforceFileLimit(prepared.manifest.originalSize,profile);
    const record=transferRecordFromPrepared(prepared,peer,profile);
    binState.active=record;binState.sessions.push(record);binState.prepared=null;dom.binFile.value="";
    addTransferLog(record,`CREATED ${record.fileName} ${record.originalSize}B SHA256 ${record.sha256Hex}`);
    if(!await saveTransfer(record)){record.state="failed";return;}
    sendFileOffer(record);
  }catch(error){binState.storageError=error.message;renderControls();}
}

function radioMessageEndpoints(item) {
  const prefix=String(item.text||"").slice(0,String(item.text||"").indexOf(Js8FileTransfer.PROTOCOL_PREFIX));
  const match=/^\s*([^:\s]+):\s+([^\s]+)/.exec(prefix);
  const calls=item.callsigns||[];
  return {from:String(match?.[1]||calls[0]||"").toUpperCase(),to:String(match?.[2]||calls[1]||"").toUpperCase()};
}

async function handleFileActivityMessage(item) {
  if(!String(item.text||"").includes(Js8FileTransfer.PROTOCOL_PREFIX))return;
  let message;try{message=Js8FileTransfer.parseMessage(item.text);}catch(error){binState.lastProtocol=`RX INVALID ${error.message}`;renderControls();return;}
  if(!message)return;
  const endpoints=radioMessageEndpoints(item),own=currentJs8().myCall;
  if(!sameCall(endpoints.to,own)||sameCall(endpoints.from,own))return;
  binState.lastProtocol=`RX ${String(item.text).slice(String(item.text).indexOf("~F1"))}`;
  if(message.type==="offer"){await handleIncomingFileOffer(message,endpoints.from,item.submode,item.snr);return;}
  const record=binState.active;
  if(!record||record.id!==message.id||!sameCall(record.peerCallsign,endpoints.from))return;
  const station=state.activity.calls.find(item=>sameCall(item.call,endpoints.from));
  if(station)record.lastSnr=station.snr;
  addTransferLog(record,binState.lastProtocol);
  clearTransferTimer();
  if(record.direction==="tx")await handleOutgoingFileResponse(record,message);
  else await handleIncomingFileMessage(record,message);
  if(!await saveTransfer(record)){record.state="paused";clearTransferTimer();}
}

async function handleIncomingFileOffer(message,peer,submode,snr) {
  const active=binState.active;
  if(active&&!terminalTransferState(active.state)){
    if(active.direction==="rx"&&active.id===message.id&&sameCall(active.peerCallsign,peer)){
      const profile=Js8FileTransfer.PROFILES[active.profileKey];queueFileProtocol(active,peer,Js8FileTransfer.encodeAccept(active.id,active.windowSize,profile),null,true);
    }else queueFileProtocol(active,peer,Js8FileTransfer.encodeReject(message.id,"BUSY"),null,true);
    return;
  }
  const profile=Js8FileTransfer.profileForSubmode(submode);
  try{
    Js8FileTransfer.enforceFileLimit(message.size,profile);
    if(message.compression!=="none"||message.blockSize!==Js8FileTransfer.DEFAULTS.blockSizeBytes||message.blockCount!==Math.ceil(message.size/message.blockSize))throw new Error("Unsupported OFFER parameters.");
  }catch(_error){queueFileProtocol(null,peer,Js8FileTransfer.encodeReject(message.id,"POLICY"),null,true);return;}
  binState.incomingOffer={...message,peer,profileKey:profile.key,snr};
  dom.binIncomingPeer.textContent=peer;dom.binIncomingFile.textContent=message.fileName;
  dom.binIncomingSize.textContent=formatBytes(message.size);dom.binIncomingHash.textContent=message.hash12;
  dom.binIncomingDialog.returnValue="";dom.binIncomingDialog.showModal();
}

async function acceptIncomingFileOffer() {
  const offer=binState.incomingOffer;if(!offer)return;
  const now=Date.now(),profile=Js8FileTransfer.PROFILES[offer.profileKey];
  const record={id:offer.id,direction:"rx",peerCallsign:offer.peer,fileName:offer.fileName,
    mimeType:"application/octet-stream",originalSize:offer.size,compression:offer.compression,
    blockSize:offer.blockSize,blockCount:offer.blockCount,hash12:offer.hash12,sha256Hex:"",
    blocks:Array(offer.blockCount+1).fill(null),profileKey:profile.key,submode:profile.submode,
    windowSize:profile.windowSize,state:"receiving",retransmittedBlocks:0,createdAt:now,
    startedAt:now,updatedAt:now,lastActivityAt:now,lastSnr:offer.snr,log:[]};
  binState.active=record;binState.sessions.push(record);binState.incomingOffer=null;
  addTransferLog(record,`ACCEPTED OFFER ${record.fileName} ${record.originalSize}B`);
  if(!await saveTransfer(record)){
    record.state="failed";queueFileProtocol(null,record.peerCallsign,Js8FileTransfer.encodeReject(record.id,"STORAGE"),null,true);return;
  }
  queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeAccept(record.id,record.windowSize,profile),null,true);
}

function rejectIncomingFileOffer(reason="POLICY") {
  const offer=binState.incomingOffer;if(!offer)return;
  queueFileProtocol(null,offer.peer,Js8FileTransfer.encodeReject(offer.id,reason),null,true);
  binState.incomingOffer=null;renderControls();
}

async function handleOutgoingFileResponse(record,message) {
  if(message.type==="accept"){
    if(!message.profile||message.windowSize<1){failTransfer(record,new Error("Peer returned an invalid ACCEPT."));return;}
    record.profileKey=message.profile.key;record.submode=message.profile.submode;
    record.windowSize=Math.min(record.windowSize,message.windowSize,8);record.accepted=true;record.state="sending";record.statusAttempts=0;sendNextFileWindow(record);return;
  }
  if(message.type==="ack"){
    acknowledgeThrough(record,message.sequence);record.statusAttempts=0;sendNextFileWindow(record);return;
  }
  if(message.type==="nack"){
    const key=`${message.id}`;let parts=binState.nackParts.get(key)||{total:message.parts,values:new Map()};parts.values.set(message.part,message.sequences);binState.nackParts.set(key,parts);
    if(parts.values.size<parts.total){armTransferTimeout(record,"status");return;}
    const missing=[...parts.values.values()].flat();binState.nackParts.delete(key);
    if(missing==="ALL"||missing.includes?.("ALL")){record.acknowledged=[];record.retransmitQueue=Array.from({length:record.blockCount+1},(_,index)=>index);}
    else{acknowledgeThrough(record,record.statusScope??Math.max(...(record.lastWindow||[0])),missing);record.retransmitQueue=[...new Set(missing)].filter(sequence=>sequence>=0&&sequence<=record.blockCount);}
    sendNextFileWindow(record);return;
  }
  if(message.type==="complete"){
    if(message.hash12!==record.hash12){failTransfer(record,new Error("Peer COMPLETE hash does not match."));return;}
    record.state="complete";record.completedAt=Date.now();addTransferLog(record,"COMPLETE verified by peer");return;
  }
  if(message.type==="reject"){record.state="rejected";addTransferLog(record,`REJECTED ${message.reason}`);return;}
  if(message.type==="cancel"){record.state="cancelled";addTransferLog(record,`CANCELLED BY PEER ${message.reason}`);return;}
  if(message.type==="query"){
    if(record.state==="complete")queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeComplete(record.id,record.hash12),null,true);
    else if(record.lastWindow?.length)queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeEnd(record.id,Math.max(...record.lastWindow)),null,true);
  }
}

function expectedIncomingBlockLength(record,sequence) {
  if(sequence===0)return 32;
  if(sequence<1||sequence>record.blockCount)throw new Error("DATA sequence is outside this transfer.");
  return sequence===record.blockCount?record.originalSize-record.blockSize*(record.blockCount-1):record.blockSize;
}

function incomingMissing(record,through=record.blockCount) {
  const result=[];for(let sequence=0;sequence<=Math.min(through,record.blockCount);sequence+=1)if(!record.blocks[sequence])result.push(sequence);return result;
}

async function finishIncomingTransfer(record) {
  record.state="verifying";await saveTransfer(record);
  try{
    const result=await Js8FileTransfer.verifyReceived(record);record.sha256Hex=result.sha256Hex;
    record.fileBytes=result.bytes;record.state="complete";record.completedAt=Date.now();
    addTransferLog(record,`SHA256 OK ${result.sha256Hex}`);
    queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeComplete(record.id,result.hash12),null,true);
  }catch(error){record.state="failed";addTransferLog(record,`HASH FAILED ${error.message}`);queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeNacks(record.id,"ALL"),null,true);}
}

async function handleIncomingFileMessage(record,message) {
  if(message.type==="data"){
    try{
      const length=expectedIncomingBlockLength(record,message.sequence),bytes=Js8FileTransfer.decodeDataMessage(message,length);
      if(!record.blocks[message.sequence])record.blocks[message.sequence]=bytes;
      else addTransferLog(record,`DUPLICATE block ${message.sequence}`);
    }catch(error){addTransferLog(record,`BAD block ${message.sequence}: ${error.message}`);}
    return;
  }
  if(message.type==="end"){
    const missing=incomingMissing(record,message.sequence);
    if(missing.length){queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeNacks(record.id,missing),null,true);return;}
    queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeAck(record.id,message.sequence),()=>{if(!incomingMissing(record).length)return finishIncomingTransfer(record);},true);return;
  }
  if(message.type==="query"){
    const missing=incomingMissing(record);
    if(missing.length)queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeNacks(record.id,missing),null,true);
    else if(record.state==="complete")queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeComplete(record.id,record.hash12),null,true);
    else queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeAck(record.id,record.blockCount),()=>finishIncomingTransfer(record),true);
    return;
  }
  if(message.type==="cancel"){record.state="cancelled";addTransferLog(record,`CANCELLED BY PEER ${message.reason}`);}
}

function pauseFileTransfer() {
  const record=binState.active;if(!record||terminalTransferState(record.state))return;
  clearTransferTimer();binState.txQueue=[];record.state="paused";addTransferLog(record,"PAUSED BY OPERATOR");saveTransfer(record);
  if(binState.txCurrent)activeEncoder.abort();
}

function resumeFileTransfer() {
  const record=binState.active;if(!record||record.state!=="paused")return;
  record.state=record.direction==="rx"?"receiving":"sending";addTransferLog(record,"RESUMED BY OPERATOR");saveTransfer(record);
  if(record.direction==="tx"){
    if(!record.accepted)sendFileOffer(record);
    else queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeQuery(record.id),()=>armTransferTimeout(record,"status"));
  }else{
    const missing=incomingMissing(record);
    queueFileProtocol(record,record.peerCallsign,missing.length?Js8FileTransfer.encodeNacks(record.id,missing):Js8FileTransfer.encodeAck(record.id,record.blockCount),null,true);
  }
}

function stopFileTransfer() {
  const record=binState.active;if(!record||terminalTransferState(record.state))return;
  clearTransferTimer();binState.txQueue=[];record.state="cancelled";addTransferLog(record,"CANCELLED BY OPERATOR");saveTransfer(record);
  const sendCancel=()=>queueFileProtocol(record,record.peerCallsign,Js8FileTransfer.encodeCancel(record.id,"USER"),null,true);
  if(binState.txCurrent){const current=binState.txCurrent;current.onSent=sendCancel;activeEncoder.abort();setTimeout(sendCancel,0);}else sendCancel();
}

function downloadReceivedFile() {
  const record=binState.active;if(!record||record.direction!=="rx"||record.state!=="complete"||!record.fileBytes)return;
  const url=URL.createObjectURL(new Blob([record.fileBytes],{type:record.mimeType||"application/octet-stream"}));
  const anchor=document.createElement("a");anchor.href=url;anchor.download=record.fileName;anchor.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}

async function restoreFileTransfers() {
  try{
    binState.sessions=await transferStore.all();
    const sorted=[...binState.sessions].sort((a,b)=>Number(b.updatedAt||0)-Number(a.updatedAt||0));
    const resumable=sorted.find(item=>!terminalTransferState(item.state));
    binState.active=resumable||sorted[0]||null;
    if(resumable){resumable.state="paused";addTransferLog(resumable,"RESTORED AFTER PAGE RELOAD");await saveTransfer(resumable);}
  }catch(error){binState.storageError=`Transfer restore failed: ${error.message}`;}
  binState.restored=true;renderControls();
}

// ---- radio commands and TX --------------------------------------------------

async function requestFrequency(frequency) {
  state.pendingFrequency=frequency; dom.frequencyMenu.hidden=true; dom.trxFrequency.setAttribute("aria-expanded","false"); renderHeader();
  try {
    const response=await fetch("/cmd",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({type:"setFrequency",frequency:String(frequency)})});
    if (!response.ok) throw new Error(`TRX request ${response.status}`);
    await ensureUsbDataMode();
    return true;
  } catch (error) { dom.modemState.textContent=error.message; dom.modemState.className="modem-state error"; state.pendingFrequency=null; renderHeader(); throw error; }
}

// Tuning a preset also prepares the radio for JS8 by switching to USB-D, but only when
// not already there. Best-effort: a failed mode set never rolls back the frequency change.
// Uses the generic civ.raw endpoint (26 00 <mode> <data> <filter>) so the firmware CAT
// code stays untouched — USB (0x01), DATA on (0x01), current FILx slot (fallback FIL1).
async function ensureUsbDataMode() {
  if (!state.radio.connected || state.radio.mode === "USB-D") return;
  const filter=[1,2,3].includes(Number(state.radio.filter)) ? Number(state.radio.filter) : 1;
  const data="26000101"+String(filter).padStart(2,"0");
  try { await fetch("/cmd",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({type:"civ.raw",data})}); }
  catch (_error) {}
}

function driveEncoder(prepared, onError) {
  Promise.resolve(prepared).then(()=>{
    if (txTick) clearInterval(txTick);
    txTick=setInterval(()=>activeEncoder.tick(Date.now()),20);
    activeEncoder.tick(Date.now());
  }).catch(onError);
}

function queueOutgoing(messageText, conversationCall="") {
  const item={direction:"outgoing",time:new Date().toISOString().slice(11,19),
    text:messageText,status:"queued",sentChars:0,activeFraction:0,txRenderKey:""};
  if(conversationCall){
    if(!state.conversations[conversationCall])state.conversations[conversationCall]=[];
    state.conversations[conversationCall].push(item);
  }
  state.activeOutgoing=item;
  state.lastOutgoing=item;
  renderConversation();
  renderTxPayload();
  return item;
}

function failOutgoing(item,error) {
  state.txStatus="fault";
  dom.modemState.textContent=error.message;
  dom.modemState.className="modem-state error";
  item.status="fault";
  item.activeFraction=0;
  if(state.activeOutgoing===item)state.activeOutgoing=null;
  renderControls();
  renderConversation();
}

function startTx(text) {
  const js8=currentJs8();
  const cq=cqType(text);
  if(cq){
    const preview=Js8Protocol.buildCqFrames({myCall:js8.myCall,grid:js8.grid,cq});
    const item=queueOutgoing(preview[0].messageText);
    activeEncoder.setToneOffset(js8.txOffsetHz).configure({myCall:js8.myCall,toCall:"",mode:selectedMode(),clockCorrectionMs:js8.clockCorrectionMs});
    driveEncoder(activeEncoder.encode("",{kind:"cq",cq,grid:js8.grid,toneHz:js8.txOffsetHz}),error=>failOutgoing(item,error));
    return;
  }
  activeEncoder.setToneOffset(js8.txOffsetHz).configure({myCall:js8.myCall,toCall:state.selectedCall,mode:selectedMode(),clockCorrectionMs:js8.clockCorrectionMs});
  const item=queueOutgoing(Js8Protocol.formatDirectedMessage({myCall:js8.myCall,
    toCall:state.selectedCall,text}),state.selectedCall);
  driveEncoder(activeEncoder.encode(text),error=>failOutgoing(item,error));
}

function startHeartbeat() {
  const js8=currentJs8(), tone=js8.txOffsetHz;
  const preview=Js8Protocol.buildHeartbeatFrames({myCall:js8.myCall,grid:js8.grid});
  const item=queueOutgoing(preview[0].messageText);
  activeEncoder.setToneOffset(tone).configure({myCall:js8.myCall,toCall:"",mode:selectedMode(),clockCorrectionMs:js8.clockCorrectionMs});
  driveEncoder(activeEncoder.encode("",{kind:"heartbeat",grid:js8.grid,toneHz:tone}),error=>failOutgoing(item,error));
}

function toggleTune() {
  if(state.tuneActive){activeEncoder.abort();return;}
  const js8=currentJs8(), tone=js8.txOffsetHz;
  activeEncoder.setToneOffset(tone).configure({myCall:js8.myCall,toCall:"",mode:selectedMode(),clockCorrectionMs:js8.clockCorrectionMs});
  driveEncoder(activeEncoder.encode("",{kind:"tune",toneHz:tone,immediate:true}),error=>{
    state.tuneActive=false; state.txStatus="fault"; dom.modemState.textContent=error.message;
    dom.modemState.className="modem-state error"; renderControls();
  });
}

function closeMessagePresets() {
  dom.messagePresetsMenu.hidden=true;
  dom.messagePresetsButton.setAttribute("aria-expanded","false");
}

function messagePresetValue(key) {
  const station=state.activity.calls.find(item=>item.call===state.selectedCall);
  return ({cq:"CQ CQ CQ",snr:station?`SNR ${formatJs8Snr(station.snr)}`:"",
    "snr-query":"SNR?","copy-query":"HW CPY?",rr:"RR",fb:"FB",qsl:"QSL",
    again:"AGN?","73":"73",sk:"SK"})[key] || "";
}

function insertMessagePreset(key) {
  const value=messagePresetValue(key);
  if(!value)return;
  dom.message.value=value;
  closeMessagePresets();
  dom.message.dispatchEvent(new Event("input",{bubbles:true}));
  dom.message.focus({preventScroll:true});
  dom.message.setSelectionRange(value.length,value.length);
}

function updateOutgoingTxProgress(txState) {
  const item=state.activeOutgoing;
  if(!item)return;
  const frames=txState.frames||[];
  let sent=Number(item.sentChars)||0;
  for(let index=0;index<Math.min(txState.frameIndex,frames.length);index+=1)
    sent=Math.max(sent,Number(frames[index].textEnd)||0);
  let activeFraction=0;
  const frame=frames[txState.frameIndex];
  const frameStart=Number(frame?.textStart)||0,frameEnd=Number(frame?.textEnd)||frameStart;
  if(frame&&frameEnd>frameStart&&txState.status==="transmitting"){
    const exact=frameStart+(frameEnd-frameStart)*Math.max(0,Math.min(1,Number(txState.frameProgress)||0));
    sent=Math.max(sent,Math.floor(exact));
    activeFraction=exact-Math.floor(exact);
  }else if(frame&&frameEnd>frameStart&&txState.status==="draining"){
    sent=Math.max(sent,frameEnd);
  }
  if(txState.status==="completed")sent=item.text.length;
  item.sentChars=Math.max(0,Math.min(item.text.length,sent));
  item.activeFraction=["aborted","fault","completed"].includes(txState.status)?0:activeFraction;
  item.status=txState.status;
  const renderKey=`${item.status}|${item.sentChars}|${Math.round(item.activeFraction*20)}`;
  if(renderKey!==item.txRenderKey){item.txRenderKey=renderKey;renderConversation();renderTxPayload();}
  if(["aborted","fault","completed"].includes(txState.status))state.activeOutgoing=null;
}

async function pollRadio() {
  if (radioPollInFlight) return;
  radioPollInFlight=true;
  try {
    const response=await fetch("/state",{cache:"no-store"}); if (!response.ok) throw new Error();
    const next=await response.json();
    state.radio={...state.radio,...next,frequency:Number(next.frequency)||0};
    const activityFrequencyChanged=selectActivityFrequency(state.radio.frequency);
    if (state.pendingFrequency && state.radio.frequency===state.pendingFrequency) state.pendingFrequency=null;
    ensureAudio(); if(activityFrequencyChanged)renderActivity(); renderHeader(); renderControls();
  } catch (_error) { state.radio.connected=false; stopAudio(); renderHeader(); renderControls(); }
  finally { radioPollInFlight=false; }
}

async function reconnectRadio() {
  if(state.reconnectPending)return;
  state.reconnectPending=true; renderHeader();
  try {
    const response=await fetch("/lan/reconnect",{method:"POST"});
    if(!response.ok)throw new Error(`Reconnect failed (HTTP ${response.status})`);
    state.radio.lanStatus="connecting";
  } catch(error) {
    dom.modemState.textContent=error.message;
    dom.modemState.className="modem-state error";
  } finally {
    state.reconnectPending=false; renderHeader();
  }
}

async function checkLanConfiguration() {
  const query=new URLSearchParams({scope:"js8call"});
  if(TEST_MODE && PAGE_PARAMS.get("lanFixture"))query.set("fixture",PAGE_PARAMS.get("lanFixture"));
  let ready=false, detail="";
  try {
    const response=await fetch(`/setup-data.json?${query}`,{cache:"no-store"});
    if(!response.ok)throw new Error(`HTTP ${response.status}`);
    const config=await response.json();
    const ip=String(config.lanip||"").trim();
    const user=String(config.lanuser||"").trim();
    const password=String(config.lanpass||"").trim();
    const missing=[];
    if(config.trx1transport!=="lan")missing.push("TRX1 connection is not LAN");
    if(!ip || ip==="0.0.0.0")missing.push("radio IP address is missing");
    if(!user)missing.push("network username is missing");
    if(!password)missing.push("network password is missing");
    ready=missing.length===0;
    detail=missing.join(" · ");
  } catch (error) {
    detail=`Unable to read TRX1 configuration: ${error.message}`;
  }
  state.lanConfig={checked:true,ready,detail};
  document.body.classList.remove("lan-config-checking");
  document.body.classList.toggle("lan-required-only",!ready);
  dom.lanRequired.hidden=ready;
  dom.lanRequiredDetail.textContent=detail;
  return ready;
}

// ---- bindings ---------------------------------------------------------------

function setJs8Setting(key,value) { currentJs8()[key]=value; persistSettings(); }
function confirmJs8Leave(event) {
  if(!state.lanConfig.ready)return;
  event.preventDefault();
  // Modern browsers intentionally replace custom text with their own warning,
  // but returnValue is still required to request the confirmation dialog.
  event.returnValue="Leaving JS8LAN will discard received data.";
  return event.returnValue;
}
function bind() {
  dom.modeSelect.addEventListener("change",()=>selectMode(dom.modeSelect.value));
  dom.trxHelpButton.addEventListener("click",()=>openTrxHelp("manual"));
  dom.trxReconnect.addEventListener("click",reconnectRadio);
  dom.trxHelpDialog.addEventListener("click",event=>{if(event.target===dom.trxHelpDialog)dom.trxHelpDialog.close();});
  dom.trxFrequency.addEventListener("click",()=>{const open=dom.frequencyMenu.hidden;dom.frequencyMenu.hidden=!open;dom.trxFrequency.setAttribute("aria-expanded",String(open));});
  dom.frequencyMenu.addEventListener("click",event=>{const button=event.target.closest("[data-frequency]");if(button)requestFrequency(Number(button.dataset.frequency)).catch(()=>{});});
  dom.waterfall.addEventListener("click",event=>{const rect=dom.waterfall.getBoundingClientRect();setJs8Setting("txOffsetHz",Math.round(RX_LOW+(event.clientX-rect.left)/rect.width*(RX_HIGH-RX_LOW)));activeEncoder&&activeEncoder.setToneOffset(currentJs8().txOffsetHz);});
  dom.recipient.addEventListener("change",()=>chooseCall(dom.recipient.value.toUpperCase().replace(/[^A-Z0-9/]/g,"")));
  dom.recipientClear.addEventListener("click",clearRecipient);
  dom.messagePresetsButton.addEventListener("click",()=>{
    const opening=dom.messagePresetsMenu.hidden;
    dom.messagePresetsMenu.hidden=!opening;
    dom.messagePresetsButton.setAttribute("aria-expanded",opening?"true":"false");
    if(opening)dom.messagePresetsMenu.querySelector("button:not(:disabled)")?.focus({preventScroll:true});
  });
  dom.messagePresetsMenu.addEventListener("click",event=>{
    const button=event.target.closest("[data-message-preset]");
    if(button&&!button.disabled)insertMessagePreset(button.dataset.messagePreset);
  });
  document.addEventListener("click",event=>{if(!event.target.closest(".message-field"))closeMessagePresets();});
  dom.txSessionMode.addEventListener("change",()=>{state.txSessionMode=dom.txSessionMode.value;renderControls();});
  dom.emailAddress.addEventListener("input",renderControls);
  dom.emailMessage.addEventListener("input",renderControls);
  dom.emailGateway.addEventListener("change",()=>{emailState.selectedId=dom.emailGateway.value;emailState.status="Draft is not stored in message history.";renderControls();});
  dom.emailGatewayAdd.addEventListener("click",()=>openEmailGatewayDialog());
  dom.emailGatewayEdit.addEventListener("click",()=>{const gateway=selectedEmailGateway();if(gateway)openEmailGatewayDialog(gateway);});
  dom.emailGatewayDelete.addEventListener("click",deleteSelectedEmailGateway);
  dom.emailGatewayFormat.addEventListener("change",()=>{
    dom.emailGatewayTemplateRow.hidden=dom.emailGatewayFormat.value!=="template";
    if(dom.emailGatewayFormat.value==="aprs-email2"&&!emailState.editingId){dom.emailGatewayMaxBody.value="40";dom.emailGatewayPolicy.value="aprs";dom.emailGatewayTarget.value=dom.emailGatewayTarget.value||"@APRSIS";}
  });
  dom.emailGatewayForm.addEventListener("submit",saveEmailGateway);
  dom.emailGatewayDialog.querySelectorAll("[data-email-dialog-close]").forEach(button=>button.addEventListener("click",()=>dom.emailGatewayDialog.close()));
  dom.emailComposer.addEventListener("submit",event=>{event.preventDefault();if(!dom.emailSend.disabled)openEmailConfirmation();});
  dom.emailMessage.addEventListener("keydown",event=>{if(event.key==="Enter"&&event.ctrlKey&&!event.isComposing){event.preventDefault();if(!dom.emailSend.disabled)openEmailConfirmation();}});
  dom.emailConfirmDialog.querySelector('header button').addEventListener("click",()=>dom.emailConfirmDialog.close("cancel"));
  dom.emailConfirmDialog.addEventListener("close",()=>{if(dom.emailConfirmDialog.returnValue==="send")transmitPendingEmail();else emailState.pendingDraft=null;});
  dom.binRecipient.addEventListener("input",()=>{binState.peerDraft=dom.binRecipient.value.toUpperCase().replace(/[^A-Z0-9/]/g,"");renderControls();});
  dom.binFile.addEventListener("change",prepareSelectedFile);
  dom.binPeerExpected.addEventListener("change",renderControls);
  dom.binComposer.addEventListener("submit",event=>{event.preventDefault();if(!dom.binOffer.disabled)openBinConfirmation();});
  dom.binConfirmDialog.querySelector('header button').addEventListener("click",()=>dom.binConfirmDialog.close("cancel"));
  dom.binCopyHash.addEventListener("click",copyPreparedFileHash);
  dom.binConfirmDialog.addEventListener("close",()=>{if(dom.binConfirmDialog.returnValue==="send")beginPreparedTransfer();});
  dom.binIncomingDialog.querySelector('header button').addEventListener("click",()=>dom.binIncomingDialog.close("reject"));
  dom.binIncomingDialog.addEventListener("close",()=>{if(dom.binIncomingDialog.returnValue==="accept")acceptIncomingFileOffer();else rejectIncomingFileOffer("POLICY");});
  dom.binPause.addEventListener("click",pauseFileTransfer);
  dom.binResume.addEventListener("click",resumeFileTransfer);
  dom.binStop.addEventListener("click",stopFileTransfer);
  dom.binDownload.addEventListener("click",downloadReceivedFile);
  for (const container of [dom.traffic,dom.stationRows]) container.addEventListener("click",event=>{const node=event.target.closest("[data-call]");if(node)chooseCall(node.dataset.call);});
  dom.trafficFilter.addEventListener("click",event=>{const button=event.target.closest("[data-traffic-filter]");if(!button||button.disabled)return;state.trafficFilter=button.dataset.trafficFilter;renderActivity();});
  dom.stationHead.addEventListener("click",event=>{const button=event.target.closest("[data-station-sort]");if(!button)return;const key=button.dataset.stationSort;if(state.stationSort.key===key)state.stationSort.direction=state.stationSort.direction==="asc"?"desc":"asc";else state.stationSort={key,direction:"asc"};renderActivity();});
  dom.txSpeed.addEventListener("change",()=>setJs8Setting("speed",dom.txSpeed.value));
  dom.txOffset.addEventListener("change",()=>setJs8Setting("txOffsetHz",Math.max(RX_LOW,Math.min(RX_HIGH,Number(dom.txOffset.value)||1500))));
  dom.myCall.addEventListener("input",()=>{state.settingsDraft.myCall=dom.myCall.value;});
  dom.myGrid.addEventListener("input",()=>{state.settingsDraft.grid=dom.myGrid.value;});
  dom.txGain.addEventListener("input",()=>{state.settingsDraft.txGain=dom.txGain.value;});
  dom.myCall.addEventListener("change",()=>{const value=dom.myCall.value.toUpperCase();state.settingsDraft.myCall=null;setJs8Setting("myCall",value);renderActivity();});
  dom.myGrid.addEventListener("change",()=>{const value=dom.myGrid.value.toUpperCase();state.settingsDraft.grid=null;setJs8Setting("grid",value);});
  dom.followSpeed.addEventListener("change",()=>setJs8Setting("followSpeed",dom.followSpeed.checked));
  dom.clockCorrection.addEventListener("change",()=>setJs8Setting("clockCorrectionMs",Number(dom.clockCorrection.value)||0));
  dom.autoTiming.addEventListener("change",()=>setJs8Setting("autoTiming",dom.autoTiming.checked));
  dom.txGain.addEventListener("change",()=>{const value=state.settingsDraft.txGain===null?dom.txGain.value:state.settingsDraft.txGain;state.settingsDraft.txGain=null;setJs8Setting("txGain",Number(value)||.25);});
  dom.txSafety.addEventListener("change",()=>setJs8Setting("txSafetyAccepted",dom.txSafety.checked));
  dom.resetSettings.addEventListener("click",()=>{const reset=Js8Settings.reset(localStorage);settings=reset.settings;state.settingsDraft={myCall:null,grid:null,txGain:null};state.activeMode=settings.activeModem;dom.storageState.textContent=reset.label;applySettingsToRuntime();renderActivity();renderControls();});
  dom.startupRetry.addEventListener("click",()=>location.reload());
  dom.heartbeat.addEventListener("click",()=>{if(!dom.heartbeat.disabled)startHeartbeat();});
  dom.tune.addEventListener("click",()=>{if(!dom.tune.disabled)toggleTune();});
  dom.composer.addEventListener("submit",event=>{event.preventDefault();const text=dom.message.value.trim();if (!text || dom.send.disabled)return;dom.message.value="";renderControls();startTx(text);});
  dom.message.addEventListener("input",renderControls);
  dom.message.addEventListener("keydown",event=>{if(event.key!=="Enter" || event.isComposing)return;event.preventDefault();if(!dom.send.disabled)dom.composer.requestSubmit();});
  dom.abort.addEventListener("click",()=>activeEncoder&&activeEncoder.abort());
  document.querySelectorAll("details[data-section]").forEach(details=>details.addEventListener("toggle",()=>{settings.ui.disclosures[details.dataset.section]=details.open;persistSettings(false);}));
  window.addEventListener("resize",resizeWaterfall);
  window.addEventListener("beforeunload",confirmJs8Leave);
  window.addEventListener("pagehide",()=>{if(activeEncoder)activeEncoder.abort();stopAudio();});
  document.addEventListener("visibilitychange",()=>{if(document.hidden&&activeEncoder)activeEncoder.abort();});
  addEventListener("keydown",event=>{if(event.key==="Escape"){if(activeEncoder)activeEncoder.abort();dom.frequencyMenu.hidden=true;closeMessagePresets();}});
}

async function init() {
  if(!await checkLanConfiguration())return;
  populateModes(); bind(); loadTxModule();
  if(!hasSeenTrxHelp())openTrxHelp("first");
  for (const details of document.querySelectorAll("details[data-section]"))
    if (Object.prototype.hasOwnProperty.call(settings.ui.disclosures,details.dataset.section)) details.open=settings.ui.disclosures[details.dataset.section];
  dom.storageState.textContent=loaded.label;
  renderStartup(); selectMode(state.activeMode); resizeWaterfall(); renderActivity(); renderDiagnostics();
  restoreFileTransfers();
  setInterval(()=>{dom.utcClock.textContent=`UTC ${new Date().toISOString().slice(11,19)}`;},250);
  renderRhythm(); setInterval(renderRhythm,100);
  pollRadio(); setInterval(pollRadio,500);
  if (TEST_MODE) self.__dataTest={
    setActivity(activity){state.testActivityLocked=true;applyDecoderActivity(activity);renderActivity();},
    setRadioFrequency(frequency){state.radio.frequency=Number(frequency)||0;if(selectActivityFrequency(state.radio.frequency))renderActivity();renderHeader();renderControls();},
    setRadioConnection(connected,lanStatus=connected?"linked":"disconnected"){state.radio.connected=Boolean(connected);state.radio.lanStatus=lanStatus;renderHeader();renderControls();},
    setAudioLive(live){state.lastAudioMs=live?performance.now():0;renderHeader();},
    activityCounts(){return {messages:state.activity.messages.length,calls:state.activity.calls.length};},
    setRadioMode(mode){state.radio.mode=mode;renderHeader();},
    setRadioTx(tx){state.radio.tx=Boolean(tx);renderHeader();},
    feedSpectrum(samples){ingestSpectrum(samples);},
    spectrumState(){return {agcLow,agcHigh,agcReady,rows:spectrumRows,fill:spectrumFill};},
    selectedCall(){return state.selectedCall;},
    fileProtocol(){return {prepared:binState.prepared,active:binState.active,lastProtocol:binState.lastProtocol};},
    receiveFileMessage(item){return handleFileActivityMessage(item);}
  };
}

init();
