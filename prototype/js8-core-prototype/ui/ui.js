// PROTOTYPE — confirmed compact JS8 operator workflow, isolated from production.

(function () {
  const modems = [
    {id: "js8call", label: "JS8Call", available: true},
    {id: "rtty45", label: "RTTY 45", available: false},
    {id: "psk31", label: "PSK31", available: false},
    {id: "ft8", label: "FT8", available: false},
    {id: "ft4", label: "FT4", available: false},
    {id: "cw", label: "CW skimmer", available: false},
  ];
  const loaded = Js8Settings.load(localStorage);
  const stored = loaded.settings;
  const js8 = stored.modems.js8call;
  const query = new URLSearchParams(location.search);
  const requestedModem = query.get("modem");
  const initialOpen = query.get("open");
  const state = {
    myCall: js8.myCall,
    grid: js8.grid,
    selectedCall: "K0OG",
    txOffsetHz: js8.txOffsetHz,
    activeModem: modems.some(modem => modem.id === requestedModem)
      ? requestedModem : stored.activeModem,
    modems,
    rxRange: [500, 2700],
    speed: js8.speed,
    clockCorrectionMs: js8.clockCorrectionMs,
    autoTiming: js8.autoTiming,
    settingsStatus: loaded.label,
    observedDtMs: {A: -20, B: -10, C: -4, E: -40, I: 1},
    disclosures: {...stored.ui.disclosures},
    trxMenuOpen: initialOpen === "frequency",
    trx: {
      connected: true,
      frequencyHz: 7078000,
      mode: "USB",
      source: "TRX state stub",
      request: null,
    },
    tx: {status: "idle", queuedText: "", frames: []},
    conversations: {
      K0OG: [
        {direction: "incoming", time: "22:12:42", text: "OK1HRA K0OG?", status: "received"},
        {direction: "outgoing", time: "22:12:58", text: "K0OG OK1HRA, good evening", status: "sent"},
        {direction: "incoming", time: "22:13:16", text: "Good evening, you are +02 here", status: "received"},
      ],
    },
    calls: [
      {call: "K0OG", snr: 2, offsetHz: 703, mode: "A", age: "12 s", grid: ""},
      {call: "KD8SKZ", snr: -14, offsetHz: 806, mode: "A", age: "18 s", grid: ""},
      {call: "KG9B", snr: -14, offsetHz: 521, mode: "A", age: "24 s", grid: ""},
      {call: "KN4CRD", snr: 33, offsetHz: 1291, mode: "E", age: "31 s", grid: ""},
      {call: "KN4ZXG", snr: -27, offsetHz: 1866, mode: "A", age: "43 s", grid: "FM16"},
    ],
    messages: [
      {time: "22:14:03", from: "KN4ZXG", to: "@HB", text: "FM16", mode: "A", offsetHz: 1866},
      {time: "22:14:11", from: "KD8SKZ", to: "KN4CRD", text: "HEARTBEAT SNR -14", mode: "A", offsetHz: 806},
      {time: "22:14:18", from: "K0OG", to: "KN4CRD", text: "SNR +2", mode: "A", offsetHz: 703},
      {time: "22:14:30", from: "KN4CRD", to: "", text: "TEST", mode: "E", offsetHz: 1291},
    ],
  };
  if (initialOpen && initialOpen !== "frequency" &&
      Object.prototype.hasOwnProperty.call(state.disclosures, initialOpen)) {
    for (const key of Object.keys(state.disclosures)) state.disclosures[key] = false;
    state.disclosures[initialOpen] = true;
  }

  const app = document.querySelector("#app");
  let trxRequestSequence = 0;
  const esc = value => String(value).replace(/[&<>\"]/g, character =>
    ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"})[character]);
  const signed = value => `${value >= 0 ? "+" : ""}${value}`;
  const opened = key => state.disclosures[key] ? " open" : "";

  function settingsSnapshot() {
    return {schemaVersion: Js8Settings.SCHEMA_VERSION,
      activeModem: state.activeModem,
      modems: {js8call: {myCall: state.myCall, grid: state.grid,
        speed: state.speed, txOffsetHz: state.txOffsetHz,
        clockCorrectionMs: state.clockCorrectionMs,
        autoTiming: state.autoTiming}},
      ui: {disclosures: state.disclosures}};
  }

  function applyStored(result) {
    const next = result.settings;
    const modem = next.modems.js8call;
    state.activeModem = next.activeModem;
    state.myCall = modem.myCall;
    state.grid = modem.grid;
    state.speed = modem.speed;
    state.txOffsetHz = modem.txOffsetHz;
    state.clockCorrectionMs = modem.clockCorrectionMs;
    state.autoTiming = modem.autoTiming;
    state.disclosures = {...next.ui.disclosures};
    state.settingsStatus = result.label;
  }

  function persistSettings() {
    applyStored(Js8Settings.save(localStorage, settingsSnapshot()));
  }

  // Deliberately isolated. Production integration will replace this with the
  // /cmd adapter and confirm actual state only through subsequent /state data.
  const trxTransportStub = {
    setFrequency(request) {
      return new Promise(resolve => setTimeout(() => resolve({ok: true,
        reqId: request.reqId, frequencyHz: request.frequency}), 220));
    },
  };

  function requestTrxFrequency(presetId) {
    const reqId = `trx-${++trxRequestSequence}`;
    const request = Js8TrxPresets.createFrequencyRequest(presetId, reqId);
    state.trx.request = {reqId, frequencyHz: request.frequency,
      status: "requesting"};
    state.trxMenuOpen = false;
    render();
    trxTransportStub.setFrequency(request).then(reply => {
      if (!state.trx.request || state.trx.request.reqId !== reply.reqId) return;
      // This simulates the later authoritative GET /state confirmation.
      state.trx.frequencyHz = reply.frequencyHz;
      state.trx.request = null;
      render();
    });
  }

  function frequencyMenu() {
    if (!state.trxMenuOpen) return "";
    const presets = Js8TrxPresets.PRESETS.map(preset => {
      const current = preset.frequencyHz === state.trx.frequencyHz ? " current" : "";
      return `<button type="button" class="frequency-preset${current}" data-trx-preset="${preset.id}">
        <strong>${preset.band}</strong><span>${Js8TrxPresets.formatFrequency(preset.frequencyHz)}</span></button>`;
    }).join("");
    return `<section class="frequency-menu" role="dialog" aria-label="JS8 dial frequencies">
      <header><strong>JS8 dial frequencies</strong><small>Choose a band to request TRX tuning</small></header>
      <div class="frequency-presets">${presets}</div>
      <footer>Preset list: bundled JS8Call source · Hz values are dial frequencies</footer>
    </section>`;
  }

  function topbar() {
    const pending = state.trx.request;
    return `<header class="topbar"><span class="brand">DATA / IC-705</span>
      <div class="trx-control">
        <button type="button" class="trx-frequency${pending ? " pending" : ""}" data-trx-menu aria-expanded="${state.trxMenuOpen}">
          <span class="trx-dot"></span><span><small>${pending ? "TUNE REQUEST" : "TRX"}</small><strong>${Js8TrxPresets.formatFrequency(pending ? pending.frequencyHz : state.trx.frequencyHz)}</strong></span><span class="unit">MHz</span><span class="chevron">▾</span>
        </button>${frequencyMenu()}
      </div>
      <span class="trx-mode" title="Mode reported by TRX state">${esc(state.trx.mode || "---")}</span>
      <span class="live">● RX LIVE</span><span class="operator">${esc(state.myCall)} · ${esc(state.grid)}</span>
      <span class="spacer"></span><span class="pill">${state.speed}</span>
      <span class="pill">TX ${state.txOffsetHz} Hz</span><span class="muted">SYS UTC ${signed(state.clockCorrectionMs)} ms</span></header>`;
  }

  function waterfall() {
    return `<section class="waterfall" data-waterfall><canvas></canvas>
      <div class="tx-marker"></div><div class="frequency-scale"><span>500 Hz</span><span>1100</span><span>1700</span><span>2200</span><span>2700 Hz</span></div></section>`;
  }

  function messages() {
    return state.messages.map(item => `<article class="message ${item.to && !item.to.startsWith("@") ? "directed" : ""}">
      <header><span>${item.time}</span><span>${item.mode}</span><span>${item.offsetHz} Hz</span></header>
      <div><strong data-call="${item.from}">${item.from}</strong>${item.to ? ` → <strong data-call="${item.to}">${item.to}</strong>` : ""}: ${esc(item.text)}</div></article>`).join("");
  }

  function conversation() {
    const items = state.conversations[state.selectedCall] || [];
    const bubbles = items.length ? items.map(item => `<article class="chat-row ${item.direction}">
      <div class="chat-bubble"><header><strong>${item.direction === "incoming" ? state.selectedCall : "You"}</strong><time>${item.time} UTC</time></header>
      <div>${esc(item.text)}</div><footer>${item.direction === "incoming" ? "received" : esc(item.status)}</footer></div>
    </article>`).join("") : `<div class="chat-empty">No directed messages with ${state.selectedCall} yet.</div>`;
    return `<section class="keyboard-session"><header><div><strong>${state.selectedCall}</strong><small>Keyboard-to-keyboard session</small></div><span class="session-active">● active</span></header><div class="chat-thread">${bubbles}</div></section>`;
  }

  function settingsPanel() {
    const speeds = ["AUTO", "A", "B", "C", "E", "I"].map(value =>
      `<option${value === state.speed ? " selected" : ""}>${value}</option>`).join("");
    return `<div class="settings">
      <label><span>My callsign</span><input value="${esc(state.myCall)}" data-setting="myCall"></label>
      <label><span>Grid</span><input value="${esc(state.grid)}" data-setting="grid"></label>
      <label><span>Speed</span><select data-setting="speed">${speeds}</select></label>
      <label class="check-setting"><span>Automatic timing</span><input type="checkbox" data-setting="autoTiming"${state.autoTiming ? " checked" : ""}></label>
      <label><span>Manual correction</span><input type="number" min="-1000" max="1000" value="${state.clockCorrectionMs}" data-setting="clockCorrectionMs"${state.autoTiming ? " disabled" : ""}><small>ms</small></label>
      <div class="settings-storage"><span>${esc(state.settingsStatus)} · schema v${Js8Settings.SCHEMA_VERSION}</span><button type="button" data-reset-settings>Restore defaults</button></div>
    </div>`;
  }

  function modemMenu() {
    const active = state.modems.find(modem => modem.id === state.activeModem) || state.modems[0];
    const options = state.modems.map(modem => `<option value="${modem.id}"${modem.id === active.id ? " selected" : ""}>${modem.label}${modem.available ? "" : " — not installed"}</option>`).join("");
    return `<nav class="modem-menu" aria-label="Modem selection"><label><span>Modem</span><select data-modem>${options}</select></label><span class="modem-state ${active.available ? "available" : "unavailable"}">${active.available ? "RX/TX prototype available" : "Decoder and encoder not installed"}</span></nav>`;
  }

  function renderPage() {
    const activeModem = state.modems.find(modem => modem.id === state.activeModem) || state.modems[0];
    if (!activeModem.available) return `<div class="variant-d">${topbar()}<div class="compact-page">${modemMenu()}
      <section class="modem-empty"><strong>${activeModem.label}</strong><span>This modem is registered as a catalogue placeholder, but its decoder and encoder are not installed.</span><span>Select JS8Call to return to the working prototype.</span></section></div></div>`;
    const rows = state.calls.map(item => `<tr class="${item.call === state.selectedCall ? "selected" : ""}" data-call="${item.call}"><td class="call">${item.call}</td><td>${signed(item.snr)}</td><td>${item.offsetHz}</td><td>${item.mode}</td><td>${item.age}</td></tr>`).join("");
    const recipients = state.calls.map(item => `<option${item.call === state.selectedCall ? " selected" : ""}>${item.call}</option>`).join("");
    const speeds = ["AUTO", "A", "B", "C", "E", "I"].map(value => `<option${value === state.speed ? " selected" : ""}>${value}</option>`).join("");
    return `<div class="variant-d">${topbar()}<div class="compact-page">${modemMenu()}
      <details class="compact-section compact-spectrum" data-section="spectrum"${opened("spectrum")}><summary><span>Spectrum &amp; TX tuning</span><span class="summary-status">TX ${state.txOffsetHz} Hz · click waterfall to tune</span></summary>${waterfall()}</details>
      <details class="compact-section compact-reply" data-section="reply"${opened("reply")}><summary><span>Reply</span><span class="summary-status">${state.selectedCall} · ${state.tx.status}</span></summary>
        <div class="quick-controls">
          <label><span>Recipient</span><select data-recipient>${recipients}</select></label>
          <label><span>TX offset</span><input type="number" min="500" max="2700" value="${state.txOffsetHz}" data-tx-offset><small>Hz</small></label>
          <label><span>Speed</span><select data-setting="speed">${speeds}</select></label>
          <div class="inline-status"><span>TX status</span><b>${state.tx.status}</b>${state.tx.status !== "idle" ? '<button class="abort" type="button" data-abort>ABORT</button>' : ""}</div>
        </div>
        ${conversation()}
        <form class="composer compact-composer"><label for="compact-message">Message</label><textarea id="compact-message" aria-label="TX message" placeholder="Message to ${state.selectedCall}">${esc(state.tx.queuedText)}</textarea><button class="send" type="submit">SEND <small>${state.selectedCall}</small></button></form>
      </details>
      <details class="compact-section compact-traffic" data-section="traffic"${opened("traffic")}><summary><span>Recent traffic</span><span class="summary-status">${state.messages.length} messages</span></summary><div class="messages">${messages()}</div></details>
      <details class="compact-section" data-section="stations"${opened("stations")}><summary><span>Stations</span><span class="summary-status">${state.calls.length} active</span></summary><table class="traffic-table"><thead><tr><th>Call</th><th>SNR</th><th>Hz</th><th>Mode</th><th>Age</th></tr></thead><tbody>${rows}</tbody></table></details>
      <details class="compact-section" data-section="settings"${opened("settings")}><summary><span>Modem settings</span><span class="summary-status">${state.myCall} · ${state.grid} · ${state.speed}</span></summary>${settingsPanel()}</details>
      <details class="compact-section" data-section="timing"${opened("timing")}><summary><span>Timing diagnostics</span><span class="summary-status">${state.autoTiming ? "AUTO" : "MANUAL"} · SYS UTC ${signed(state.clockCorrectionMs)} ms</span></summary><div class="compact-diagnostics"><span>Browser/system clock</span><code>${state.autoTiming ? "automatic bounded correction" : `manual ${signed(state.clockCorrectionMs)} ms`}</code><span>Observed dt</span><code>${Object.entries(state.observedDtMs).map(([key, value]) => `${key}: ${value} ms`).join(" · ")}</code></div></details>
      <details class="state-dump" data-section="prototypeState"${opened("prototypeState")}><summary>Complete prototype state</summary><pre>${esc(JSON.stringify(state, null, 2))}</pre></details>
    </div></div>`;
  }

  function drawWaterfall() {
    document.querySelectorAll("[data-waterfall]").forEach(box => {
      const canvas = box.querySelector("canvas");
      const width = Math.max(600, Math.floor(box.clientWidth));
      const height = Math.max(130, Math.floor(box.clientHeight));
      canvas.width = width; canvas.height = height;
      const context = canvas.getContext("2d");
      const image = context.createImageData(width, height);
      for (let y = 0; y < height; y += 1) for (let x = 0; x < width; x += 1) {
        const ridge = state.calls.reduce((sum, call) => {
          const px = (call.offsetHz - 500) / 2200 * width;
          return sum + Math.exp(-((x - px) ** 2) / 18) * (0.35 + (y % 17) / 30);
        }, 0);
        const noise = ((x * 17 + y * 31 + (x * y) % 29) % 37) / 90;
        const value = Math.min(1, noise + ridge);
        const index = (y * width + x) * 4;
        image.data[index] = Math.floor(15 + 70 * value);
        image.data[index + 1] = Math.floor(25 + 210 * value);
        image.data[index + 2] = Math.floor(35 + 170 * value + 40 * Math.max(0, value - .7));
        image.data[index + 3] = 255;
      }
      context.putImageData(image, 0, 0);
      box.querySelector(".tx-marker").style.left = `${(state.txOffsetHz - 500) / 2200 * 100}%`;
      box.addEventListener("click", event => {
        const rect = box.getBoundingClientRect();
        state.txOffsetHz = Math.round(500 + (event.clientX - rect.left) / rect.width * 2200);
        persistSettings(); render();
      });
    });
  }

  function bind() {
    document.querySelectorAll("[data-trx-menu]").forEach(button => button.addEventListener("click", () => {
      state.trxMenuOpen = !state.trxMenuOpen; render();
    }));
    document.querySelectorAll("[data-trx-preset]").forEach(button => button.addEventListener("click", () =>
      requestTrxFrequency(button.dataset.trxPreset)));
    document.querySelectorAll("[data-call]").forEach(element => element.addEventListener("click", event => {
      const call = event.currentTarget.dataset.call;
      if (call && !call.startsWith("@")) state.selectedCall = call;
      render();
    }));
    document.querySelectorAll(".composer").forEach(form => form.addEventListener("submit", event => {
      event.preventDefault();
      const text = form.querySelector("textarea").value.trim();
      if (!text) return;
      state.tx = {status: "queued", queuedText: text,
        frames: Js8Protocol.buildReplyFrames({myCall: state.myCall,
          toCall: state.selectedCall, text})};
      if (!state.conversations[state.selectedCall]) state.conversations[state.selectedCall] = [];
      state.conversations[state.selectedCall].push({direction: "outgoing",
        time: new Date().toISOString().slice(11, 19), text, status: "queued"});
      render();
    }));
    document.querySelectorAll("[data-abort]").forEach(button => button.addEventListener("click", () => {
      state.tx = {status: "idle", queuedText: "", frames: []}; render();
    }));
    document.querySelectorAll("[data-setting]").forEach(input => input.addEventListener("change", () => {
      const key = input.dataset.setting;
      state[key] = input.type === "checkbox" ? input.checked
        : key === "clockCorrectionMs" ? Number(input.value) : input.value;
      persistSettings(); render();
    }));
    document.querySelectorAll("[data-recipient]").forEach(input => input.addEventListener("change", () => {
      state.selectedCall = input.value; render();
    }));
    document.querySelectorAll("[data-tx-offset]").forEach(input => input.addEventListener("change", () => {
      state.txOffsetHz = Math.max(500, Math.min(2700, Number(input.value) || 1500));
      persistSettings(); render();
    }));
    document.querySelectorAll("[data-modem]").forEach(input => input.addEventListener("change", () => {
      if (state.modems.some(modem => modem.id === input.value)) state.activeModem = input.value;
      persistSettings();
      const url = new URL(location.href); url.searchParams.set("modem", state.activeModem);
      history.replaceState({}, "", url); render();
    }));
    document.querySelectorAll("[data-reset-settings]").forEach(button => button.addEventListener("click", () => {
      applyStored(Js8Settings.reset(localStorage)); render();
    }));
    document.querySelectorAll("details[data-section]").forEach(details => details.addEventListener("toggle", () => {
      state.disclosures[details.dataset.section] = details.open;
      persistSettings();
    }));
  }

  function render() {
    app.innerHTML = renderPage();
    bind(); drawWaterfall();
  }

  addEventListener("keydown", event => {
    if (event.key !== "Escape" || !state.trxMenuOpen) return;
    state.trxMenuOpen = false; render();
  });
  addEventListener("popstate", render);
  window.__js8Prototype = {state, requestTrxFrequency};
  render();
})();
