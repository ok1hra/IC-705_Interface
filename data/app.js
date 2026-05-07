const wsStatus = document.getElementById("wsStatus");
const radioStatus = document.getElementById("radioStatus");
const powerStatus = document.getElementById("powerStatus");
const frequencyStatus = document.getElementById("frequencyStatus");
const modeStatus = document.getElementById("modeStatus");
const addressStatus = document.getElementById("addressStatus");
const log = document.getElementById("log");
const payload = document.getElementById("hexPayload");
const diagnosticModeInput = document.getElementById("diagnosticMode");
const diagnosticFilterInput = document.getElementById("diagnosticFilter");
const diagnosticRequestsOnlyInput = document.getElementById("diagnosticRequestsOnly");
const diagnosticPrettyInput = document.getElementById("diagnosticPretty");
const expectReplyInput = document.getElementById("expectReply");
const expectAckInput = document.getElementById("expectAck");
const setFrequencyInput = document.getElementById("setFrequency");
const setModeInput = document.getElementById("setMode");
const setFilterInput = document.getElementById("setFilter");
const cwTextInput = document.getElementById("cwText");

let socket;
let reqId = 1;
let diagnosticEntries = [];

function isDiagnosticMessage(message) {
  if (typeof message === "string") {
    return message.startsWith("WebSocket");
  }

  if (!message || typeof message !== "object") {
    return false;
  }

  return [
    "hello",
    "queued",
    "reply",
    "error",
    "state",
    "civ.tx",
    "civ.rx"
  ].includes(message.type);
}

function isRequestBoundMessage(message) {
  if (!message || typeof message !== "object") {
    return false;
  }

  if (message.reqId) {
    return true;
  }

  if (message.type === "reply" || message.type === "queued" || message.type === "error") {
    return true;
  }

  return false;
}

function formatLogLine(message) {
  if (typeof message === "string") {
    return message;
  }

  if (diagnosticPrettyInput.checked) {
    return JSON.stringify(message, null, 2);
  }

  return JSON.stringify(message);
}

function appendLog(line, parsedMessage = null) {
  if (diagnosticFilterInput.checked) {
    const diagnosticCandidate = parsedMessage ?? line;
    if (!isDiagnosticMessage(diagnosticCandidate)) {
      return;
    }
  }

  if (diagnosticRequestsOnlyInput.checked) {
    const candidate = parsedMessage ?? line;
    if (!isRequestBoundMessage(candidate)) {
      return;
    }
  }

  const stamp = new Date().toLocaleTimeString();
  const formatted = formatLogLine(parsedMessage ?? line);
  log.textContent += `[${stamp}] ${formatted}\n`;
  log.scrollTop = log.scrollHeight;

  if (diagnosticModeInput.checked) {
    diagnosticEntries.push({
      stamp,
      entry: parsedMessage ?? line
    });
  }
}

function fallbackCopyText(text) {
  const textarea = document.createElement("textarea");
  textarea.value = text;
  textarea.setAttribute("readonly", "");
  textarea.style.position = "fixed";
  textarea.style.top = "-1000px";
  textarea.style.left = "-1000px";
  document.body.appendChild(textarea);
  textarea.focus();
  textarea.select();

  let copied = false;
  try {
    copied = document.execCommand("copy");
  } catch (error) {
    copied = false;
  }

  document.body.removeChild(textarea);
  return copied;
}

function copyDiagnostic() {
  const exported = diagnosticEntries.map(({ stamp, entry }) => {
    if (typeof entry === "string") {
      return `[${stamp}] ${entry}`;
    }
    const req = entry.reqId ? ` reqId=${entry.reqId}` : "";
    const kind = entry.type ? `${entry.type}${req}` : "message";
    return `# ${stamp} ${kind}\n${JSON.stringify(entry, null, 2)}`;
  }).join("\n\n");

  const text = exported || "No diagnostic entries captured yet.";
  if (navigator.clipboard && window.isSecureContext) {
    navigator.clipboard.writeText(text).then(() => {
      appendLog("Diagnostic log copied to clipboard");
    }).catch(() => {
      if (fallbackCopyText(text)) {
        appendLog("Diagnostic log copied to clipboard");
      } else {
        appendLog("Copy to clipboard failed");
      }
    });
    return;
  }

  if (fallbackCopyText(text)) {
    appendLog("Diagnostic log copied to clipboard");
  } else {
    appendLog("Copy to clipboard failed");
  }
}

function setState(state) {
  if (!state) {
    return;
  }
  radioStatus.textContent = state.connected ? "Connected" : "Disconnected";
  powerStatus.textContent = state.power ? "On" : "Off";
  frequencyStatus.textContent = `${state.frequency ?? 0} Hz`;
  modeStatus.textContent = state.mode ?? "OFF";
  addressStatus.textContent = state.radioAddress ?? "0x00";
}

function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);

  socket.addEventListener("open", () => {
    wsStatus.textContent = "Connected";
    appendLog("WebSocket connected");
  });

  socket.addEventListener("close", () => {
    wsStatus.textContent = "Disconnected";
    appendLog("WebSocket disconnected, retry in 2 s");
    setTimeout(connect, 2000);
  });

  socket.addEventListener("message", (event) => {
    try {
      const msg = JSON.parse(event.data);
      appendLog(event.data, msg);
      if (msg.type === "hello" && msg.state) {
        setState(msg.state);
      } else if (msg.type === "state" || msg.type === "reply") {
        setState(msg.state ?? msg);
      }
    } catch (error) {
      appendLog(event.data);
      appendLog(`JSON parse error: ${error.message}`);
    }
  });
}

function send(message) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    appendLog("WebSocket is not connected");
    return;
  }
  socket.send(JSON.stringify(message));
}

document.getElementById("refreshState").addEventListener("click", () => {
  send({ type: "getState", reqId: String(reqId++) });
});

document.getElementById("readFrequency").addEventListener("click", () => {
  send({ type: "readFrequency", reqId: String(reqId++) });
});

document.getElementById("readMode").addEventListener("click", () => {
  send({ type: "readMode", reqId: String(reqId++) });
});

document.getElementById("readVfoFrequency").addEventListener("click", () => {
  send({ type: "readVfoFrequency", reqId: String(reqId++) });
});

document.getElementById("readVfoMode").addEventListener("click", () => {
  send({ type: "readVfoMode", reqId: String(reqId++) });
});

document.getElementById("readRit").addEventListener("click", () => {
  send({ type: "readRit", reqId: String(reqId++) });
});

document.getElementById("readSplit").addEventListener("click", () => {
  send({ type: "readSplit", reqId: String(reqId++) });
});

document.getElementById("readSmeter").addEventListener("click", () => {
  send({ type: "readSmeter", reqId: String(reqId++) });
});

document.getElementById("readPower").addEventListener("click", () => {
  send({ type: "readPower", reqId: String(reqId++) });
});

document.getElementById("readSwr").addEventListener("click", () => {
  send({ type: "readSwr", reqId: String(reqId++) });
});

document.getElementById("readAlc").addEventListener("click", () => {
  send({ type: "readAlc", reqId: String(reqId++) });
});

document.getElementById("clearRit").addEventListener("click", () => {
  send({ type: "setRitClear", reqId: String(reqId++) });
});

document.getElementById("sendPayload").addEventListener("click", () => {
  send({
    type: "civ.raw",
    reqId: String(reqId++),
    data: payload.value,
    expectReply: expectReplyInput.checked,
    expectAck: expectAckInput.checked
  });
});

document.getElementById("sendFramed").addEventListener("click", () => {
  send({
    type: "civ.raw",
    reqId: String(reqId++),
    data: payload.value,
    framed: true,
    expectReply: expectReplyInput.checked,
    expectAck: expectAckInput.checked
  });
});

document.getElementById("sendFrequency").addEventListener("click", () => {
  send({ type: "setFrequency", reqId: String(reqId++), frequency: setFrequencyInput.value });
});

document.getElementById("sendMode").addEventListener("click", () => {
  send({ type: "setMode", reqId: String(reqId++), mode: setModeInput.value, filter: setFilterInput.value });
});

document.getElementById("sendCw").addEventListener("click", () => {
  send({ type: "sendCw", reqId: String(reqId++), text: cwTextInput.value });
});

document.getElementById("clearLog").addEventListener("click", () => {
  log.textContent = "";
  diagnosticEntries = [];
});

document.getElementById("copyDiagnostic").addEventListener("click", copyDiagnostic);

connect();
