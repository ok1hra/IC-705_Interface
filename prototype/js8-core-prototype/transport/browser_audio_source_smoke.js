#!/usr/bin/env node
// PROTOTYPE — tiny local HTTP/WebSocket fixture server for headless Chrome.

const crypto = require("crypto");
const fs = require("fs");
const http = require("http");
const path = require("path");
const {spawn} = require("child_process");

const root = path.resolve(__dirname, "..");
const mime = {".html":"text/html", ".js":"text/javascript"};
let connectionCount = 0;
let chrome = null;
let chromeError = "";
let completionTimer = null;
let completed = false;

function complete(pass, text) {
  if (completed) return;
  completed = true;
  clearTimeout(completionTimer);
  if (chrome) chrome.kill("SIGTERM");
  server.close();
  if (pass) console.log(text);
  else { console.error(text || chromeError); process.exitCode = 1; }
}

function aud1(streamId, sequence, firstSample, flags = 0) {
  const wire = Buffer.alloc(200, 0xff);
  wire.write("AUD1", 0, "ascii");
  wire[4] = 1; wire[5] = 1;
  wire.writeUInt16BE(flags, 6); wire.writeUInt16BE(40, 8);
  wire.writeUInt16BE(0, 10); wire.writeUInt32BE(streamId, 12);
  wire.writeUInt32BE(sequence, 16); wire.writeUInt32BE(8000, 20);
  wire.writeBigUInt64BE(BigInt(firstSample), 24); wire.writeUInt32BE(0, 32);
  wire.writeUInt32BE(160, 36);
  return wire;
}

function wsFrame(opcode, payload) {
  const body = Buffer.isBuffer(payload) ? payload : Buffer.from(payload);
  const header = body.length < 126 ? Buffer.from([0x80 | opcode, body.length])
    : Buffer.from([0x80 | opcode, 126, body.length >> 8, body.length & 0xff]);
  return Buffer.concat([header, body]);
}

function sendEpoch(socket, streamId, descriptors, closeAfter) {
  const hello = JSON.stringify({type:"hello", protocol:"AUD1", version:1,
    streamId, rx:[{kind:"RX_ULAW", sampleRate:8000}], maxPayloadBytes:2048});
  socket.write(wsFrame(1, hello));
  for (const descriptor of descriptors)
    socket.write(wsFrame(2, aud1(streamId, ...descriptor)));
  if (closeAfter) {
    socket.write(wsFrame(8, Buffer.alloc(0)));
    socket.end();
  }
}

const server = http.createServer((request, response) => {
  const pathname = new URL(request.url, "http://localhost").pathname;
  if (pathname === "/result" && request.method === "POST") {
    let body = "";
    request.on("data", chunk => { body += chunk; });
    request.on("end", () => {
      response.writeHead(204).end();
      try {
        const result = JSON.parse(body);
        complete(result.pass === true, result.text);
      } catch (error) { complete(false, error.stack || String(error)); }
    });
    return;
  }
  const relative = pathname === "/" ? "transport/browser-audio-source-smoke.html"
    : pathname.slice(1);
  const target = path.resolve(root, relative);
  if (!target.startsWith(`${root}${path.sep}`)) return response.writeHead(403).end();
  fs.readFile(target, (error, bytes) => {
    if (error) return response.writeHead(404).end();
    response.setHeader("Content-Type", mime[path.extname(target)] || "application/octet-stream");
    response.end(bytes);
  });
});

server.on("upgrade", (request, socket) => {
  if (request.url !== "/audiows") return socket.destroy();
  const key = request.headers["sec-websocket-key"];
  const accept = crypto.createHash("sha1")
    .update(`${key}258EAFA5-E914-47DA-95CA-C5AB0DC85B11`).digest("base64");
  socket.write("HTTP/1.1 101 Switching Protocols\r\n" +
    "Upgrade: websocket\r\nConnection: Upgrade\r\n" +
    `Sec-WebSocket-Accept: ${accept}\r\n\r\n`);
  connectionCount += 1;
  if (connectionCount === 1)
    sendEpoch(socket, 101, [[0, 0, 1], [1, 160], [1, 160],
      [3, 480, 4], [4, 640], [5, 800]], true);
  else
    sendEpoch(socket, 202, [[0, 0, 1], [1, 160], [2, 320],
      [3, 480], [4, 640], [5, 800], [6, 960], [7, 1120],
      [8, 1280], [9, 1440]], false);
  socket.on("error", () => {});
});

server.listen(0, "127.0.0.1", () => {
  const url = `http://127.0.0.1:${server.address().port}/`;
  chrome = spawn("google-chrome", ["--headless=new", "--no-sandbox",
    "--disable-gpu", "--disable-dev-shm-usage", "--remote-debugging-port=0", url]);
  chrome.stderr.on("data", chunk => { chromeError += chunk; });
  chrome.on("close", code => {
    if (!completed) complete(false,
      `BROWSER AUDIO SOURCE FAIL Chrome exited ${code}\n${chromeError}`);
  });
  completionTimer = setTimeout(() => complete(false,
    `BROWSER AUDIO SOURCE FAIL server timeout connections=${connectionCount}\n${chromeError}`),
  7000);
});
