#!/usr/bin/env node
// PROTOTYPE — stage a conservative production asset set without touching data/.

const fs = require("fs");
const path = require("path");
const zlib = require("zlib");

if (process.argv.length !== 5) {
  console.error("usage: build-budget-assets.js REPOSITORY PROTOTYPE STAGE");
  process.exit(2);
}

const repository = path.resolve(process.argv[2]);
const prototype = path.resolve(process.argv[3]);
const stage = path.resolve(process.argv[4]);
fs.mkdirSync(stage, {recursive: true});

const entries = [];
function write(name, bytes, group, encoding) {
  const target = path.join(stage, name);
  fs.writeFileSync(target, bytes);
  entries.push({name, group, encoding, bytes: bytes.length});
}

for (const name of fs.readdirSync(path.join(repository, "data")).sort()) {
  const source = path.join(repository, "data", name);
  if (!fs.statSync(source).isFile() || name.endsWith(".gz")) continue;
  const bytes = fs.readFileSync(source);
  if (/\.(?:html|css|js)$/.test(name))
    write(`${name}.gz`, zlib.gzipSync(bytes, {level: 9, mtime: 0}), "existing-web", "gzip");
  else
    write(name, bytes, "existing-web", "identity");
}

const js8 = [
  ["js8-jsc.bin", "build-protocol/jsc-map.bin"],
  ["js8-decoder.wasm", "build-decoder-wasm/js8-decoder.wasm"],
  ["js8-decoder.js", "build-decoder-wasm/js8-decoder.js"],
  ["js8-prototype.wasm", "build-wasm/js8-prototype.wasm"],
  ["js8-prototype.js", "build-wasm/js8-prototype.js"],
  ["js8-worker.js", "wasm/worker_entry.js"],
  ["js8-worker-runtime.js", "wasm/worker_runtime.js"],
  ["js8-protocol.js", "protocol/protocol_runtime.js"],
  ["js8-tx.js", "tx/tx_runtime.js"],
  ["aud1-websocket.js", "transport/aud1_websocket_session.js"],
  ["js8-adapter.js", "integration/js8_modem_adapter.js"],
];
for (const [name, relative] of js8) {
  const bytes = fs.readFileSync(path.join(prototype, relative));
  const compressed = zlib.brotliCompressSync(bytes, {params: {
    [zlib.constants.BROTLI_PARAM_QUALITY]: 11,
  }});
  write(`${name}.br`, compressed, "js8", "br");
}

const totals = {};
for (const entry of entries) totals[entry.group] = (totals[entry.group] || 0) + entry.bytes;
process.stdout.write(`${JSON.stringify({format: "JS8 asset budget v1", entries, totals,
  payloadBytes: entries.reduce((sum, entry) => sum + entry.bytes, 0),
  requirements: ["HTTP Content-Encoding: br", "application/wasm MIME",
    "application/octet-stream JSC MIME"]}, null, 2)}\n`);
