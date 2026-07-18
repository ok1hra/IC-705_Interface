#!/usr/bin/env node
// PROTOTYPE — serves the real classic Worker and asks headless Chrome to run it.

const fs = require("fs");
const http = require("http");
const os = require("os");
const path = require("path");
const {spawn} = require("child_process");

const root = path.resolve(process.argv[2]);
const pageMode = process.argv[3] === "--page";
const fixture = pageMode ? null : path.resolve(process.argv[3]);
const page = pageMode ? process.argv[4] : "browser/worker-smoke.html";
const mime = {".html":"text/html", ".js":"text/javascript",
              ".wasm":"application/wasm", ".bin":"application/octet-stream"};
const server = http.createServer((request, response) => {
  const target = request.url === "/fixture.bin" && fixture ? fixture :
    path.join(root, request.url === "/" ? page : request.url.split("?", 1)[0]);
  if (target !== fixture && !target.startsWith(`${root}${path.sep}`)) {
    response.writeHead(403).end();
    return;
  }
  fs.readFile(target, (error, bytes) => {
    if (error) response.writeHead(404).end(String(error));
    else {
      response.setHeader("Content-Type", mime[path.extname(target)] || "application/octet-stream");
      response.setHeader("Cache-Control", "no-store");
      response.end(bytes);
    }
  });
});

class PipeCdp {
  constructor(input, output) {
    this.input = input;
    this.nextId = 1;
    this.pending = new Map();
    let buffered = Buffer.alloc(0);
    output.on("data", chunk => {
      buffered = Buffer.concat([buffered, chunk]);
      for (let boundary; (boundary = buffered.indexOf(0)) >= 0;) {
        const raw = buffered.subarray(0, boundary).toString("utf8");
        buffered = buffered.subarray(boundary + 1);
        if (!raw) continue;
        const message = JSON.parse(raw);
        if (message.id && this.pending.has(message.id)) {
          const {resolve, reject} = this.pending.get(message.id);
          this.pending.delete(message.id);
          if (message.error) reject(new Error(message.error.message));
          else resolve(message.result);
        }
      }
    });
  }

  send(method, params = {}, sessionId) {
    const id = this.nextId++;
    const message = {id, method, params};
    if (sessionId) message.sessionId = sessionId;
    return new Promise((resolve, reject) => {
      this.pending.set(id, {resolve, reject});
      this.input.write(`${JSON.stringify(message)}\0`);
    });
  }
}

const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

server.listen(0, "127.0.0.1", async () => {
  const profile = fs.mkdtempSync(path.join(os.tmpdir(), "js8-chrome-"));
  const url = `http://127.0.0.1:${server.address().port}/`;
  const chrome = spawn("google-chrome", ["--headless=new", "--no-sandbox",
    "--disable-gpu", "--disable-dev-shm-usage", `--user-data-dir=${profile}`,
    "--remote-debugging-pipe", "about:blank"],
    {stdio: ["ignore", "ignore", "pipe", "pipe", "pipe"]});
  let stderr = "";
  chrome.stderr.on("data", chunk => { stderr += chunk; });
  const cdp = new PipeCdp(chrome.stdio[3], chrome.stdio[4]);
  try {
    const target = await cdp.send("Target.createTarget", {url});
    const attached = await cdp.send("Target.attachToTarget",
                                    {targetId: target.targetId, flatten: true});
    const session = attached.sessionId;
    await cdp.send("Runtime.enable", {}, session);
    let text = "";
    const deadline = Date.now() + 60000;
    while (Date.now() < deadline) {
      const evaluated = await cdp.send("Runtime.evaluate", {
        expression: "document.querySelector('#result')?.textContent || ''",
        returnByValue: true,
      }, session);
      text = evaluated.result.value || "";
      if (/^(PASS|FAIL|ERROR) /.test(text)) break;
      await delay(250);
    }
    if (!text.startsWith("PASS ")) throw new Error(text || "browser smoke timeout");
    console.log(`BROWSER ${text}`);
  } catch (error) {
    console.error(String(error.stack || error));
    if (stderr) console.error(stderr);
    process.exitCode = 1;
  } finally {
    try { await cdp.send("Browser.close"); } catch {}
    await Promise.race([new Promise(resolve => chrome.once("close", resolve)), delay(3000)]);
    if (chrome.exitCode === null) chrome.kill("SIGKILL");
    server.close();
    fs.rmSync(profile, {recursive: true, force: true});
  }
});
