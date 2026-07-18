#!/usr/bin/env node
// PROTOTYPE — local-only UI server; optional screenshots close automatically.

const fs = require("fs");
const http = require("http");
const os = require("os");
const path = require("path");
const {spawn} = require("child_process");

const root = path.resolve(__dirname, "..");
const screenshotFlag = process.argv.indexOf("--screenshots");
const screenshotDir = screenshotFlag >= 0 ? path.resolve(process.argv[screenshotFlag + 1]) : null;
const mime = {".html":"text/html", ".js":"text/javascript", ".css":"text/css"};
const server = http.createServer((request, response) => {
  const pathname = new URL(request.url, "http://localhost").pathname;
  const target = path.join(root, pathname === "/" ? "ui/index.html" : pathname);
  if (!target.startsWith(`${root}${path.sep}`)) return response.writeHead(403).end();
  fs.readFile(target, (error, bytes) => {
    if (error) response.writeHead(404).end(String(error));
    else { response.setHeader("Content-Type", mime[path.extname(target)] || "application/octet-stream"); response.end(bytes); }
  });
});

function runChrome(url, output, profile, windowSize = "1440,900") {
  return new Promise((resolve, reject) => {
    const chrome = spawn("google-chrome", ["--headless=new", "--no-sandbox", "--disable-gpu",
      "--disable-dev-shm-usage", "--hide-scrollbars", `--window-size=${windowSize}`,
      `--user-data-dir=${profile}`, `--screenshot=${output}`, url]);
    let error = "";
    chrome.stderr.on("data", chunk => { error += chunk; });
    chrome.on("close", code => code === 0 ? resolve() : reject(new Error(error)));
  });
}

function runChromeDom(url, profile, expected) {
  return new Promise((resolve, reject) => {
    const chrome = spawn("google-chrome", ["--headless=new", "--no-sandbox",
      "--disable-gpu", "--disable-dev-shm-usage", "--virtual-time-budget=1000", `--user-data-dir=${profile}`,
      "--dump-dom", url]);
    let output = ""; let error = "";
    chrome.stdout.on("data", chunk => { output += chunk; });
    chrome.stderr.on("data", chunk => { error += chunk; });
    chrome.on("close", code => code === 0 && output.includes(expected)
      ? resolve() : reject(new Error(`${error}\n${output}`)));
  });
}

server.listen(0, "127.0.0.1", async () => {
  const base = `http://127.0.0.1:${server.address().port}/`;
  if (!screenshotDir) {
    console.log(`PROTOTYPE UI ${base} (confirmed compact workflow)`);
    return;
  }
  fs.mkdirSync(screenshotDir, {recursive: true});
  const profile = fs.mkdtempSync(path.join(os.tmpdir(), "js8-ui-chrome-"));
  const smokeProfile = fs.mkdtempSync(path.join(os.tmpdir(), "js8-settings-chrome-"));
  try {
    await runChromeDom(`${base}ui/settings_browser_smoke.html`, smokeProfile,
      "SETTINGS BROWSER PASS");
    await runChromeDom(`${base}ui/operator_browser_smoke.html`, smokeProfile,
      "OPERATOR BROWSER PASS");
    await runChrome(base, path.join(screenshotDir, "variant-D.png"), profile);
    await runChrome(`${base}?viewport=mobile`, path.join(screenshotDir, "variant-D-mobile.png"), profile, "390,844");
    await runChrome(`${base}?open=settings`, path.join(screenshotDir, "variant-D-settings.png"), profile);
    await runChrome(`${base}?open=frequency`, path.join(screenshotDir, "variant-D-frequency-menu.png"), profile);
    await runChrome(`${base}?open=frequency&viewport=mobile`, path.join(screenshotDir, "variant-D-frequency-menu-mobile.png"), profile, "390,844");
    await runChrome(`${base}?modem=ft8`, path.join(screenshotDir, "variant-D-modem-placeholder.png"), profile);
    console.log("SETTINGS BROWSER PASS");
    console.log("OPERATOR BROWSER PASS");
    console.log(`UI screenshots ${screenshotDir}`);
  } catch (error) {
    console.error(error.stack || error); process.exitCode = 1;
  } finally {
    server.close(); fs.rmSync(profile, {recursive: true, force: true});
    fs.rmSync(smokeProfile, {recursive: true, force: true});
  }
});
