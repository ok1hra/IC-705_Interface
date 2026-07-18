// PROTOTYPE — real localStorage save/reload check in headless Chrome.
(function () {
  const phase = new URLSearchParams(location.search).get("phase");
  if (!phase) {
    Js8Settings.reset(localStorage);
    Js8Settings.save(localStorage, {schemaVersion: 2, activeModem: "js8call",
      modems: {js8call: {myCall: "K0OG", grid: "FN31", speed: "B",
        txOffsetHz: 1777, clockCorrectionMs: 35, autoTiming: false}},
      ui: {disclosures: {settings: true}}});
    location.replace(`${location.pathname}?phase=reload`);
    return;
  }
  const loaded = Js8Settings.load(localStorage);
  const js8 = loaded.settings.modems.js8call;
  const pass = loaded.status === "loaded" && js8.myCall === "K0OG" &&
    js8.grid === "FN31" && js8.speed === "B" && js8.txOffsetHz === 1777 &&
    js8.clockCorrectionMs === 35 && js8.autoTiming === false &&
    loaded.settings.ui.disclosures.settings;
  document.body.textContent = `SETTINGS BROWSER ${pass ? "PASS" : "FAIL"}`;
  document.body.dataset.pass = String(pass);
})();
