// PROTOTYPE — real-browser state transition check for a TRX preset request.
(function () {
  const frame = document.querySelector("#operator-ui");
  const result = document.querySelector("#result");
  frame.addEventListener("load", () => {
    const prototype = frame.contentWindow.__js8Prototype;
    if (!prototype) {
      result.textContent = "OPERATOR BROWSER FAIL missing prototype boundary";
      return;
    }
    const initial = prototype.state.trx.frequencyHz === 7078000 &&
      prototype.state.trx.mode === "USB" && prototype.state.trx.request === null;
    prototype.requestTrxFrequency("20m");
    const pending = prototype.state.trx.frequencyHz === 7078000 &&
      prototype.state.trx.request?.frequencyHz === 14078000 &&
      prototype.state.trx.request?.status === "requesting";
    setTimeout(() => {
      const confirmed = prototype.state.trx.frequencyHz === 14078000 &&
        prototype.state.trx.mode === "USB" && prototype.state.trx.request === null;
      const pass = initial && pending && confirmed;
      result.textContent = `OPERATOR BROWSER ${pass ? "PASS" : "FAIL"}`;
      document.body.dataset.pass = String(pass);
    }, 350);
  });
})();
