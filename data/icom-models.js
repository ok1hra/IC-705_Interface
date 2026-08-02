// What this interface knows about each Icom transceiver, in one table.
//
// There used to be two half-tables: RADIO_FULL_POWER_W in wspr-core.js (watts
// only) and a single hand-written IC-705 procedure in data.html. Both answered
// "which radio is this?" and they could not both be right for five models, so an
// IC-7760 could not transmit WSPR at all (no watts entry -> refuse) while the
// setup help confidently explained an IC-705 menu tree to its operator.
//
// Keyed on the model NUMBER, not the reported string. The number is what the
// radio is; everything around it drifts. "IC-705", "IC705", a trailing firmware
// revision -- all parse to 705.
//
// DOM-free on purpose: wspr-core.js requires this under Node in the smoke tools,
// and trx-help.js builds the setup guides from the same rows in the browser.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.IcomModels = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {

  // net:      the word this radio uses for the network audio path. The IC-705 is
  //           wireless and says WLAN; every Ethernet model says LAN. It appears in
  //           three menu items, so it is data rather than three separate strings.
  // netMenu:  where Network Control and the credentials live. THREE variants, not
  //           two: the IC-705 keeps them under WLAN Set, the IC-7300MK2 and
  //           IC-7760 under Network > Remote Settings, and the IC-7610 and IC-9700
  //           have no Remote Settings submenu at all -- the items sit directly
  //           under Network. Verified in each radio's own manual, in docs/.
  // hasPreset: PRESET stores a whole named combination of settings, so the
  //           operator gets one-touch return to a working digital setup. Only some
  //           radios have it; on the others the guide has to say "set these by
  //           hand" rather than silently dropping three steps.
  // watts:    full output at 100 % of the CI-V power scale. This is the number
  //           that makes WSPR's dBm honest, and getting it wrong is a
  //           factor-of-ten error, so an unknown radio deliberately has no row.
  // civAddr:  the radio's default CI-V address. Note the IC-7300MK2 is B6h, NOT
  //           the original IC-7300's 94h -- same number, different address.
  const MODELS = [
    {
      number: 705, label: "IC-705", watts: 10, civAddr: "A4",
      net: "WLAN", netMenu: "MENU → SET → WLAN Set → Remote Settings",
      hasPreset: true, tested: true,
      firmwareNote: "Use firmware 1.20 or newer, which is where PRESET appears.",
    },
    {
      number: 7300, label: "IC-7300MK2", watts: 100, civAddr: "B6",
      net: "LAN", netMenu: "MENU → SET → Network → Remote Settings",
      hasPreset: true,
    },
    {
      number: 7610, label: "IC-7610", watts: 100, civAddr: "98",
      net: "LAN", netMenu: "MENU → SET → Network",
      hasPreset: false,
    },
    {
      number: 9700, label: "IC-9700", watts: 100, civAddr: "A2",
      net: "LAN", netMenu: "MENU → SET → Network",
      hasPreset: false, vhfUhfOnly: true,
    },
    {
      number: 7760, label: "IC-7760", watts: 200, civAddr: "B2",
      net: "LAN", netMenu: "MENU → SET → Network → Remote Settings",
      hasPreset: true,
    },
  ];

  // Radios that speak CI-V but have no network control of their own. They can
  // still turn up over LAN behind a wfview or RS-BA1 server, which reports the
  // radio's real model while the network settings live on the PC -- so their
  // watts have to be known even though no menu path can be offered.
  const BRIDGED_ONLY_WATTS = {7100: 100, 7851: 200, 9100: 100};

  // First run of 3-4 digits wins. Anything else -- a non-Icom radio behind a
  // bridge, a blank capabilities packet -- returns null, and every caller treats
  // null as "do not guess".
  function modelNumber(reported) {
    const match = /(\d{3,4})/.exec(String(reported == null ? "" : reported));
    return match ? Number(match[1]) : null;
  }

  // A model row, or null. There is deliberately no fallback to the IC-705: a page
  // that names a radio which is not connected is worse than one that admits it
  // does not know.
  function findModel(reported) {
    const number = modelNumber(reported);
    if (number == null) return null;
    return MODELS.find(model => model.number === number) || null;
  }

  // Full output at 100 % of the CI-V scale. Reads the model table first, then the
  // bridged-only radios, then gives up -- WSPR refuses to key on null rather than
  // assume, which is the only safe answer when 100 % could mean 10 W or 200 W.
  function fullPowerWatts(reported) {
    const model = findModel(reported);
    if (model) return model.watts;
    const number = modelNumber(reported);
    return (number != null && BRIDGED_ONLY_WATTS[number]) || null;
  }

  // For the setup-help model switcher and the WSPR manual power override, so both
  // lists are in one order and neither can gain a model the other lacks.
  function models() { return MODELS.slice(); }

  return {MODELS, BRIDGED_ONLY_WATTS, modelNumber, findModel, fullPowerWatts, models};
});
