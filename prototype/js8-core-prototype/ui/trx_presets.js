// PROTOTYPE — JS8 dial-frequency catalogue and future TRX request contract.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8TrxPresets = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  // Pinned from JS8Call-improved-master/JS8_Main/FrequencyList.cpp.
  const PRESETS = Object.freeze([
    {id: "160m", band: "160 m", frequencyHz: 1843500},
    {id: "80m", band: "80 m", frequencyHz: 3578000},
    {id: "60m", band: "60 m", frequencyHz: 5363000},
    {id: "40m", band: "40 m", frequencyHz: 7078000},
    {id: "30m", band: "30 m", frequencyHz: 10130000},
    {id: "20m", band: "20 m", frequencyHz: 14078000},
    {id: "17m", band: "17 m", frequencyHz: 18104000},
    {id: "15m", band: "15 m", frequencyHz: 21078000},
    {id: "12m", band: "12 m", frequencyHz: 24922000},
    {id: "10m", band: "10 m", frequencyHz: 28078000},
    {id: "6m", band: "6 m", frequencyHz: 50318000},
    {id: "2m", band: "2 m", frequencyHz: 144178000},
  ]);

  function formatFrequency(frequencyHz) {
    const hz = Math.max(0, Math.round(Number(frequencyHz) || 0));
    const mhz = Math.floor(hz / 1000000);
    const rest = String(hz % 1000000).padStart(6, "0");
    return `${mhz}.${rest.slice(0, 3)}.${rest.slice(3)}`;
  }

  function find(id) {
    return PRESETS.find(preset => preset.id === id) || null;
  }

  function createFrequencyRequest(id, reqId) {
    const preset = find(id);
    if (!preset) throw new Error(`Unknown JS8 frequency preset: ${id}`);
    return {type: "setFrequency", reqId: String(reqId),
      frequency: preset.frequencyHz};
  }

  return {PRESETS, formatFrequency, find, createFrequencyRequest};
});
