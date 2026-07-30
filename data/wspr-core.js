// WSPR Type 1 encoder and on-the-fly 4-FSK generator. See
// docs/wspr-majak-implementace.md. No DOM dependency: the same file runs in the
// browser and under Node, which is what lets tools/wspr-*-smoke.js exercise it
// against wsprcode and wsprd without a radio.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.WsprCore = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  const SYMBOL_COUNT = 162;
  const SAMPLE_RATE = 48000;
  const SAMPLES_PER_SYMBOL = 32768;          // 48000 * 8192 / 12000, exact
  const SIGNAL_SAMPLES = SYMBOL_COUNT * SAMPLES_PER_SYMBOL;   // 5 308 416
  const TONE_SPACING_HZ = 12000 / 8192;      // 1.46484375
  const DURATION_S = SIGNAL_SAMPLES / SAMPLE_RATE;            // 110.592

  // Only these are legal in the 7-bit power field; the encoder refuses anything
  // else rather than rounding, so what goes on the air is what wsprnet stores.
  const POWER_LEVELS = [0, 3, 7, 10, 13, 17, 20, 23, 27, 30,
                        33, 37, 40, 43, 47, 50, 53, 57, 60];

  // 162 sync bits packed MSB-first into 21 bytes (the last 6 bits are padding).
  // Stored packed rather than as 162 array entries purely for SPIFFS budget.
  const SYNC_PACKED = new Uint8Array([
    0xc0, 0x8e, 0x25, 0xe0, 0x25, 0x02, 0xcd, 0x1a, 0x1a, 0xa9, 0x2c,
    0x6a, 0x20, 0x93, 0xb3, 0x47, 0x05, 0x30, 0x1a, 0xc6, 0x00]);

  function syncBit(index) {
    return (SYNC_PACKED[index >> 3] >> (7 - (index & 7))) & 1;
  }

  const ALPHABET_37 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ ";
  const ALPHABET_36 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  // Space is LAST here and last in ALPHABET_37 too. Getting this backwards still
  // produces the right top 16 bits, so it survives a casual eyeball and only
  // shows up as symbols that no decoder anywhere will accept.
  const ALPHABET_27 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ ";

  class WsprError extends Error {}

  // ---- validation ----------------------------------------------------------

  // Type 1 only. A compound callsign needs Type 2 and a different transmission
  // sequence, so it is refused loudly instead of being silently mangled.
  function normalizeCallsign(input) {
    const call = String(input == null ? "" : input).trim().toUpperCase();
    if (!call) throw new WsprError("callsign is empty");
    if (call.indexOf("/") >= 0)
      throw new WsprError(
        `compound callsign ${call} needs a Type 2 message, which this version does not encode`);
    if (!/^[A-Z0-9]{3,6}$/.test(call))
      throw new WsprError(`callsign ${call} is not a plain 3-6 character call`);
    // The packed field is six characters with the digit fixed at position 3.
    const padded = /[0-9]/.test(call[2]) ? call : " " + call;
    if (padded.length > 6 || !/[0-9]/.test(padded[2]))
      throw new WsprError(`callsign ${call} does not fit the Type 1 six-character field`);
    return {call, field: padded.padEnd(6, " ")};
  }

  // A six-character locator is accepted and kept, but only the first four are
  // transmittable in Type 1; callers show both so the truncation is visible.
  function normalizeLocator(input) {
    const locator = String(input == null ? "" : input).trim().toUpperCase();
    if (!/^[A-R]{2}[0-9]{2}([A-X]{2})?$/.test(locator))
      throw new WsprError(`locator ${locator || "(empty)"} is not a valid Maidenhead square`);
    return {locator, transmitted: locator.slice(0, 4)};
  }

  function normalizePower(input) {
    const dbm = Number(input);
    if (!POWER_LEVELS.includes(dbm))
      throw new WsprError(`${input} dBm is not one of the legal WSPR power levels`);
    return dbm;
  }

  function validate(message) {
    const {call, field} = normalizeCallsign(message && message.callsign);
    const {locator, transmitted} = normalizeLocator(message && message.locator);
    const powerDbm = normalizePower(message && message.powerDbm);
    return {callsign: call, callField: field, locator, transmittedLocator: transmitted,
            powerDbm, text: `${call} ${transmitted} ${powerDbm}`};
  }

  // ---- Maidenhead ----------------------------------------------------------

  function latLonToGrid(lat, lon, characters = 6) {
    if (!Number.isFinite(lat) || !Number.isFinite(lon))
      throw new WsprError("coordinates must be finite numbers");
    // Clamp rather than wrap: a pole or the date line must not roll into a
    // field that does not exist.
    const la = Math.min(Math.max(lat, -90), 90) + 90;
    const lo = Math.min(Math.max(lon, -180), 180) + 180;
    const fieldLon = Math.min(17, Math.floor(lo / 20));
    const fieldLat = Math.min(17, Math.floor(la / 10));
    let grid = String.fromCharCode(65 + fieldLon, 65 + fieldLat)
             + Math.min(9, Math.floor((lo - fieldLon * 20) / 2))
             + Math.min(9, Math.floor(la - fieldLat * 10));
    if (characters >= 6) {
      const restLon = lo - fieldLon * 20 - Math.floor((lo - fieldLon * 20) / 2) * 2;
      const restLat = la - fieldLat * 10 - Math.floor(la - fieldLat * 10);
      grid += String.fromCharCode(65 + Math.min(23, Math.floor(restLon / (2 / 24))),
                                  65 + Math.min(23, Math.floor(restLat / (1 / 24))));
    }
    return grid;
  }

  // Accepts either a ready-made locator or a coordinate pair, so one input field
  // can serve both without the operator having to say which is which.
  function parseLocatorInput(input) {
    const text = String(input == null ? "" : input).trim();
    if (/^[A-Za-z]{2}[0-9]{2}([A-Za-z]{2})?$/.test(text))
      return normalizeLocator(text).locator;
    const pair = text.split(/[,;\s]+/).filter(Boolean);
    if (pair.length !== 2) throw new WsprError(`cannot read "${text}" as a locator or coordinate pair`);
    const lat = Number(pair[0]), lon = Number(pair[1]);
    if (!Number.isFinite(lat) || !Number.isFinite(lon) ||
        Math.abs(lat) > 90 || Math.abs(lon) > 180)
      throw new WsprError(`cannot read "${text}" as a locator or coordinate pair`);
    return latLonToGrid(lat, lon, 6);
  }

  // ---- packing -------------------------------------------------------------

  function packCallsign(field) {
    let n = ALPHABET_37.indexOf(field[0]);
    n = n * 36 + ALPHABET_36.indexOf(field[1]);
    n = n * 10 + ALPHABET_36.indexOf(field[2]);
    n = n * 27 + ALPHABET_27.indexOf(field[3]);
    n = n * 27 + ALPHABET_27.indexOf(field[4]);
    n = n * 27 + ALPHABET_27.indexOf(field[5]);
    return n;
  }

  function packLocatorAndPower(locator, powerDbm) {
    const lonField = locator.charCodeAt(0) - 65, latField = locator.charCodeAt(1) - 65;
    const lonSquare = locator.charCodeAt(2) - 48, latSquare = locator.charCodeAt(3) - 48;
    const grid = 180 * (179 - 10 * lonField - lonSquare) + 10 * latField + latSquare;
    return grid * 128 + powerDbm + 64;
  }

  // 28-bit callsign followed by the 22-bit locator/power word, MSB-first, then a
  // zero tail — 11 bytes, matching `wsprcode -d`.
  function packMessage(message) {
    const checked = validate(message);
    const call = packCallsign(checked.callField);
    const grid = packLocatorAndPower(checked.transmittedLocator, checked.powerDbm);
    const bytes = new Uint8Array(11);
    bytes[0] = (call >>> 20) & 0xff;
    bytes[1] = (call >>> 12) & 0xff;
    bytes[2] = (call >>> 4) & 0xff;
    bytes[3] = ((call << 4) & 0xf0) | ((grid >>> 18) & 0x0f);
    bytes[4] = (grid >>> 10) & 0xff;
    bytes[5] = (grid >>> 2) & 0xff;
    bytes[6] = (grid << 6) & 0xc0;
    return {bytes, checked};
  }

  // ---- FEC -----------------------------------------------------------------

  const POLY_0 = 0xf2d05351 >>> 0;
  const POLY_1 = 0xe4613c47 >>> 0;

  function parity32(value) {
    value >>>= 0;
    value ^= value >>> 16;
    value ^= value >>> 8;
    value ^= value >>> 4;
    value &= 0x0f;
    return (0x6996 >>> value) & 1;
  }

  // r=1/2, K=32. 50 message bits plus a 31-bit zero tail give exactly 162 bits.
  function convolutionalEncode(bytes) {
    const out = new Uint8Array(162);
    let register = 0, at = 0;
    for (let i = 0; i < 81; i++) {
      const bit = (bytes[i >> 3] >> (7 - (i & 7))) & 1;
      register = ((register << 1) | bit) >>> 0;
      out[at++] = parity32(register & POLY_0);
      out[at++] = parity32(register & POLY_1);
    }
    return out;
  }

  function reverse8(value) {
    let out = 0;
    for (let i = 0; i < 8; i++) out = (out << 1) | ((value >> i) & 1);
    return out;
  }

  function interleave(bits) {
    const out = new Uint8Array(162);
    let source = 0;
    for (let i = 0; i < 256 && source < 162; i++) {
      const destination = reverse8(i);
      if (destination < 162) out[destination] = bits[source++];
    }
    return out;
  }

  function encode(message) {
    const {bytes, checked} = packMessage(message);
    const data = interleave(convolutionalEncode(bytes));
    const symbols = new Uint8Array(162);
    for (let i = 0; i < 162; i++) symbols[i] = syncBit(i) + 2 * data[i];
    return {symbols, packed: bytes, dataBits: data, message: checked};
  }

  // ---- audio ---------------------------------------------------------------

  const TWO_PI = 2 * Math.PI;
  const RAMP_SAMPLES = Math.round(SAMPLE_RATE * 0.005);   // 5 ms, kills key clicks

  // Generates the transmission 20 ms at a time and forgets each block, so the
  // whole 110.592 s costs ~1 kB of state instead of the 10.8 MB a materialised
  // packet array would need.
  class WsprStream {
    constructor(symbols, options = {}) {
      if (!symbols || symbols.length !== SYMBOL_COUNT)
        throw new WsprError("a WSPR frame is exactly 162 symbols");
      this.symbols = symbols;
      this.baseHz = Number(options.baseHz ?? 1500);
      this.amplitude = Number(options.amplitude ?? 0.25);
      if (!(this.amplitude > 0 && this.amplitude <= 1))
        throw new WsprError("amplitude must be in (0, 1]");
      this.streamId = options.streamId >>> 0;
      this.txId = options.txId >>> 0;
      this.samplesPerPacket = options.samplesPerPacket ?? 960;   // 20 ms
      this.reset();
    }

    reset() { this.phase = 0; this.sample = 0; this.sequence = 0; }

    get totalSamples() { return SIGNAL_SAMPLES; }
    get totalPackets() { return Math.ceil(SIGNAL_SAMPLES / this.samplesPerPacket); }
    get done() { return this.sample >= SIGNAL_SAMPLES; }

    // One block of PCM16. Phase carries across both symbol and packet
    // boundaries — that continuity is the whole point of 4-FSK.
    nextSamples(count) {
      const wanted = Math.min(count, SIGNAL_SAMPLES - this.sample);
      const out = new Int16Array(wanted);
      const scale = this.amplitude * 32767;
      for (let i = 0; i < wanted; i++) {
        const at = this.sample + i;
        const symbol = this.symbols[(at / SAMPLES_PER_SYMBOL) | 0];
        const step = TWO_PI * (this.baseHz + symbol * TONE_SPACING_HZ) / SAMPLE_RATE;
        let gain = scale;
        if (at < RAMP_SAMPLES)
          gain *= 0.5 - 0.5 * Math.cos(Math.PI * at / RAMP_SAMPLES);
        else if (at >= SIGNAL_SAMPLES - RAMP_SAMPLES)
          gain *= 0.5 - 0.5 * Math.cos(Math.PI * (SIGNAL_SAMPLES - 1 - at) / RAMP_SAMPLES);
        out[i] = Math.max(-32768, Math.min(32767, Math.round(gain * Math.sin(this.phase))));
        this.phase += step;
        if (this.phase >= TWO_PI) this.phase -= TWO_PI;
      }
      this.sample += wanted;
      return out;
    }

    // AUD1 v1 kind 3 wire frame, byte-for-byte what aud1AcceptTxPacket accepts:
    // 40-byte big-endian header, little-endian PCM16 payload, length % 12 == 0.
    nextPacket() {
      if (this.done) return null;
      const first = this.sample === 0;
      const firstSample = this.sample;
      const sequence = this.sequence++;
      const pcm = this.nextSamples(this.samplesPerPacket);
      const last = this.sample >= SIGNAL_SAMPLES;
      const wire = new Uint8Array(40 + pcm.length * 2);
      const view = new DataView(wire.buffer);
      wire.set([0x41, 0x55, 0x44, 0x31], 0);          // "AUD1"
      wire[4] = 1; wire[5] = 3;                        // version 1, TX_PCM16
      view.setUint16(6, (first ? 1 : 0) | (last ? 2 : 0), false);
      view.setUint16(8, 40, false);                    // header length
      view.setUint32(12, this.streamId, false);
      view.setUint32(16, sequence, false);
      view.setUint32(20, SAMPLE_RATE, false);
      view.setBigUint64(24, BigInt(firstSample), false);
      view.setUint32(32, this.txId, false);
      view.setUint32(36, pcm.length * 2, false);
      for (let i = 0; i < pcm.length; i++) view.setInt16(40 + i * 2, pcm[i], true);
      return {wire, sequence, firstSample, samples: pcm.length, first, last};
    }
  }

  // Standard WSPR dial frequencies (USB). The signal sits in the 1400-1600 Hz
  // audio window above these, which is why the beacon randomises its tone there.
  const PRESETS = [
    {band: "160m", hz: 1836600},   {band: "80m", hz: 3568600},
    {band: "60m", hz: 5287200},    {band: "40m", hz: 7038600},
    {band: "30m", hz: 10138700},   {band: "20m", hz: 14095600},
    {band: "17m", hz: 18104600},   {band: "15m", hz: 21094600},
    {band: "12m", hz: 24924600},   {band: "10m", hz: 28124600},
    {band: "6m", hz: 50293000},    {band: "2m", hz: 144489000},
  ];

  // Full output at 100 % of the CI-V power scale, per radio. The LAN transport is
  // configured as "IC-705-LAN" but IC-7610 and IC-9700 speak the same protocol,
  // so trusting the profile name instead of /state.radioName is a factor-of-ten
  // error. An unknown model deliberately has no entry: the UI must refuse to
  // start rather than guess.
  const RADIO_FULL_POWER_W = {
    "IC-705": 10, "IC-7610": 100, "IC-9700": 100, "IC-7300": 100,
    "IC-7100": 100, "IC-7851": 200, "IC-9100": 100,
  };

  function fullPowerWatts(radioName) {
    const key = String(radioName || "").trim().toUpperCase().replace(/\s+/g, "");
    for (const [model, watts] of Object.entries(RADIO_FULL_POWER_W))
      if (key.startsWith(model.replace("-", "")) || key.startsWith(model)) return watts;
    return null;
  }

  const dbmToWatts = dbm => Math.pow(10, (Number(dbm) - 30) / 10);
  const wattsToDbm = watts => 10 * Math.log10(Math.max(1e-6, Number(watts))) + 30;

  // dBm the operator picked -> percent of the radio's scale -> the 0..255 CI-V
  // level -> the two BCD bytes encodeCivLevel produces (255 -> 02 55).
  // 5 W is 36.99 dBm, not 37, so an exact comparison would forbid the level the
  // whole WSPR world reports for a five-watt radio and silently cap an IC-705 on
  // battery at 2 W. One percent of headroom covers the rounding without letting
  // anything genuinely over-powered through.
  const POWER_TOLERANCE = 1.01;

  function powerCommand(dbm, fullWatts) {
    if (!(fullWatts > 0)) throw new WsprError("full power is unknown for this radio");
    const watts = dbmToWatts(dbm);
    if (watts > fullWatts * POWER_TOLERANCE)
      throw new WsprError(`${dbm} dBm is ${watts.toFixed(2)} W, above this radio's ${fullWatts} W`);
    const percent = 100 * watts / fullWatts;
    const level = Math.max(0, Math.min(255, Math.round(percent * 255 / 100)));
    const hex = value => value.toString(16).toUpperCase().padStart(2, "0");
    return {watts, percent, level,
            data: "140A" + hex(Math.floor(level / 100)) +
                  hex((((level / 10) | 0) % 10) * 16 + (level % 10))};
  }

  // Highest legal WSPR power level this radio can actually produce.
  function maxPowerDbm(fullWatts) {
    const ceiling = fullWatts * POWER_TOLERANCE;
    return POWER_LEVELS.filter(dbm => dbmToWatts(dbm) <= ceiling).pop() ?? POWER_LEVELS[0];
  }

  // WSPR starts one second after an even UTC minute.
  function nextSlotUtcMs(nowUtcMs) {
    const period = 120000;
    const slot = Math.ceil((nowUtcMs - 1000) / period) * period + 1000;
    return slot <= nowUtcMs ? slot + period : slot;
  }

  // ---- schedule prediction --------------------------------------------------
  //
  // A schedule here is anything carrying {slots, periodFrames, randomizeFrame} --
  // the page's settings object does. The beacon keys one frame out of every
  // periodFrames, and which one is either fixed or moved every half hour. Three
  // different views ask that question (the countdown, the panel preview, the
  // activity block's future), so the arithmetic lives here once: a preview that
  // re-implements it is a preview that can lie.

  const FRAME_MS = 120000, DAY_MS = 86400000, SLOTS_PER_DAY = 48;

  const slotIndexAt = utcMs => {
    const at = new Date(utcMs);
    return at.getUTCHours() * 2 + (at.getUTCMinutes() >= 30 ? 1 : 0);
  };
  const slotLabel = index =>
    `${String(Math.floor(index / 2)).padStart(2, "0")}:${index % 2 ? "30" : "00"}`;

  // Deterministic but different every half hour, so the beacon does not always
  // land on the same two-minute frame while staying predictable enough to show.
  //
  // The final >>> 0 is not decoration. `a ^ b` in JS produces a SIGNED 32-bit int,
  // so whenever bit 31 came out set the offset was negative -- and the caller
  // compares it against `frameIndex % period`, which never is. That silently
  // killed every frame of the affected half hours: with randomise on, 40 % of
  // scheduled slots never transmitted at all.
  function frameOffset(slotIndex, dayNumber, schedule) {
    const period = Math.max(1, Number(schedule.periodFrames) || 1);
    if (!schedule.randomizeFrame) return 0;
    let hash = (dayNumber * SLOTS_PER_DAY + slotIndex) >>> 0;
    hash = (hash ^ (hash >>> 13)) * 0x5bd1e995 >>> 0;
    return ((hash ^ (hash >>> 15)) >>> 0) % period;
  }

  // Does the frame starting at this instant key, and on what band? null is
  // silence -- either the half hour has no band or this is not its frame.
  function frameTransmission(frameUtcMs, schedule) {
    const index = slotIndexAt(frameUtcMs);
    const slot = (schedule.slots || {})[index];
    if (!slot) return null;
    const period = Math.max(1, Number(schedule.periodFrames) || 1);
    const at = new Date(frameUtcMs);
    const frameIndex = Math.floor((at.getUTCHours() * 60 + at.getUTCMinutes()) / 2);
    if (frameIndex % period !== frameOffset(index, Math.floor(frameUtcMs / DAY_MS), schedule))
      return null;
    return {slotUtcMs: frameUtcMs, slot, index};
  }

  // The next frame this schedule keys in, or null when it stays silent for the
  // whole 24-hour cycle.
  function nextTransmission(fromUtcMs, schedule) {
    for (let step = 0; step < 720; step++) {
      const frame = frameTransmission(nextSlotUtcMs(fromUtcMs + step * FRAME_MS - 1000), schedule);
      if (frame) return frame;
    }
    return null;
  }

  // Every frame that keys inside a window. Frames, not half hours: the minute is
  // the one thing the timetable's band-per-half-hour geometry cannot show, and
  // it is exactly what the operator cannot currently predict.
  function plannedFrames(fromUtcMs, hours, schedule) {
    const until = fromUtcMs + hours * 3600000, frames = [];
    for (let step = 0; step < Math.ceil(hours * 30) + 2; step++) {
      const frameUtcMs = nextSlotUtcMs(fromUtcMs + step * FRAME_MS - 1000);
      if (frameUtcMs >= until) break;
      const frame = frameTransmission(frameUtcMs, schedule);
      if (frame) frames.push(frame);
    }
    return frames;
  }

  // Which half-hour slots one edit touches. The ranges run forward from the
  // clicked slot and wrap past midnight, because the schedule is a 24-hour cycle
  // and a 21:00-03:00 night block is the case this exists for; "day" is the
  // explicit non-wrapping choice.
  const SPANS = [
    {id: "slot", label: "slot"}, {id: "hour", label: "hour"},
    {id: "3h", label: "+3h", slots: 6}, {id: "6h", label: "+6h", slots: 12},
    {id: "day", label: "to 24:00"}, {id: "all", label: "all day"},
  ];

  function spanSlots(index, spanId) {
    const start = ((Number(index) % SLOTS_PER_DAY) + SLOTS_PER_DAY) % SLOTS_PER_DAY;
    const span = SPANS.find(entry => entry.id === spanId) || SPANS[0];
    if (span.id === "all") return Array.from({length: SLOTS_PER_DAY}, (_, slot) => slot);
    if (span.id === "day")
      return Array.from({length: SLOTS_PER_DAY - start}, (_, offset) => start + offset);
    if (span.id === "hour") { const hour = Math.floor(start / 2) * 2; return [hour, hour + 1]; }
    const count = span.slots || 1;
    return Array.from({length: count}, (_, offset) => (start + offset) % SLOTS_PER_DAY);
  }

  return {
    SYMBOL_COUNT, SAMPLE_RATE, SAMPLES_PER_SYMBOL, SIGNAL_SAMPLES,
    TONE_SPACING_HZ, DURATION_S, POWER_LEVELS, WsprError,
    normalizeCallsign, normalizeLocator, normalizePower, validate,
    latLonToGrid, parseLocatorInput,
    packCallsign, packLocatorAndPower, packMessage,
    convolutionalEncode, interleave, syncBit, encode,
    WsprStream, nextSlotUtcMs,
    PRESETS, RADIO_FULL_POWER_W, fullPowerWatts, dbmToWatts, wattsToDbm,
    powerCommand, maxPowerDbm,
    FRAME_MS, SLOTS_PER_DAY, SPANS, slotIndexAt, slotLabel, frameOffset,
    frameTransmission, nextTransmission, plannedFrames, spanSlots,
  };
});
