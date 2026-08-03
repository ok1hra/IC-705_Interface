// Runtime ALC limiter: the half of the auto-gain design that runs during real
// transmissions. See docs/tx-auto-gain-implementace.md, decisions 1 and 10.
//
// It can only ever reduce. Raising the level is the calibration carrier's job,
// where the operator asked for it and nothing is at stake but the carrier
// itself; doing it inside a real frame would turn every transmission into an
// experiment and split one signal into two levels.
//
// It never aborts a frame either. A WSPR frame 6 dB down still decodes; a frame
// cut in half is a burnt slot and a truncated signal on the air. The same goes
// for JS8, which cannot even change level mid-frame -- its modulator bakes the
// gain into the whole frame, so a reduction there applies to the NEXT one. The
// asymmetry lives in the caller; this module just answers "what should the gain
// be now".
//
// Persisting a reduction takes two witnesses. With a zero-margin calibration the
// first real transmission after a battery sag or an antenna change may well trip
// once, and writing that straight into the station's table would let a single
// anomaly walk the stored level down. Two separate transmissions saying the same
// thing is evidence; one is weather.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.TxAlcGuard = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  "use strict";

  const DEFAULTS = {
    stepDb: 1,            // per confirmed deflection
    floorDb: -6,          // below the calibrated level, the limiter stops trying
    // Every change this module makes is a REDUCTION, and after one the ALC
    // needle falls on the radio's schedule, not ours. A reading taken too soon
    // still describes the level we just left, and acting on it would reduce
    // again for the same deflection -- six dB down in three seconds on a single
    // event. 500 ms of played audio is the price of not doing that; during
    // normal TX the ALC answers once a second anyway.
    settleBytes: 4000,    // 500 ms of the new level before a reading counts
    rampBytes: 960,       // 120 ms ramp, same as the calibration carrier
    witnessesToPersist: 2,
  };

  const fromDb = db => Math.pow(10, db / 20);

  class TxAlcGuard {
    constructor(options = {}) {
      this.config = {...DEFAULTS, ...options};
      // key -> transmissions that reported ALC action since the last write.
      this.witnesses = new Map();
      this.tx = null;
    }

    // One transmission. `gain` is the level resolved for this key, `key` the
    // model|band|percent the table is filed under.
    beginTx({key, gain}) {
      this.tx = {
        key: String(key || ""),
        startGain: Number(gain) || 0,
        gain: Number(gain) || 0,
        floor: (Number(gain) || 0) * fromDb(this.config.floorDb),
        consumed: 0,
        lastSeq: 0,
        liveFromBytes: 0,
        reductions: 0,
        atFloor: false,
      };
      return this.gain;
    }

    get gain() { return this.tx ? this.tx.gain : 0; }
    get active() { return Boolean(this.tx); }

    // One tx-level frame. Returns the gain the caller should be producing.
    noteLevel({consumed, alc, alcSeq} = {}) {
      if (!this.tx) return 0;
      const played = Number(consumed);
      if (Number.isFinite(played) && played > this.tx.consumed) this.tx.consumed = played;

      const seq = Number(alcSeq) || 0;
      if (seq <= this.tx.lastSeq) return this.tx.gain;   // repeat, not a new reading
      this.tx.lastSeq = seq;
      if ((Number(alc) || 0) <= 0) return this.tx.gain;
      if (this.tx.consumed < this.tx.liveFromBytes + this.config.settleBytes)
        return this.tx.gain;                              // still the old level talking

      this.tx.reductions += 1;
      const next = this.tx.gain * fromDb(-this.config.stepDb);
      if (next < this.tx.floor) {
        // Six dB down and still limiting means the stored entry no longer
        // describes this radio. Hold the floor -- quietly dropping further would
        // hide a setup that needs recalibrating.
        this.tx.gain = this.tx.floor;
        this.tx.atFloor = true;
      } else {
        this.tx.gain = next;
      }
      this.tx.liveFromBytes = this.tx.consumed + this.config.rampBytes;
      return this.tx.gain;
    }

    // Ends the transmission and reports what, if anything, the table should
    // learn from it. `persistGain` is null unless this transmission was the
    // second witness for its key.
    endTx() {
      if (!this.tx) return null;
      const tx = this.tx;
      this.tx = null;
      if (!tx.reductions) {
        // A clean transmission does not merely fail to accuse -- it clears the
        // standing accusation. Otherwise two trips a month apart, with fifty
        // good transmissions between them, would count as corroboration.
        this.witnesses.delete(tx.key);
        return {key: tx.key, reduced: false, witnesses: 0, persistGain: null,
                needsRecalibration: false, gain: tx.gain};
      }
      const witnesses = (this.witnesses.get(tx.key) || 0) + 1;
      this.witnesses.set(tx.key, witnesses);
      const persist = witnesses >= this.config.witnessesToPersist;
      if (persist) this.witnesses.delete(tx.key);
      return {
        key: tx.key,
        reduced: true,
        witnesses,
        // Only ever the level this transmission actually finished at, and only
        // ever downwards -- the caller never has to check the direction.
        persistGain: persist ? Math.min(tx.gain, tx.startGain) : null,
        needsRecalibration: tx.atFloor,
        gain: tx.gain,
      };
    }

    // A page reload must not resurrect half-collected evidence from a previous
    // session's radio state; the caller decides when that is appropriate.
    forget(key) {
      if (key === undefined) this.witnesses.clear();
      else this.witnesses.delete(String(key));
    }
  }

  return {TxAlcGuard, DEFAULTS};
});
