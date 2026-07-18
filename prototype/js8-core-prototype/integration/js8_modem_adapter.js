// PROTOTYPE — candidate adapter only. It deliberately does not call registerModem().

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.createJs8ModemAdapter = value.createJs8ModemAdapter;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  function createJs8ModemAdapter(environment) {
    const {DecoderBase, EncoderBase, createWorker, createTxController} = environment;
    if (!DecoderBase || !EncoderBase || !createWorker || !createTxController)
      throw new Error("JS8 adapter requires base classes and injected runtimes");

    class Js8CallDecoder extends DecoderBase {
      constructor(sampleRate) {
        super(sampleRate);
        this._onEvent = null;
        this._status = "loading";
        this._epochKey = null;
        this._worker = createWorker();
        this._worker.onmessage = event => this._receive(event.data);
        this._worker.postMessage({type: "init"});
      }

      onEvent(callback) { this._onEvent = callback; return this; }

      beginEpoch(epoch) {
        if (!epoch || !Number.isInteger(Number(epoch.streamId)) ||
            !Number.isFinite(Number(epoch.anchorUtcMs)))
          throw new Error("JS8 decoder requires a locked media epoch");
        const mediaEpoch = Number(epoch.mediaEpoch || 0);
        this._epochKey = `${epoch.streamId}|${mediaEpoch}`;
        this._worker.postMessage({type:"epoch", streamId:Number(epoch.streamId),
          anchorUtcMs:Number(epoch.anchorUtcMs)});
      }

      // Current DATA page must call this overload with the untouched AUD1 wire.
      pushAud1(wire, arrivalMs = performance.now()) {
        if (!wire) throw new Error("JS8 decoder requires an AUD1 packet");
        this._worker.postMessage({type: "audio", wire, arrivalMs});
      }

      pushSamples(_samples, metadata) {
        if (!metadata || !metadata.aud1Wire ||
            !Number.isInteger(Number(metadata.streamId)) ||
            !Number.isFinite(Number(metadata.anchorUtcMs)))
          throw new Error("JS8 timing requires AUD1 sequence/firstSample metadata");
        const key = `${metadata.streamId}|${Number(metadata.mediaEpoch || 0)}`;
        if (this._epochKey !== key) this.beginEpoch(metadata);
        this.pushAud1(metadata.aud1Wire, metadata.arrivalMs);
      }

      reset() { this._epochKey = null; this._worker.postMessage({type: "reset"}); }
      close() { this._worker.postMessage({type: "close"}); this._worker.terminate(); }

      _receive(message) {
        if (message.type === "ready") this._status = "ready";
        if (message.type === "error") this._status = "error";
        if (message.state && message.state.activity) {
          const activity = message.state.activity;
          if (this._onEvent) this._onEvent({type: "activity", activity});
          if (this._emit) {
            for (const item of activity.messages || []) this._emit(`${item.text}\n`);
          }
        }
      }
    }

    class Js8CallEncoder extends EncoderBase {
      constructor(sampleRate) {
        super(sampleRate);
        this._onStatus = null;
        this._controller = createTxController({sampleRate});
        this._context = {myCall: "", toCall: "", mode: 0};
      }

      configure(context) { this._context = {...this._context, ...context}; return this; }
      onStatus(callback) { this._onStatus = callback; return this; }

      encode(text, options = {}) {
        const request = {...this._context, ...options, text,
          toneHz:Number.isFinite(Number(options.toneHz)) ? Number(options.toneHz) : this.toneHz};
        const state = this._controller.queue(request, Date.now());
        this._notify(state);
        return this._controller.prepare(Date.now());
      }

      tick(nowUtcMs = Date.now()) {
        const state = this._controller.tick(nowUtcMs);
        this._notify(state);
        return state;
      }

      abort() { const state = this._controller.abort("operator", Date.now()); this._notify(state); }
      disconnect() { const state = this._controller.disconnect(Date.now()); this._notify(state); }
      _notify(state) { if (this._onStatus) this._onStatus(state); }
    }

    return {
      id: "js8call",
      definition: {
        label: "JS8Call",
        Decoder: Js8CallDecoder,
        Encoder: Js8CallEncoder,
        minTone: 500,
        maxTone: 2700,
        requiresAud1Metadata: true,
        requiresAsyncTxSink: true,
      },
      Decoder: Js8CallDecoder,
      Encoder: Js8CallEncoder,
    };
  }

  return {createJs8ModemAdapter};
});
