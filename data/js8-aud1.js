// PROTOTYPE — future versioned AUD1 RX source + asynchronous TX sink.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8Aud1Transport = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  const ULAW = new Float32Array(256);
  for (let i = 0; i < 256; i += 1) {
    const u = ~i & 0xff;
    let sample = (((u & 0x0f) << 3) + 0x84) << ((u >> 4) & 7);
    sample -= 0x84;
    ULAW[i] = ((u & 0x80) ? -sample : sample) / 32768;
  }

  function parseAud1(wire) {
    const bytes = wire instanceof Uint8Array ? wire : new Uint8Array(wire);
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    if (bytes.length < 40 || String.fromCharCode(...bytes.subarray(0, 4)) !== "AUD1" ||
        bytes[4] !== 1 || view.getUint16(8, false) !== 40 ||
        view.getUint16(10, false) !== 0 || view.getUint32(36, false) !== bytes.length - 40)
      throw new Error("invalid AUD1 envelope");
    const kind = bytes[5];
    const flags = view.getUint16(6, false);
    const sampleRate = view.getUint32(20, false);
    if (![1, 2, 3].includes(kind) || (flags & ~0x0f) || sampleRate === 0 ||
        ((kind === 2 || kind === 3) && (bytes.length - 40) % 2))
      throw new Error("unsupported AUD1 envelope");
    return {kind, flags, streamId:view.getUint32(12, false),
      sequence:view.getUint32(16, false), sampleRate,
      firstSample:view.getBigUint64(24, false), txId:view.getUint32(32, false),
      payload:bytes.subarray(40), wire:bytes};
  }

  class Aud1WebSocketSession {
    // `now` is monotonic (transport delay); `wallNow` is the UTC source the
    // firmware compares against. It must be the same clock the TX controller
    // used for `slotUtcMs`, because the firmware validates
    // `slotUtcMs - clientUtcMs` against a 100..35000 ms window.
    constructor({url, WebSocketImpl, reconnectMs = 1000, readyTimeoutMs = 3000,
                 now = () => performance.now(), wallNow = () => Date.now()}) {
      this.url = url; this.WebSocketImpl = WebSocketImpl;
      this.reconnectMs = reconnectMs; this.readyTimeoutMs = readyTimeoutMs; this.now = now;
      this.wallNow = wallNow;
      this.socket = null; this.running = false; this.hello = null; this.ptt = false;
      this.sampleCallback = null; this.statusCallback = null; this.packetCallback = null;
      this.pendingPrepare = new Map(); this.drained = new Set(); this.activeTxId = 0;
      this.txFault = null;
    }

    onSamples(callback) { this.sampleCallback = callback; return this; }
    onPacket(callback) { this.packetCallback = callback; return this; }
    onStatus(callback) { this.statusCallback = callback; return this; }
    status(type, detail = {}) { if (this.statusCallback) this.statusCallback({type, ...detail}); }

    start() {
      if (this.running) return;
      this.running = true; this.connect();
    }

    connect() {
      const socket = new this.WebSocketImpl(this.url);
      socket.binaryType = "arraybuffer";
      socket.onopen = () => this.status("open");
      socket.onmessage = event => Promise.resolve(this.receive(event.data))
        .catch(error => {
          this.status("protocol-error", {message:error.message});
          try { socket.close(1002, "AUD1 protocol error"); } catch {}
        });
      socket.onerror = () => this.status("error");
      socket.onclose = () => {
        if (this.socket !== socket) return;
        this.socket = null; this.hello = null; this.ptt = false;
        this.txFault = null;
        for (const pending of this.pendingPrepare.values()) pending.reject(new Error("WebSocket lost"));
        this.pendingPrepare.clear(); this.status("closed", {ptt:false});
        if (this.running) {
          if (this.reconnectMs <= 0) this.connect();
          else setTimeout(() => this.connect(), this.reconnectMs);
        }
      };
      this.socket = socket;
    }

    stop() {
      this.running = false;
      if (this.activeTxId) this.abort(this.activeTxId, "session stop");
      if (this.socket) this.socket.close();
      this.socket = null;
    }

    async receive(data) {
      if (typeof data === "string") return this.receiveControl(JSON.parse(data));
      const buffer = data instanceof Blob ? await data.arrayBuffer() : data;
      const packet = parseAud1(buffer);
      if (packet.kind !== 1 && packet.kind !== 2) return;
      if (!this.hello) throw new Error("AUD1 packet received before hello");
      if (packet.streamId !== this.hello.streamId)
        throw new Error("AUD1 packet stream does not match hello");
      let samples;
      if (packet.kind === 1) {
        samples = new Float32Array(packet.payload.length);
        for (let i = 0; i < samples.length; i += 1) samples[i] = ULAW[packet.payload[i]];
      } else {
        samples = new Float32Array(packet.payload.length / 2);
        const view = new DataView(packet.payload.buffer, packet.payload.byteOffset,
                                  packet.payload.byteLength);
        for (let i = 0; i < samples.length; i += 1) samples[i] = view.getInt16(i * 2, true) / 32768;
      }
      const metadata = {streamId:packet.streamId, sequence:packet.sequence,
        firstSample:packet.firstSample, flags:packet.flags, kind:packet.kind,
        sampleRate:packet.sampleRate, sampleCount:samples.length,
        arrivalMs:this.now(), aud1Wire:packet.wire};
      if (this.packetCallback) this.packetCallback(packet, metadata);
      if (this.sampleCallback) this.sampleCallback(samples, packet.sampleRate, metadata);
    }

    receiveControl(message) {
      if (message.type === "hello") {
        if (this.hello) throw new Error("duplicate AUD1 hello");
        if (message.protocol !== "AUD1" || message.version !== 1 ||
            !Number.isInteger(message.streamId) || message.streamId <= 0)
          throw new Error("unsupported audio WebSocket protocol");
        this.hello = message; this.status("ready", {streamId:message.streamId});
      } else if (message.type === "tx-ready") {
        const pending = this.pendingPrepare.get(message.txId);
        if (pending) { clearTimeout(pending.timeout); this.pendingPrepare.delete(message.txId); pending.resolve(true); }
      } else if (message.type === "tx-drained") {
        this.drained.add(message.txId); this.ptt = false;
        this.status("tx-drained", {txId:message.txId, ptt:false});
      } else if (message.type === "tx-state") {
        this.ptt = Boolean(message.ptt); this.status("tx-state", {txId:message.txId, ptt:this.ptt});
      } else if (message.type === "tx-error") {
        const pending = this.pendingPrepare.get(message.txId);
        if (pending) { clearTimeout(pending.timeout); this.pendingPrepare.delete(message.txId); pending.reject(new Error(message.reason)); }
        this.txFault = {txId:message.txId, reason:message.reason || "remote TX error"};
        this.ptt = false; this.status("tx-error", {txId:message.txId, reason:message.reason, ptt:false});
      }
    }

    sendControl(message) {
      if (!this.socket || this.socket.readyState !== this.WebSocketImpl.OPEN)
        throw new Error("audio WebSocket is not open");
      this.socket.send(JSON.stringify(message));
    }

    prepare(txId, metadata) {
      if (!this.hello) return Promise.reject(new Error("AUD1 hello not received"));
      if (!metadata || !Number.isFinite(metadata.slotUtcMs) ||
          !Number.isInteger(metadata.prebufferSamples) || metadata.prebufferSamples <= 0 ||
          metadata.packetMs !== 20)
        return Promise.reject(new Error("TX prepare requires slot and 20 ms pacing"));
      this.activeTxId = txId; this.drained.delete(txId); this.txFault = null;
      this.sendControl({type:"tx.prepare", txId, sampleRate:48000,
        samples:metadata.samples, packets:metadata.packets, mode:metadata.mode,
        toneHz:metadata.toneHz, slotUtcMs:metadata.slotUtcMs,
        clientUtcMs:this.wallNow(),
        prebufferSamples:metadata.prebufferSamples, packetMs:metadata.packetMs});
      return new Promise((resolve, reject) => {
        const timeout = setTimeout(() => {
          this.pendingPrepare.delete(txId); this.abort(txId, "tx-ready timeout");
          reject(new Error("tx-ready timeout"));
        }, this.readyTimeoutMs);
        this.pendingPrepare.set(txId, {resolve, reject, timeout});
      });
    }

    begin(txId) { this.activeTxId = txId; this.status("tx-stream", {txId}); }
    write(wire) {
      if (this.txFault && this.txFault.txId === this.activeTxId)
        throw new Error(this.txFault.reason);
      if (!this.socket || this.socket.readyState !== this.WebSocketImpl.OPEN)
        throw new Error("audio WebSocket lost during TX");
      this.socket.send(wire);
    }
    end(txId) { this.status("tx-audio-end", {txId}); }
    isDrained(txId) { return this.drained.has(txId); }
    complete(txId) { this.activeTxId = 0; this.drained.delete(txId); this.txFault = null; this.ptt = false; }
    abort(txId, reason) {
      try { this.sendControl({type:"tx.abort", txId, reason}); } catch {}
      this.activeTxId = 0; this.drained.delete(txId); this.txFault = null; this.ptt = false;
      this.status("tx-abort", {txId, reason, ptt:false});
    }
  }

  return {Aud1WebSocketSession, parseAud1};
});
