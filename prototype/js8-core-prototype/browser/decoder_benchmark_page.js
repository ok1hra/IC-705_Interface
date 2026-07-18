// PROTOTYPE — all-speed phone benchmark for the complete RX Worker pipeline.

(async function () {
  const output = document.querySelector("#result");
  let worker;

  function show(text) {
    output.textContent = text;
  }

  function readPackets(dump) {
    const packets = [];
    const view = new DataView(dump.buffer, dump.byteOffset, dump.byteLength);
    let offset = 0;
    while (offset < dump.length) {
      if (offset + 4 > dump.length) throw new Error("truncated packet length");
      const length = view.getUint32(offset, false);
      offset += 4;
      if (offset + length > dump.length) throw new Error("truncated AUD1 packet");
      const wire = dump.slice(offset, offset + length);
      offset += length;
      const header = new DataView(wire.buffer, wire.byteOffset, wire.byteLength);
      packets.push({wire, arrivalMs: Number(header.getBigUint64(24, false)) / 8});
    }
    return packets;
  }

  function request(message, transfers = []) {
    return new Promise((resolve, reject) => {
      const cleanup = () => {
        worker.removeEventListener("message", receive);
        worker.removeEventListener("error", fail);
      };
      const receive = event => {
        cleanup();
        if (event.data.type === "error") reject(new Error(event.data.message));
        else resolve(event.data);
      };
      const fail = event => {
        cleanup();
        reject(event.error || new Error(event.message));
      };
      worker.addEventListener("message", receive);
      worker.addEventListener("error", fail);
      worker.postMessage(message, transfers);
    });
  }

  try {
    show("LOADING golden AUD1 fixtures");
    const manifestResponse = await fetch("/build-golden/manifest.json");
    if (!manifestResponse.ok)
      throw new Error(`manifest fetch failed: ${manifestResponse.status}`);
    const manifest = await manifestResponse.json();
    const fixtures = await Promise.all(manifest.vectors.map(async vector => {
      const name = vector.wav.replace(/\.wav$/, ".aud1.bin");
      const response = await fetch(`/build-phone-benchmark/${name}`);
      if (!response.ok) throw new Error(`${name} fetch failed: ${response.status}`);
      return {...vector,
        packets: readPackets(new Uint8Array(await response.arrayBuffer()))};
    }));

    const overallStarted = performance.now();
    worker = new Worker("/wasm/worker_entry.js");
    const initStarted = performance.now();
    const ready = await request({type: "init", runtimeJs: "/wasm/worker_runtime.js",
      portableJs: "/build-wasm/js8-prototype.js",
      portableWasm: "/build-wasm/js8-prototype.wasm",
      decoderJs: "/build-decoder-wasm/js8-decoder.js",
      decoderWasm: "/build-decoder-wasm/js8-decoder.wasm",
      protocolJs: "/protocol/protocol_runtime.js",
      jscUrl: "/build-protocol/jsc-map.bin", anchorUtcMs: 0});
    if (ready.type !== "ready") throw new Error("Worker did not become ready");
    const initMs = performance.now() - initStarted;

    const vectors = [];
    let processingMs = 0;
    let maxAudioMemoryBytes = ready.state.audioMemoryBytes;
    let maxDecoderMemoryBytes = ready.state.decoderMemoryBytes;
    for (let index = 0; index < fixtures.length; index += 1) {
      const fixture = fixtures[index];
      show(`RUNNING ${index + 1}/${fixtures.length}: ${fixture.letter} mode ${fixture.submode}`);
      await request({type: "reset"});
      const started = performance.now();
      const transfers = fixture.packets.map(packet => packet.wire.buffer);
      await request({type: "audioBatch", packets: fixture.packets}, transfers);
      const finished = await request({type: "finish"});
      const elapsedMs = performance.now() - started;
      processingMs += elapsedMs;
      const state = finished.state;
      maxAudioMemoryBytes = Math.max(maxAudioMemoryBytes, state.audioMemoryBytes);
      maxDecoderMemoryBytes = Math.max(maxDecoderMemoryBytes, state.decoderMemoryBytes);
      const framePass = state.frames.some(frame =>
        frame.raw === manifest.raw && frame.submode === fixture.submode);
      const messagePass = state.activity.messages.some(message =>
        message.text === manifest.text && message.submode === fixture.submode);
      const callPass = state.activity.calls.some(item => item.call === "KN4CRD");
      const samplePass = state.totalSamples12k === fixture.samples;
      const schedulerPass = state.windowsByMode[fixture.submode] > 0;
      const pass = framePass && messagePass && callPass && samplePass && schedulerPass;
      vectors.push({letter: fixture.letter, submode: fixture.submode,
        audioSeconds: fixture.samples / 12000, packets: fixture.packets.length,
        samples12k: state.totalSamples12k, windows: state.windows,
        windowsByMode: state.windowsByMode, frames: state.frames.length,
        elapsedMs: Number(elapsedMs.toFixed(1)), framePass, messagePass,
        callPass, samplePass, schedulerPass, pass});
    }

    const closed = await request({type: "close"});
    if (closed.type !== "closed") throw new Error("Worker did not close");
    worker.terminate();
    worker = null;
    const state = {vectors, initMs: Number(initMs.toFixed(1)),
      processingMs: Number(processingMs.toFixed(1)),
      totalMs: Number((performance.now() - overallStarted).toFixed(1)),
      maxAudioMemoryBytes, maxDecoderMemoryBytes,
      pass: vectors.every(vector => vector.pass), userAgent: navigator.userAgent};
    show(`${state.pass ? "PASS" : "FAIL"} ${JSON.stringify(state)}`);
  } catch (error) {
    if (worker) worker.terminate();
    show(`ERROR ${String(error.stack || error)}`);
  }
})();
