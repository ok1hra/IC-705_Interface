// PROTOTYPE — this page deliberately has no UI beyond its complete result.

(async function () {
  const result = document.querySelector("#result");
  try {
    const dump = new Uint8Array(await (await fetch("/fixture.bin")).arrayBuffer());
    const packets = [];
    const view = new DataView(dump.buffer);
    for (let offset = 0; offset < dump.length;) {
      const length = view.getUint32(offset, false);
      offset += 4;
      const wire = dump.slice(offset, offset + length);
      offset += length;
      const header = new DataView(wire.buffer, wire.byteOffset, wire.byteLength);
      packets.push({wire, arrivalMs: Number(header.getBigUint64(24, false)) / 8});
    }
    const worker = new Worker("/wasm/worker_entry.js");
    const request = (message) => new Promise((resolve, reject) => {
      const receive = (event) => {
        worker.removeEventListener("error", fail);
        if (event.data.type === "error") reject(new Error(event.data.message));
        else resolve(event.data);
      };
      const fail = (event) => {
        worker.removeEventListener("message", receive);
        reject(event.error || new Error(event.message));
      };
      worker.addEventListener("message", receive, {once: true});
      worker.addEventListener("error", fail, {once: true});
      worker.postMessage(message);
    });
    await request({type: "init", runtimeJs: "/wasm/worker_runtime.js",
      portableJs: "/build-wasm/js8-prototype.js",
      portableWasm: "/build-wasm/js8-prototype.wasm",
      decoderJs: "/build-decoder-wasm/js8-decoder.js",
      decoderWasm: "/build-decoder-wasm/js8-decoder.wasm",
      protocolJs: "/protocol/protocol_runtime.js",
      jscUrl: "/build-protocol/jsc-map.bin", anchorUtcMs: 0});
    const started = performance.now();
    await request({type: "audioBatch", packets});
    const finished = await request({type: "finish"});
    const elapsedMs = performance.now() - started;
    const state = finished.state;
    const summary = {packets: packets.length, frames: state.frames.length,
      windows: state.windows, windowsByMode: state.windowsByMode,
      calls: state.activity.calls.map(item => item.call),
      messages: state.activity.messages.map(item => item.text),
      decoderMemoryBytes: state.decoderMemoryBytes,
      elapsedMs: Number(elapsedMs.toFixed(1))};
    const pass = state.frames.length === 1 &&
      summary.messages.includes("KN4CRD: TEST") && summary.calls.includes("KN4CRD") &&
      Object.keys(state.windowsByMode).length === 5;
    result.textContent = `${pass ? "PASS" : "FAIL"} ${JSON.stringify(summary)}`;
    await request({type: "close"});
    worker.terminate();
  } catch (error) {
    result.textContent = `ERROR ${String(error.stack || error)}`;
  }
})();
