// PROTOTYPE — browser/phone throughput harness for the actual audio WASM C ABI.

(async function () {
  const result = document.querySelector("#result");
  try {
    const module = await createJs8Prototype({locateFile: name => `/build-wasm/${name}`});
    const seconds = Math.max(1, Math.min(3600,
      Number(new URLSearchParams(location.search).get("seconds")) || 600));
    const packetSamples = 160;
    const payloadPtr = module._malloc(packetSamples);
    const windowPtr = module._malloc(32);
    const payload = new Uint8Array(packetSamples);
    for (let i = 0; i < payload.length; i += 1)
      payload[i] = 128 + ((i * 29 + 17) & 127);
    module.HEAPU8.set(payload, payloadPtr);

    function execute(audioSeconds) {
      const frontend = module._js8_audio_create(0n);
      const packets = audioSeconds * 50;
      let produced = 0;
      let windows = 0;
      const started = performance.now();
      for (let sequence = 0; sequence < packets; sequence += 1) {
        produced += module._js8_audio_push_ulaw(frontend, sequence,
          BigInt(sequence * packetSamples), sequence * 20, 0,
          payloadPtr, packetSamples);
        while (module._js8_audio_next_window(frontend, windowPtr)) windows += 1;
      }
      produced += module._js8_audio_finish(frontend);
      while (module._js8_audio_next_window(frontend, windowPtr)) windows += 1;
      const elapsedMs = performance.now() - started;
      module._js8_audio_destroy(frontend);
      return {audioSeconds, packets, produced, windows, elapsedMs,
        realtimeFactor: audioSeconds * 1000 / elapsedMs};
    }

    execute(2); // Exclude module/JIT warm-up from the measured run.
    const measured = execute(seconds);
    const expected = seconds * 12000;
    const state = {...measured, expected, sampleCountPass: measured.produced === expected,
      wasmMemoryBytes: module.HEAPU8.buffer.byteLength,
      userAgent: navigator.userAgent};
    module._free(windowPtr); module._free(payloadPtr);
    result.textContent = `${state.sampleCountPass ? "PASS" : "FAIL"} ${JSON.stringify(state)}`;
  } catch (error) {
    result.textContent = `ERROR ${String(error.stack || error)}`;
  }
})();
