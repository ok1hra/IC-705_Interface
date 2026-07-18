// PROTOTYPE — classic Web Worker / Node worker_threads adapter.

(function () {
  let post;
  let subscribe;
  let loadRuntime;
  let loadModules;
  let loadProtocol;
  let loadBrotli;
  const loading = (stage, label, progress, loaded, total) => post({
    type:"loading", stage, label, progress:Math.max(0, Math.min(100, Math.round(progress))),
    loaded:Number(loaded || 0), total:Number(total || 0),
  });

  if (typeof process === "object" && process.versions && process.versions.node) {
    const {parentPort} = require("worker_threads");
    const fs = require("fs");
    const path = require("path");
    post = (value) => parentPort.postMessage(value);
    subscribe = (handler) => parentPort.on("message", handler);
    loadRuntime = (url) => require(path.resolve(url)).Js8WorkerRuntime;
    loadBrotli = async () => null;
    loadModules = async (message) => {
      const portablePath = path.resolve(message.portableJs);
      const decoderPath = path.resolve(message.decoderJs);
      const createPortable = require(portablePath);
      const createDecoder = require(decoderPath);
      return {
        audio: await createPortable({
          wasmBinary: fs.readFileSync(portablePath.replace(/\.js$/, ".wasm")),
        }),
        decoder: await createDecoder({
          wasmBinary: fs.readFileSync(decoderPath.replace(/\.js$/, ".wasm")),
        }),
      };
    };
    loadProtocol = (message) => {
      if (!message.protocolJs || !message.jscPath) return null;
      const protocol = require(path.resolve(message.protocolJs));
      const dictionary = new protocol.JscDictionary(
        fs.readFileSync(path.resolve(message.jscPath))
      );
      return new protocol.ActivityStore(dictionary);
    };
  } else {
    post = (value) => self.postMessage(value);
    subscribe = (handler) => { self.onmessage = (event) => handler(event.data); };
    loadRuntime = (url) => {
      importScripts(url);
      return self.Js8WorkerRuntime;
    };
    loadBrotli = async (message) => {
      loading("brotli", "Loading Brotli decoder", 2);
      importScripts(message.brotliJs);
      const module = await self.createJs8Brotli({locateFile: () => message.brotliWasm});
      loading("brotli", "Brotli decoder ready", 8);
      return module;
    };
    const fetchBytes = async (url, label, stage, progressFrom, progressTo) => {
      const response = await fetch(url);
      if (!response.ok) throw new Error(`${label} fetch failed: ${response.status}`);
      const total = Number(response.headers.get("Content-Length")) || 0;
      if (!response.body || !response.body.getReader) {
        const bytes = new Uint8Array(await response.arrayBuffer());
        loading(stage, label, progressTo, bytes.length, total || bytes.length);
        return bytes;
      }
      const reader = response.body.getReader();
      const chunks = [];
      let loaded = 0;
      for (;;) {
        const result = await reader.read();
        if (result.done) break;
        chunks.push(result.value);
        loaded += result.value.length;
        const ratio = total > 0 ? Math.min(1, loaded / total) : 0;
        loading(stage, label, progressFrom + (progressTo - progressFrom) * ratio,
          loaded, total);
      }
      const bytes = new Uint8Array(loaded);
      let at = 0;
      for (const chunk of chunks) { bytes.set(chunk, at); at += chunk.length; }
      loading(stage, label, progressTo, loaded, total || loaded);
      return bytes;
    };
    const decompressBrotli = async (module, url, outputSize, label, stage,
                                    progressFrom, progressTo, decodeProgress) => {
      const input = await fetchBytes(url, label, stage, progressFrom, progressTo);
      loading(stage, `Expanding ${label}`, decodeProgress, input.length, input.length);
      const inputPtr = module._malloc(input.length);
      const sizePtr = module._malloc(4);
      const outputPtr = module._malloc(outputSize);
      try {
        module.HEAPU8.set(input, inputPtr);
        module.HEAPU32[sizePtr >> 2] = outputSize;
        const result = module._BrotliDecoderDecompress(
          input.length, inputPtr, sizePtr, outputPtr);
        const written = module.HEAPU32[sizePtr >> 2];
        if (result !== 1 || written !== outputSize)
          throw new Error(`${label} Brotli decode failed (${result}, ${written}/${outputSize})`);
        return module.HEAPU8.slice(outputPtr, outputPtr + written);
      } finally {
        module._free(outputPtr); module._free(sizePtr); module._free(inputPtr);
      }
    };
    loadModules = async (message, brotli) => {
      loading("decoder", "Loading decoder runtime", 9);
      importScripts(message.portableJs, message.decoderJs);
      const decoderWasm = await decompressBrotli(brotli,
        message.decoderWasmBr, message.decoderWasmSize, "decoder WASM",
        "decoder", 10, 34, 37);
      loading("decoder", "Initializing decoder", 39);
      const modules = {
        audio: await self.createJs8Prototype({
          locateFile: () => message.portableWasm,
        }),
        decoder: await self.createJs8DecoderProbe({
          wasmBinary: decoderWasm,
        }),
      };
      loading("decoder", "Decoder ready", 44);
      return modules;
    };
    loadProtocol = async (message, brotli) => {
      if (!message.protocolJs || !message.jscUrlBr) return null;
      loading("dictionary", "Loading JS8 dictionary runtime", 45);
      importScripts(message.protocolJs);
      const jsc = await decompressBrotli(brotli, message.jscUrlBr,
        message.jscSize, "JSC dictionary", "dictionary", 46, 92, 95);
      loading("dictionary", "Indexing JS8 dictionary", 97);
      const dictionary = new self.Js8Protocol.JscDictionary(
        jsc
      );
      return new self.Js8Protocol.ActivityStore(dictionary);
    };
  }

  let runtime;
  subscribe(async (message) => {
    try {
      if (message.type === "init") {
        loading("runtime", "Starting JS8Call modem", 1);
        const Runtime = loadRuntime(message.runtimeJs);
        const brotli = await loadBrotli(message);
        const modules = await loadModules(message, brotli);
        const protocol = await loadProtocol(message, brotli);
        runtime = new Runtime(modules.audio, modules.decoder,
                              BigInt(message.anchorUtcMs || 0), protocol,
                              {strictEpochAnchoring:
                                Boolean(message.strictEpochAnchoring)});
        loading("ready", "JS8Call modem ready", 100);
        post({type: "ready", state: runtime.state()});
      } else if (message.type === "epoch") {
        post({type: "epoch", state: runtime.beginEpoch(
          message.streamId, message.anchorUtcMs)});
      } else if (message.type === "audio") {
        post({type: "state", state: runtime.pushAud1(message.wire,
                                                     message.arrivalMs)});
      } else if (message.type === "audioBatch") {
        for (const packet of message.packets)
          runtime.pushAud1(packet.wire, packet.arrivalMs);
        post({type: "state", state: runtime.state()});
      } else if (message.type === "finish") {
        post({type: "finished", state: runtime.finish()});
      } else if (message.type === "reset") {
        runtime.reset(null);
        post({type: "state", state: runtime.state()});
      } else if (message.type === "close") {
        runtime.destroy();
        post({type: "closed"});
      }
    } catch (error) {
      post({type: "error", message: String(error.stack || error)});
    }
  });
})();
