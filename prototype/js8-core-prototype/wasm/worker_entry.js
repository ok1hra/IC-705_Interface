// PROTOTYPE — classic Web Worker / Node worker_threads adapter.

(function () {
  let post;
  let subscribe;
  let loadRuntime;
  let loadModules;
  let loadProtocol;
  let loadBrotli;

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
      importScripts(message.brotliJs);
      return self.createJs8Brotli({locateFile: () => message.brotliWasm});
    };
    const fetchBytes = async (url, label) => {
      const response = await fetch(url);
      if (!response.ok) throw new Error(`${label} fetch failed: ${response.status}`);
      return new Uint8Array(await response.arrayBuffer());
    };
    const decompressBrotli = async (module, url, outputSize, label) => {
      const input = await fetchBytes(url, label);
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
      importScripts(message.portableJs, message.decoderJs);
      const decoderWasm = await decompressBrotli(brotli,
        message.decoderWasmBr, message.decoderWasmSize, "decoder WASM");
      return {
        audio: await self.createJs8Prototype({
          locateFile: () => message.portableWasm,
        }),
        decoder: await self.createJs8DecoderProbe({
          wasmBinary: decoderWasm,
        }),
      };
    };
    loadProtocol = async (message, brotli) => {
      if (!message.protocolJs || !message.jscUrlBr) return null;
      importScripts(message.protocolJs);
      const jsc = await decompressBrotli(brotli, message.jscUrlBr,
        message.jscSize, "JSC");
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
        const Runtime = loadRuntime(message.runtimeJs);
        const brotli = await loadBrotli(message);
        const modules = await loadModules(message, brotli);
        const protocol = await loadProtocol(message, brotli);
        runtime = new Runtime(modules.audio, modules.decoder,
                              BigInt(message.anchorUtcMs || 0), protocol,
                              {strictEpochAnchoring:
                                Boolean(message.strictEpochAnchoring)});
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
