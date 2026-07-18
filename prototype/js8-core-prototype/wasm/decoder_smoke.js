// PROTOTYPE — host for a separately linked reference-decoder WASM probe.

const fs = require("fs");
const path = require("path");

function wavePayload(file) {
  const wave = fs.readFileSync(file);
  const marker = wave.indexOf(Buffer.from("data"));
  if (marker < 0) throw new Error("WAV data chunk not found");
  const bytes = wave.readUInt32LE(marker + 4);
  return wave.subarray(marker + 8, marker + 8 + bytes);
}

async function main() {
  const modulePath = path.resolve(process.argv[2]);
  const createModule = require(modulePath);
  const wasmBinary = fs.readFileSync(modulePath.replace(/\.js$/, ".wasm"));
  const decoder = await createModule({wasmBinary});
  const handle = decoder._js8_wasm_decoder_create();
  const eventPtr = decoder._malloc(44);
  if (process.argv.length < 5 || (process.argv.length - 3) % 2 !== 0)
    throw new Error("usage: decoder_smoke.js MODULE WAV SUBMODE [WAV SUBMODE]");

  for (let index = 3; index < process.argv.length; index += 2) {
    const wavePath = path.resolve(process.argv[index]);
    const submode = Number(process.argv[index + 1]);
    const pcm = wavePayload(wavePath);
    const samples = pcm.length / 2;
    const pcmPtr = decoder._malloc(pcm.length);
    decoder.HEAPU8.set(pcm, pcmPtr);

    const started = process.hrtime.bigint();
    const count = decoder._js8_wasm_decoder_run(
      handle, pcmPtr, samples, submode, 0, 5000, 1500
    );
    const elapsedMs = Number(process.hrtime.bigint() - started) / 1e6;
    const frames = [];
    while (decoder._js8_wasm_decoder_next_event(handle, eventPtr)) {
      const bytes = decoder.HEAPU8.slice(eventPtr + 16, eventPtr + 29);
      const zero = bytes.indexOf(0);
      frames.push(Buffer.from(zero < 0 ? bytes : bytes.slice(0, zero))
                        .toString("ascii"));
    }
    console.log(`WASM decoder samples=${samples} submode=${submode} ` +
                `unique_decodes=${count} elapsed_ms=${elapsedMs.toFixed(1)} ` +
                `memory_bytes=${decoder.HEAPU8.buffer.byteLength} ` +
                `frames=${JSON.stringify(frames)}`);
    decoder._free(pcmPtr);
    if (count < 0) process.exitCode = 1;
  }
  decoder._free(eventPtr);
  decoder._js8_wasm_decoder_destroy(handle);
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
