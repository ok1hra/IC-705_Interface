// PROTOTYPE — Node host proving the generated WASM C ABI is callable.

const path = require("path");
const fs = require("fs");

async function main() {
  const modulePath = path.resolve(process.argv[2]);
  const createModule = require(modulePath);
  const wasmPath = modulePath.replace(/\.js$/, ".wasm");
  const wasm = await createModule({wasmBinary: fs.readFileSync(wasmPath)});

  const framePtr = wasm._malloc(12);
  const tonesPtr = wasm._malloc(79);
  const crcPtr = wasm._malloc(2);
  wasm.HEAPU8.set(Buffer.from("000000000000", "ascii"), framePtr);
  const encoded = wasm._js8_proto_encode_frame(
    framePtr, 0, 0, tonesPtr, crcPtr
  );
  const tones = Array.from(wasm.HEAPU8.slice(tonesPtr, tonesPtr + 79));
  const crc = wasm.HEAPU16[crcPtr >> 1];

  const ulawPtr = wasm._malloc(160);
  wasm.HEAPU8.fill(0xff, ulawPtr, ulawPtr + 160);
  const outputPtr = wasm._malloc(1024 * 4);
  const audio = wasm._js8_audio_create(0n);
  const firstProduced = wasm._js8_audio_push_ulaw(
    audio, 0, 0n, 0, 0, ulawPtr, 160
  );
  const gapProduced = wasm._js8_audio_push_ulaw(
    audio, 2, 320n, 40, 0, ulawPtr, 160
  );
  const copied = wasm._js8_audio_copy_output(audio, outputPtr, 1024);
  const duplicateProduced = wasm._js8_audio_push_ulaw(
    audio, 2, 320n, 40, 0, ulawPtr, 160
  );
  const tailProduced = wasm._js8_audio_finish(audio);

  const result = {
    encoder: {
      accepted: encoded === 1,
      crc: `0x${crc.toString(16).padStart(3, "0")}`,
      firstCostas: tones.slice(0, 7),
      toneCount: tones.length,
    },
    audio: {
      firstProduced,
      gapProduced,
      copied,
      duplicateProduced,
      tailProduced,
      totalProduced: firstProduced + gapProduced + tailProduced,
    },
  };
  console.log(`WASM smoke ${JSON.stringify(result)}`);

  wasm._js8_audio_destroy(audio);
  wasm._free(outputPtr);
  wasm._free(ulawPtr);
  wasm._free(crcPtr);
  wasm._free(tonesPtr);
  wasm._free(framePtr);

  if (!result.encoder.accepted || crc !== 0x02a ||
      tones.slice(0, 7).join(",") !== "4,2,5,6,1,3,0" ||
      firstProduced !== 192 || gapProduced !== 480 || copied !== 480 ||
      duplicateProduced !== 0 || tailProduced !== 48 ||
      result.audio.totalProduced !== 720) {
    process.exitCode = 1;
  }
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
