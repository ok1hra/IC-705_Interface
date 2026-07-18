# JS8Call browser modem: reproducible build

The production modem consists of readable browser sources in `data/js8-*.js`,
generated WASM/JSC assets in `data/`, the isolated build source under
`prototype/js8-core-prototype/`, and a pinned subset of the original JS8Call
source under `JS8Call-improved-master/`.

## Debian 12 dependencies

Enable Debian source packages in APT (`deb-src ... bookworm main` and the
matching security source), then install:

```sh
sudo apt update
sudo apt install \
  build-essential ca-certificates cmake dpkg-dev emscripten git gzip \
  libboost1.81-dev libfftw3-dev nodejs p7zip-full python3 terser xz-utils
```

Reviewed versions are pinned in
`prototype/js8-core-prototype/toolchain/toolchain.lock`: Emscripten 3.1.6,
CMake 3.25.1 and Node 18.20.4 for a release build. Local tests accept Node
18–20. Verify the installed toolchain with:

```sh
./prototype/js8-core-prototype/toolchain/check-toolchain.sh
```

The decoder builder obtains the pinned FFTW 3.3.10 source with
`apt-get source fftw3=3.3.10-1`. To rebuild the small Worker-side Brotli
decoder as well, obtain the Debian Brotli source and point the build at it:

```sh
cd /tmp
apt-get source brotli
cd -
BROTLI_SOURCE_DIR=/tmp/brotli-1.0.9 ./tools/build-brotli-decoder.sh data
```

`google-chrome` is optional and is used only by the headless browser regression
tests. Firmware compilation additionally requires Arduino IDE, ESP32 Arduino
core 2.0.14, board `ESP32 Dev Module`, partition scheme
`No OTA (2MB APP/2MB SPIFFS)`, and TrxNet 0.3.0 from
`https://github.com/ok1hra/TrxNet` in the Arduino libraries directory.

## Rebuild

Rebuild the encoder, decoder and JSC dictionary and stage their production
browser assets with one command:

```sh
./tools/build-js8-assets.sh
```

For firmware and Pages releases, first use Arduino IDE
`Sketch -> Export Compiled Binary`, then run one of:

```sh
./tools/upload-firmware-spiffs.sh --port /dev/ttyUSB0
./tools/gh-pages.sh
./tools/gh-pages.sh --publish
```

All three deployment paths regenerate minified, gzip and Brotli assets. The
Pages script refuses a stale firmware export, a missing JS8 runtime asset, a
missing LittleFS builder (`mklittlefs` from the ESP32 package), or an unsafe output directory.

## Source files that must be tracked

Track these project-owned files:

- `data/data.html`, `data/data.css`, `data/data.js`, `data/dxcc.js` and all
  readable `data/js8-*.js` sources;
- generated-but-required `data/js8-core.wasm`, `data/js8-decoder.wasm`,
  `data/js8-brotli.wasm`, `data/js8-jsc.bin` and their Emscripten loaders;
- `data/THIRD-PARTY-NOTICES.txt` and `data/BROTLI-LICENSE.txt`;
- source, CMake files, shell/JS build scripts and test fixtures under
  `prototype/js8-core-prototype/`, excluding every `build*` directory;
- `tools/build-js8-assets.sh`, `tools/build-brotli-decoder.sh`,
  `tools/brotli-js8-assets.js`, `tools/minify-spiffs-js.sh`,
  `tools/gzip-assets.sh`, `tools/prepare-spiffs-tree.sh`, upload scripts,
  `tools/data-browser-smoke.js` and `tools/gh-pages.sh`.

The minimal pinned upstream subset needed to compile is:

- `JS8Call-improved-master/LICENSE`;
- `JS8Call-improved-master/JS8_Mode/`;
- `JS8Call-improved-master/JS8_Include/commons.h`;
- `JS8Call-improved-master/JS8_JSC/JSC_map.cpp`;
- `JS8Call-improved-master/vendor/Eigen/`.

Do not add prototype `build*` directories, `build/`, exported firmware images,
compressed `*.gz`/`*.br` companions that can be regenerated, screenshots,
captures, IDE state or downloaded full upstream documentation. Reference WAV
files are optional regression fixtures, not compile dependencies.
