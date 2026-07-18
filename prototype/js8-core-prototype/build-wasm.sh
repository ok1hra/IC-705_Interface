#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_dir="$prototype_dir/build-wasm"

command -v emcmake >/dev/null
command -v emcc >/dev/null
command -v node >/dev/null

emcmake cmake -S "$prototype_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$build_dir" --parallel >/dev/null

wasm="$build_dir/js8-prototype.wasm"
javascript="$build_dir/js8-prototype.js"

raw_bytes=$(wc -c < "$wasm")
gzip_bytes=$(gzip -9 -c "$wasm" | wc -c)
js_bytes=$(wc -c < "$javascript")

echo "WASM raw_bytes=$raw_bytes gzip_bytes=$gzip_bytes js_bytes=$js_bytes"
node "$prototype_dir/wasm/smoke.js" "$javascript"
