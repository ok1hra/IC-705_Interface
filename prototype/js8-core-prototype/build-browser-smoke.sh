#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$prototype_dir/../.." && pwd)
if [ "$#" -ne 2 ]; then
  echo "usage: $0 WAV SUBMODE" >&2
  exit 2
fi
dump=$(mktemp /tmp/js8-browser-worker-XXXXXX.bin)
trap 'rm -f "$dump"' EXIT HUP INT TERM
cmake -S "$prototype_dir" -B "$prototype_dir/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$prototype_dir/build" --target js8-stream-prototype --parallel >/dev/null
"$prototype_dir/build-wasm.sh"
"$prototype_dir/build-decoder-wasm.sh" "$1" "$2"
mkdir -p "$prototype_dir/build-protocol"
node "$prototype_dir/protocol/build-jsc-map.js" \
  "$repo_dir/JS8Call-improved-master/JS8_JSC/JSC_map.cpp" \
  "$prototype_dir/build-protocol/jsc-map.bin"
"$prototype_dir/build/js8-stream-prototype" --aud1-dump "$1" "$dump"
node "$prototype_dir/browser/browser_smoke.js" "$prototype_dir" "$dump"
