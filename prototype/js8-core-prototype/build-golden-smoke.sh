#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$prototype_dir/../.." && pwd)
golden_dir="$prototype_dir/build-golden"
dump=$(mktemp /tmp/js8-golden-worker-XXXXXX.bin)
trap 'rm -f "$dump"' EXIT HUP INT TERM

cmake -S "$prototype_dir" -B "$prototype_dir/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$prototype_dir/build" --target js8-golden-fixtures js8-stream-prototype --parallel >/dev/null
"$prototype_dir/build/js8-golden-fixtures" "$golden_dir"
"$prototype_dir/build-wasm.sh"
"$prototype_dir/build-decoder-wasm.sh" "$golden_dir/A_golden_KN4CRD_TEST.wav" 0
mkdir -p "$prototype_dir/build-protocol"
node "$prototype_dir/protocol/build-jsc-map.js" \
  "$repo_dir/JS8Call-improved-master/JS8_JSC/JSC_map.cpp" \
  "$prototype_dir/build-protocol/jsc-map.bin"

for vector in \
  "A_golden_KN4CRD_TEST.wav:0" \
  "B_golden_KN4CRD_TEST.wav:1" \
  "C_golden_KN4CRD_TEST.wav:2" \
  "E_golden_KN4CRD_TEST.wav:4" \
  "I_golden_KN4CRD_TEST.wav:8"
do
  wav=${vector%:*}
  mode=${vector#*:}
  "$prototype_dir/build/js8-stream-prototype" --aud1-dump "$golden_dir/$wav" "$dump"
  node "$prototype_dir/wasm/worker_smoke.js" "$prototype_dir" "$dump" \
    "TrMcT8++++++" "$mode"
done
