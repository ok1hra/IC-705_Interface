#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$prototype_dir/../.." && pwd)
fixture_dir="$prototype_dir/build-reconnect-epoch"
golden_dir="$prototype_dir/build-golden"

"$prototype_dir/toolchain/check-toolchain.sh"
cmake -S "$prototype_dir" -B "$prototype_dir/build" \
  -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$prototype_dir/build" \
  --target js8-drift-replay js8-golden-fixtures --parallel >/dev/null
"$prototype_dir/build/js8-drift-replay" "$fixture_dir" >/dev/null
"$prototype_dir/build/js8-golden-fixtures" "$golden_dir" >/dev/null

"$prototype_dir/build-wasm.sh"
"$prototype_dir/build-decoder-wasm.sh" \
  "$golden_dir/A_golden_KN4CRD_TEST.wav" 0
mkdir -p "$prototype_dir/build-protocol"
node "$prototype_dir/protocol/build-jsc-map.js" \
  "$repo_dir/JS8Call-improved-master/JS8_JSC/JSC_map.cpp" \
  "$prototype_dir/build-protocol/jsc-map.bin"

exec node "$prototype_dir/wasm/reconnect_epoch_worker.js" \
  "$prototype_dir" "$fixture_dir"
