#!/usr/bin/env sh
set -eu
prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
sh "$prototype_dir/check-runtime-sync.sh"
node "$prototype_dir/scheduler/scheduler_smoke.js"
node "$prototype_dir/transport/tx_clock_consistency_smoke.js"
node "$prototype_dir/protocol/restrictions_smoke.js"
node "$prototype_dir/protocol/autoreply_smoke.js"
node "$prototype_dir/protocol/heartbeat_smoke.js"
node "$prototype_dir/protocol/relay_smoke.js"
node "$prototype_dir/protocol/inbox_smoke.js"
node "$prototype_dir/protocol/reassembly_smoke.js"
node "$prototype_dir/protocol/directed_tx_smoke.js"
node "$prototype_dir/tx/txqueue_smoke.js"
node "$prototype_dir/tx/tx_progress_smoke.js"
cmake -S "$prototype_dir" -B "$prototype_dir/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$prototype_dir/build" --target js8-golden-fixtures --parallel >/dev/null
"$prototype_dir/build/js8-golden-fixtures" "$prototype_dir/build-golden" >/dev/null
"$prototype_dir/build-wasm.sh"
"$prototype_dir/build-decoder-wasm.sh" \
  "$prototype_dir/build-golden/B_golden_KN4CRD_TEST.wav" 1
node "$prototype_dir/protocol/comms_loopback_smoke.js" \
  "$prototype_dir/build-wasm/js8-prototype.js" \
  "$prototype_dir/build-decoder-wasm/js8-decoder.js"
node "$prototype_dir/tx/tx_smoke.js" \
  "$prototype_dir/build-wasm/js8-prototype.js" \
  "$prototype_dir/build-decoder-wasm/js8-decoder.js"
