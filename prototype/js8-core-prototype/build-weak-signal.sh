#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$prototype_dir/../.." && pwd)
weak_dir="$prototype_dir/build-weak-signal"

"$prototype_dir/toolchain/check-toolchain.sh"
cmake -S "$prototype_dir" -B "$prototype_dir/build" \
  -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$prototype_dir/build" \
  --target js8-weak-signal js8-stream-prototype --parallel >/dev/null
"$prototype_dir/build/js8-weak-signal" "$weak_dir"

"$prototype_dir/build-wasm.sh"
"$prototype_dir/build-decoder-wasm.sh" \
  "$weak_dir/A_weak_selected.wav" 0 \
  "$weak_dir/B_weak_selected.wav" 1 \
  "$weak_dir/C_weak_selected.wav" 2 \
  "$weak_dir/E_weak_selected.wav" 4 \
  "$weak_dir/I_weak_selected.wav" 8

mkdir -p "$prototype_dir/build-protocol"
node "$prototype_dir/protocol/build-jsc-map.js" \
  "$repo_dir/JS8Call-improved-master/JS8_JSC/JSC_map.cpp" \
  "$prototype_dir/build-protocol/jsc-map.bin"

for letter in A B C E I
do
  "$prototype_dir/build/js8-stream-prototype" --aud1-dump \
    "$weak_dir/${letter}_weak_selected.wav" \
    "$weak_dir/${letter}_weak_selected.aud1.bin"
done

exec node "$prototype_dir/wasm/weak_signal_worker.js" \
  "$prototype_dir" "$weak_dir"
