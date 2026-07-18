#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$prototype_dir/../.." && pwd)
fixture_dir="$prototype_dir/build-capture-replay"
media_dir="$repo_dir/JS8Call-improved-master/media/tests"

if [ "$#" -eq 0 ]; then
  set -- \
    "$media_dir/E_1_1.wav" 4 \
    "$media_dir/E_2_1.wav" 4 \
    "$media_dir/A_1_4.wav" 0 \
    "$media_dir/A_2_1.wav" 0 \
    "$media_dir/A_2_3.wav" 0 \
    "$media_dir/A_2_5.wav" 0 \
    "$media_dir/A_2_6.wav" 0 \
    "$media_dir/A_2_9.wav" 0 \
    "$media_dir/A_3_3.wav" 0
elif [ $(( $# % 2 )) -ne 0 ]; then
  echo "usage: $0 [WAV MODE]..." >&2
  exit 2
fi

"$prototype_dir/toolchain/check-toolchain.sh"
cmake -S "$prototype_dir" -B "$prototype_dir/build" \
  -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$prototype_dir/build" \
  --target js8-capture-replay --parallel >/dev/null
"$prototype_dir/build/js8-capture-replay" "$fixture_dir" "$@"

"$prototype_dir/build-wasm.sh"
"$prototype_dir/build-decoder-wasm.sh" "$media_dir/A_1_4.wav" 0
mkdir -p "$prototype_dir/build-protocol"
node "$prototype_dir/protocol/build-jsc-map.js" \
  "$repo_dir/JS8Call-improved-master/JS8_JSC/JSC_map.cpp" \
  "$prototype_dir/build-protocol/jsc-map.bin"

exec node "$prototype_dir/wasm/capture_replay_worker.js" \
  "$prototype_dir" "$fixture_dir"
