#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$prototype_dir/../.." && pwd)
golden_dir="$prototype_dir/build-golden"
benchmark_dir="$prototype_dir/build-phone-benchmark"

"$prototype_dir/toolchain/check-toolchain.sh"
cmake -S "$prototype_dir" -B "$prototype_dir/build" \
  -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$prototype_dir/build" \
  --target js8-golden-fixtures js8-stream-prototype --parallel >/dev/null
"$prototype_dir/build/js8-golden-fixtures" "$golden_dir"
"$prototype_dir/build-wasm.sh"
"$prototype_dir/build-decoder-wasm.sh" \
  "$golden_dir/A_golden_KN4CRD_TEST.wav" 0 \
  "$golden_dir/B_golden_KN4CRD_TEST.wav" 1 \
  "$golden_dir/C_golden_KN4CRD_TEST.wav" 2 \
  "$golden_dir/E_golden_KN4CRD_TEST.wav" 4 \
  "$golden_dir/I_golden_KN4CRD_TEST.wav" 8

mkdir -p "$prototype_dir/build-protocol" "$benchmark_dir"
node "$prototype_dir/protocol/build-jsc-map.js" \
  "$repo_dir/JS8Call-improved-master/JS8_JSC/JSC_map.cpp" \
  "$prototype_dir/build-protocol/jsc-map.bin"

for letter in A B C E I
do
  wav="$golden_dir/${letter}_golden_KN4CRD_TEST.wav"
  dump="$benchmark_dir/${letter}_golden_KN4CRD_TEST.aud1.bin"
  "$prototype_dir/build/js8-stream-prototype" --aud1-dump "$wav" "$dump"
done

if [ "${1:-}" = "--serve" ]; then
  port=${2:-8765}
  echo "PHONE DECODER BENCHMARK http://<this-computer-ip>:$port/browser/decoder-benchmark.html"
  exec python3 -m http.server "$port" --bind 0.0.0.0 --directory "$prototype_dir"
fi

exec node "$prototype_dir/browser/browser_smoke.js" "$prototype_dir" \
  --page browser/decoder-benchmark.html
