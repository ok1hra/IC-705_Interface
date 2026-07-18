#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_dir="$prototype_dir/build"

if [ "${1:-}" = "--wasm" ]; then
  exec "$prototype_dir/build-wasm.sh"
fi

if [ "${1:-}" = "--wasm-decoder" ]; then
  shift
  exec "$prototype_dir/build-decoder-wasm.sh" "$@"
fi

if [ "${1:-}" = "--worker-fixture" ]; then
  shift
  exec "$prototype_dir/build-worker-smoke.sh" "$@"
fi

if [ "${1:-}" = "--protocol" ]; then
  shift
  exec "$prototype_dir/build-protocol-smoke.sh" "$@"
fi

if [ "${1:-}" = "--timebase" ]; then
  shift
  exec node "$prototype_dir/timebase/timebase_tui.js" "$@"
fi

if [ "${1:-}" = "--weak-signal" ]; then
  shift
  exec "$prototype_dir/build-weak-signal.sh" "$@"
fi

if [ "${1:-}" = "--channel-matrix" ]; then
  shift
  exec "$prototype_dir/build-channel-matrix.sh" "$@"
fi

if [ "${1:-}" = "--drift-replay" ]; then
  shift
  exec "$prototype_dir/build-drift-replay.sh" "$@"
fi

if [ "${1:-}" = "--reconnect-epoch" ]; then
  shift
  exec "$prototype_dir/build-reconnect-epoch.sh" "$@"
fi

if [ "${1:-}" = "--capture-replay" ]; then
  shift
  exec "$prototype_dir/build-capture-replay.sh" "$@"
fi

if [ "${1:-}" = "--browser-fixture" ]; then
  shift
  exec "$prototype_dir/build-browser-smoke.sh" "$@"
fi

if [ "${1:-}" = "--golden" ]; then
  shift
  exec "$prototype_dir/build-golden-smoke.sh" "$@"
fi

if [ "${1:-}" = "--ui" ]; then
  shift
  exec "$prototype_dir/build-ui-smoke.sh" "$@"
fi

if [ "${1:-}" = "--asset-budget" ]; then
  shift
  exec "$prototype_dir/build-asset-budget.sh" "$@"
fi

if [ "${1:-}" = "--resampler-benchmark" ]; then
  shift
  exec "$prototype_dir/build-resampler-benchmark.sh" "$@"
fi

if [ "${1:-}" = "--browser-resampler-benchmark" ]; then
  shift
  exec "$prototype_dir/build-browser-resampler-benchmark.sh" "$@"
fi

if [ "${1:-}" = "--browser-decoder-benchmark" ]; then
  shift
  exec "$prototype_dir/build-browser-decoder-benchmark.sh" "$@"
fi

if [ "${1:-}" = "--tx" ]; then
  shift
  exec "$prototype_dir/build-tx-smoke.sh" "$@"
fi

if [ "${1:-}" = "--tx-tui" ]; then
  shift
  exec node "$prototype_dir/tx/tx_tui.js" "$@"
fi

if [ "${1:-}" = "--adapter" ]; then
  shift
  exec node "$prototype_dir/integration/adapter_smoke.js" "$@"
fi

if [ "${1:-}" = "--transport" ]; then
  shift
  exec node "$prototype_dir/transport/transport_smoke.js" "$@"
fi

if [ "${1:-}" = "--audio-source" ]; then
  shift
  exec node "$prototype_dir/transport/ws_audio_source_smoke.js" "$@"
fi

if [ "${1:-}" = "--browser-audio-source" ]; then
  shift
  exec sh "$prototype_dir/build-browser-audio-source.sh" "$@"
fi

if [ "${1:-}" = "--firmware-aud1" ]; then
  shift
  exec sh "$prototype_dir/build-firmware-aud1-smoke.sh" "$@"
fi

if [ "${1:-}" = "--firmware-tx-gate" ]; then
  shift
  exec sh "$prototype_dir/build-firmware-tx-gate-smoke.sh" "$@"
fi

if [ "${1:-}" = "--software-lifecycle" ]; then
  shift
  exec sh "$prototype_dir/build-software-lifecycle.sh" "$@"
fi

cmake -S "$prototype_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$build_dir" --parallel >/dev/null

if [ "${1:-}" = "--decode" ]; then
  shift
  exec "$build_dir/js8-reference-decoder" "$@"
fi

if [ "${1:-}" = "--stream" ]; then
  shift
  exec "$build_dir/js8-stream-prototype" "$@"
fi

if [ "${1:-}" = "--stream-fixture" ]; then
  shift
  exec "$build_dir/js8-stream-prototype" --fixture "$@"
fi

exec "$build_dir/js8-core-prototype" "$@"
