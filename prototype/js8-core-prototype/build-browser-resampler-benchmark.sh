#!/usr/bin/env sh
set -eu
prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
"$prototype_dir/build-wasm.sh" >/dev/null
if [ "${1:-}" = "--serve" ]; then
  port=${2:-8765}
  echo "PHONE BENCHMARK http://<this-computer-ip>:$port/browser/resampler-benchmark.html?seconds=600"
  exec python3 -m http.server "$port" --bind 0.0.0.0 --directory "$prototype_dir"
fi
exec node "$prototype_dir/browser/browser_smoke.js" "$prototype_dir" \
  --page browser/resampler-benchmark.html
