#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$prototype_dir/../.." && pwd)
mkdir -p "$prototype_dir/build-protocol"
node "$prototype_dir/protocol/build-jsc-map.js" \
  "$repo_dir/JS8Call-improved-master/JS8_JSC/JSC_map.cpp" \
  "$prototype_dir/build-protocol/jsc-map.bin"
exec node "$prototype_dir/protocol/protocol_tui.js" \
  "$prototype_dir/build-protocol/jsc-map.bin" "$@"
