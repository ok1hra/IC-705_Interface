#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
output_dir="$prototype_dir/build-channel-matrix"

"$prototype_dir/toolchain/check-toolchain.sh"
cmake -S "$prototype_dir" -B "$prototype_dir/build" \
  -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$prototype_dir/build" --target js8-channel-matrix \
  --parallel >/dev/null
exec "$prototype_dir/build/js8-channel-matrix" "$output_dir"
