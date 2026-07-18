#!/usr/bin/env sh
set -eu
prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
node "$prototype_dir/ui/settings_store_smoke.js"
node "$prototype_dir/ui/trx_presets_smoke.js"
if [ "${1:-}" = "--screenshots" ]; then
  exec node "$prototype_dir/ui/serve_ui.js" --screenshots "$prototype_dir/build-ui"
fi
exec node "$prototype_dir/ui/serve_ui.js"
