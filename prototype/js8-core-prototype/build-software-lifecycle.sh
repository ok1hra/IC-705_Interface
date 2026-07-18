#!/usr/bin/env sh
set -eu
prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
node "$prototype_dir/timebase/timebase_tui.js" --snapshot
node "$prototype_dir/ui/settings_store_smoke.js"
node "$prototype_dir/transport/transport_smoke.js"
node "$prototype_dir/transport/ws_audio_source_smoke.js"
node "$prototype_dir/integration/adapter_smoke.js"
sh "$prototype_dir/build-firmware-aud1-smoke.sh"
sh "$prototype_dir/build-firmware-tx-gate-smoke.sh"
sh "$prototype_dir/build-ui-smoke.sh" --screenshots
sh "$prototype_dir/build-browser-audio-source.sh"
echo "SOFTWARE LIFECYCLE PASS"
