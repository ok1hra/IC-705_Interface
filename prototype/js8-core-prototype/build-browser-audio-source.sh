#!/usr/bin/env sh
set -eu
prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec node "$prototype_dir/transport/browser_audio_source_smoke.js"
