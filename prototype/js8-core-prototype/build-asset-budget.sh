#!/usr/bin/env sh
set -eu

prototype_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repository_dir=$(CDPATH= cd -- "$prototype_dir/../.." && pwd)
build_dir="$prototype_dir/build-budget"
partition_bytes=1966080
minimum_reserve_percent=10

mkdir -p "$build_dir"
stage_dir=$(mktemp -d "$build_dir/stage.XXXXXX")
trap 'rm -rf "$stage_dir"' EXIT HUP INT TERM

"$repository_dir/tools/prepare-spiffs-tree.sh" \
  "$repository_dir/data" "$stage_dir" >/dev/null

mkspiffs=${MKSPIFFS_BIN:-}
if [ -z "$mkspiffs" ]; then
  for root in "$HOME/Arduino/libraries/.arduino15" "$HOME/.arduino15"; do
    [ -d "$root/packages/esp32/tools/mkspiffs" ] || continue
    mkspiffs=$(find "$root/packages/esp32/tools/mkspiffs" \
      -type f -name mkspiffs 2>/dev/null | sort -V | tail -n 1)
    [ -n "$mkspiffs" ] && break
  done
fi
if [ -z "$mkspiffs" ] || [ ! -x "$mkspiffs" ]; then
  echo "ERROR: mkspiffs not found; set MKSPIFFS_BIN" >&2
  exit 1
fi

image="$build_dir/spiffs-js8-candidate.bin"
"$mkspiffs" -c "$stage_dir" -b 4096 -p 256 -s "$partition_bytes" "$image" >/dev/null
visual=$($mkspiffs -i "$image" 2>&1)
used=$(printf '%s\n' "$visual" | awk '/^used: +[0-9]+ of [0-9]+/ {print $2; exit}')
total=$(printf '%s\n' "$visual" | awk '/^used: +[0-9]+ of [0-9]+/ {print $4; exit}')
if [ -z "$used" ] || [ -z "$total" ]; then
  echo "ERROR: could not parse mkspiffs usage" >&2
  exit 1
fi
free=$((total - used))
reserve=$(awk -v free="$free" -v total="$total" 'BEGIN {printf "%.2f", free * 100 / total}')
payload=$(du -sb "$stage_dir" | awk '{print $1}')

printf 'ASSET BUDGET payload=%s spiffs_used=%s spiffs_total=%s free=%s reserve=%s%%\n' \
  "$payload" "$used" "$total" "$free" "$reserve"
awk -v reserve="$reserve" -v minimum="$minimum_reserve_percent" \
  'BEGIN {if (reserve + 0 < minimum + 0) exit 1}' || {
    echo "ASSET BUDGET FAIL minimum_reserve=${minimum_reserve_percent}%" >&2
    exit 1
  }
echo "ASSET BUDGET PASS minimum_reserve=${minimum_reserve_percent}% image=$image"
