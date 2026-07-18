#!/usr/bin/env bash
set -euo pipefail

# Build and optionally upload LittleFS to the partition named "spiffs". The source data/
# directory is never renamed or modified except for regenerated compressed
# assets. Partition offset and size come from the installed no_ota.csv.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT=""
WRITE=false
BAUD=921600

usage() {
  cat <<'EOF'
Usage:
  tools/upload-spiffs.sh                         # build and validate only
  tools/upload-spiffs.sh --write --port DEVICE   # build, validate and upload

Options:
  --port DEVICE   Serial port, for example /dev/ttyUSB0 or /dev/ttyACM0
  --baud RATE     Upload baud rate (default 921600)
  --write         Explicitly permit writing the filesystem partition
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="${2:-}"; shift 2 ;;
    --baud) BAUD="${2:-}"; shift 2 ;;
    --write) WRITE=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

ARDUINO15_DIR="${ARDUINO15_DIR:-}"
if [[ -z "$ARDUINO15_DIR" ]]; then
  for candidate in "${HOME}/Arduino/libraries/.arduino15" "${HOME}/.arduino15"; do
    if [[ -d "$candidate/packages/esp32" ]]; then ARDUINO15_DIR="$candidate"; break; fi
  done
fi
if [[ -z "$ARDUINO15_DIR" ]]; then
  echo "ERROR: ESP32 Arduino package directory not found; set ARDUINO15_DIR" >&2
  exit 1
fi

ESP32_HW_ROOT="$(find "$ARDUINO15_DIR/packages/esp32/hardware/esp32" -mindepth 1 -maxdepth 1 -type d | sort -V | tail -1)"
PARTITION_CSV="$ESP32_HW_ROOT/tools/partitions/no_ota.csv"
MKLITTLEFS_BIN="$(find "$ARDUINO15_DIR/packages/esp32/tools/mklittlefs" -type f -name mklittlefs -perm -u+x | sort -V | tail -1)"
ESPTOOL_PY="$(find "$ARDUINO15_DIR/packages/esp32/tools/esptool_py" -type f -name esptool.py | sort -V | tail -1)"

for required in "$PARTITION_CSV" "$MKLITTLEFS_BIN" "$ESPTOOL_PY"; do
  [[ -f "$required" ]] || { echo "ERROR: required tool/file missing: $required" >&2; exit 1; }
done

partition_line="$(awk -F, '$1 ~ /^[[:space:]]*spiffs[[:space:]]*$/ {print; exit}' "$PARTITION_CSV")"
[[ -n "$partition_line" ]] || { echo "ERROR: SPIFFS partition not found in $PARTITION_CSV" >&2; exit 1; }
SPIFFS_OFFSET="$(printf '%s\n' "$partition_line" | awk -F, '{gsub(/[[:space:]]/,"",$4); print $4}')"
SPIFFS_SIZE="$(printf '%s\n' "$partition_line" | awk -F, '{gsub(/[[:space:]]/,"",$5); print $5}')"
SPIFFS_SIZE_DEC=$((SPIFFS_SIZE))

BUILD_DIR="${ROOT_DIR}/build/spiffs-upload"
STAGING_DIR="${BUILD_DIR}/data"
IMAGE="${BUILD_DIR}/spiffs.bin"
mkdir -p "$BUILD_DIR"
"${ROOT_DIR}/tools/prepare-spiffs-tree.sh" "${ROOT_DIR}/data" "$STAGING_DIR"
staging_bytes="$(du -sb "$STAGING_DIR" | awk '{print $1}')"
runtime_reserve_bytes=$((256 * 1024))
metadata_budget_bytes=$((64 * 1024))
max_payload_bytes=$((SPIFFS_SIZE_DEC - runtime_reserve_bytes - metadata_budget_bytes))
[[ "$staging_bytes" -le "$max_payload_bytes" ]] || {
  echo "ERROR: LittleFS deployment leaves less than the required 256 KiB runtime reserve." >&2
  echo "       Payload: $staging_bytes B; limit: $max_payload_bytes B; partition: $SPIFFS_SIZE_DEC B." >&2
  exit 1
}
"$MKLITTLEFS_BIN" -c "$STAGING_DIR" -b 4096 -p 256 -s "$SPIFFS_SIZE_DEC" "$IMAGE"

IMAGE_SIZE="$(stat -c '%s' "$IMAGE")"
[[ "$IMAGE_SIZE" -eq "$SPIFFS_SIZE_DEC" ]] || {
  echo "ERROR: image size $IMAGE_SIZE differs from partition size $SPIFFS_SIZE_DEC" >&2
  exit 1
}

echo "==> Partition CSV : $PARTITION_CSV"
echo "==> LittleFS offset: $SPIFFS_OFFSET (partition label: spiffs)"
echo "==> LittleFS size : $SPIFFS_SIZE ($SPIFFS_SIZE_DEC bytes)"
echo "==> Runtime reserve gate: $runtime_reserve_bytes bytes (payload $staging_bytes / $max_payload_bytes)"
echo "==> Image         : $IMAGE ($IMAGE_SIZE bytes)"

if [[ "$WRITE" != true ]]; then
  echo "==> Validation complete; flash was NOT modified"
  echo "    Upload with: $0 --write --port /dev/ttyUSB0"
  exit 0
fi

[[ -n "$PORT" ]] || { echo "ERROR: --write requires --port DEVICE" >&2; exit 2; }
[[ -e "$PORT" ]] || { echo "ERROR: serial port does not exist: $PORT" >&2; exit 1; }
[[ "$BAUD" =~ ^[0-9]+$ ]] || { echo "ERROR: invalid baud rate: $BAUD" >&2; exit 2; }

echo "==> Writing only LittleFS at $SPIFFS_OFFSET on $PORT"
python3 "$ESPTOOL_PY" --chip esp32 --port "$PORT" --baud "$BAUD" \
  --before default_reset --after hard_reset write_flash "$SPIFFS_OFFSET" "$IMAGE"
