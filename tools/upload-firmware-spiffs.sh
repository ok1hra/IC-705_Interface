#!/usr/bin/env bash
set -euo pipefail

# Release upload after Arduino IDE: Sketch -> Export Compiled Binary.
# Builds LittleFS in the partition named "spiffs" and writes APP + filesystem in one
# esptool write session. Bootloader and partition table are deliberately
# untouched; the device partition table is verified before any write.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT=""
FIRMWARE=""
BAUD=921600
DRY_RUN=false

usage() {
  cat <<'EOF'
Usage:
  tools/upload-firmware-spiffs.sh --port DEVICE [options]
  tools/upload-firmware-spiffs.sh --dry-run [options]

Options:
  --port DEVICE    Serial port, e.g. /dev/ttyUSB0 or /dev/serial/by-id/...
  --firmware FILE  Exported application binary
                   (default: IC-705_Interface.ino.esp32.bin)
  --baud RATE      Upload baud rate (default: 921600)
  --dry-run        Build and validate both images without writing flash
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="${2:-}"; shift 2 ;;
    --firmware) FIRMWARE="${2:-}"; shift 2 ;;
    --baud) BAUD="${2:-}"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$FIRMWARE" ]]; then
  if [[ -f "${ROOT_DIR}/IC-705_Interface.ino.esp32.bin" ]]; then
    FIRMWARE="${ROOT_DIR}/IC-705_Interface.ino.esp32.bin"
  elif [[ -f "${ROOT_DIR}/IC-705_Interface.ino.bin" ]]; then
    FIRMWARE="${ROOT_DIR}/IC-705_Interface.ino.bin"
  else
    echo "ERROR: exported firmware not found; run Sketch -> Export Compiled Binary" >&2
    exit 1
  fi
else
  FIRMWARE="$(realpath "$FIRMWARE")"
fi
[[ -f "$FIRMWARE" ]] || { echo "ERROR: firmware not found: $FIRMWARE" >&2; exit 1; }

firmware_magic="$(od -An -t x1 -N1 "$FIRMWARE" | tr -d '[:space:]')"
[[ "$firmware_magic" == "e9" ]] || {
  echo "ERROR: $FIRMWARE is not an ESP32 application image (magic=$firmware_magic)" >&2
  exit 1
}

# Refuse an export older than any root-level firmware source file.
while IFS= read -r -d '' source; do
  if [[ "$source" -nt "$FIRMWARE" ]]; then
    echo "ERROR: exported firmware is stale; newer source: $source" >&2
    echo "       Run Sketch -> Export Compiled Binary again." >&2
    exit 1
  fi
done < <(find "$ROOT_DIR" -maxdepth 1 -type f \
  \( -name '*.ino' -o -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \) -print0)

ARDUINO15_DIR="${ARDUINO15_DIR:-}"
if [[ -z "$ARDUINO15_DIR" ]]; then
  for candidate in "${HOME}/Arduino/libraries/.arduino15" "${HOME}/.arduino15"; do
    if [[ -d "$candidate/packages/esp32" ]]; then ARDUINO15_DIR="$candidate"; break; fi
  done
fi
[[ -n "$ARDUINO15_DIR" ]] || {
  echo "ERROR: ESP32 Arduino package directory not found; set ARDUINO15_DIR" >&2
  exit 1
}

ESP32_HW_ROOT="$(find "$ARDUINO15_DIR/packages/esp32/hardware/esp32" -mindepth 1 -maxdepth 1 -type d | sort -V | tail -1)"
PARTITION_CSV="$ESP32_HW_ROOT/tools/partitions/no_ota.csv"
MKLITTLEFS_BIN="$(find "$ARDUINO15_DIR/packages/esp32/tools/mklittlefs" -type f -name mklittlefs -perm -u+x | sort -V | tail -1)"
ESPTOOL_PY="$(find "$ARDUINO15_DIR/packages/esp32/tools/esptool_py" -type f -name esptool.py | sort -V | tail -1)"
GEN_PARTITIONS="$ESP32_HW_ROOT/tools/gen_esp32part.py"
for required in "$PARTITION_CSV" "$MKLITTLEFS_BIN" "$ESPTOOL_PY" "$GEN_PARTITIONS"; do
  [[ -f "$required" ]] || { echo "ERROR: required tool/file missing: $required" >&2; exit 1; }
done

partition_field() {
  local name="$1" field="$2"
  awk -F, -v wanted="$name" -v column="$field" '
    $1 ~ "^[[:space:]]*" wanted "[[:space:]]*$" {
      gsub(/[[:space:]]/, "", $column); print $column; exit
    }' "$PARTITION_CSV"
}

APP_OFFSET="$(partition_field app0 4)"
APP_SIZE="$(partition_field app0 5)"
SPIFFS_OFFSET="$(partition_field spiffs 4)"
SPIFFS_SIZE="$(partition_field spiffs 5)"
[[ -n "$APP_OFFSET" && -n "$APP_SIZE" && -n "$SPIFFS_OFFSET" && -n "$SPIFFS_SIZE" ]] || {
  echo "ERROR: app0/SPIFFS layout missing in $PARTITION_CSV" >&2
  exit 1
}
APP_SIZE_DEC=$((APP_SIZE))
SPIFFS_SIZE_DEC=$((SPIFFS_SIZE))
FIRMWARE_SIZE="$(stat -c '%s' "$FIRMWARE")"
[[ "$FIRMWARE_SIZE" -le "$APP_SIZE_DEC" ]] || {
  echo "ERROR: firmware $FIRMWARE_SIZE B exceeds app0 partition $APP_SIZE_DEC B" >&2
  exit 1
}

BUILD_DIR="${ROOT_DIR}/build/release-upload"
STAGING_DIR="${BUILD_DIR}/data"
SPIFFS_IMAGE="${BUILD_DIR}/spiffs.bin"
EXPECTED_PARTITIONS="${BUILD_DIR}/expected-partitions.bin"
DEVICE_PARTITIONS="${BUILD_DIR}/device-partitions.bin"
mkdir -p "$BUILD_DIR"
"${ROOT_DIR}/tools/prepare-spiffs-tree.sh" "${ROOT_DIR}/data" "$STAGING_DIR"
staging_bytes="$(du -sb "$STAGING_DIR" | awk '{print $1}')"
staging_files="$(find "$STAGING_DIR" -type f | wc -l)"
runtime_reserve_bytes=$((256 * 1024))
metadata_budget_bytes=$((64 * 1024))
max_payload_bytes=$((SPIFFS_SIZE_DEC - runtime_reserve_bytes - metadata_budget_bytes))
if [[ "$staging_bytes" -gt "$max_payload_bytes" ]]; then
  echo "ERROR: LittleFS deployment leaves less than the required 256 KiB runtime reserve." >&2
  echo "       Payload: $staging_bytes B; limit: $max_payload_bytes B; partition: $SPIFFS_SIZE_DEC B." >&2
  exit 1
fi
if ! "$MKLITTLEFS_BIN" -c "$STAGING_DIR" -b 4096 -p 256 -s "$SPIFFS_SIZE_DEC" "$SPIFFS_IMAGE"; then
  echo "ERROR: LittleFS deployment does not fit after filesystem overhead." >&2
  echo "       Payload: $staging_bytes B in $staging_files files; partition: $SPIFFS_SIZE_DEC B." >&2
  echo "       Reduce or recompress web assets; increasing the image past the partition is unsafe." >&2
  exit 1
fi
python3 "$GEN_PARTITIONS" "$PARTITION_CSV" "$EXPECTED_PARTITIONS" >/dev/null
SPIFFS_IMAGE_SIZE="$(stat -c '%s' "$SPIFFS_IMAGE")"
[[ "$SPIFFS_IMAGE_SIZE" -eq "$SPIFFS_SIZE_DEC" ]] || {
  echo "ERROR: LittleFS image size $SPIFFS_IMAGE_SIZE != partition $SPIFFS_SIZE_DEC" >&2
  exit 1
}

echo "==> Partition map : $PARTITION_CSV"
echo "==> Firmware      : $FIRMWARE ($FIRMWARE_SIZE / $APP_SIZE_DEC bytes)"
echo "==> APP write     : $APP_OFFSET"
echo "==> LittleFS image: $SPIFFS_IMAGE ($SPIFFS_IMAGE_SIZE bytes)"
echo "==> Runtime reserve gate: $runtime_reserve_bytes bytes (payload $staging_bytes / $max_payload_bytes)"
echo "==> Filesystem write: $SPIFFS_OFFSET (partition label: spiffs)"
echo "==> Preserved     : bootloader, partition table, NVS and coredump"
echo "==> Safety check  : device partition table must exactly match no_ota.csv"

if [[ "$DRY_RUN" == true ]]; then
  echo "==> Dry run complete; flash was NOT modified"
  exit 0
fi

[[ -n "$PORT" ]] || { echo "ERROR: upload requires --port DEVICE" >&2; exit 2; }
[[ -e "$PORT" ]] || { echo "ERROR: serial port does not exist: $PORT" >&2; exit 1; }
[[ "$BAUD" =~ ^[0-9]+$ ]] || { echo "ERROR: invalid baud rate: $BAUD" >&2; exit 2; }

PARTITION_TABLE_OFFSET=0x8000
PARTITION_TABLE_SIZE="$(stat -c '%s' "$EXPECTED_PARTITIONS")"
echo "==> Verifying the device partition table before writing"
python3 "$ESPTOOL_PY" --chip esp32 --port "$PORT" --baud "$BAUD" \
  --before default_reset --after no_reset read_flash \
  "$PARTITION_TABLE_OFFSET" "$PARTITION_TABLE_SIZE" "$DEVICE_PARTITIONS"
if ! cmp -s "$EXPECTED_PARTITIONS" "$DEVICE_PARTITIONS"; then
  echo "ERROR: device partition table does not match $PARTITION_CSV" >&2
  echo "       Nothing was written. Perform a full Arduino IDE upload with" >&2
  echo "       'No OTA (2MB APP/2MB SPIFFS)' before using this script." >&2
  exit 1
fi

echo "==> Writing firmware + LittleFS on $PORT"
python3 "$ESPTOOL_PY" --chip esp32 --port "$PORT" --baud "$BAUD" \
  --before default_reset --after hard_reset write_flash \
  "$APP_OFFSET" "$FIRMWARE" \
  "$SPIFFS_OFFSET" "$SPIFFS_IMAGE"
