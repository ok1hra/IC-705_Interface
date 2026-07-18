#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

OUTPUT_DIR="${ROOT_DIR}/build/gh-pages"
FIRMWARE_BIN="${ROOT_DIR}/IC-705_Interface.ino.esp32.bin"
DATA_DIR="${ROOT_DIR}/data"
SKETCH_FILE="${ROOT_DIR}/IC-705_Interface.ino"
GZIP_ASSETS_SCRIPT="${ROOT_DIR}/tools/gzip-assets.sh"

# no_ota partition scheme (No OTA — 2MB APP / 2MB SPIFFS)
PARTITIONS_CSV_NAME="no_ota"
# IMPORTANT: DIO, not QIO. These IC-705 interface boards ship a Zbit (0x5e) clone
# flash chip whose QIO reads are unreliable — a QIO bootloader makes the ROM loader
# read garbage after the first segment and the board never boots. DIO 80 MHz is
# stable. The bootloader's flash mode is baked in at elf2image time below (its
# SHA256 digest prevents patching the mode afterwards).
FLASH_MODE="dio"
FLASH_FREQ="80m"
FLASH_SIZE="4MB"

# Offsets from no_ota.csv
BOOTLOADER_OFFSET=0x1000
PARTITIONS_OFFSET=0x8000
BOOT_APP0_OFFSET=0xe000
APP_OFFSET=0x10000
SPIFFS_OFFSET=0x210000
SPIFFS_SIZE_DEC=$((0x1E0000))   # 1966080; exact no_ota.csv partition size

ESP32_CORE_ROOT="${ESP32_CORE_ROOT:-}"
BOOTLOADER_BIN="${BOOTLOADER_BIN:-}"
BOOTLOADER_ELF="${BOOTLOADER_ELF:-}"
BOOT_APP0_BIN="${BOOT_APP0_BIN:-}"
GEN_PART_BIN="${GEN_PART_BIN:-}"
ESPTOOL_BIN="${ESPTOOL_BIN:-}"
MKLITTLEFS_BIN="${MKLITTLEFS_BIN:-}"

DO_PUBLISH=0
PUBLISH_BRANCH="gh-pages"
PUBLISH_REMOTE="origin"
PUBLISH_MESSAGE=""

usage() {
  cat <<'EOF'
Usage: tools/gh-pages.sh [options]

Build a GitHub Pages web flasher for IC-705_Interface (blank ESP32 via USB).
Generates build/gh-pages/ with manifest.json and index.html for esp-web-tools.

Steps:
  1. Read firmware version from IC-705_Interface.ino
  2. Build LittleFS image from data/
  3. Generate partition table binary
  4. Create manifest.json and index.html
  5. Optionally publish to gh-pages branch

Options:
  --output-dir PATH      Output directory       (default: ./build/gh-pages)
  --firmware PATH        Firmware .bin file      (default: ./IC-705_Interface.ino.esp32.bin)
  --esp32-core PATH      ESP32 Arduino core root (auto-detected if not set)
  --bootloader PATH      Prebuilt bootloader .bin (skips ELF conversion)
  --bootloader-elf PATH  Bootloader ELF          (auto-detected; DIO .bin built from it)
  --boot-app0 PATH       boot_app0.bin           (auto-detected)
  --gen-part PATH        gen_esp32part.py        (auto-detected)
  --esptool PATH         esptool.py              (auto-detected)
  --mklittlefs PATH      mklittlefs binary       (auto-detected)
  --publish              Push build/gh-pages to gh-pages branch
  --branch NAME          Target Pages branch     (default: gh-pages)
  --remote NAME          Git remote to push to   (default: origin)
  --message TEXT         Publish commit message   (auto-generated if not set)
  -h, --help             Show this help

Examples:
  bash tools/gh-pages.sh
  bash tools/gh-pages.sh --publish
  bash tools/gh-pages.sh --publish --message "Release 20260509"
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)   OUTPUT_DIR="$2";          shift 2 ;;
    --firmware)     FIRMWARE_BIN="$2";        shift 2 ;;
    --esp32-core)   ESP32_CORE_ROOT="$2";     shift 2 ;;
    --bootloader)     BOOTLOADER_BIN="$2";    shift 2 ;;
    --bootloader-elf) BOOTLOADER_ELF="$2";    shift 2 ;;
    --boot-app0)    BOOT_APP0_BIN="$2";       shift 2 ;;
    --gen-part)     GEN_PART_BIN="$2";        shift 2 ;;
    --esptool)      ESPTOOL_BIN="$2";         shift 2 ;;
    --mklittlefs)   MKLITTLEFS_BIN="$2";      shift 2 ;;
    --publish)      DO_PUBLISH=1;             shift ;;
    --branch)       PUBLISH_BRANCH="$2";      shift 2 ;;
    --remote)       PUBLISH_REMOTE="$2";      shift 2 ;;
    --message)      PUBLISH_MESSAGE="$2";     shift 2 ;;
    -h|--help)      usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

# ---------------------------------------------------------------------------
# Auto-detect ESP32 Arduino core root
# ---------------------------------------------------------------------------

detect_esp32_core_root() {
  local candidates=()
  [[ -n "${ESP32_CORE_ROOT:-}" ]] && candidates+=("${ESP32_CORE_ROOT}")
  [[ -n "${HOME:-}" ]] && candidates+=(
    "${HOME}/Arduino/hardware/espressif/esp32"
    "${HOME}/.arduino15/packages/esp32/hardware/esp32"
  )

  # Marker file present inside the core across versions. (esptool.py used to
  # live here too, but core 2.x moved it to a separate package tool — see below.)
  local marker="tools/gen_esp32part.py"
  local dir version_dir
  for dir in "${candidates[@]}"; do
    if [[ -f "${dir}/${marker}" ]]; then
      echo "$dir"; return 0
    fi
    if [[ -d "$dir" ]]; then
      version_dir="$(find "$dir" -mindepth 1 -maxdepth 1 -type d | sort -V | tail -n 1)"
      if [[ -n "$version_dir" && -f "${version_dir}/${marker}" ]]; then
        echo "$version_dir"; return 0
      fi
    fi
  done
  return 1
}

# In core 2.x, esptool.py and mklittlefs ship as separate package tools at
#   packages/esp32/tools/<subdir>/<version>/<file>
# The core lives at packages/esp32/hardware/esp32/<ver>, so the package tools
# dir is three levels up from the core root.
find_pkg_tool() {
  local subdir="$1" file="$2"
  local pkg_tools="${ESP32_CORE_ROOT}/../../../tools/${subdir}"
  [[ -d "$pkg_tools" ]] || return 1
  local ver_dir
  ver_dir="$(find "$pkg_tools" -mindepth 1 -maxdepth 1 -type d | sort -V | tail -n 1)"
  [[ -n "$ver_dir" && -f "${ver_dir}/${file}" ]] && { echo "${ver_dir}/${file}"; return 0; }
  return 1
}

if [[ -z "$ESP32_CORE_ROOT" ]]; then
  ESP32_CORE_ROOT="$(detect_esp32_core_root || true)"
fi

# Core 2.0.14 ships only the bootloader ELF (not a .bin) in tools/sdk/esp32/bin.
# When no prebuilt --bootloader .bin is given, derive it from the matching ELF via
# elf2image (below) so the DIO flash mode is set in the header.
[[ -z "$BOOTLOADER_BIN" && -z "$BOOTLOADER_ELF" && -n "$ESP32_CORE_ROOT" ]] && \
  BOOTLOADER_ELF="${ESP32_CORE_ROOT}/tools/sdk/esp32/bin/bootloader_${FLASH_MODE}_${FLASH_FREQ}.elf"
[[ -z "$BOOT_APP0_BIN" && -n "$ESP32_CORE_ROOT" ]] && \
  BOOT_APP0_BIN="${ESP32_CORE_ROOT}/tools/partitions/boot_app0.bin"
[[ -z "$GEN_PART_BIN" && -n "$ESP32_CORE_ROOT" ]] && \
  GEN_PART_BIN="${ESP32_CORE_ROOT}/tools/gen_esp32part.py"
if [[ -z "$ESPTOOL_BIN" && -n "$ESP32_CORE_ROOT" ]]; then
  if [[ -f "${ESP32_CORE_ROOT}/tools/esptool.py" ]]; then
    ESPTOOL_BIN="${ESP32_CORE_ROOT}/tools/esptool.py"
  else
    ESPTOOL_BIN="$(find_pkg_tool esptool_py esptool.py || true)"
  fi
fi
if [[ -z "$MKLITTLEFS_BIN" && -n "$ESP32_CORE_ROOT" ]]; then
  if [[ -f "${ESP32_CORE_ROOT}/tools/mklittlefs/mklittlefs" ]]; then
    MKLITTLEFS_BIN="${ESP32_CORE_ROOT}/tools/mklittlefs/mklittlefs"
  else
    MKLITTLEFS_BIN="$(find_pkg_tool mklittlefs mklittlefs || true)"
  fi
fi

require_file() {
  local path="$1" label="$2"
  if [[ ! -f "$path" ]]; then
    echo "ERROR: $label not found: $path" >&2
    exit 1
  fi
}

require_file "$FIRMWARE_BIN"   "Firmware binary"
require_file "$SKETCH_FILE"    "Sketch file"
require_file "$BOOT_APP0_BIN"  "boot_app0.bin"
require_file "$GEN_PART_BIN"   "gen_esp32part.py"
require_file "$ESPTOOL_BIN"    "esptool.py"
require_file "$MKLITTLEFS_BIN" "mklittlefs"
# Bootloader: either a prebuilt --bootloader .bin, or the core ELF we convert below.
if [[ -n "$BOOTLOADER_BIN" ]]; then
  require_file "$BOOTLOADER_BIN" "Bootloader binary"
else
  require_file "$BOOTLOADER_ELF" "Bootloader ELF"
fi

# Refuse a release built from a stale Arduino export.
while IFS= read -r -d '' source; do
  if [[ "$source" -nt "$FIRMWARE_BIN" ]]; then
    echo "ERROR: exported firmware is stale; newer source: $source" >&2
    echo "       Run Sketch -> Export Compiled Binary again." >&2
    exit 1
  fi
done < <(find "$ROOT_DIR" -maxdepth 1 -type f \
  \( -name '*.ino' -o -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \) -print0)

# A Pages release must always contain the complete DATA/JS8 runtime.
for asset in data.html data.css data.js dxcc.js js8-adapter.js js8-aud1.js \
  js8-audio.js js8-brotli.js js8-brotli.wasm js8-core.js js8-core.wasm \
  js8-decoder.js js8-decoder.wasm js8-jsc.bin js8-presets.js js8-protocol.js \
  js8-settings.js js8-timebase.js js8-tx.js js8-worker-runtime.js js8-worker.js \
  BROTLI-LICENSE.txt THIRD-PARTY-NOTICES.txt; do
  require_file "${DATA_DIR}/${asset}" "DATA/JS8 asset ${asset}"
done

# ---------------------------------------------------------------------------
# Read firmware version
# ---------------------------------------------------------------------------

FW_REV="$(
  awk '/#define REV / { print $3; exit }' "$SKETCH_FILE"
)"

if [[ -z "$FW_REV" ]]; then
  echo "ERROR: Could not read REV from $SKETCH_FILE" >&2
  exit 1
fi

echo "==> Firmware REV: $FW_REV"

# ---------------------------------------------------------------------------
# Locate partition CSV
# ---------------------------------------------------------------------------

PARTITIONS_CSV="${ESP32_CORE_ROOT}/tools/partitions/${PARTITIONS_CSV_NAME}.csv"
if [[ ! -f "$PARTITIONS_CSV" ]]; then
  echo "ERROR: Partition CSV not found: $PARTITIONS_CSV" >&2
  exit 1
fi

OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"
ROOT_REAL="$(realpath "$ROOT_DIR")"
case "${OUTPUT_DIR}/" in
  "${ROOT_REAL}/build/"*|/tmp/*) ;;
  *) echo "ERROR: --output-dir must be below ${ROOT_REAL}/build or /tmp: ${OUTPUT_DIR}" >&2; exit 1 ;;
esac
mkdir -p "$OUTPUT_DIR"
find "$OUTPUT_DIR" -mindepth 1 -depth -delete

# ---------------------------------------------------------------------------
# Bootloader binary (from ELF, with DIO flash mode baked into the header)
# ---------------------------------------------------------------------------

if [[ -z "$BOOTLOADER_BIN" ]]; then
  echo "==> Generating ${FLASH_MODE} bootloader from ELF"
  BOOTLOADER_BIN="${OUTPUT_DIR}/bootloader.bin"
  python3 "$ESPTOOL_BIN" --chip esp32 elf2image \
    --flash_mode "$FLASH_MODE" --flash_freq "$FLASH_FREQ" --flash_size "$FLASH_SIZE" \
    -o "$BOOTLOADER_BIN" "$BOOTLOADER_ELF"
fi

# ---------------------------------------------------------------------------
# Build partition table binary
# ---------------------------------------------------------------------------

echo "==> Generating partition table"
PARTITIONS_BIN="${OUTPUT_DIR}/partitions.bin"
python3 "$GEN_PART_BIN" "$PARTITIONS_CSV" "$PARTITIONS_BIN"

# ---------------------------------------------------------------------------
# Build LittleFS image from data/
# ---------------------------------------------------------------------------

SPIFFS_BIN="${OUTPUT_DIR}/spiffs.bin"
SPIFFS_DATA_DIR="${OUTPUT_DIR}/spiffs-data"

if [[ ! -x "${ROOT_DIR}/tools/prepare-spiffs-tree.sh" ]]; then
  echo "ERROR: filesystem staging helper not found" >&2
  exit 1
fi
"${ROOT_DIR}/tools/prepare-spiffs-tree.sh" "$DATA_DIR" "$SPIFFS_DATA_DIR"
staging_bytes="$(du -sb "$SPIFFS_DATA_DIR" | awk '{print $1}')"
runtime_reserve_bytes=$((256 * 1024))
metadata_budget_bytes=$((64 * 1024))
max_payload_bytes=$((SPIFFS_SIZE_DEC - runtime_reserve_bytes - metadata_budget_bytes))
[[ "$staging_bytes" -le "$max_payload_bytes" ]] || {
  echo "ERROR: LittleFS deployment leaves less than the required 256 KiB runtime reserve." >&2
  echo "       Payload: $staging_bytes B; limit: $max_payload_bytes B; partition: $SPIFFS_SIZE_DEC B." >&2
  exit 1
}
echo "==> Building LittleFS image from data/ (payload $staging_bytes / $max_payload_bytes B)"
"$MKLITTLEFS_BIN" -c "$SPIFFS_DATA_DIR" -b 4096 -p 256 -s "$SPIFFS_SIZE_DEC" "$SPIFFS_BIN"

# ---------------------------------------------------------------------------
# Copy binaries
# ---------------------------------------------------------------------------

echo "==> Copying binaries"
# BOOTLOADER_BIN may already be ${OUTPUT_DIR}/bootloader.bin (generated above).
[[ "$BOOTLOADER_BIN" -ef "${OUTPUT_DIR}/bootloader.bin" ]] || \
  cp "$BOOTLOADER_BIN" "${OUTPUT_DIR}/bootloader.bin"
cp "$BOOT_APP0_BIN"  "${OUTPUT_DIR}/boot_app0.bin"
cp "$FIRMWARE_BIN"   "${OUTPUT_DIR}/firmware.bin"

# ---------------------------------------------------------------------------
# Build merged binary (for manual esptool flashing)
# ---------------------------------------------------------------------------

echo "==> Building merged binary"
MERGE_ARGS=(
  --chip esp32 merge_bin
  -o "${OUTPUT_DIR}/ic705-${FW_REV}-full.bin"
  --flash_mode "$FLASH_MODE"
  --flash_freq "$FLASH_FREQ"
  --flash_size "$FLASH_SIZE"
  "${BOOTLOADER_OFFSET}" "${OUTPUT_DIR}/bootloader.bin"
  "${PARTITIONS_OFFSET}" "${OUTPUT_DIR}/partitions.bin"
  "${BOOT_APP0_OFFSET}"  "${OUTPUT_DIR}/boot_app0.bin"
  "${APP_OFFSET}"        "${OUTPUT_DIR}/firmware.bin"
)
if [[ -n "$SPIFFS_BIN" ]]; then
  MERGE_ARGS+=("${SPIFFS_OFFSET}" "$SPIFFS_BIN")
fi
python3 "$ESPTOOL_BIN" "${MERGE_ARGS[@]}"

# ---------------------------------------------------------------------------
# Generate manifest.json
# ---------------------------------------------------------------------------

echo "==> Generating manifest.json"

PARTS_JSON="        { \"path\": \"bootloader.bin\",  \"offset\": $((BOOTLOADER_OFFSET)) },
        { \"path\": \"partitions.bin\",  \"offset\": $((PARTITIONS_OFFSET)) },
        { \"path\": \"boot_app0.bin\",   \"offset\": $((BOOT_APP0_OFFSET)) },
        { \"path\": \"firmware.bin\",    \"offset\": $((APP_OFFSET)) }"

if [[ -n "$SPIFFS_BIN" ]]; then
  PARTS_JSON="${PARTS_JSON},
        { \"path\": \"spiffs.bin\",      \"offset\": $((SPIFFS_OFFSET)) }"
fi

cat > "${OUTPUT_DIR}/manifest.json" <<EOF
{
  "name": "IC-705 Interface",
  "version": "${FW_REV}",
  "new_install_prompt_erase": true,
  "new_install_improv_wait_time": 0,
  "builds": [
    {
      "chipFamily": "ESP32",
      "parts": [
${PARTS_JSON}
      ]
    }
  ]
}
EOF

# ---------------------------------------------------------------------------
# Generate index.html
# ---------------------------------------------------------------------------

echo "==> Generating index.html"

cat > "${OUTPUT_DIR}/index.html" <<EOF
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
  <meta http-equiv="Pragma" content="no-cache">
  <meta http-equiv="Expires" content="0">
  <title>IC-705 Interface — firmware installer</title>
  <script type="module" src="https://unpkg.com/esp-web-tools@10.4.0/dist/web/install-button.js?module"></script>
  <style>
    :root {
      --accent: #3b82f6;
      --accent-glow: rgba(59, 130, 246, 0.22);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Trebuchet MS", "Segoe UI", sans-serif;
      color: #f1f5f9;
      background:
        radial-gradient(circle at top right, rgba(59, 130, 246, 0.25), transparent 30rem),
        linear-gradient(150deg, #0f172a 0%, #111827 50%, #1e293b 100%);
      min-height: 100vh;
    }
    main {
      width: min(44rem, calc(100% - 2rem));
      margin: 0 auto;
      padding: 3.5rem 0 5rem;
    }
    .card {
      background: rgba(15, 23, 42, 0.78);
      border: 1px solid rgba(59, 130, 246, 0.28);
      border-radius: 1.25rem;
      padding: 2rem 2rem 2.25rem;
      box-shadow: 0 2rem 4rem rgba(0, 0, 0, 0.3);
      backdrop-filter: blur(14px);
    }
    h1 {
      margin: 0 0 0.5rem;
      font-size: clamp(1.8rem, 4vw, 2.8rem);
      line-height: 1.1;
    }
    .subtitle {
      margin: 0 0 1.75rem;
      color: #94a3b8;
      font-size: 1rem;
    }
    .subtitle code {
      background: rgba(148, 163, 184, 0.15);
      padding: 0.1rem 0.35rem;
      border-radius: 0.3rem;
      color: #93c5fd;
    }
    .divider {
      border: none;
      border-top: 1px solid rgba(59, 130, 246, 0.2);
      margin: 1.5rem 0;
    }
    h2 {
      margin: 0 0 0.6rem;
      font-size: 1.15rem;
      color: #93c5fd;
    }
    p, li {
      font-size: 0.97rem;
      line-height: 1.65;
      color: #cbd5e1;
    }
    ul, ol {
      margin: 0.5rem 0 0;
      padding-left: 1.35rem;
    }
    li { margin-bottom: 0.35rem; }
    .highlight { color: #fde68a; }
    .warn-box {
      background: rgba(220, 38, 38, 0.15);
      border: 2px solid #dc2626;
      border-radius: 0.75rem;
      padding: 1rem 1.25rem;
      margin-bottom: 1.5rem;
      color: #fca5a5;
    }
    .warn-box strong { color: #f87171; }
    .warn-box p { margin: 0; color: #fca5a5; font-size: 0.97rem; line-height: 1.6; }
    .warn-box p + p { margin-top: 0.4rem; }
    .hardware-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.65rem;
      margin: 0.85rem 0 0;
    }
    .hardware-item {
      margin: 0;
      padding: 0.8rem 0.9rem;
      border: 1px solid rgba(96, 165, 250, 0.22);
      border-radius: 0.65rem;
      background: rgba(30, 41, 59, 0.55);
    }
    .hardware-item dt {
      margin-bottom: 0.25rem;
      color: #93c5fd;
      font-size: 0.78rem;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
    }
    .hardware-item dd {
      margin: 0;
      color: #e2e8f0;
      font-size: 0.93rem;
      line-height: 1.45;
    }
    .compatibility-note { margin: 0.8rem 0 0; }
    @media (max-width: 34rem) {
      .hardware-grid { grid-template-columns: 1fr; }
    }
    code {
      background: rgba(148, 163, 184, 0.15);
      padding: 0.1rem 0.35rem;
      border-radius: 0.3rem;
    }
    .cta {
      margin-top: 1.5rem;
    }
    esp-web-install-button {
      --esp-tools-button-color: var(--accent);
      --esp-tools-button-text-color: #fff;
      --esp-tools-button-border-radius: 999px;
    }
    .muted { color: #64748b; font-size: 0.88rem; margin-top: 0.75rem; }
    a { color: #60a5fa; }
  </style>
</head>
<body>
  <main>
    <div class="card">
      <h1>IC-705 Interface</h1>
      <p class="subtitle">
        Firmware installer &mdash; version <code>${FW_REV}</code> &nbsp;&bull;&nbsp;
        <a href="https://github.com/ok1hra/IC-705_Interface" target="_blank">GitHub</a>
      </p>

      <hr class="divider">

      <div class="warn-box">
        <p><strong>⚠ Back up your configuration before upgrading firmware!</strong></p>
        <p>
          Flashing new firmware <strong>erases your entire configuration</strong> — all radio settings,
          TRX addresses, contest logs, and setup options will be lost.
          Before proceeding, open the <strong>Setup page</strong> on your device and
          download a configuration backup.
        </p>
      </div>

      <section aria-labelledby="hardware-title">
        <h2 id="hardware-title">Required hardware</h2>
        <dl class="hardware-grid">
          <div class="hardware-item">
            <dt>Processor</dt>
            <dd>Original Espressif ESP32 (Xtensa LX6), compatible with the <strong>ESP32 Dev Module</strong> target.</dd>
          </div>
          <div class="hardware-item">
            <dt>Flash memory</dt>
            <dd><strong>4 MB minimum</strong>, configured for DIO mode at 80 MHz.</dd>
          </div>
          <div class="hardware-item">
            <dt>RAM</dt>
            <dd>Standard ESP32 internal SRAM (520 KB). External PSRAM is not required.</dd>
          </div>
          <div class="hardware-item">
            <dt>Flash layout</dt>
            <dd>No OTA: 2 MB application and approximately 2 MB LittleFS filesystem.</dd>
          </div>
        </dl>
        <p class="muted compatibility-note">
          This firmware image targets the original ESP32. It is not compatible with ESP32-C3,
          ESP32-S2, ESP32-S3, or other ESP32 chip families.
        </p>
      </section>

      <hr class="divider">

      <h2>Flash firmware via USB</h2>
      <p>
        Open this page in <strong>Google Chrome</strong> or <strong>Microsoft Edge</strong>
        (Web Serial is not supported in Firefox or Safari).
        Connect the ESP32 to your computer via USB, then click the button below.
      </p>
      <ul>
        <li>After connecting, select the correct <code>CP210x</code> / <code>CH340</code> / <code>JTAG</code> serial device.</li>
        <li>Choose <strong>Install IC-705 Interface</strong> and follow the prompts.</li>
      </ul>

      <div class="cta">
        <esp-web-install-button manifest="manifest.json?v=${FW_REV}" baudrate="9600"></esp-web-install-button>
      </div>

      <hr class="divider">

      <h2>Open the interface after flashing</h2>
      <ol>
        <li>On its first boot, the device creates the WiFi network <code>IC705-if</code>. Connect to it using password <code>remoteqth</code>.</li>
        <li>Open <code>http://192.168.4.1/setup</code> in your browser. You can also try <code>http://ic705.local/setup</code>; some phones open the setup portal automatically.</li>
        <li>Configure your normal WiFi and the IC-705 <strong>TRX1 LAN</strong> address, username, and password. Select <strong>Save &amp; Restart</strong>, then reconnect your phone or computer to the normal WiFi.</li>
        <li>Find the interface's new IP address in your router's DHCP client list, or open the installer serial console at <code>9600 baud</code> and press <strong>Reset Device</strong> to read it from the boot log.</li>
        <li>Open <code>http://&lt;assigned-IP&gt;/</code> or try <code>http://ic705.local/</code>. Bookmark the working address.</li>
      </ol>

      <p class="muted">
        Flashes: bootloader, partition table, boot_app0, firmware$(if [[ -n "$SPIFFS_BIN" ]]; then echo ", LittleFS filesystem"; fi).
        For manual flashing use <code>ic705-${FW_REV}-full.bin</code> at offset <code>0x0</code>
        with <code>esptool.py</code>.
      </p>
    </div>
  </main>
</body>
</html>
EOF

touch "${OUTPUT_DIR}/.nojekyll"

echo ""
echo "==> Build complete: ${OUTPUT_DIR}"
echo "    Firmware REV : ${FW_REV}"
echo "    Partitions   : ${PARTITIONS_CSV_NAME} (no OTA — 2MB APP / 2MB SPIFFS)"
  echo "    Flash mode   : ${FLASH_MODE} ${FLASH_FREQ} (DIO required — Zbit clone flash)"
if [[ -n "$SPIFFS_BIN" ]]; then
  echo "    LittleFS     : included"
else
  echo "    LittleFS     : skipped"
fi
echo ""

# ---------------------------------------------------------------------------
# Publish to gh-pages
# ---------------------------------------------------------------------------

if [[ "$DO_PUBLISH" -eq 0 ]]; then
  echo "To publish to GitHub Pages, run:"
  echo "  bash tools/gh-pages.sh --publish"
  exit 0
fi

echo "==> Publishing to ${PUBLISH_REMOTE}/${PUBLISH_BRANCH}"

if ! git -C "$ROOT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "ERROR: Not a git repository: $ROOT_DIR" >&2; exit 1
fi
if ! git -C "$ROOT_DIR" remote get-url "$PUBLISH_REMOTE" >/dev/null 2>&1; then
  echo "ERROR: Git remote not found: $PUBLISH_REMOTE" >&2; exit 1
fi

[[ -z "$PUBLISH_MESSAGE" ]] && \
  PUBLISH_MESSAGE="Publish IC-705 Interface firmware ${FW_REV} — $(date -u +%Y-%m-%dT%H:%M:%SZ)"

TMP_DIR="$(mktemp -d)"
cleanup() { rm -rf "$TMP_DIR"; }
trap cleanup EXIT

git init "$TMP_DIR" >/dev/null
git -C "$TMP_DIR" remote add "$PUBLISH_REMOTE" \
  "$(git -C "$ROOT_DIR" remote get-url "$PUBLISH_REMOTE")"

if git -C "$TMP_DIR" ls-remote --exit-code --heads \
    "$PUBLISH_REMOTE" "$PUBLISH_BRANCH" >/dev/null 2>&1; then
  git -C "$TMP_DIR" fetch --depth 1 "$PUBLISH_REMOTE" "$PUBLISH_BRANCH"
  git -C "$TMP_DIR" checkout -B "$PUBLISH_BRANCH" FETCH_HEAD
else
  git -C "$TMP_DIR" checkout --orphan "$PUBLISH_BRANCH"
fi

find "$TMP_DIR" -mindepth 1 -maxdepth 1 ! -name '.git' -exec rm -rf {} +
for release_file in .nojekyll index.html manifest.json bootloader.bin partitions.bin \
  boot_app0.bin firmware.bin spiffs.bin "ic705-${FW_REV}-full.bin"; do
  require_file "${OUTPUT_DIR}/${release_file}" "release artifact ${release_file}"
  cp "${OUTPUT_DIR}/${release_file}" "$TMP_DIR/${release_file}"
done

git -C "$TMP_DIR" add --all

if git -C "$TMP_DIR" diff --cached --quiet; then
  echo "No changes to publish."
  exit 0
fi

GIT_NAME="$(git -C "$ROOT_DIR" config user.name  2>/dev/null || echo "IC-705 Publisher")"
GIT_EMAIL="$(git -C "$ROOT_DIR" config user.email 2>/dev/null || echo "publish@example.invalid")"
git -C "$TMP_DIR" config user.name  "$GIT_NAME"
git -C "$TMP_DIR" config user.email "$GIT_EMAIL"

git -C "$TMP_DIR" commit -m "$PUBLISH_MESSAGE"

echo "Pushing to ${PUBLISH_REMOTE}/${PUBLISH_BRANCH}..."
git -C "$TMP_DIR" push "$PUBLISH_REMOTE" "$PUBLISH_BRANCH"

echo ""
echo "==> Published successfully."
echo "    Enable GitHub Pages on branch '${PUBLISH_BRANCH}' (root) in repository settings."
echo "    URL: https://ok1hra.github.io/IC-705_Interface/"
