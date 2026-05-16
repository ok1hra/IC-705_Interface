#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

OUTPUT_DIR="${ROOT_DIR}/build/gh-pages"
FIRMWARE_BIN="${ROOT_DIR}/IC-705_Interface.ino.esp32.bin"
DATA_DIR="${ROOT_DIR}/data"
SKETCH_FILE="${ROOT_DIR}/IC-705_Interface.ino"

# no_ota partition scheme (No OTA — 2MB APP / 2MB SPIFFS)
PARTITIONS_CSV_NAME="no_ota"
FLASH_MODE="qio"
FLASH_FREQ="80m"
FLASH_SIZE="4MB"

# Offsets from no_ota.csv
BOOTLOADER_OFFSET=0x1000
PARTITIONS_OFFSET=0x8000
BOOT_APP0_OFFSET=0xe000
APP_OFFSET=0x10000
SPIFFS_OFFSET=0x210000
SPIFFS_SIZE_DEC=$((0x1F0000))   # 1966080

ESP32_CORE_ROOT="${ESP32_CORE_ROOT:-}"
BOOTLOADER_BIN="${BOOTLOADER_BIN:-}"
BOOT_APP0_BIN="${BOOT_APP0_BIN:-}"
GEN_PART_BIN="${GEN_PART_BIN:-}"
ESPTOOL_BIN="${ESPTOOL_BIN:-}"
MKSPIFFS_BIN="${MKSPIFFS_BIN:-}"

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
  2. Build SPIFFS image from data/
  3. Generate partition table binary
  4. Create manifest.json and index.html
  5. Optionally publish to gh-pages branch

Options:
  --output-dir PATH      Output directory       (default: ./build/gh-pages)
  --firmware PATH        Firmware .bin file      (default: ./IC-705_Interface.ino.esp32.bin)
  --esp32-core PATH      ESP32 Arduino core root (auto-detected if not set)
  --bootloader PATH      Bootloader binary       (auto-detected)
  --boot-app0 PATH       boot_app0.bin           (auto-detected)
  --gen-part PATH        gen_esp32part.py        (auto-detected)
  --esptool PATH         esptool.py              (auto-detected)
  --mkspiffs PATH        mkspiffs binary         (auto-detected)
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
    --bootloader)   BOOTLOADER_BIN="$2";      shift 2 ;;
    --boot-app0)    BOOT_APP0_BIN="$2";       shift 2 ;;
    --gen-part)     GEN_PART_BIN="$2";        shift 2 ;;
    --esptool)      ESPTOOL_BIN="$2";         shift 2 ;;
    --mkspiffs)     MKSPIFFS_BIN="$2";        shift 2 ;;
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

  local dir version_dir
  for dir in "${candidates[@]}"; do
    if [[ -f "${dir}/tools/esptool.py" ]]; then
      echo "$dir"; return 0
    fi
    if [[ -d "$dir" ]]; then
      version_dir="$(find "$dir" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
      if [[ -n "$version_dir" && -f "${version_dir}/tools/esptool.py" ]]; then
        echo "$version_dir"; return 0
      fi
    fi
  done
  return 1
}

if [[ -z "$ESP32_CORE_ROOT" ]]; then
  ESP32_CORE_ROOT="$(detect_esp32_core_root || true)"
fi

[[ -z "$BOOTLOADER_BIN" && -n "$ESP32_CORE_ROOT" ]] && \
  BOOTLOADER_BIN="${ESP32_CORE_ROOT}/tools/sdk/esp32/bin/bootloader_${FLASH_MODE}_${FLASH_FREQ}.bin"
[[ -z "$BOOT_APP0_BIN" && -n "$ESP32_CORE_ROOT" ]] && \
  BOOT_APP0_BIN="${ESP32_CORE_ROOT}/tools/partitions/boot_app0.bin"
[[ -z "$GEN_PART_BIN" && -n "$ESP32_CORE_ROOT" ]] && \
  GEN_PART_BIN="${ESP32_CORE_ROOT}/tools/gen_esp32part.py"
[[ -z "$ESPTOOL_BIN" && -n "$ESP32_CORE_ROOT" ]] && \
  ESPTOOL_BIN="${ESP32_CORE_ROOT}/tools/esptool.py"
[[ -z "$MKSPIFFS_BIN" && -n "$ESP32_CORE_ROOT" ]] && \
  MKSPIFFS_BIN="${ESP32_CORE_ROOT}/tools/mkspiffs/mkspiffs"

require_file() {
  local path="$1" label="$2"
  if [[ ! -f "$path" ]]; then
    echo "ERROR: $label not found: $path" >&2
    exit 1
  fi
}

require_file "$FIRMWARE_BIN"   "Firmware binary"
require_file "$SKETCH_FILE"    "Sketch file"
require_file "$BOOTLOADER_BIN" "Bootloader binary"
require_file "$BOOT_APP0_BIN"  "boot_app0.bin"
require_file "$GEN_PART_BIN"   "gen_esp32part.py"
require_file "$ESPTOOL_BIN"    "esptool.py"

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

mkdir -p "$OUTPUT_DIR"

# ---------------------------------------------------------------------------
# Build partition table binary
# ---------------------------------------------------------------------------

echo "==> Generating partition table"
PARTITIONS_BIN="${OUTPUT_DIR}/partitions.bin"
python3 "$GEN_PART_BIN" "$PARTITIONS_CSV" "$PARTITIONS_BIN"

# ---------------------------------------------------------------------------
# Build SPIFFS image from data/
# ---------------------------------------------------------------------------

SPIFFS_BIN="${OUTPUT_DIR}/spiffs.bin"

if [[ -d "$DATA_DIR" && -x "$MKSPIFFS_BIN" ]]; then
  echo "==> Building SPIFFS image from data/"
  "$MKSPIFFS_BIN" -c "$DATA_DIR" -b 4096 -p 256 -s "$SPIFFS_SIZE_DEC" "$SPIFFS_BIN"
else
  if [[ ! -d "$DATA_DIR" ]]; then
    echo "WARN: data/ directory not found — skipping SPIFFS" >&2
  else
    echo "WARN: mkspiffs not found at $MKSPIFFS_BIN — skipping SPIFFS" >&2
  fi
  SPIFFS_BIN=""
fi

# ---------------------------------------------------------------------------
# Copy binaries
# ---------------------------------------------------------------------------

echo "==> Copying binaries"
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
  <script type="module" src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js?module"></script>
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
    ul {
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

      <h2>Flash firmware via USB</h2>
      <p>
        Open this page in <strong>Google Chrome</strong> or <strong>Microsoft Edge</strong>
        (Web Serial is not supported in Firefox or Safari).
        Connect the ESP32 to your computer via USB, then click the button below.
      </p>
      <ul>
        <li>After connecting, select the correct <code>CP210x</code> / <code>CH340</code> / <code>JTAG</code> serial device.</li>
        <li>Choose <strong>Install IC-705 Interface</strong> and follow the prompts.</li>
        <li>
          After flashing, open the serial console and press <strong>Reset Device</strong>
          to see the boot log with the assigned <strong>IP address</strong>.
        </li>
      </ul>

      <div class="cta">
        <esp-web-install-button manifest="manifest.json" baudrate="9600"></esp-web-install-button>
      </div>

      <p class="muted">
        Flashes: bootloader, partition table, boot_app0, firmware$(if [[ -n "$SPIFFS_BIN" ]]; then echo ", SPIFFS filesystem"; fi).
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
if [[ -n "$SPIFFS_BIN" ]]; then
  echo "    SPIFFS       : included"
else
  echo "    SPIFFS       : skipped"
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
cp -R "${OUTPUT_DIR}/." "$TMP_DIR/"

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
