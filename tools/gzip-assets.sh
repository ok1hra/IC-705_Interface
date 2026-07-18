#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_DIR="${1:-${ROOT_DIR}/data}"

if [[ ! -d "$DATA_DIR" ]]; then
  echo "ERROR: data directory not found: $DATA_DIR" >&2
  exit 1
fi

if ! command -v 7z >/dev/null 2>&1; then
  echo "ERROR: 7z not found (install package p7zip-full)" >&2
  exit 1
fi

echo "==> Generating gzip web assets in ${DATA_DIR}"

find "$DATA_DIR" -type f \
  \( -name '*.html' -o -name '*.css' -o -name '*.js' -o -name '*.wasm' -o -name '*.bin' -o -name '*.txt' \) \
  -print0 | sort -z | while IFS= read -r -d '' file; do
    case "$(basename "$file")" in
      js8-jsc.bin|js8-decoder.wasm) continue ;;
    esac
    input="$file"
    if [[ "$file" == *.js && -f "${file}.min" ]]; then
      manifest="${DATA_DIR}/.minified-js.sha256"
      expected="$(awk -v name="$(basename "$file")" '$2 == name {print $1; exit}' "$manifest" 2>/dev/null || true)"
      actual="$(sha256sum "$file" | awk '{print $1}')"
      if [[ -z "$expected" || "$actual" != "$expected" ]]; then
        echo "ERROR: stale minified asset: ${file}.min" >&2
        echo "       Run tools/minify-spiffs-js.sh" >&2
        exit 1
      fi
      input="${file}.min"
    fi
    rm -f "${file}.gz"
    7z a -tgzip -mx=9 -mfb=258 -mpass=15 "${file}.gz" "$input" >/dev/null
  done

# These two large assets are fetched as opaque Brotli data and expanded in the
# Worker. Keeping stale gzip copies would exceed the SPIFFS working limit.
find "$DATA_DIR" -maxdepth 1 -type f \
  \( -name 'js8-jsc.bin.gz' -o -name 'js8-decoder.wasm.gz' \) -delete

# Remove orphaned generated files when a source asset was renamed/deleted.
find "$DATA_DIR" -type f -name '*.gz' -print0 | while IFS= read -r -d '' gz; do
  src="${gz%.gz}"
  if [[ ! -f "$src" ]]; then
    echo "    removing stale $(basename "$gz")"
    rm -f "$gz"
  fi
done

# Brotli copies are retained as optional development artifacts. Deployment
# prefers gzip because browsers do not advertise Brotli over plain device HTTP.
if command -v node >/dev/null 2>&1; then
  node "${ROOT_DIR}/tools/brotli-js8-assets.js" "$DATA_DIR"
else
  echo "ERROR: node is required to create Brotli JS8 assets" >&2
  exit 1
fi

echo "==> Gzip assets ready"
