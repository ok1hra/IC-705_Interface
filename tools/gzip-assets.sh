#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_DIR="${1:-${ROOT_DIR}/data}"

if [[ ! -d "$DATA_DIR" ]]; then
  echo "ERROR: data directory not found: $DATA_DIR" >&2
  exit 1
fi

if ! command -v gzip >/dev/null 2>&1; then
  echo "ERROR: gzip not found in PATH" >&2
  exit 1
fi

echo "==> Generating gzip web assets in ${DATA_DIR}"

find "$DATA_DIR" -maxdepth 1 -type f \
  \( -name '*.html' -o -name '*.css' -o -name '*.js' \) \
  -print0 | sort -z | while IFS= read -r -d '' file; do
    gzip -kf "$file"
  done

# Remove orphaned generated files when a source asset was renamed/deleted.
find "$DATA_DIR" -maxdepth 1 -type f -name '*.gz' -print0 | while IFS= read -r -d '' gz; do
  src="${gz%.gz}"
  if [[ ! -f "$src" ]]; then
    echo "    removing stale $(basename "$gz")"
    rm -f "$gz"
  fi
done

echo "==> Gzip assets ready"
