#!/usr/bin/env bash
set -euo pipefail

# Build the minimal MIT Brotli decoder used by the DATA Worker. Obtain the
# Debian source with: (cd /tmp && apt-get source brotli)

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BROTLI_SOURCE_DIR="${BROTLI_SOURCE_DIR:-/tmp/brotli-1.0.9}"
OUTPUT_DIR="${1:-${ROOT_DIR}/data}"

[[ -f "${BROTLI_SOURCE_DIR}/c/dec/decode.c" ]] || {
  echo "ERROR: Brotli source not found: ${BROTLI_SOURCE_DIR}" >&2
  exit 1
}
command -v emcc >/dev/null || { echo "ERROR: emcc not found" >&2; exit 1; }
mkdir -p "$OUTPUT_DIR"

emcc "${BROTLI_SOURCE_DIR}"/c/common/*.c "${BROTLI_SOURCE_DIR}"/c/dec/*.c \
  -I "${BROTLI_SOURCE_DIR}/c/include" -Oz -flto \
  -s MODULARIZE=1 -s EXPORT_NAME=createJs8Brotli \
  -s ENVIRONMENT=web,worker -s FILESYSTEM=0 -s ALLOW_MEMORY_GROWTH=1 \
  -s INITIAL_MEMORY=4194304 -s TOTAL_STACK=65536 -s MALLOC=emmalloc \
  -s SUPPORT_LONGJMP=0 \
  -s EXPORTED_FUNCTIONS=_malloc,_free,_BrotliDecoderDecompress \
  -o "${OUTPUT_DIR}/js8-brotli.js"

# Emscripten 3.1.6 emits whitespace-only wrapper lines. Normalize them so a
# clean rebuild also passes Git's whitespace check.
sed -i 's/[[:space:]]\+$//' "${OUTPUT_DIR}/js8-brotli.js"

cp "${BROTLI_SOURCE_DIR}/LICENSE" "${OUTPUT_DIR}/BROTLI-LICENSE.txt"
echo "==> Brotli Worker decoder ready in ${OUTPUT_DIR}"
