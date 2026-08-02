#!/usr/bin/env bash
# Are the sketch, the page sources and the *served* (compressed) assets all the
# same generation?
#
# Worth its own script because git cannot answer it: data/*.br and most data/*.gz
# are gitignored, so a branch switch reverts the sources and leaves the compressed
# copies behind -- and the firmware serves .br in preference to everything else.
# The result is old firmware serving new pages, with nothing in `git status` to
# show for it.
set -uo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR" || exit 1

SKETCH="$(ls -1 *.ino 2>/dev/null | head -1)"
host="$(grep -oP 'deviceHostname = "\K[^"]+' "$SKETCH" 2>/dev/null)"
echo "branch      $(git rev-parse --abbrev-ref HEAD)  ($(git log --oneline -1))"
echo "sketch      $SKETCH  hostname=${host:-?}"

# The old identity must not survive anywhere that is actually served. The dead
# Bluetooth-era identifiers are expected and excluded by name.
stale=0
for f in data/*.gz; do
  zcat "$f" 2>/dev/null | grep -q "ic705\|IC-705 Setup\|IC-705 IP interface" && { echo "STALE  $f"; stale=1; }
done
if command -v brotli >/dev/null 2>&1; then
  for f in data/*.br; do
    brotli -dc "$f" 2>/dev/null | grep -q "ic705\|IC-705 Setup\|IC-705 IP interface" && { echo "STALE  $f"; stale=1; }
  done
fi

# Compare CONTENT, not timestamps. mtime looked like the obvious signal until a
# fast-forward proved otherwise: git rewrites the tracked sources (mtime = now) and
# leaves the gitignored .br and untracked .gz alone, so every branch switch produced
# a page of false alarms -- exactly when the check most needs to be trusted.
#
# The .gz/.br of a .js holds the *minified* companion when one exists, so that is
# what it must be compared against.
behind=0
for src in data/*.js data/*.html data/*.css data/*.txt; do
  [[ -f "$src" ]] || continue
  case "$src" in *.min) continue ;; esac
  reference="$src"
  [[ -f "$src.min" ]] && reference="$src.min"
  if [[ -f "$src.gz" ]] && ! cmp -s <(zcat "$src.gz" 2>/dev/null) "$reference"; then
    echo "STALE  $src.gz does not match $reference"; behind=1
  fi
  if [[ -f "$src.br" ]] && command -v brotli >/dev/null 2>&1 &&
     ! cmp -s <(brotli -dc "$src.br" 2>/dev/null) "$reference"; then
    echo "STALE  $src.br does not match $reference"; behind=1
  fi
done

if (( stale || behind )); then
  echo "FAIL — run tools/minify-spiffs-js.sh then tools/gzip-assets.sh before building"
  exit 1
fi
echo "OK — sketch, sources and served assets are the same generation"
