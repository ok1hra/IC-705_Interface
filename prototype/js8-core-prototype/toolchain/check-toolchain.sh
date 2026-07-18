#!/usr/bin/env sh
set -eu

toolchain_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
# shellcheck source=toolchain.lock
. "$toolchain_dir/toolchain.lock"

profile=local-compatible
if [ "${1:-}" = "--release" ]; then
  profile=release
  shift
fi
if [ "$#" -ne 0 ]; then
  echo "usage: $0 [--release]" >&2
  exit 2
fi

fail() { echo "TOOLCHAIN FAIL $*" >&2; exit 1; }
actual_emcc=$(emcc --version | awk 'NR == 1 {for (i = 1; i <= NF; i++) if ($i ~ /^[0-9]+\.[0-9]+\.[0-9]+$/) {print $i; exit}}')
[ "$actual_emcc" = "$EMSCRIPTEN_VERSION" ] ||
  fail "emcc=$actual_emcc expected=$EMSCRIPTEN_VERSION"
actual_cmake=$(cmake --version | awk 'NR == 1 {print $3; exit}')
[ "$actual_cmake" = "$CMAKE_VERSION" ] ||
  fail "cmake=$actual_cmake expected=$CMAKE_VERSION"
actual_node=$(node --version | sed 's/^v//')
if [ "$profile" = "release" ]; then
  [ "$actual_node" = "$NODE_VERSION" ] ||
    fail "node=$actual_node expected=$NODE_VERSION profile=$profile"
else
  node_major=${actual_node%%.*}
  case "$node_major" in *[!0-9]*|'') fail "invalid node version: $actual_node";; esac
  [ "$node_major" -ge "$NODE_LOCAL_MIN_MAJOR" ] &&
    [ "$node_major" -le "$NODE_LOCAL_MAX_MAJOR" ] ||
    fail "node=$actual_node supported=${NODE_LOCAL_MIN_MAJOR}-${NODE_LOCAL_MAX_MAJOR}.x profile=$profile"
fi

if command -v dpkg-query >/dev/null 2>&1; then
  package_version() { dpkg-query -W -f='${Version}' "$1" 2>/dev/null || true; }
  [ "$(package_version emscripten)" = "$EMSCRIPTEN_PACKAGE_VERSION" ] ||
    fail "Debian emscripten package mismatch"
  [ "$(package_version cmake)" = "$CMAKE_PACKAGE_VERSION" ] ||
    fail "Debian cmake package mismatch"
  if [ "$profile" = "release" ]; then
    [ "$(package_version nodejs)" = "$NODE_PACKAGE_VERSION" ] ||
      fail "Debian nodejs package mismatch"
  fi
  [ "$(package_version "$BOOST_PACKAGE")" = "$BOOST_PACKAGE_VERSION" ] ||
    fail "Debian $BOOST_PACKAGE package mismatch"
fi

echo "TOOLCHAIN PASS profile=$profile emscripten=$actual_emcc cmake=$actual_cmake node=$actual_node"
