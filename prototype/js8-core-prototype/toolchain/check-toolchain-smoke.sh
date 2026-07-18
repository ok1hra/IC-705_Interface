#!/usr/bin/env sh
set -eu

toolchain_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
shim_dir=$(mktemp -d /tmp/js8-node-policy-XXXXXX)
trap 'rm -rf "$shim_dir"' EXIT HUP INT TERM

printf '%s\n' '#!/usr/bin/env sh' 'echo v20.20.0' > "$shim_dir/node"
chmod +x "$shim_dir/node"
PATH="$shim_dir:$PATH" "$toolchain_dir/check-toolchain.sh" >/dev/null

printf '%s\n' '#!/usr/bin/env sh' 'echo v21.0.0' > "$shim_dir/node"
if PATH="$shim_dir:$PATH" "$toolchain_dir/check-toolchain.sh" >/dev/null 2>&1; then
  echo "TOOLCHAIN SMOKE FAIL unsupported Node 21 was accepted" >&2
  exit 1
fi

echo "TOOLCHAIN SMOKE PASS local_node=20.20.0 rejected_node=21.0.0"
