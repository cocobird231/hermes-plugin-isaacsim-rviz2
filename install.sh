#!/usr/bin/env bash
# Install the already cloned plugins; no downloads, sudo, or automatic enablement.
set -euo pipefail
SIM_PLUGIN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SIM_PLUGIN_ROOT/scripts/install_plugins.py" "$@"
