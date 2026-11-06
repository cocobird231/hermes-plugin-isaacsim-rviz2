#!/usr/bin/env bash
# Configured user-service launch; does not restart/stop existing services.
set -euo pipefail
SIM_PLUGIN_SCRIPTS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SIM_PLUGIN_SCRIPTS/launch_bridges.py" rviz2 "$@"
