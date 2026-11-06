#!/usr/bin/env bash
# Extract dependencies into hunter/.runtime; does not install system packages.
set -euo pipefail
hunter_nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$hunter_nav_root/scripts/setup_ros.sh"
exec /usr/bin/python3 "$hunter_nav_root/tools/setup_nav2.py"
