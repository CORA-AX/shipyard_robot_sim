#!/usr/bin/env bash
# Install project-local ROS tools. Default: Nav2 and remote RViz.
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$workspace_dir/tools/setup.py" "$@"
