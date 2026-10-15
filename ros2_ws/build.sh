#!/usr/bin/env bash
set -e
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname "$workspace_dir")"
source /opt/ros/humble/setup.bash
cd "$workspace_dir"
if command -v colcon >/dev/null 2>&1; then
    exec colcon build --symlink-install --executor sequential "$@"
fi
tools_dir="$project_dir/.runtime/ros2_build_tools"
if [ ! -f "$tools_dir/bin/colcon" ]; then
    echo 'Install python3-colcon-common-extensions, or install colcon-common-extensions into .runtime/ros2_build_tools.' >&2
    exit 1
fi
export PYTHONPATH="$tools_dir${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$tools_dir/bin/colcon" build --symlink-install --executor sequential "$@"
