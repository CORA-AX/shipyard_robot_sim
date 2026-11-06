# Source this file in a system-Python terminal, not the Isaac Conda terminal.
source /opt/ros/humble/setup.bash
_hunter_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_hunter_ros_prefix="$_hunter_root/.runtime/ros_dependencies/sysroot/opt/ros/humble"
if [[ -d "$_hunter_ros_prefix" ]]; then
    export AMENT_PREFIX_PATH="$_hunter_ros_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export PYTHONPATH="$_hunter_ros_prefix/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="$_hunter_ros_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
unset _hunter_root _hunter_ros_prefix
