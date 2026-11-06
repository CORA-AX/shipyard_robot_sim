# Source in a new system-Python terminal, separate from Isaac's Conda terminal.
_hunter_nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_hunter_nav_root/scripts/ros_env.bash"
_hunter_nav_sysroot="$_hunter_nav_root/.runtime/nav2/sysroot"
_hunter_nav_prefix="$_hunter_nav_sysroot/opt/ros/humble"
if [[ -d "$_hunter_nav_prefix/share/ament_index" ]]; then
    export AMENT_PREFIX_PATH="$_hunter_nav_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$_hunter_nav_prefix/lib:$_hunter_nav_prefix/lib/x86_64-linux-gnu:$_hunter_nav_sysroot/usr/lib/x86_64-linux-gnu:$_hunter_nav_sysroot/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$_hunter_nav_prefix/local/lib/python3.10/dist-packages:$_hunter_nav_prefix/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
unset _hunter_nav_root _hunter_nav_sysroot _hunter_nav_prefix
