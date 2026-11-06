# Source from a system ROS terminal (not the Isaac Conda environment).
source /opt/ros/humble/setup.bash
navigation_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
navigation_sysroot="$(dirname "$navigation_workspace")/.runtime/nav2/sysroot"
navigation_prefix="$navigation_sysroot/opt/ros/humble"
if [ -d "$navigation_prefix/share/ament_index" ]; then
    export AMENT_PREFIX_PATH="$navigation_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="$navigation_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$navigation_prefix/lib:$navigation_sysroot/usr/lib/x86_64-linux-gnu:$navigation_sysroot/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$navigation_prefix/local/lib/python3.10/dist-packages:$navigation_prefix/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
if [ -f "$navigation_workspace/install/local_setup.bash" ]; then
    source "$navigation_workspace/install/local_setup.bash"
fi
unset navigation_workspace navigation_sysroot navigation_prefix
