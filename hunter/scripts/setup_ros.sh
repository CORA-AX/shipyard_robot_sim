#!/usr/bin/env bash
# Install the system-Python Ackermann messages locally; no sudo or Kit startup.
set -euo pipefail
hunter_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "$hunter_root/scripts/ros_env.bash"
set -u
if /usr/bin/python3 -c 'from ackermann_msgs.msg import AckermannDriveStamped' 2>/dev/null; then
    echo 'Hunter ROS environment already provides ackermann_msgs.'
    exit 0
fi
download_dir="$hunter_root/.runtime/ros_dependencies/downloads"
install_dir="$hunter_root/.runtime/ros_dependencies/sysroot"
mkdir -p "$download_dir" "$install_dir"
cd "$download_dir"
apt-get download ros-humble-ackermann-msgs
for package in ros-humble-ackermann-msgs_*.deb; do
    dpkg-deb -x "$package" "$install_dir"
done
set +u
source "$hunter_root/scripts/ros_env.bash"
set -u
/usr/bin/python3 -c 'from ackermann_msgs.msg import AckermannDriveStamped; print("Hunter Ackermann messages ready.")'
