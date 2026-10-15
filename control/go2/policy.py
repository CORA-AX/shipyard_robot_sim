"""Compatibility imports for the workspace-owned implementation."""
from control.workspace import add_ros_packages
add_ros_packages()
from robot_control.go2.policy import CONTRACT, JOINT_NAMES, FlatPolicy  # noqa: E402,F401
