"""Legacy Go2 entrypoint delegates to the ROS workspace runtime."""
from control.workspace import add_ros_packages
add_ros_packages()
from robot_sim.runtime import run_ros  # noqa: E402,F401
