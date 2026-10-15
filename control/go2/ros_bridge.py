"""Compatibility imports for legacy Go2 ROS entrypoints."""
from control.workspace import add_ros_packages
add_ros_packages()
from robot_control.commands import VelocityCommand  # noqa: E402,F401
from robot_sim.environment import ros_environment  # noqa: E402,F401
from robot_sim.ros_bridge import RobotRosBridge, clock_qos  # noqa: E402,F401

Go2RosBridge = RobotRosBridge
