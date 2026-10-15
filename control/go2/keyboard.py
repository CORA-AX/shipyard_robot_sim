"""Compatibility imports for the workspace-owned implementation."""
from control.workspace import add_ros_packages
add_ros_packages()
from robot_control.keyboard import DEFAULT_KEYBOARD_SPEED, DEFAULT_KEYBOARD_TURN_SPEED, HELP, KeyboardCommand, TerminalKeyboard, add_keyboard_arguments, validate_keyboard_arguments  # noqa: E402,F401
