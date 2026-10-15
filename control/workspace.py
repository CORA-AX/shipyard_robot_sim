"""Make project ROS Python packages available to legacy standalone entrypoints."""
from pathlib import Path
import sys


def add_ros_packages():
    source = Path(__file__).resolve().parents[1] / 'ros2_ws/src'
    for name in ('robot_control', 'robot_sim'):
        path = str(source / name)
        if path not in sys.path:
            sys.path.insert(0, path)
