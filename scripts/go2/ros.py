#!/usr/bin/env python3
"""Compatibility entrypoint for the workspace's Go2 ROS launcher."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from control.workspace import add_ros_packages
add_ros_packages()
from robot_sim.launcher import add_arguments, main, validate, worker

if __name__ == '__main__':
    if '--worker' in sys.argv:
        sys.argv.remove('--worker')
        worker()
    else:
        main()
