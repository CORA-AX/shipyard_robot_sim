#!/usr/bin/env python3
"""Run Hunter teleoperation with the system ROS Humble Python."""
from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from hunter.ros.teleop import main


if __name__ == '__main__':
    main()
