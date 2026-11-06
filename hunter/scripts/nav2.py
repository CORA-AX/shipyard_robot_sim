#!/usr/bin/env python3
"""Run Hunter Nav2 and the RViz goal interface in a system ROS terminal."""
from pathlib import Path
import sys

HUNTER_ROOT = Path(__file__).resolve().parents[1]
if not __package__:
    sys.path.insert(0, str(HUNTER_ROOT.parent))

from hunter.ros.nav2.runner import main

if __name__ == '__main__':
    raise SystemExit(main(HUNTER_ROOT))
