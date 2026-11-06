#!/usr/bin/env python3
"""Run the Hunter GUI with fixed-speed or ROS control."""
from pathlib import Path
import sys

HUNTER_ROOT = Path(__file__).resolve().parent
if not __package__:
    sys.path.insert(0, str(HUNTER_ROOT.parent))

from hunter.sim.runtime import main


if __name__ == '__main__':
    main(HUNTER_ROOT)
