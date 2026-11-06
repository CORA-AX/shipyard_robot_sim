#!/usr/bin/env python3
"""Visit stored shipyard waypoints in order using the Hunter Nav2 stack."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.ros.nav2.runner import main

if __name__ == '__main__':
    raise SystemExit(main(ROOT, ['--environment', 'shipyard', '--route', 'smoke', *sys.argv[1:]]))
