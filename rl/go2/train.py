#!/usr/bin/env python3
"""Register project tasks and launch the pinned Isaac Lab RSL-RL trainer."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / 'third_party/IsaacLab/scripts/reinforcement_learning/rsl_rl'
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(RUNNER))
import rl.go2.tasks.flat_tracking  # noqa: E402,F401; registration only, no simulator imports

if __name__ == '__main__':
    runpy.run_path(str(RUNNER / 'train.py'), run_name='__main__')
