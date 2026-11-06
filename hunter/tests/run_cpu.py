#!/usr/bin/env python3
"""Run Hunter CPU tests with OpenUSD, without starting Kit or physics."""
import os
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    prefix = Path(os.environ.get('HUNTER_ISAAC_PREFIX', root.parent / '.conda/isaacsim-5.0'))
    python = prefix / 'bin/python'
    libraries = sorted((prefix / 'lib/python3.11/site-packages/isaacsim/extscache').glob('omni.usd.libs-*'))
    if not python.is_file() or not libraries:
        raise SystemExit('Set HUNTER_ISAAC_PREFIX to the installed Isaac Sim 5.0 environment.')
    usd = libraries[-1]
    environment = dict(os.environ)
    environment['PYTHONPATH'] = str(usd)
    environment['LD_LIBRARY_PATH'] = os.pathsep.join(filter(None, (
        str(usd / 'bin'), environment.get('LD_LIBRARY_PATH', ''))))
    result = subprocess.run([str(python), '-m', 'unittest', 'discover',
                             '-s', str(root / 'tests'), '-p', 'test_*.py', '-v'],
                            cwd=root, env=environment)
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
