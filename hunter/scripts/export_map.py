#!/usr/bin/env python3
"""Build the Hunter shipyard map with OpenUSD CPU libraries; never start Kit."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', choices=('shipyard',), default='shipyard')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.worker:
        prefix = Path(os.environ.get('HUNTER_ISAAC_PREFIX', ROOT.parent/'.conda/isaacsim-5.0'))
        libraries = sorted((prefix/'lib/python3.11/site-packages/isaacsim/extscache').glob('omni.usd.libs-*'))
        if not libraries or not (prefix/'bin/python').is_file():
            parser.error('Set HUNTER_ISAAC_PREFIX to the installed Isaac environment.')
        usd = libraries[-1]
        env = dict(os.environ, PYTHONPATH=os.pathsep.join((str(usd),str(ROOT.parent))),
                   LD_LIBRARY_PATH=str(usd/'bin'))
        return subprocess.call([str(prefix/'bin/python'),str(Path(__file__).resolve()),
                                '--environment',args.environment,'--worker'],env=env)
    from hunter.environments.config import load_world
    from hunter.environments.raster import export
    export(ROOT,load_world(ROOT,args.environment))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
