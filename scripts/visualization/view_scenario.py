#!/usr/bin/env python3
"""View a scenario without robots, locally or through WebRTC."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from scenarios.registry import SCENARIOS
from visualization.display import add_display_arguments, launch_display


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=SCENARIOS, default='shipyard')
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--steps', type=int, default=0, help='Viewer frames; 0 runs until stopped.')
    parser.add_argument('--report', type=Path)
    add_display_arguments(parser)
    args = parser.parse_args()
    if Path(sys.prefix).resolve() != (ROOT / '.conda/isaacsim-5.0').resolve():
        parser.error(f'Activate {ROOT / ".conda/isaacsim-5.0"} first.')
    if args.steps < 0:
        parser.error('--steps must be nonnegative.')
    command = [str(Path(__file__).with_name('scenario_viewer.py')),
               '--scenario', args.scenario, '--device', args.device, '--steps', str(args.steps)]
    if args.report:
        command += ['--report', str(args.report.resolve())]
    try:
        status = launch_display(command, root=ROOT, robot='scenario_viewer',
                                webrtc=args.webrtc, server_ip=args.server_ip)
    except ValueError as exc:
        parser.error(str(exc))
    raise SystemExit(status)


if __name__ == '__main__':
    main()
