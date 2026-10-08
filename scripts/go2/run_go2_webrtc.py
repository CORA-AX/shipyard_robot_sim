#!/usr/bin/env python3
"""Select a Go2 scenario and launch it with the shared WebRTC display support."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT))
from assets.go2.scenarios import SCENARIOS
from visualization.webrtc import WebRTCConfig
from visualization.display import add_display_arguments, launch_display


def scenario_arguments(args, display: WebRTCConfig | None):
    common = ['--device', args.device]
    command = [str(ROOT / 'scripts/go2/run_go2.py'),
               '--mode', args.mode, '--scenario', args.scenario,
               '--real-time', *common]
    if display is not None:
        command += [*display.app_arguments(), '--kit_args', display.kit_arguments()]
    if args.vx is not None:
        command += ['--vx', str(args.vx)]
    steps = args.steps if args.steps is not None else (1500 if args.mode == 'check' else None)
    if steps is not None:
        command += ['--steps', str(steps)]
    if args.report is not None:
        command += ['--report', str(args.report.resolve())]
    return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('view', 'policy', 'check'), nargs='?', default='policy')
    parser.add_argument('--scenario', choices=SCENARIOS, default='flat')
    add_display_arguments(parser, default=True)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--steps', type=int, help='Control steps; check defaults to 1500, view/policy run until stopped.')
    parser.add_argument('--vx', type=float, help='Override forward command, in m/s; check defaults to 0.4.')
    parser.add_argument('--report', type=Path, help='Optional JSON report path.')
    args = parser.parse_args()
    if args.steps is not None and (args.steps < 0 or (args.mode == 'check' and args.steps < 100)):
        parser.error('Check requires at least 100 steps; other modes accept 0 for continuous execution.')
    if args.vx is not None and not 0.2 <= args.vx <= 0.8:
        parser.error('Forward speed must be between 0.2 and 0.8 m/s.')
    if Path(sys.prefix).resolve() != (ROOT / '.conda/isaacsim-5.0').resolve():
        parser.error(f'Activate {ROOT / ".conda/isaacsim-5.0"} first.')
    try:
        status = launch_display(scenario_arguments(args, None), root=ROOT, robot='go2',
                                webrtc=args.webrtc, server_ip=args.server_ip)
    except ValueError as exc:
        parser.error(str(exc))
    raise SystemExit(status)


if __name__ == '__main__':
    main()
