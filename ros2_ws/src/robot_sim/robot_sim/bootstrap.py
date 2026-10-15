"""Start the common Isaac launcher using validated workspace configuration."""
import argparse
import os
from pathlib import Path
import sys

from .config import load_world_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--robots-file', type=Path, required=True)
    args, remaining = parser.parse_known_args()
    root = args.project_root.resolve()
    if not (root / 'assets/go2/config/sources.lock.json').is_file():
        parser.error('project-root must point to the robot simulation repository.')
    world = load_world_config(args.robots_file)
    os.environ['WAREHOUSE_SIM_ROOT'] = str(root)
    sys.path.insert(0, str(root))
    if len(world['robots']) > 1:
        sys.argv = ['robot_sim.launcher', '--robot-type', 'multi',
                    '--robots-file', str(args.robots_file.resolve()), *remaining]
    else:
        config = world['robots'][0]
        sys.argv = ['robot_sim.launcher', '--robot-type', config['robot_type'],
                '--scenario', world['scenario'], '--robot-id', config['robot_id'],
                '--namespace', config['namespace'], '--cmd-vel-topic', '/' + config['namespace'] + '/cmd_vel',
                '--initial-position', *map(str, config['position']), '--initial-yaw', str(config['yaw']),
                '--seed', str(world['seed']), *remaining]
    from robot_sim.launcher import main as run_simulator
    run_simulator()


if __name__ == '__main__':
    main()
