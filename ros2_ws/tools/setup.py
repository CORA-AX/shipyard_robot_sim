"""Install Nav2 and remote RViz dependencies inside the project, without sudo."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
NAV2_PACKAGES = [
    'nav2-map-server', 'nav2-planner', 'nav2-navfn-planner', 'nav2-controller',
    'dwb-core', 'dwb-plugins', 'dwb-critics', 'nav2-bt-navigator',
    'nav2-lifecycle-manager', 'nav2-velocity-smoother',
]
RVIZ_PACKAGES = [
    'xvfb', 'x11vnc', 'novnc', 'python3-novnc', 'python3-websockify',
    'openbox', 'libvncclient1', 'libid3tag0', 'libimlib2', 'libobrender32v5', 'libobt2v5',
]


def install_packages(packages, destination, validate):
    """Validate an extracted installation before replacing the current one."""
    destination.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='download.', dir=destination) as temporary:
        temporary = Path(temporary)
        if packages:
            subprocess.run(['apt-get', 'download', *packages], cwd=temporary, check=True)
        staging = temporary / 'sysroot'
        staging.mkdir()
        versions = []
        for deb in sorted(temporary.glob('*.deb')):
            subprocess.run(['dpkg-deb', '-x', str(deb), str(staging)], check=True)
            versions.append(subprocess.check_output(
                ['dpkg-deb', '-f', str(deb), 'Package', 'Version'], text=True).strip())
        validate(staging)
        target = destination / 'sysroot'
        if target.exists():
            shutil.rmtree(target)
        shutil.move(str(staging), str(target))
        (destination / 'packages.json').write_text(json.dumps(versions, indent=2) + '\n')
    print(f'Ready: {target}')


def validate_nav2(staging):
    prefix = staging / 'opt/ros/humble'
    env = dict(os.environ, LD_LIBRARY_PATH=':'.join([
        str(prefix / 'lib'), str(staging / 'usr/lib/x86_64-linux-gnu'), str(staging / 'usr/lib'),
        '/opt/ros/humble/lib']))
    for package, executable in [('nav2_map_server', 'map_server'),
                                ('nav2_controller', 'controller_server'),
                                ('nav2_planner', 'planner_server'),
                                ('nav2_bt_navigator', 'bt_navigator')]:
        binary = prefix / 'lib' / package / executable
        if not binary.is_file():
            binary = Path('/opt/ros/humble/lib') / package / executable
        result = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
        if 'not found' in result:
            raise RuntimeError(f'Missing libraries for {binary}:\n{result}')


def validate_rviz_web(staging):
    env = dict(os.environ, LD_LIBRARY_PATH=str(staging / 'usr/lib/x86_64-linux-gnu'),
               PYTHONPATH=str(staging / 'usr/lib/python3/dist-packages'))
    for name in ('Xvfb', 'x11vnc', 'openbox'):
        output = subprocess.check_output(['ldd', str(staging / 'usr/bin' / name)], env=env, text=True)
        if 'not found' in output:
            raise RuntimeError(f'Missing host libraries for {name}:\n{output}')
    subprocess.run(['/usr/bin/python3', '-c', 'import websockify'], env=env, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only', choices=('nav2', 'rviz-web'),
                        help='Install only this component; default: install both.')
    args = parser.parse_args()
    if args.only in (None, 'nav2'):
        packages = ['ros-humble-' + name for name in NAV2_PACKAGES]
        plan = subprocess.check_output(
            ['apt-get', '-s', 'install', '--no-install-recommends', *packages], text=True)
        pending = re.findall(r'^Inst (\S+) (?:\[[^\]]+\] )?\((\S+)', plan, re.MULTILINE)
        install_packages([f'{p}={v}' for p, v in pending], ROOT / '.runtime/nav2', validate_nav2)
    if args.only in (None, 'rviz-web'):
        install_packages(RVIZ_PACKAGES, ROOT / '.runtime/rviz_web', validate_rviz_web)
    print('Next: bash ros2_ws/build.sh, then source ros2_ws/nav2_env.bash')


if __name__ == '__main__':
    main()
