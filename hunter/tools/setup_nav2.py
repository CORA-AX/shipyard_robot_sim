"""Download the Humble Nav2 runtime into Hunter's private dependency prefix."""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ['ros-humble-' + name for name in (
    'nav2-planner', 'nav2-smac-planner', 'nav2-controller',
    'nav2-regulated-pure-pursuit-controller', 'nav2-bt-navigator',
    'nav2-lifecycle-manager', 'rviz2')]


def validate(staging):
    prefix = staging/'opt/ros/humble'
    env = dict(os.environ, LD_LIBRARY_PATH=':'.join((str(prefix/'lib'), str(prefix/'lib/x86_64-linux-gnu'),
        str(staging/'usr/lib/x86_64-linux-gnu'), str(staging/'usr/lib'), '/opt/ros/humble/lib')))
    targets = [('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server'),
               ('nav2_bt_navigator', 'bt_navigator'), ('nav2_lifecycle_manager', 'lifecycle_manager')]
    binaries = []
    for package, executable in targets:
        relative = Path('lib')/package/executable
        binary = next((p/relative for p in (prefix, Path('/opt/ros/humble')) if (p/relative).is_file()), None)
        if binary is None:
            raise RuntimeError(f'Missing {package}/{executable}')
        binaries.append(binary)
    for name in ('libnav2_smac_planner.so', 'libnav2_regulated_pure_pursuit_controller.so',
                 'libposition_goal_checker.so'):
        binary = next((p/'lib'/name for p in (prefix, Path('/opt/ros/humble')) if (p/'lib'/name).is_file()), None)
        if binary is None:
            raise RuntimeError(f'Missing Nav2 plugin {name}')
        binaries.append(binary)
    for binary in binaries:
        result = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
        if 'not found' in result:
            missing = '\n'.join(line for line in result.splitlines() if 'not found' in line)
            raise RuntimeError(f'Missing runtime library for {binary}:\n{missing}')


def main():
    destination = ROOT/'.runtime/nav2'
    destination.mkdir(parents=True, exist_ok=True)
    try:
        validate(destination/'sysroot')
    except (RuntimeError, subprocess.CalledProcessError):
        pass
    else:
        print('Hunter Nav2 dependencies ready. Source hunter/scripts/nav2_env.bash.')
        return
    plan = subprocess.check_output(['apt-get', '-s', 'install', '--no-install-recommends', *PACKAGES], text=True)
    pending = re.findall(r'^Inst (\S+) (?:\[[^\]]+\] )?\((\S+)', plan, re.MULTILINE)
    downloads = destination/'downloads'
    downloads.mkdir(exist_ok=True)
    cached = {}
    for deb in downloads.glob('*.deb'):
        package = subprocess.check_output(['dpkg-deb', '-f', str(deb), 'Package'], text=True).strip()
        version = subprocess.check_output(['dpkg-deb', '-f', str(deb), 'Version'], text=True).strip()
        cached[(package, version)] = deb
    missing = [f'{p}={v}' for p, v in pending if (p, v) not in cached]
    if missing:
        subprocess.run(['apt-get', 'download', *missing], cwd=downloads, check=True)
    with tempfile.TemporaryDirectory(prefix='install_', dir=destination) as folder:
        folder = Path(folder)
        staging = folder/'sysroot'
        staging.mkdir()
        for deb in sorted(downloads.glob('*.deb')):
            subprocess.run(['dpkg-deb', '-x', str(deb), str(staging)], check=True)
        validate(staging)
        target = destination/'sysroot'
        if target.exists():
            # Replace only this script's disposable dependency installation.
            shutil.rmtree(target)
        shutil.move(str(staging), target)
        (destination/'packages.json').write_text(json.dumps(dict(pending), indent=2)+'\n')
    print('Hunter Nav2 dependencies ready. Source hunter/scripts/nav2_env.bash.')


if __name__ == '__main__':
    main()
