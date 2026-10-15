#!/usr/bin/env bash
# Ubuntu 22.04: extract distribution packages into the project, without sudo.
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tools_dir="$(dirname "$workspace_dir")/.runtime/rviz_web"
mkdir -p "$tools_dir"
download_dir="$(mktemp -d "$tools_dir/download.XXXXXX")"
trap 'rm -rf "$download_dir"' EXIT
cd "$download_dir"
apt-get download xvfb x11vnc novnc python3-novnc python3-websockify \
  openbox libvncclient1 libid3tag0 libimlib2 libobrender32v5 libobt2v5
/usr/bin/python3 - "$tools_dir" "$download_dir" <<'PY'
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

tools, downloads = map(Path, sys.argv[1:])
staging = downloads / 'sysroot'
versions = []
for package in sorted(downloads.glob('*.deb')):
    subprocess.run(['dpkg-deb', '-x', str(package), str(staging)], check=True)
    versions.append(subprocess.check_output(
        ['dpkg-deb', '-f', str(package), 'Package', 'Version'], text=True).strip())
env = dict(os.environ, LD_LIBRARY_PATH=str(staging / 'usr/lib/x86_64-linux-gnu'),
           PYTHONPATH=str(staging / 'usr/lib/python3/dist-packages'))
for name in ('Xvfb', 'x11vnc', 'openbox'):
    output = subprocess.check_output(['ldd', str(staging / 'usr/bin' / name)], env=env, text=True)
    if 'not found' in output:
        raise SystemExit(f'Missing host libraries for {name}:\n{output}')
subprocess.run(['/usr/bin/python3', '-c', 'import websockify'], env=env, check=True)
if (tools / 'sysroot').exists():
    shutil.rmtree(tools / 'sysroot')
shutil.move(str(staging), str(tools / 'sysroot'))
(tools / 'packages.json').write_text(json.dumps(versions, indent=2) + '\n')
print(f'RViz web tools ready: {tools / "sysroot"}')
PY
