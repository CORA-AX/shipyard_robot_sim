#!/usr/bin/env python3
"""Download and verify a Go2 checkpoint described by the project manifest."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control-policy', action='store_true',
                        help='Prepare the selected default control policy instead of the NVIDIA checkpoint.')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    manifest = json.loads((root / 'assets/go2/config/sources.lock.json').read_text())
    policy = manifest['control_policy' if args.control_policy else 'policy']
    target = root / policy['path']
    if target.exists():
        if hashlib.sha256(target.read_bytes()).hexdigest() != policy['sha256']:
            raise SystemExit(f'Existing checkpoint checksum mismatch: {target}')
        print(f'Verified {target}')
        return
    if not policy.get('url'):
        raise SystemExit('Selected checkpoint has no download URL; restore its source_checkpoint from the training run.')
    with urlopen(policy['url'], timeout=60) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != policy['sha256']:
        raise SystemExit('Downloaded checkpoint checksum mismatch; no file was installed.')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
    print(f'Downloaded and verified {target}')


if __name__ == '__main__':
    main()
