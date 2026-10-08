#!/usr/bin/env python3
"""Package/CUDA preflight, without launching Kit or accepting its license."""
from importlib.metadata import version
import json
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
assert sys.version_info[:2] == (3, 11), sys.version
assert Path(sys.prefix).resolve() == ROOT/'.conda/isaacsim-5.0', sys.prefix
packages = {name: version(name) for name in ('isaacsim','torch','torchvision','isaacsim-kernel','isaacsim-app')}
assert packages['isaacsim'] == '5.0.0.0', packages
subprocess.run([sys.executable,'-m','pip','check'],check=True)
import torch
assert torch.cuda.is_available(), 'CUDA unavailable; run on the host with NVIDIA device access'
assert torch.ones(4,device='cuda:0').sum().item() == 4
report = {'python':sys.version,'python_executable':sys.executable,'platform':platform.platform(),
          'packages':packages,'torch_cuda':torch.version.cuda,'cuda_tensor_test':'pass',
          'gpus':[{'name':torch.cuda.get_device_name(i),'vram_bytes':torch.cuda.get_device_properties(i).total_memory}
                  for i in range(torch.cuda.device_count())],
          'kit_started':False,'hunter_usd_created':False}
out=ROOT/'docs/isaacsim_install_evidence';out.mkdir(exist_ok=True)
(out/'package_cuda_check.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
