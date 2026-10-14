"""Standalone flat-checkpoint inference. Requires PyTorch, not Isaac Sim."""
import hashlib
import json
from pathlib import Path

JOINT_NAMES = [f'{leg}_{joint}_joint' for joint in ('hip', 'thigh', 'calf')
               for leg in ('FL', 'FR', 'RL', 'RR')]
CONTRACT = 'go2-flat-48x12-v1'


class FlatPolicy:
    def __init__(self, root, device='cpu', checkpoint=None):
        import torch
        from torch import nn

        manifest = json.loads((root / 'assets/go2/config/sources.lock.json').read_text())
        specification = manifest.get('control_policy', manifest['policy'])
        checkpoint = Path(checkpoint).resolve() if checkpoint is not None else root / specification['path']
        if not checkpoint.is_file():
            raise ValueError(f'Go2 checkpoint not found: {checkpoint}. Use --checkpoint to select an available model.')
        digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        for registered in (manifest['policy'], manifest.get('control_policy')):
            if (registered is not None and checkpoint.resolve() == (root / registered['path']).resolve()
                    and digest != registered['sha256']):
                raise ValueError('Go2 checkpoint checksum mismatch.')
        payload = torch.load(checkpoint, map_location='cpu', weights_only=True)
        if payload.get('obs_norm_state_dict') is not None:
            raise ValueError('Go2 control requires a checkpoint without empirical observation normalization.')
        self.device = device
        self.checkpoint_path = checkpoint.resolve()
        self.checkpoint_sha256 = digest
        self.iteration = payload.get('iter')
        self.actor = nn.Sequential(nn.Linear(48, 128), nn.ELU(), nn.Linear(128, 128), nn.ELU(),
                                   nn.Linear(128, 128), nn.ELU(), nn.Linear(128, 12))
        state = payload['model_state_dict']
        self.actor.load_state_dict({k.removeprefix('actor.'): v for k, v in state.items()
                                   if k.startswith('actor.')}, strict=True)
        if not all(torch.isfinite(value).all() for value in self.actor.state_dict().values()):
            raise ValueError('Go2 checkpoint contains non-finite actor parameters.')
        self.actor.to(device).eval()

    def actions(self, observation):
        import torch

        with torch.inference_mode():
            values = torch.tensor(observation, dtype=torch.float32, device=self.device).reshape(1, 48)
            actions = self.actor(values)
        if not torch.isfinite(actions).all():
            raise ValueError('Policy produced non-finite actions.')
        return actions[0].cpu().tolist()
