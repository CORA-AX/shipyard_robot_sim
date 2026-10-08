"""Standalone flat-checkpoint inference. Requires PyTorch, not Isaac Sim."""
import hashlib
import json

JOINT_NAMES = [f'{leg}_{joint}_joint' for joint in ('hip', 'thigh', 'calf')
               for leg in ('FL', 'FR', 'RL', 'RR')]
CONTRACT = 'go2-flat-48x12-v1'


class FlatPolicy:
    def __init__(self, root, device='cpu'):
        import torch
        from torch import nn

        manifest = json.loads((root / 'assets/go2/config/sources.lock.json').read_text())
        checkpoint = root / manifest['policy']['path']
        if not checkpoint.is_file():
            raise ValueError('Run python scripts/go2/prepare_go2_policy.py first.')
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != manifest['policy']['sha256']:
            raise ValueError('Go2 checkpoint checksum mismatch.')
        self.device = device
        self.actor = nn.Sequential(nn.Linear(48, 128), nn.ELU(), nn.Linear(128, 128), nn.ELU(),
                                   nn.Linear(128, 128), nn.ELU(), nn.Linear(128, 12))
        state = torch.load(checkpoint, map_location='cpu', weights_only=True)['model_state_dict']
        self.actor.load_state_dict({k.removeprefix('actor.'): v for k, v in state.items()
                                   if k.startswith('actor.')}, strict=True)
        self.actor.to(device).eval()

    def actions(self, observation):
        import torch

        with torch.inference_mode():
            values = torch.tensor(observation, dtype=torch.float32, device=self.device).reshape(1, 48)
            actions = self.actor(values)
        if not torch.isfinite(actions).all():
            raise ValueError('Policy produced non-finite actions.')
        return actions[0].cpu().tolist()
