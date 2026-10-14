"""Direct velocity commands with explicit turns and low-speed forward samples."""
import torch

from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg
from isaaclab.utils import configclass

from .mdp import sample_command_modes


class TrackingVelocityCommand(UniformVelocityCommand):
    def __init__(self, cfg, env):
        if cfg.heading_command:
            raise ValueError('TrackingVelocityCommand requires direct yaw velocity, heading_command=False.')
        fractions = (cfg.rel_standing_envs, cfg.rel_pure_yaw_envs,
                     cfg.rel_forward_envs, cfg.rel_forward_turn_envs)
        if not (all(0 <= fraction <= 1 for fraction in fractions) and sum(fractions) <= 1):
            raise ValueError('Command mode fractions must be nonnegative and sum to at most one.')
        if cfg.min_abs_yaw <= 0:
            raise ValueError('min_abs_yaw must be positive.')
        if cfg.rel_pure_yaw_envs > 0 or cfg.rel_forward_turn_envs > 0:
            lower, upper = cfg.ranges.ang_vel_z
            if lower > -cfg.min_abs_yaw or upper < cfg.min_abs_yaw:
                raise ValueError('Turning modes require yaw bounds covering both ±min_abs_yaw.')
        if not 0 < cfg.forward_speed_range[0] <= cfg.forward_speed_range[1]:
            raise ValueError('Forward speeds must be positive and ordered.')
        if cfg.rel_forward_envs > 0 or cfg.rel_forward_turn_envs > 0:
            lower, upper = cfg.ranges.lin_vel_x
            if lower > cfg.forward_speed_range[0] or upper < cfg.forward_speed_range[1]:
                raise ValueError('Forward speed range must fit inside lin_vel_x bounds.')
        super().__init__(cfg, env)
        self.is_pure_yaw_env = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        commands, standing, turning = sample_command_modes(
            self.vel_command_b[env_ids], self.cfg.rel_standing_envs,
            self.cfg.rel_pure_yaw_envs, self.cfg.min_abs_yaw,
            self.cfg.rel_forward_envs, self.cfg.rel_forward_turn_envs,
            self.cfg.forward_speed_range)
        self.vel_command_b[env_ids] = commands
        self.is_standing_env[env_ids] = standing
        self.is_pure_yaw_env[env_ids] = turning


@configclass
class TrackingVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = TrackingVelocityCommand
    rel_pure_yaw_envs: float = 0.30
    min_abs_yaw: float = 0.30
    rel_forward_envs: float = 0.0
    rel_forward_turn_envs: float = 0.0
    forward_speed_range: tuple[float, float] = (0.2, 0.6)
