"""Register project Go2 tracking tasks without importing simulator modules."""
import gymnasium as gym

for task, config in (
    ('Shipyard-Go2-Flat-Tracking-v0', 'Go2TrackingEnvCfg'),
    ('Shipyard-Go2-Flat-Tracking-Play-v0', 'Go2TrackingEnvCfg_PLAY'),
):
    gym.register(
        id=task,
        entry_point='isaaclab.envs:ManagerBasedRLEnv',
        disable_env_checker=True,
        kwargs={
            'env_cfg_entry_point': f'{__name__}.env_cfg:{config}',
            'rsl_rl_cfg_entry_point': f'{__name__}.env_cfg:Go2TrackingPPORunnerCfg',
        },
    )
