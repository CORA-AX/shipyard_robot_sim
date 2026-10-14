"""Go2 tracking v4: fresh trot training with persistent airborne-foot costs."""
from isaaclab.envs import mdp
from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.flat_env_cfg import UnitreeGo2FlatEnvCfg
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.agents.rsl_rl_ppo_cfg import (
    UnitreeGo2FlatPPORunnerCfg,
)
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import RewardsCfg

from .commands import TrackingVelocityCommandCfg
from .mdp import (
    FOOT_NAMES, feet_air_time_excess, feet_air_time_tracking, feet_slide,
    track_ang_vel_z_error, track_lin_vel_xy_error, trot_gait,
)


@configclass
class TrackingCommandsCfg:
    base_velocity = TrackingVelocityCommandCfg(
        asset_name='robot', resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02, rel_pure_yaw_envs=0.30, min_abs_yaw=0.30,
        rel_forward_envs=0.20, rel_forward_turn_envs=0.20, forward_speed_range=(0.2, 0.6),
        heading_command=False, rel_heading_envs=0.0, debug_vis=False,
        ranges=TrackingVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0), lin_vel_y=(-1.0, 1.0), ang_vel_z=(-1.0, 1.0)),
    )


@configclass
class TrackingRewardsCfg(RewardsCfg):
    # Order is explicit: sensor/articulation body indices need not coincide.
    trot_gait = RewardTermCfg(
        func=trot_gait, weight=0.5,
        params={'command_name': 'base_velocity',
                'sensor_cfg': SceneEntityCfg('contact_forces', body_names=list(FOOT_NAMES), preserve_order=True),
                'timing_std': 0.1, 'min_swing_time': 0.06, 'max_swing_time': 0.4,
                'linear_threshold': 0.1, 'angular_threshold': 0.1,
                'ramp_steps': 0, 'initial_scale': 1.0},
    )
    feet_air_time_excess = RewardTermCfg(
        func=feet_air_time_excess, weight=-2.0,
        params={'command_name': 'base_velocity',
                'sensor_cfg': SceneEntityCfg('contact_forces', body_names=list(FOOT_NAMES), preserve_order=True),
                'max_air_time': 0.4, 'saturation_time': 0.2,
                'linear_threshold': 0.1, 'angular_threshold': 0.1},
    )
    feet_slide = RewardTermCfg(
        func=feet_slide, weight=-0.1,
        params={'sensor_cfg': SceneEntityCfg('contact_forces', body_names=list(FOOT_NAMES), preserve_order=True),
                'asset_cfg': SceneEntityCfg('robot', body_names=list(FOOT_NAMES), preserve_order=True),
                'force_threshold': 1.0},
    )
    # Flat ground is z=0. This is a fixed posture objective, not spawn height.
    base_height = RewardTermCfg(func=mdp.base_height_l2, weight=-20.0, params={'target_height': 0.28})
    lin_vel_xy_error = RewardTermCfg(
        func=track_lin_vel_xy_error, weight=-1.0,
        params={'command_name': 'base_velocity', 'asset_cfg': SceneEntityCfg('robot')},
    )
    ang_vel_z_error = RewardTermCfg(
        func=track_ang_vel_z_error, weight=-0.5,
        params={'command_name': 'base_velocity', 'asset_cfg': SceneEntityCfg('robot'),
                'pure_yaw_scale': 3.0, 'linear_threshold': 0.1, 'angular_threshold': 0.1},
    )
    feet_air_time = RewardTermCfg(
        func=feet_air_time_tracking, weight=1.0,
        params={'command_name': 'base_velocity',
                'sensor_cfg': SceneEntityCfg('contact_forces', body_names='.*_foot'),
                'min_air_time': 0.1, 'max_air_time': 0.4,
                'linear_threshold': 0.1, 'angular_threshold': 0.1},
    )


@configclass
class Go2TrackingEnvCfg(UnitreeGo2FlatEnvCfg):
    commands = TrackingCommandsCfg()
    rewards = TrackingRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_lin_vel_xy_exp.params['std'] = 0.25
        self.rewards.track_ang_vel_z_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.params['std'] = 0.35
        self.rewards.feet_air_time.weight = 1.0


@configclass
class Go2TrackingEnvCfg_PLAY(Go2TrackingEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None


@configclass
class Go2TrackingPPORunnerCfg(UnitreeGo2FlatPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = 'go2_flat_tracking'
        self.max_iterations = 5000
