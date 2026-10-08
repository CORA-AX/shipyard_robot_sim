"""Go2 flat scenario owned by this project, using Isaac Lab framework APIs.

Observation order, action scaling and physics timing match the official flat
checkpoint. Rewards below support inspection; this runner does not train.
"""
import isaaclab.sim as sim
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg, mdp
from isaaclab.managers import (
    EventTermCfg, ObservationGroupCfg, ObservationTermCfg, RewardTermCfg,
    SceneEntityCfg, TerminationTermCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

from assets.go2.config.robot_cfg import GO2_CFG


@configclass
class FlatSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path='/World/ground', terrain_type='plane', collision_group=-1,
        physics_material=sim.RigidBodyMaterialCfg(
            friction_combine_mode='multiply', restitution_combine_mode='multiply',
            static_friction=1.0, dynamic_friction=1.0),
        visual_material=sim.PreviewSurfaceCfg(diffuse_color=(0.35, 0.38, 0.40)),
    )
    robot = GO2_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')
    contact_forces = ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/Robot/.*', history_length=3, track_air_time=True)
    sky_light = AssetBaseCfg(
        prim_path='/World/skyLight', spawn=sim.DomeLightCfg(intensity=750.0))


@configclass
class CommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name='robot', resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.0, heading_command=False, debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.4, 0.4), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0)),
    )


@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(
        asset_name='robot', joint_names=['.*'], scale=0.25, use_default_offset=True)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        # 3 + 3 + 3 + 3 + 12 + 12 + 12 = 48. Preserve this order.
        base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel)
        base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel)
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity)
        velocity_commands = ObservationTermCfg(
            func=mdp.generated_commands, params={'command_name': 'base_velocity'})
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel)
        actions = ObservationTermCfg(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy = PolicyCfg()


@configclass
class EventsCfg:
    reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform, mode='reset',
        params={'pose_range': {'x': (0.0, 0.0), 'y': (0.0, 0.0), 'yaw': (0.0, 0.0)},
                'velocity_range': {axis: (0.0, 0.0) for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}},
    )
    reset_robot_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale, mode='reset',
        params={'position_range': (1.0, 1.0), 'velocity_range': (0.0, 0.0)},
    )


@configclass
class RewardsCfg:
    track_lin_vel_xy = RewardTermCfg(
        func=mdp.track_lin_vel_xy_exp, weight=1.5,
        params={'command_name': 'base_velocity', 'std': 0.5})
    track_ang_vel_z = RewardTermCfg(
        func=mdp.track_ang_vel_z_exp, weight=0.75,
        params={'command_name': 'base_velocity', 'std': 0.5})
    lin_vel_z = RewardTermCfg(func=mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy = RewardTermCfg(func=mdp.ang_vel_xy_l2, weight=-0.05)
    joint_torques = RewardTermCfg(func=mdp.joint_torques_l2, weight=-0.0002)
    joint_acc = RewardTermCfg(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.01)
    orientation = RewardTermCfg(func=mdp.flat_orientation_l2, weight=-2.5)


@configclass
class TerminationsCfg:
    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)
    base_contact = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={'sensor_cfg': SceneEntityCfg('contact_forces', body_names='base'), 'threshold': 1.0})


@configclass
class FlatGo2EnvCfg(ManagerBasedRLEnvCfg):
    scene = FlatSceneCfg(num_envs=1, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    commands = CommandsCfg()
    events = EventsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()

    def __post_init__(self):
        self.seed = 42
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        self.scene.contact_forces.update_period = self.sim.dt
