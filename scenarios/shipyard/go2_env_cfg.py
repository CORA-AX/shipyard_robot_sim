"""Bind the existing Go2 controller to the persistent shipyard environment."""
from isaaclab.utils import configclass
from scenarios.flat.go2_env_cfg import FlatGo2EnvCfg, FlatSceneCfg
from .scene_cfg import ShipyardSceneCfg, CAMERA_EYE, CAMERA_TARGET, ROBOT_START

_GO2 = FlatSceneCfg(num_envs=1, env_spacing=2.5)


@configclass
class ShipyardGo2SceneCfg(ShipyardSceneCfg):
    robot = _GO2.robot.copy()
    robot.init_state.pos = ROBOT_START
    contact_forces = _GO2.contact_forces.copy()


@configclass
class ShipyardGo2EnvCfg(FlatGo2EnvCfg):
    scene = ShipyardGo2SceneCfg(num_envs=1, env_spacing=2.5)

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = CAMERA_EYE
        self.viewer.lookat = CAMERA_TARGET
