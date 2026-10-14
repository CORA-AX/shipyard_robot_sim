"""Bind the current Go2 flat policy contract to the shared warehouse."""
from isaaclab.utils import configclass
from scenarios.flat.go2_env_cfg import FlatGo2EnvCfg, FlatSceneCfg
from .scene_cfg import WarehouseSceneCfg, CAMERA_EYE, CAMERA_TARGET, ROBOT_START

_GO2_SCENE_CFG = FlatSceneCfg(num_envs=1, env_spacing=2.5)


@configclass
class WarehouseGo2SceneCfg(WarehouseSceneCfg):
    robot = _GO2_SCENE_CFG.robot.copy()
    robot.init_state.pos = ROBOT_START
    contact_forces = _GO2_SCENE_CFG.contact_forces.copy()


@configclass
class WarehouseGo2EnvCfg(FlatGo2EnvCfg):
    scene = WarehouseGo2SceneCfg(num_envs=1, env_spacing=2.5)

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = CAMERA_EYE
        self.viewer.lookat = CAMERA_TARGET
