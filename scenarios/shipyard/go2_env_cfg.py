"""Bind Go2 observations and control to the shared shipyard layout."""
from isaaclab.utils import configclass
from scenarios.flat.go2_env_cfg import FlatGo2EnvCfg, FlatSceneCfg
from .scene_cfg import ShipyardSceneCfg, CAMERA_EYE, CAMERA_TARGET

_GO2_SCENE_CFG = FlatSceneCfg(num_envs=1, env_spacing=2.5)


@configclass
class ShipyardGo2SceneCfg(ShipyardSceneCfg):
    robot = _GO2_SCENE_CFG.robot.copy()
    contact_forces = _GO2_SCENE_CFG.contact_forces.copy()


@configclass
class ShipyardGo2EnvCfg(FlatGo2EnvCfg):
    scene = ShipyardGo2SceneCfg(num_envs=1, env_spacing=2.5)

    def __post_init__(self):
        super().__post_init__()
        self.viewer.eye = CAMERA_EYE
        self.viewer.lookat = CAMERA_TARGET
