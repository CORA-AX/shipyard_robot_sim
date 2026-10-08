"""Lazy registry for robot-free scenes and robot environment bindings."""
from importlib import import_module

SCENARIOS = {
    'flat': 'scenarios.flat.scene_cfg',
    'shipyard': 'scenarios.shipyard.scene_cfg',
}


def load_scene(name):
    module = import_module(SCENARIOS[name])
    return module.SceneCfg(num_envs=1, env_spacing=2.5), module.CAMERA_EYE, module.CAMERA_TARGET


GO2_SCENARIOS = {
    'flat': ('scenarios.flat.go2_env_cfg', 'FlatGo2EnvCfg'),
    'shipyard': ('scenarios.shipyard.go2_env_cfg', 'ShipyardGo2EnvCfg'),
}


def load_go2_scenario(name):
    module, config = GO2_SCENARIOS[name]
    return getattr(import_module(module), config)()
