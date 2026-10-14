"""Lazy registry for robot-free scenes and robot environment bindings."""
from importlib import import_module

DEFAULT_SCENARIO = 'warehouse'

SCENARIOS = {
    'flat': 'scenarios.flat.scene_cfg',
    'warehouse': 'scenarios.warehouse.scene_cfg',
}


def load_scene(name):
    module = import_module(SCENARIOS[name])
    return module.SceneCfg(num_envs=1, env_spacing=2.5), module.CAMERA_EYE, module.CAMERA_TARGET


GO2_SCENARIOS = {
    'flat': ('scenarios.flat.go2_env_cfg', 'FlatGo2EnvCfg'),
    'warehouse': ('scenarios.warehouse.go2_env_cfg', 'WarehouseGo2EnvCfg'),
}


def load_go2_scenario(name):
    module, config = GO2_SCENARIOS[name]
    return getattr(import_module(module), config)()
