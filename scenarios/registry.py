"""Lazy robot-free scene registry for the common viewer."""
from importlib import import_module

SCENARIOS = {
    'flat': 'scenarios.flat.scene_cfg',
    'shipyard': 'scenarios.shipyard.scene_cfg',
}


def load_scene(name):
    module = import_module(SCENARIOS[name])
    return module.SceneCfg(num_envs=1, env_spacing=2.5), module.CAMERA_EYE, module.CAMERA_TARGET
