"""Lazy scenario loading avoids Kit imports during CLI setup."""
from importlib import import_module

SCENARIOS = {
    'flat': ('assets.go2.scenarios.flat_env_cfg', 'FlatGo2EnvCfg'),
    'shipyard': ('scenarios.shipyard.go2_env_cfg', 'ShipyardGo2EnvCfg'),
}


def load_scenario(name):
    module, config = SCENARIOS[name]
    return getattr(import_module(module), config)()
