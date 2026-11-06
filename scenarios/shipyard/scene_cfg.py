"""Load the local, editable shipyard world in meters and Z-up coordinates."""
from pathlib import Path

from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from scenarios.common.scene_cfg import BaseSceneCfg

WORLD_USD = Path(__file__).resolve().parents[2] / 'assets/environments/shipyard/world.usda'
CAMERA_EYE = (48.0, 58.0, 42.0)
CAMERA_TARGET = (2.0, 0.0, 0.0)
# Yard apron near the rear entrance, away from the recessed dry dock.
ROBOT_START = (-16.0, 18.0, 0.4)
_BASE = BaseSceneCfg(num_envs=1, env_spacing=2.5)


@configclass
class ShipyardSceneCfg(BaseSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path='/World/ShipyardEnvironment', terrain_type='usd',
        usd_path=str(WORLD_USD), collision_group=-1,
        physics_material=_BASE.terrain.physics_material.copy(), visual_material=None,
    )
    # Lighting is persisted in layout.usda with the rest of the environment.
    sky_light = None


SceneCfg = ShipyardSceneCfg
