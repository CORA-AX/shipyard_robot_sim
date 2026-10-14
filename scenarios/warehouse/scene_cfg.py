"""The upstream warehouse layout, using the Isaac Sim 5.0 asset release."""
import json
from pathlib import Path

from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from scenarios.common.scene_cfg import BaseSceneCfg

_SOURCE = json.loads((Path(__file__).resolve().parents[2]
                      / 'assets/environments/warehouse/sources.lock.json').read_text())
WAREHOUSE_URL = _SOURCE['warehouse']['url']
WAREHOUSE_PRIM_PATH = '/World/Warehouse'
CAMERA_EYE = (4.0, -6.0, 3.5)
CAMERA_TARGET = (0.0, 1.0, 0.4)
ROBOT_START = (0.0, 0.0, 0.4)
_BASE_SCENE = BaseSceneCfg(num_envs=1, env_spacing=2.5)


@configclass
class WarehouseSceneCfg(BaseSceneCfg):
    # The USD supplies its own collision plane, walls, racks and materials.
    # Import the whole scene and avoid a second overlapping ground plane.
    terrain = TerrainImporterCfg(
        prim_path=WAREHOUSE_PRIM_PATH, terrain_type='usd', usd_path=WAREHOUSE_URL,
        collision_group=-1, physics_material=_BASE_SCENE.terrain.physics_material.copy(),
        visual_material=None,
    )


SceneCfg = WarehouseSceneCfg
