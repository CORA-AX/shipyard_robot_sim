"""Shared shipyard ground, lighting and cargo; no robot or policy imports."""
import isaaclab.sim as sim
from isaaclab.assets import AssetBaseCfg
from isaaclab.utils import configclass
from scenarios.common.scene_cfg import BaseSceneCfg

CAMERA_EYE = (8.0, 7.0, 5.0)
CAMERA_TARGET = (2.0, 0.0, 0.3)


def cargo_box(name, position, size, color):
    return AssetBaseCfg(
        prim_path=f'{{ENV_REGEX_NS}}/{name}',
        spawn=sim.CuboidCfg(
            size=size, collision_props=sim.CollisionPropertiesCfg(),
            visual_material=sim.PreviewSurfaceCfg(diffuse_color=color)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=position),
    )


@configclass
class ShipyardSceneCfg(BaseSceneCfg):
    cargo_left = cargo_box('CargoLeft', (3.0, 2.0, 0.5), (1.5, 1.0, 1.0), (0.8, 0.35, 0.1))
    cargo_right = cargo_box('CargoRight', (5.0, -2.0, 0.75), (2.0, 1.0, 1.5), (0.15, 0.4, 0.65))
    cargo_rear = cargo_box('CargoRear', (-2.0, -2.0, 0.4), (1.0, 1.0, 0.8), (0.6, 0.6, 0.15))



# Common viewer registry entry; existing robot bindings retain ShipyardSceneCfg.
SceneCfg = ShipyardSceneCfg
