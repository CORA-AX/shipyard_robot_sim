"""Shared robot-free ground and lighting."""
import isaaclab.sim as sim
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass


@configclass
class BaseSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground", terrain_type="plane", collision_group=-1,
        physics_material=sim.RigidBodyMaterialCfg(
            friction_combine_mode="multiply", restitution_combine_mode="multiply",
            static_friction=1.0, dynamic_friction=1.0),
        visual_material=sim.PreviewSurfaceCfg(diffuse_color=(0.35, 0.38, 0.40)),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight", spawn=sim.DomeLightCfg(intensity=750.0))
