"""World/map consistency and geometry raster checks; no Isaac app or physics."""
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent))
from hunter.environments.config import load_world
from hunter.environments.raster import export, rasterize
from hunter.navigation.maps import load_map
from hunter.navigation.nav2.config import FOOTPRINT, rviz_configuration
from hunter.control.controller import load_parameters
from hunter.sim.scene import prepare_hunter2_stage

try:
    from pxr import Usd
except ImportError:
    Usd = None


def plane(x0,y0,x1,y1,z=0.):
    return [[[x0,y0,z],[x1,y0,z],[x1,y1,z]], [[x0,y0,z],[x1,y1,z],[x0,y1,z]]]


class HunterWorldTests(unittest.TestCase):
    def test_profiles_share_absolute_coordinates_with_rviz(self):
        flat, yard = load_world(ROOT), load_world(ROOT,'shipyard')
        self.assertIsNone(flat.usd_path)
        self.assertEqual(flat.position,(0.,0.,.37))
        self.assertEqual(yard.position,(-16.,18.,.37))
        self.assertAlmostEqual(yard.yaw,math.pi)
        view = rviz_configuration('hunter2_0',yard)['Visualization Manager']['Views']['Current']
        self.assertEqual((view['X'],view['Y']),yard.view_center)
        with self.assertRaises(ValueError):
            load_world(ROOT,'warehouse')

    def test_pgm_row_order_negative_bounds_and_stale_source_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            profile = replace(load_world(ROOT), map_file=root/'map.yaml',bounds=(0.,0.,2.,2.),resolution=1.)
            raster = b'P5\n2 2\n255\n'+bytes([205,0,254,254])
            (root/'map.pgm').write_bytes(raster)
            (root/'source.usda').write_text('original')
            profile.map_file.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=1.,origin=[0.,0.,0.])))
            metadata = dict(image_sha256=hashlib.sha256(raster).hexdigest(),sources=[dict(
                path='source.usda',sha256=hashlib.sha256(b'original').hexdigest())])
            (root/'map.json').write_text(json.dumps(metadata))
            grid = load_map(root,profile)
            self.assertTrue(grid.free(.5,.5))
            self.assertTrue(grid.free(1.5,.5))
            for point in ((.5,1.5),(1.5,1.5),(-.01,.5),(2.,.5)):
                self.assertFalse(grid.free(*point))
            (root/'source.usda').write_text('edited')
            with self.assertRaisesRegex(ValueError,'stale'):
                load_map(root,profile)
            (root/'source.usda').write_text('original')
            (root/'map.pgm').write_bytes(raster[:-1]+b'\0')
            with self.assertRaisesRegex(ValueError,'image changed'):
                load_map(root,profile)

    def test_vertical_wall_blocks_cells_and_disconnects_floor(self):
        triangles = plane(0.,0.,2.,2.)+[
            [[1.,0.,0.],[1.,2.,0.],[1.,2.,1.]], [[1.,0.,0.],[1.,2.,1.],[1.,0.,1.]]]
        grid = rasterize(np.array(triangles),(0.,0.,3.,3.),.1,(.5,.5))
        self.assertEqual(grid[5,5],254)
        self.assertEqual(grid[5,9],0)  # Vertical face occupies both adjacent cells.
        self.assertEqual(grid[5,10],0)
        self.assertEqual(grid[5,15],205)  # Supported but disconnected by the wall.
        self.assertEqual(grid[25,25],205)  # No physical support.

    def test_raised_deck_and_ramp_do_not_become_flat_floor(self):
        triangles = plane(0.,0.,1.,1.)+plane(1.,0.,2.,1.,1.2)+[
            [[0.,1.,0.],[1.,1.,0.],[1.,2.,.2]], [[0.,1.,0.],[1.,2.,.2],[0.,2.,.2]]]
        grid = rasterize(np.array(triangles),(0.,0.,3.,3.),.1,(.5,.5))
        self.assertEqual(grid[5,5],254)
        self.assertNotEqual(grid[5,15],254)
        self.assertNotEqual(grid[15,5],254)

    def test_shipyard_map_contains_start_and_bypass_but_not_dock_or_deck(self):
        world = load_world(ROOT,'shipyard')
        grid = load_map(ROOT,world)
        for point in ((-16.,18.),(-22.,18.),(-25.,12.),(-25.,-18.)):
            self.assertTrue(grid.free(*point),point)
        for point in ((0.,0.),(-37.,0.),(-37.,8.)):
            self.assertFalse(grid.free(*point),point)
        self.assertTrue(grid.footprint_free(-15.71991,18.,math.pi,FOOTPRINT))
        self.assertTrue(grid.footprint_free(-25.,10.,-math.pi/2.,FOOTPRINT))
        self.assertFalse(grid.footprint_free(-37.,0.,0.,FOOTPRINT))

    @unittest.skipIf(Usd is None,'Requires OpenUSD CPU libraries.')
    def test_world_has_local_roads_and_saved_robot_uses_profile_spawn(self):
        world = load_world(ROOT,'shipyard')
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim('/World/Environment','Xform').GetReferences().AddReference(str(world.usd_path))
        self.assertTrue(stage.GetPrimAtPath('/World/Environment/RoadNetwork'))
        self.assertFalse(stage.GetPrimAtPath('/World/Environment/warehouse_reference'))
        self.assertFalse(stage.GetPrimAtPath('/World/ground'))
        errors = [str(e) for p in stage.Traverse() for e in p.GetPrimIndex().localErrors]
        self.assertEqual(errors,[])
        self.assertTrue(all(not layer.realPath.startswith('http') for layer in stage.GetUsedLayers()))
        robot = load_parameters(ROOT)
        stage.DefinePrim('/World/Robot','Xform').GetReferences().AddReference(str(ROOT/robot['usd_path']))
        details = prepare_hunter2_stage(stage,'/World/Robot',robot,world.position,world.yaw)
        self.assertAlmostEqual(details['rear_axle_offset_base_m'][0],-.28009,places=5)

    @unittest.skipIf(Usd is None,'Requires OpenUSD CPU libraries.')
    def test_map_regeneration_is_reproducible_and_preserves_source_layers(self):
        world = load_world(ROOT,'shipyard')
        metadata = json.loads(world.map_file.with_suffix('.json').read_text())
        sources = {ROOT/item['path']:item['sha256'] for item in metadata['sources']}
        with tempfile.TemporaryDirectory() as folder:
            result = export(ROOT,replace(world,map_file=Path(folder)/'shipyard.yaml'))
            self.assertEqual(result['image_sha256'],metadata['image_sha256'])
        self.assertTrue(all(hashlib.sha256(path.read_bytes()).hexdigest()==digest for path,digest in sources.items()))


if __name__ == '__main__':
    unittest.main()
