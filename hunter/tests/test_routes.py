"""Stored round trips use supported ground and the rear-axle start reference."""
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
from hunter.environments.config import load_world
from hunter.navigation.maps import load_map
from hunter.navigation.routes import load_route, patrol_parameters, route_path_clear
from hunter.ros.nav2.runner import arguments


class RouteTests(unittest.TestCase):
    def test_shipyard_routes_order_return_and_start_guard(self):
        grid = load_map(ROOT, load_world(ROOT, 'shipyard'))
        route = load_route(ROOT, 'flat_only', grid)
        self.assertEqual([p.name for p in route.points], [
            'north_junction', 'bypass_access', 'south_junction',
            'bypass_return', 'north_return', 'parking'])
        self.assertEqual((route.points[-1].x, route.points[-1].y), route.start)
        self.assertTrue(route.at_start(-15.71991, 18.00001))
        self.assertFalse(route.at_start(-16.2, 18.0))
        self.assertEqual(len(load_route(ROOT, 'smoke', grid).points), 2)

    def test_invalid_ground_and_non_round_trip_rejected(self):
        grid = load_map(ROOT, load_world(ROOT, 'shipyard'))
        original = yaml.safe_load((ROOT/'config/routes/shipyard.yaml').read_text())
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            folder = root/'config/routes'
            folder.mkdir(parents=True)
            for bad in ((0., 0.), (float('nan'), 18.), (-22., 18.)):
                original['routes']['smoke'][-1]['xy'] = list(bad)
                (folder/'shipyard.yaml').write_text(yaml.safe_dump(original))
                with self.assertRaises(ValueError):
                    load_route(root, 'smoke', grid)

    def test_route_cli_requires_matching_world(self):
        from contextlib import redirect_stderr
        from io import StringIO
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            arguments(ROOT, ['--route', 'smoke'])
        args, _, _ = arguments(ROOT, ['--environment','shipyard','--route','smoke','--route-idle'])
        self.assertTrue(args.route_idle)

    def test_patrol_clearance_does_not_modify_click_mode_parameters(self):
        _, _, original = arguments(ROOT, [])
        result = patrol_parameters(original, 'hunter2_0')
        for costmap in ('global_costmap', 'local_costmap'):
            key = f'/hunter2_0/{costmap}/{costmap}'
            self.assertEqual(original[key]['ros__parameters']['footprint_padding'], .02)
            self.assertEqual(result[key]['ros__parameters']['footprint_padding'], .20)
            self.assertTrue(result[key]['ros__parameters']['inflation_layer']['inflate_around_unknown'])
            self.assertNotIn('inflate_around_unknown', original[key]['ros__parameters']['inflation_layer'])

    def test_center_on_road_does_not_suffice_for_full_vehicle_clearance(self):
        grid = load_map(ROOT, load_world(ROOT, 'shipyard'))
        self.assertTrue(grid.free(-22.88, 14.36))
        self.assertFalse(route_path_clear(grid, [(-22.88, 14.36, -.06)]))
        self.assertTrue(route_path_clear(grid, [(-25., 12., -1.5707963267948966)]))


if __name__ == '__main__':
    unittest.main()
