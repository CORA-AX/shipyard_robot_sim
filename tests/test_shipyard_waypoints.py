"""Route topology, capability filtering and additive launcher regression."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/shipyard'))
from common import FLEET, HINTS, load_fleet
from route_plan import build_plan, distance_to_segment


class ShipyardRoutes(unittest.TestCase):
    def setUp(self):
        self.robots = load_fleet()['robots']

    def test_flat_round_trips_avoid_parked_robot(self):
        for robot in self.robots:
            plan = build_plan(HINTS,robot)
            points = plan['points']
            self.assertEqual(points[0],points[-1])
            self.assertEqual(points[0][:2],robot['position'][:2])
            self.assertTrue(all(p[2]==0 for p in points))
            self.assertTrue(all(math.dist(a,b)<=2.000001 for a,b in zip(points,points[1:])))
            other = next(r for r in self.robots if r!=robot)
            for a,b in zip(points,points[1:]):
                self.assertGreaterEqual(distance_to_segment(other['position'],a,b),1.)

    def test_capability_checks(self):
        with self.assertRaisesRegex(ValueError,'allow-ramps'):
            build_plan(HINTS,self.robots[0],'service_loop')
        plan = build_plan(HINTS,self.robots[0],'service_loop',True)
        self.assertAlmostEqual(max(p[2] for p in plan['points']),1.2)
        with self.assertRaisesRegex(ValueError,'Stairs'):
            build_plan(HINTS,self.robots[0],'stair_access',True)
        with self.assertRaisesRegex(ValueError,'Lekiwi'):
            build_plan(HINTS,self.robots[1],'service_loop',True)

    def test_missing_reverse_edge_is_rejected(self):
        hints = json.loads(HINTS.read_text())
        for edge in hints['edges']:
            edge['bidirectional'] = False
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'hints.json'
            path.write_text(json.dumps(hints))
            with self.assertRaisesRegex(ValueError,'No directed connection'):
                build_plan(path,self.robots[1])

    def test_existing_validator_still_rejects_shipyard(self):
        from robot_sim.config import load_world_config
        with self.assertRaisesRegex(ValueError,'flat or warehouse'):
            load_world_config(FLEET)

    def test_invalid_coordinates_and_spawns(self):
        for spacing in (0,float('nan'),10):
            with self.assertRaises(ValueError):
                build_plan(HINTS,self.robots[0],spacing=spacing)
        robot = dict(self.robots[0],position=[0,0,.4])
        with self.assertRaisesRegex(ValueError,'Parking'):
            build_plan(HINTS,robot)


if __name__=='__main__':
    unittest.main()
