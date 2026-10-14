"""Check gait metrics distinguish trot, static stance, pace and reset gaps."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl.go2.gait_metrics import FOOT_NAMES, summarize_gait


class GaitMetricsTests(unittest.TestCase):
    def summarize(self, contact):
        height = np.where(contact > .5, .025, .075)
        speed = np.where(contact > .5, .02, .5)
        return summarize_gait(contact, height, speed, .02)

    def test_trot_has_opposite_diagonals_matching_events_and_measured_cycles(self):
        phase = (np.arange(100) // 10) % 2
        contact = np.stack([phase, 1 - phase, 1 - phase, phase], axis=-1)[:, None, :].astype(float)
        result = self.summarize(contact)
        self.assertEqual(result['alternating_diagonal_support_fraction'], 1.)
        for foot in FOOT_NAMES:
            self.assertAlmostEqual(result['feet'][foot]['cycle_period_s']['mean'], .4)
            self.assertAlmostEqual(result['feet'][foot]['swing_peak_lift_above_stance_m']['mean'], .05)
            self.assertAlmostEqual(result['feet'][foot]['stance_slip_m_s']['mean'], .02)
        for entry in result['diagonal_event_delay_s'].values():
            self.assertEqual(entry['matched_delay']['mean'], 0.)
            self.assertEqual(entry['unmatched_events'], 0)

    def test_static_stance_agreement_does_not_count_as_trot(self):
        result = self.summarize(np.ones((30, 1, 4)))
        self.assertEqual(result['alternating_diagonal_support_fraction'], 0.)
        self.assertEqual(result['all_stance_fraction'], 1.)
        for foot in FOOT_NAMES:
            self.assertIsNone(result['feet'][foot]['cycle_period_s'])

    def test_pace_is_not_diagonal_support(self):
        phase = (np.arange(60) // 10) % 2
        c = np.stack([phase, 1 - phase, phase, 1 - phase], axis=-1)[:, None, :]
        self.assertEqual(self.summarize(c)['alternating_diagonal_support_fraction'], 0.)

    def test_reset_gap_does_not_create_cycle_or_pair_events_across_gap(self):
        c = np.ones((12, 1, 4))
        c[2:4, 0, [0, 3]] = 0.
        c[4:8] = np.nan
        c[8:10, 0, [0, 3]] = 0.
        result = self.summarize(c)
        self.assertEqual(result['valid_frames'], 8)
        self.assertIsNone(result['feet']['FL_foot']['cycle_period_s'])
        self.assertIsNone(result['feet']['FL_foot']['swing_duration_s'])

    def test_all_invalid_is_serializable_without_nan(self):
        import json
        result = self.summarize(np.full((10, 1, 4), np.nan))
        self.assertEqual(result['valid_frames'], 0)
        json.dumps(result, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
