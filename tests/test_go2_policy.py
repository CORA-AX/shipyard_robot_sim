"""Default policy selection and checkpoint integrity without Isaac Sim."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control.go2.policy import FlatPolicy


class PolicyChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'assets/go2/config').mkdir(parents=True)
        (self.root/'assets/go2/policies').mkdir(parents=True)
        self.original = self.write_checkpoint('flat.pt', -.5, 299)
        self.selected = self.write_checkpoint('selected.pt', .5, 10000)
        self.manifest = {'policy': self.spec(self.original), 'control_policy': self.spec(self.selected)}
        self.write_manifest()

    def write_checkpoint(self, name, bias, iteration):
        state = {}
        for index, inputs, outputs in [(0,48,128),(2,128,128),(4,128,128),(6,128,12)]:
            state[f'actor.{index}.weight'] = torch.zeros(outputs, inputs)
            state[f'actor.{index}.bias'] = torch.full((outputs,), bias if index == 6 else 0.)
        path = self.root/'assets/go2/policies'/name
        torch.save({'model_state_dict':state, 'iter':iteration}, path)
        return path

    def spec(self, path):
        return {'path':str(path.relative_to(self.root)),
                'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}

    def write_manifest(self):
        (self.root/'assets/go2/config/sources.lock.json').write_text(json.dumps(self.manifest))

    def test_default_selects_trained_policy_and_explicit_override_selects_original(self):
        selected = FlatPolicy(self.root)
        self.assertEqual(selected.iteration, 10000)
        self.assertEqual(selected.checkpoint_path, self.selected)
        self.assertEqual(selected.actions([0.]*48), [.5]*12)
        original = FlatPolicy(self.root, checkpoint=self.original)
        self.assertEqual(original.iteration, 299)
        self.assertEqual(original.actions([0.]*48), [-.5]*12)

    def test_original_manifest_remains_supported(self):
        self.manifest.pop('control_policy')
        self.write_manifest()
        self.assertEqual(FlatPolicy(self.root).checkpoint_path, self.original)

    def test_corrupt_default_is_rejected_before_loading(self):
        self.selected.write_bytes(b'corrupt checkpoint')
        with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
            FlatPolicy(self.root)

    def test_corrupt_registered_override_is_also_rejected(self):
        self.original.write_bytes(b'corrupt official checkpoint')
        with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
            FlatPolicy(self.root, checkpoint=self.original)

    def test_missing_selected_policy_does_not_silently_switch_models(self):
        self.selected.unlink()
        with self.assertRaisesRegex(ValueError, 'checkpoint not found'):
            FlatPolicy(self.root)

    def test_normalized_checkpoint_is_rejected(self):
        payload = torch.load(self.selected, weights_only=True)
        payload['obs_norm_state_dict'] = {'mean':torch.zeros(48)}
        torch.save(payload, self.selected)
        self.manifest['control_policy'] = self.spec(self.selected)
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'observation normalization'):
            FlatPolicy(self.root)

    def test_nonfinite_actor_is_rejected(self):
        payload = torch.load(self.selected, weights_only=True)
        payload['model_state_dict']['actor.6.bias'][0] = float('nan')
        torch.save(payload, self.selected)
        self.manifest['control_policy'] = self.spec(self.selected)
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, 'non-finite actor'):
            FlatPolicy(self.root)


if __name__ == '__main__':
    unittest.main()
