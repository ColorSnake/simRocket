"""Configuration hand-off checks without requiring a ROS installation."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_sim import prepare_config

class ConfigHandoff(unittest.TestCase):
    def test_yaml_resolved_once_and_shared(self):
        with tempfile.TemporaryDirectory(prefix='scenario with spaces ') as directory:
            snapshot = Path(directory) / 'fmu.json'
            data = {'telemetry': {}, 'visuals': {'rocket_length_m': 7},
                    'external_modules': {'imu': {'path': '/absolute/sensor.fmu'}}}
            with patch('run_sim.subprocess.run') as run:
                run.return_value.stdout = json.dumps(data)
                selected = prepare_config('examples/component_config/scenarios/fmu_imu.yaml', snapshot)
            self.assertEqual(selected, str(snapshot))
            self.assertEqual(json.loads(snapshot.read_text())['visuals'], data['visuals'])
            self.assertTrue(Path(json.loads(snapshot.read_text())['telemetry']['log_filename']).is_absolute())
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.args[0][-1], '--dump-config')

    def test_json_preserves_relative_asset_base(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'original.json'
            source.write_text('{"visuals":{"rocket_length_m":9}}')
            snapshot = Path(directory) / 'bridge.json'
            self.assertEqual(prepare_config(source, snapshot), str(source))
            self.assertEqual(json.loads(snapshot.read_text()), json.loads(source.read_text()))

    def test_invalid_config_does_not_create_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'bad.json'; source.write_text('{')
            snapshot = Path(directory) / 'bridge.json'
            with self.assertRaises(ValueError): prepare_config(source, snapshot)
            self.assertFalse(snapshot.exists())

if __name__ == '__main__': unittest.main()
