import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml
from scenario_loader import compile_scenario, ConfigError

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT/'examples/component_config'
EXE = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else ROOT/'build/simRocket'
if len(sys.argv)>1: del sys.argv[1:]


class LoaderTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)

    def write(self, name, obj):
        path=self.root/name;path.write_text(yaml.safe_dump(obj,sort_keys=False));return path

    def case(self, overrides=None, fmu=False):
        doc={'extends':str(EXAMPLES/'scenarios/common.yaml'),
             'simulation':{'duration_s':.1},
             'record':{'directory':str(self.root/'logs')},
             'components':{'imu':{'model':str(EXAMPLES/('models/fmu_imu.yaml' if fmu else 'models/native_imu.yaml'))}}}
        from scenario_loader import merge
        return self.write('scenario.yaml',merge(doc,overrides or {}))

    def rejects(self, overrides, message):
        with self.assertRaisesRegex(ConfigError,message): compile_scenario(self.case(overrides))

    def test_native_and_fmu_share_interfaces_and_parameters(self):
        native=compile_scenario(self.case());fmu=compile_scenario(self.case(fmu=True))
        self.assertEqual(native['rocket'],fmu['rocket'])
        self.assertEqual(native['component_runtime']['connections'],fmu['component_runtime']['connections'])
        self.assertEqual(native['component_runtime']['imu_model']['ports'],fmu['component_runtime']['imu_model']['ports'])
        self.assertFalse(fmu['sensors']['imu']['enabled']);self.assertTrue(native['sensors']['imu']['enabled'])
        self.assertEqual(fmu['external_modules']['imu_fmu']['sample_time_policy'],'latched_input')

    def test_override_parameters_and_source_relative_paths(self):
        config=compile_scenario(self.case({'components':{'imu':{'parameters':{'gyro_noise_std_rad_s':.02}}}},fmu=True))
        module=config['external_modules']['imu_fmu']
        self.assertEqual(module['parameters']['param.gyro_noise_std'],.02)
        self.assertEqual(module['parameters']['param.accel_noise_std'],0)
        self.assertEqual(module['path'],str(ROOT/'build/fmu/imu_sensor.fmu'))

    def test_unknown_field_and_parameter(self):
        self.rejects({'simulation':{'duraton_s':1}},'unknown fields')
        self.rejects({'components':{'imu':{'parameters':{'typo':1}}}},'unknown fields')

    def test_duplicate_keys(self):
        path=self.root/'bad.yaml';path.write_text('schema: one\nschema: two\n')
        with self.assertRaisesRegex(ConfigError,'duplicate'):compile_scenario(path)

    def test_cycle_and_alias(self):
        path=self.write('cycle.yaml',{'extends':'cycle.yaml'})
        with self.assertRaisesRegex(ConfigError,'cycle'):compile_scenario(path)
        path.write_text('a: &a [1]\nb: *a\n')
        with self.assertRaisesRegex(ConfigError,'aliases'):compile_scenario(path)

    def test_unknown_factory_and_topology(self):
        self.rejects({'components':{'controller':{'model':'native/unknown'}}},'controller.model')
        system=yaml.safe_load((EXAMPLES/'systems/rocket_tvc.yaml').read_text())
        system['components']['imu']['model']=str(EXAMPLES/'models/native_imu.yaml')
        system['topology']='wrong'
        self.rejects({'system':str(self.write('system.yaml',system))},'topology')

    def test_invalid_time_and_unsupported_independent_rates(self):
        self.rejects({'components':{'imu':{'rate_hz':800}}},'multiple')
        self.assertEqual(compile_scenario(self.case({'components':{'estimator':{'rate_hz':100}}}))['component_runtime']['components']['estimator']['rate_hz'],100)
        self.rejects({'simulation':{'duration_s':float('nan')}},'finite')
        self.rejects({'simulation':{'duration_s':.1001}},'align')

    def test_prior_and_initial_state_validation(self):
        self.rejects({'estimator_prior':{'orientation_body_to_world_wxyz':[0,0,0,0]}},'unit norm')
        self.rejects({'initial_state':{'position_world_m':[0,0,-1]}},'nonnegative')

    def test_record_and_policies_not_silently_ignored(self):
        self.rejects({'record':{'signals':['unknown']}},'record.signals')
        self.rejects({'components':{'estimator':{'inputs':{'imu':{'read':'latest'}}}}},'imu.read')

    def test_fmu_binding_errors(self):
        original=yaml.safe_load((EXAMPLES/'models/fmu_imu.yaml').read_text())
        original['implementation']['file']=str(ROOT/'build/fmu/imu_sensor.fmu')
        for change,message in [('name','Unknown FMI'),('frame','frame'),('unit','unit'),('direction','expected input Real'),('time','timing')]:
            with self.subTest(change=change):
                model=copy.deepcopy(original);binding=model['bindings']['truth.angular_velocity_body']
                if change=='name':binding['variables'][0]='absent'
                elif change=='direction':binding['variables'][0]='out.gyro_x'
                elif change=='time':model['timing']['sample_time']='step_end'
                else:binding[change]='wrong'
                path=self.write('model.yaml',model)
                with self.assertRaisesRegex(ConfigError,message):compile_scenario(self.case({'components':{'imu':{'model':str(path)}}}))

    def command(self, path, *args, success=True):
        result=subprocess.run([str(EXE),str(path),*args],cwd=self.root,capture_output=True,text=True,timeout=40)
        self.assertEqual(result.returncode,0 if success else 1,result.stdout+result.stderr)
        return result

    def samples(self, result):
        prefix='[simRocket] Run directory: '
        directory=next(line[len(prefix):] for line in result.stdout.splitlines() if line.startswith(prefix))
        self.assertTrue((Path(directory)/'resolved.json').exists())
        return [json.loads(line) for line in (Path(directory)/'samples.jsonl').read_text().splitlines()]

    def test_cli_validate_dump_do_not_create_logs(self):
        path=self.case(fmu=True)
        self.command(path,'--validate')
        resolved=json.loads(self.command(path,'--dump-config').stdout)
        self.assertEqual(resolved['component_runtime']['runtime'],'phased-preset-v1')
        self.assertFalse((self.root/'logs').exists())

    def test_cli_native_fmu_and_translated_json(self):
        for fmu in (False,True):
            with self.subTest(fmu=fmu):
                path=self.case(fmu=fmu)
                first=self.samples(self.command(path));self.assertEqual(len(first),10)
                self.assertEqual(set(first[0]),{'time_s','truth','imu.measurement','estimator.state','actuator.state'})
                for row in first[1:]:
                    imu=row['imu.measurement']
                    self.assertLessEqual(imu['available_time_s'],row['time_s'])
                    self.assertAlmostEqual(imu['available_time_s']-imu['sample_time_s'],.001 if fmu and imu['sample_time_s'] > 0 else 0)
                legacy=self.root/'translated.json';legacy.write_text(json.dumps(compile_scenario(path)))
                self.assertEqual(first,self.samples(self.command(legacy)))

    def test_phased_observation_and_multirate_estimator(self):
        path=self.case({'simulation':{'base_rate_hz':4000}, 'components':{
            'imu':{'rate_hz':800},
            'estimator':{'rate_hz':100,'inputs':{'imu':{'max_age_s':.03}}},
            'actuator':{'rate_hz':4000}, 'propulsion':{'rate_hz':4000},
            'dynamics':{'rate_hz':4000}}})
        rows=self.samples(self.command(path))
        self.assertEqual(rows[0]['time_s'],0)
        self.assertEqual(rows[0]['truth']['pos'],[0,0,0])
        for row in rows:
            self.assertLessEqual(row['imu.measurement']['available_time_s'],row['time_s'])
            self.assertLessEqual(row['estimator.state']['sample_time_s'],row['time_s'])
        self.assertAlmostEqual(rows[-1]['estimator.state']['sample_time_s'],.09)

    def test_bus_trace_captures_publications_without_changing_results(self):
        path=self.case()
        baseline=self.samples(self.command(path))
        trace=self.root/'trace with spaces'/'bus.jsonl'
        traced=self.samples(self.command(path,'--log-bus',str(trace)))
        self.assertEqual(baseline,traced)
        rows=[json.loads(line) for line in trace.read_text().splitlines()]
        imu=[row for row in rows if row['source']=='imu.measurement']
        self.assertEqual(len(imu),100)  # full 1000 Hz, despite telemetry at 100 Hz
        self.assertEqual([row['sequence'] for row in imu],list(range(100)))
        self.assertEqual(imu[0]['payload']['angular_velocity_body_rad_s'],[0,0,0])
        self.assertEqual(imu[-1]['sample_time_ns'],99000000)
        self.assertEqual({row['transport'] for row in rows},{'bus','direct'})
        for row in rows:
            self.assertLessEqual(row['sample_time_ns'],row['available_time_ns'])
            self.assertEqual(row['quality'],'valid')
            self.assertIsNone(row['valid_until_ns'])
        original=trace.read_bytes()
        self.assertIn('already exists',self.command(path,'--log-bus',str(trace),success=False).stderr)
        self.assertEqual(trace.read_bytes(),original)

    def test_bus_trace_preserves_fmu_time_and_validation_is_readonly(self):
        path=self.case(fmu=True);trace=self.root/'trace.jsonl'
        self.command(path,'--log-bus',str(trace),'--validate')
        self.assertFalse(trace.exists())
        self.command(path,'--log-bus',str(trace))
        imu=[json.loads(line) for line in trace.read_text().splitlines()
             if json.loads(line)['source']=='imu.measurement']
        self.assertEqual(len(imu),99)
        self.assertEqual(imu[0]['available_time_ns'],0)
        for row in imu[1:]:
            self.assertEqual(row['available_time_ns']-row['sample_time_ns'],1000000)
        self.assertIn('requires an output',self.command(path,'--log-bus',success=False).stderr)

    def test_orchestrator_forwards_bus_log_path(self):
        path=self.case(); trace=self.root/'orchestrator trace.jsonl'
        result=subprocess.run([sys.executable,str(ROOT/'run_sim.py'),str(path),
                               '--log-bus',str(trace)],cwd=ROOT,capture_output=True,text=True,timeout=40)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(len(self.samples(result)),10)
        self.assertTrue(trace.is_file())

    def test_runtime_prior_and_selection(self):
        path=self.case({'estimator_prior':{'position_world_m':[12,13,14],'velocity_world_m_s':[1,2,3]},
                        'record':{'signals':['estimator.state']}})
        row=self.samples(self.command(path))[0]
        self.assertEqual(set(row),{'time_s','estimator.state'})
        self.assertEqual(row['estimator.state']['pos'],[12,13,14])
        self.assertEqual(row['estimator.state']['vel'],[1,2,3])

    def test_runtime_stale_data_fails(self):
        path=self.case({'components':{'estimator':{'inputs':{'imu':{'max_age_s':0}}}}},fmu=True)
        result=self.command(path,success=False)
        self.assertIn('estimator.imu: stale',result.stderr)

    def test_invalid_yaml_does_not_start_simulation(self):
        path=self.case({'components':{'imu':{'parameters':{'typo':1}}}})
        result=self.command(path,success=False)
        self.assertIn('typo',result.stderr)
        self.assertNotIn('Entering Hot-Loop',result.stdout)
        self.assertFalse((self.root/'logs').exists())


if __name__=='__main__': unittest.main()
