"""End-to-end checks using temporary configs/logs, including non-IMU FMUs."""
import copy
import csv
import json
import math
import pathlib
import subprocess
import sys
import tempfile

exe, source, build = map(pathlib.Path, sys.argv[1:])
base = json.loads((source/'test_fmu.json').read_text())
base['environment']['simulation_time_s'] = .1
base['telemetry']['log_filename'] = 'log.csv'
base['external_modules']['imu_fmu']['path'] = str(build/'fmu/imu_sensor.fmu')

def run(config, expect_error=None):
    with tempfile.TemporaryDirectory() as tmp:
        root=pathlib.Path(tmp); path=root/'config.json'; path.write_text(json.dumps(config))
        result=subprocess.run([str(exe), str(path)], cwd=root, text=True, capture_output=True, timeout=30)
        if expect_error:
            assert result.returncode != 0, result.stdout
            assert expect_error in result.stderr, result.stderr
            return
        assert result.returncode == 0, result.stderr
        rows=list(csv.DictReader((root/'log.csv').open()))
        assert len(rows)==10, len(rows)
        assert all(math.isfinite(float(v)) for row in rows for v in row.values())
        return rows

run(base)
fast=copy.deepcopy(base); fast['scheduler']['base_rate_hz']=4000
fast['external_modules']['imu_fmu']['update_rate_hz']=800
run(fast)
bad=copy.deepcopy(fast); bad['scheduler']['base_rate_hz']=1000
run(bad, 'base_rate_hz must be a multiple')
bad=copy.deepcopy(base); bad['external_modules']['imu_fmu']['inputs']['in.gyro_x']='typo'
run(bad,'Unknown signal')
bad=copy.deepcopy(base); bad['sensors']['imu']['enabled']=True
run(bad,'Multiple IMU publishers')
native=copy.deepcopy(base); native.pop('external_modules'); native['sensors']['imu']['enabled']=True
native['scheduler']['execution_order'].remove('imu_fmu')
assert run(native)==run(native), 'Seeded native simulation must be repeatable'
custom=copy.deepcopy(native)
custom['external_modules']={'custom':{'path':str(build/'fmu/integrator.fmu'),'update_rate_hz':100,
    'parameters':{'gain':2},'inputs':{'drive':'truth.specific_force_body.z'},'outputs':{'integral':'custom.velocity'}}}
custom['scheduler']['execution_order'].insert(0,'custom')
run(custom)
tvc=copy.deepcopy(custom)
tvc['control']['tvc']['enabled']=False
tvc['external_modules']['custom']['publish']={'type':'tvc','pitch':'custom.velocity','yaw':'custom.velocity'}
run(tvc)
engine=copy.deepcopy(custom)
engine['control']['engine_controllers']=[]
engine['external_modules']['custom']['publish']={'type':'engine','engine_id':0,'throttle':'custom.velocity','active':'custom.velocity'}
run(engine)
print('CLI: FMU IMU, 800 Hz, native reproducibility, generic FMU, validation and CSV passed')
