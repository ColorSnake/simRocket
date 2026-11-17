"""Compile component YAML into a validated configuration for the transitional runtime.

No FMU code is executed here. Only the rocket-tvc/v1 preset is supported; this
is intentionally not the future graph scheduler or communication middleware.
"""
import copy
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile
import yaml


class ConfigError(ValueError):
    pass


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    result = {}
    for key, value in node.value:
        name = loader.construct_object(key, deep=deep)
        if not isinstance(name, str) or name in result:
            raise ConfigError(f"{node.start_mark}: duplicate or non-string key: {name}")
        result[name] = loader.construct_object(value, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def fields(obj, allowed, where, required=None):
    if not isinstance(obj, dict):
        raise ConfigError(f"{where}: expected mapping")
    unknown = obj.keys() - set(allowed.split())
    missing = set((required if required is not None else allowed).split()) - obj.keys()
    if unknown or missing:
        raise ConfigError(f"{where}: unknown fields {sorted(unknown)}, missing fields {sorted(missing)}")
    return obj


def number(value, where, minimum=None, positive=False):
    if type(value) not in (float, int) or not math.isfinite(value):
        raise ConfigError(f"{where}: expected finite number")
    if (minimum is not None and value < minimum) or (positive and value <= 0):
        raise ConfigError(f"{where}: number outside allowed range")
    return value


def vector(value, where, n=3):
    if not isinstance(value, list) or len(value) != n:
        raise ConfigError(f"{where}: expected {n} numbers")
    return [number(v, where) for v in value]


def quaternion(value, where):
    values = vector(value, where, 4)
    norm = math.sqrt(sum(v*v for v in values))
    if abs(norm-1) > 1e-6:
        raise ConfigError(f"{where}: quaternion must have unit norm")
    return [v/norm for v in values]


def expect(value, expected, where):
    if value != expected:
        raise ConfigError(f"{where}: supported value is {expected!r}, received {value!r}")


def merge(base, override):
    result = copy.deepcopy(base)
    for k, v in override.items():
        result[k] = merge(result[k], v) if isinstance(v, dict) and isinstance(result.get(k), dict) else copy.deepcopy(v)
    return result


def load(path):
    path = Path(path).resolve()
    try:
        if path.stat().st_size > 1024*1024:
            raise ConfigError(f"{path}: YAML exceeds 1 MiB")
        text = path.read_text()
        depth = 0
        for event in yaml.parse(text):
            if isinstance(event, yaml.AliasEvent):
                raise ConfigError(f"{path}: YAML aliases are not supported; use extends")
            if isinstance(event, (yaml.MappingStartEvent, yaml.SequenceStartEvent)):
                depth += 1
                if depth > 32: raise ConfigError(f"{path}: YAML nesting limit exceeded")
            if isinstance(event, (yaml.MappingEndEvent, yaml.SequenceEndEvent)): depth -= 1
        doc = yaml.load(text, Loader=UniqueLoader)
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f"{path}: {error}") from error
    if not isinstance(doc, dict): raise ConfigError(f"{path}: expected mapping")
    # Resolve at declaration, before inheritance; preserve each file's base directory.
    def resolve(obj):
        if isinstance(obj, dict):
            for k, v in obj.items():
                if k in ('extends', 'vehicle', 'system', 'file', 'directory') or (k == 'model' and isinstance(v, str) and v.endswith(('.yaml', '.yml'))):
                    if not isinstance(v, str) or not v: raise ConfigError(f"{path}: invalid path in {k}")
                    obj[k] = str((path.parent/v).resolve())
                else: resolve(v)
        elif isinstance(obj, list):
            for v in obj: resolve(v)
    resolve(doc)
    return doc


SCENARIO_FIELDS = 'extends schema vehicle system simulation environment initial_state estimator_prior record components'


def scenario(path, seen=()):
    path = Path(path).resolve()
    if path in seen or len(seen) >= 8: raise ConfigError(f"{path}: inheritance cycle or excessive depth")
    doc = load(path)
    fields(doc, SCENARIO_FIELDS, str(path), '')
    if 'extends' in doc:
        parent = doc.pop('extends')
        doc = merge(scenario(parent, (*seen, path)), doc)
    return doc


PORTS = {
    'truth': {'direction': 'input', 'interface': 'ImuTruth/v1', 'transport': 'direct'},
    'measurement': {'direction': 'output', 'interface': 'ImuMeasurement/v1', 'transport': 'bus'},
}
PARAMS = ('gyro_noise_std_rad_s', 'accel_noise_std_m_s2')
NATIVE_TIMING = dict(mode='sampled', feedthrough=True, sample_time='truth.sample_time', available_time='activation_time')
FMU_TIMING = dict(mode='co_simulation', feedthrough=False, sample_time='latched_truth.sample_time', available_time='step_end',
                  initialization=dict(inputs='initial_truth', outputs='read_after_initialization', sample_time='initial_truth.sample_time',
                                      available_time='simulation_start', repeated_sample_time='discard'))
# An inspectable description of the preset. Runtime currently realizes these
# connections through existing native composition/adapters, not a generic graph.
CONNECTIONS = [
    ('dynamics.imu_truth','imu.truth','ImuTruth/v1','direct'),
    ('dynamics.state','gps.truth','KinematicTruth/v1','direct'),
    ('imu.measurement','estimator.imu','ImuMeasurement/v1','avionics'),
    ('gps.measurement','estimator.gps','GpsMeasurement/v1','avionics'),
    ('estimator.state','controller.state','StateEstimate/v1','avionics'),
    ('controller.demand','allocator.demand','TvcDemand/v1','avionics'),
    ('allocator.setpoint','actuator.setpoint','ActuatorSetpoint/v1','direct'),
    ('actuator.state','propulsion.actuator_state','ActuatorState/v1','direct'),
    ('propulsion.wrench','dynamics.propulsion_wrench','Wrench/v1','direct'),
    ('propulsion.mass_flow','dynamics.propellant_flow','PropellantFlow/v1','direct'),
    ('dynamics.propellant','propulsion.propellant','PropellantState/v1','direct'),
    ('dynamics.mass_properties','propulsion.mass_properties','MassProperties/v1','direct'),
    ('dynamics.ambient_pressure','propulsion.ambient_pressure','AmbientPressure/v1','direct'),
]


def model_config(path, parameters):
    model = load(path)
    fields(model, 'schema id role category implementation ports parameters timing bindings', 'imu model', 'schema id role category implementation ports parameters timing')
    expect(model['schema'], 'simrocket/model-v1', 'model.schema')
    if not isinstance(model['id'], str) or not model['id']: raise ConfigError('model.id: expected nonempty string')
    expect(model['role'], 'imu', 'model.role'); expect(model['category'], 'plant', 'model.category')
    expect(model['ports'], PORTS, 'model.ports')
    impl = model['implementation']; kind = impl.get('kind') if isinstance(impl, dict) else None
    if kind == 'native':
        fields(impl, 'kind factory', 'implementation'); expect(impl['factory'], 'imu', 'factory')
        expect(model['timing'], NATIVE_TIMING, 'native timing')
        if 'bindings' in model: raise ConfigError('Native IMU cannot declare FMU bindings')
    elif kind == 'fmu':
        fields(impl, 'kind file fmi_version mode platform', 'implementation')
        for key, val in [('fmi_version','2.0'),('mode','co_simulation'),('platform','linux64')]: expect(impl[key], val, key)
        expect(model['timing'], FMU_TIMING, 'FMU timing')
    else: raise ConfigError('IMU implementation.kind must be native or fmu')
    fields(model['parameters'], ' '.join(PARAMS), 'model.parameters')
    fields(parameters, ' '.join(PARAMS), 'imu.parameters', '')
    values = {}
    for name, desc in model['parameters'].items():
        fields(desc, 'type default minimum' + (' variable' if kind == 'fmu' else ''), name)
        expect(desc['type'], 'Real', name+'.type'); expect(desc['minimum'], 0, name+'.minimum')
        number(desc['default'], name+'.default', minimum=0)
        values[name] = number(parameters.get(name, desc['default']), name, minimum=0)
    module = None
    if kind == 'fmu':
        try:
            with zipfile.ZipFile(impl['file']) as archive:
                info = archive.getinfo('modelDescription.xml')
                if info.file_size > 4*1024*1024: raise ConfigError('FMU XML exceeds 4 MiB')
                xml = ET.fromstring(archive.read(info))
                expect(xml.get('fmiVersion'), '2.0', 'FMU XML version')
                cs = xml.find('CoSimulation')
                if cs is None: raise ConfigError('FMU does not declare CoSimulation')
                identifier = cs.get('modelIdentifier','')
                if not identifier or not identifier.isascii() or not identifier.replace('_','a').isalnum() or identifier[0].isdigit():
                    raise ConfigError('Invalid FMU modelIdentifier')
                if not xml.get('guid'): raise ConfigError('FMU GUID missing')
                if cs.get('canRunAsynchronuously','false') not in ('false','0'): raise ConfigError('Asynchronous FMU unsupported')
                archive.getinfo(f'binaries/linux64/{identifier}.so')
                variables = {}
                for var in xml.findall('./ModelVariables/ScalarVariable'):
                    name = var.get('name')
                    if not name or name in variables: raise ConfigError('Duplicate/empty FMU variable name')
                    variables[name] = var
        except (OSError, KeyError, zipfile.BadZipFile, ET.ParseError) as error:
            raise ConfigError(f"{impl['file']}: invalid FMU archive/metadata: {error}") from error
        def check_var(name, causality, unit=None):
            if not isinstance(name, str) or name not in variables: raise ConfigError(f'Unknown FMI variable: {name}')
            var = variables[name]; real = var.find('Real')
            if var.get('causality') != causality or real is None: raise ConfigError(f'{name}: expected {causality} Real')
            xml_unit = real.get('unit')
            if real.get('declaredType'): raise ConfigError(f'{name}: inherited type metadata not yet supported')
            if unit and xml_unit and xml_unit != unit: raise ConfigError(f'{name}: unit mismatch ({xml_unit} vs {unit})')
        mapping = {
            'truth.angular_velocity_body': ('input', 'rad/s', 'truth.gyro_body'),
            'truth.specific_force_body': ('input', 'm/s^2', 'truth.specific_force_body'),
            'measurement.angular_velocity_body': ('output', 'rad/s', 'external.imu.gyro'),
            'measurement.specific_force_body': ('output', 'm/s^2', 'external.imu.accel'),
        }
        fields(model.get('bindings'), ' '.join(mapping), 'bindings')
        module = {'path': impl['file'], 'inputs': {}, 'outputs': {}, 'units': {}, 'parameters': {},
                  'sample_time_policy': 'latched_input', 'discard_repeated_samples': True,
                  'publish': {'type': 'imu', 'gyro': ['external.imu.gyro.'+a for a in 'xyz'],
                              'accel': ['external.imu.accel.'+a for a in 'xyz']}}
        used = set()
        for field, (causality, unit, signal) in mapping.items():
            binding = model['bindings'][field]; fields(binding, 'variables unit frame', field)
            expect(binding['unit'], unit, field+'.unit'); expect(binding['frame'], 'body', field+'.frame')
            names = binding['variables']
            if not isinstance(names, list) or len(names) != 3: raise ConfigError(field+': expected three variables')
            for axis, name in zip('xyz', names):
                check_var(name, causality, unit)
                if name in used: raise ConfigError(f'Duplicate FMU binding: {name}')
                used.add(name)
                module['inputs' if causality == 'input' else 'outputs'][name] = signal+'.'+axis
                if causality == 'output': module['units'][signal+'.'+axis] = unit
        for name, desc in model['parameters'].items():
            var = desc['variable']; check_var(var, 'parameter')
            if var in used: raise ConfigError('Duplicate parameter binding: '+var)
            used.add(var); module['parameters'][var] = values[name]
        # Unmapped required inputs are almost certainly an incomplete adapter.
        for name, var in variables.items():
            if var.get('causality') == 'input' and name not in used: raise ConfigError('Unmapped FMU input: '+name)
    return model, values, module


def vehicle_config(path):
    vehicle = load(path); fields(vehicle, 'schema id rocket visuals', 'vehicle', 'schema id rocket')
    expect(vehicle['schema'], 'simrocket/vehicle-v1', 'vehicle.schema')
    rocket = vehicle['rocket']; fields(rocket, 'engines actuators mass aerodynamics', 'rocket')
    for name in ('engines','actuators'):
        if not isinstance(rocket[name], list) or len(rocket[name]) != 1: raise ConfigError('rocket-tvc/v1 supports one engine and one actuator')
    engine = rocket['engines'][0]; fields(engine, 'engine_id type max_thrust_vac_n max_thrust_sl_n max_mass_flow_kg_s', 'engine')
    expect(engine['type'], 'liquid', 'engine.type')
    for key in ('max_thrust_vac_n','max_thrust_sl_n','max_mass_flow_kg_s'): number(engine[key], key, positive=True)
    actuator = rocket['actuators'][0]; fields(actuator, 'actuator_id engine_id position_m', 'actuator')
    for obj, key in [(engine,'engine_id'),(actuator,'engine_id'),(actuator,'actuator_id')]:
        if type(obj[key]) is not int or not 0 <= obj[key] < 2**32: raise ConfigError(key+': invalid identifier')
    expect(actuator['engine_id'], engine['engine_id'], 'actuator.engine_id'); vector(actuator['position_m'], 'actuator.position_m')
    mass = rocket['mass']; fields(mass, 'type dry_mass_kg inertia_diagonal_kg_m2 dry_cg_z_m tanks', 'mass')
    expect(mass['type'], 'dynamic', 'mass.type'); number(mass['dry_mass_kg'], 'dry_mass', positive=True)
    for value in vector(mass['inertia_diagonal_kg_m2'],'inertia'): number(value,'inertia',positive=True)
    number(mass['dry_cg_z_m'], 'dry_cg')
    if not isinstance(mass['tanks'], list) or not mass['tanks']: raise ConfigError('mass.tanks must not be empty')
    ids=set()
    for tank in mass['tanks']:
        fields(tank, 'tank_id z_bottom_m radius_m max_height_m propellant_density_kg_m3 mass_kg', 'tank')
        if type(tank['tank_id']) is not int or tank['tank_id'] < 0 or tank['tank_id'] in ids: raise ConfigError('Invalid/duplicate tank_id')
        ids.add(tank['tank_id']); number(tank['z_bottom_m'], 'tank.z_bottom_m')
        for k in ('radius_m','max_height_m','propellant_density_kg_m3'): number(tank[k], k, positive=True)
        number(tank['mass_kg'], 'tank.mass', minimum=0)
        if tank['mass_kg'] > math.pi*tank['radius_m']**2*tank['max_height_m']*tank['propellant_density_kg_m3']: raise ConfigError('Tank mass exceeds capacity')
    aero=rocket['aerodynamics']; fields(aero, 'drag_coefficient normal_force_coefficient reference_area_m2 center_of_pressure_z_m pitch_yaw_damping_coefficient roll_damping_coefficient','aerodynamics')
    for k,v in aero.items(): number(v,k,minimum=None if k=='center_of_pressure_z_m' else 0,positive=k=='reference_area_m2')
    if 'visuals' in vehicle:
        fields(vehicle['visuals'],'rocket_length_m rocket_radius_m','visuals')
        for k,v in vehicle['visuals'].items(): number(v,k,positive=True)
    return vehicle


def compile_scenario(path):
    doc=scenario(path)
    fields(doc, SCENARIO_FIELDS, 'scenario', 'schema vehicle system simulation environment initial_state estimator_prior record')
    expect(doc['schema'],'simrocket/scenario-v1','scenario.schema')
    sim=doc['simulation'];fields(sim,'duration_s base_rate_hz random_seed real_time_factor','simulation')
    base=number(sim['base_rate_hz'],'base_rate_hz',positive=True);duration=number(sim['duration_s'],'duration_s',positive=True)
    if base>1e6 or base*duration>1e12: raise ConfigError('Simulation limits exceeded')
    number(sim['real_time_factor'],'real_time_factor',minimum=0)
    if type(sim['random_seed']) is not int or not 0<=sim['random_seed']<2**32-1: raise ConfigError('Invalid random_seed')
    if abs(1e9/base-round(1e9/base))>1e-3: raise ConfigError('base_rate_hz must have an integer nanosecond period')
    def rate(hz, where):
        number(hz,where,positive=True)
        ticks=base/hz
        if ticks<1 or abs(ticks-round(ticks))>1e-7: raise ConfigError(where+': base_rate_hz must be a multiple')
    if abs(duration*base-round(duration*base))>1e-7: raise ConfigError('duration_s must align with base step')
    env=doc['environment'];fields(env,'gravity_world_m_s2 wind_world_m_s origin','environment')
    gravity=vector(env['gravity_world_m_s2'],'gravity');wind=vector(env['wind_world_m_s'],'wind')
    if gravity[:2]!=[0,0]: raise ConfigError('Transitional environment supports vertical gravity only')
    origin=env['origin'];fields(origin,'latitude_deg longitude_deg altitude_m','origin')
    for k,v in origin.items(): number(v,k)
    if abs(origin['latitude_deg'])>=89.9 or abs(origin['longitude_deg'])>180: raise ConfigError('Origin outside supported local ENU region')
    def state(section, angular):
        obj=doc[section];fields(obj,'position_world_m velocity_world_m_s orientation_body_to_world_wxyz'+(' angular_velocity_body_rad_s' if angular else ''),section)
        for k,v in obj.items(): obj[k]=quaternion(v,k) if k.startswith('orientation') else vector(v,k)
    state('initial_state',True);state('estimator_prior',False)
    if doc['initial_state']['position_world_m'][2]<0: raise ConfigError('Initial altitude must be nonnegative')
    rec=doc['record'];fields(rec,'rate_hz signals directory','record');rate(rec['rate_hz'],'record.rate_hz')
    if not isinstance(rec['signals'],list) or not rec['signals'] or any(s not in ('truth','imu.measurement','estimator.state','actuator.state') for s in rec['signals']) or len(set(rec['signals']))!=len(rec['signals']): raise ConfigError('record.signals: unsupported, empty or duplicate selection')
    system=load(doc['system']);fields(system,'schema communication topology components','system')
    expect(system['schema'],'simrocket/system-v1','system.schema');expect(system['topology'],'rocket-tvc/v1','topology')
    expect(system['communication'],{'avionics':{'model':'ideal'}},'communication')
    components=merge(system['components'],doc.get('components',{}))
    allowed={
        'imu':'role model rate_hz parameters', 'gps':'model rate_hz parameters',
        'estimator':'model rate_hz inputs', 'controller':'model rate_hz parameters inputs',
        'allocator':'model rate_hz', 'actuator':'model vehicle_actuator_id rate_hz initial_state initial_setpoint inputs',
        'propulsion':'model vehicle_engine_id rate_hz throttle_profile', 'dynamics':'model rate_hz integrator'}
    fields(components,' '.join(allowed),'components')
    factories={'gps':'gps','estimator':'error-state-kalman-filter','controller':'tvc-controller','allocator':'tvc-mixer','actuator':'ideal-tvc-actuator','propulsion':'liquid-propulsion','dynamics':'rocket-dynamics'}
    for name,c in components.items():
        fields(c,allowed[name],name);rate(c['rate_hz'],name+'.rate_hz')
        if name in factories: expect(c['model'],'native/'+factories[name],name+'.model')
    imu=components['imu'];expect(imu['role'],'imu','imu.role')
    for name in ('actuator','propulsion','dynamics'): expect(components[name]['rate_hz'],base,name+'.rate_hz (current native composition)')
    expect(components['allocator']['rate_hz'],components['controller']['rate_hz'],'allocator.rate_hz')
    expect(components['dynamics']['integrator'],'rk4','dynamics.integrator')
    for name,policies in {'estimator':{'imu':('all_samples',True),'gps':('all_samples',False)},'controller':{'state':('latest',True)},'actuator':{'setpoint':('latest',True)}}.items():
        fields(components[name]['inputs'],' '.join(policies),name+'.inputs')
        for port,(read,required) in policies.items():
            policy=components[name]['inputs'][port];fields(policy,'read required max_age_s',name+'.'+port)
            expect(policy['read'],read,port+'.read')
            if type(policy['required']) is not bool: raise ConfigError(port+'.required must be boolean')
            expect(policy['required'],required,port+'.required');number(policy['max_age_s'],port+'.max_age_s',minimum=0)
    gps=components['gps']['parameters'];fields(gps,'delay_s position_noise_std_m','gps.parameters')
    for k,v in gps.items(): number(v,k,minimum=0)
    tvc=components['controller']['parameters'];fields(tvc,'kp kd max_gimbal_rad','controller.parameters')
    for k,v in tvc.items():number(v,k,minimum=0)
    if tvc['max_gimbal_rad']>math.pi/2: raise ConfigError('max_gimbal_rad exceeds pi/2')
    act=components['actuator'];fields(act['initial_state'],'angle_x_rad angle_y_rad','actuator.initial_state')
    fields(act['initial_setpoint'],'angle_x_rad angle_y_rad startup_deadline_s','actuator.initial_setpoint')
    for k,v in act['initial_state'].items():number(v,k)
    for k,v in act['initial_setpoint'].items():number(v,k,minimum=0 if k=='startup_deadline_s' else None)
    vehicle=vehicle_config(doc['vehicle']);rocket=vehicle['rocket']
    expect(act['vehicle_actuator_id'],rocket['actuators'][0]['actuator_id'],'vehicle_actuator_id')
    expect(components['propulsion']['vehicle_engine_id'],rocket['engines'][0]['engine_id'],'vehicle_engine_id')
    profile=components['propulsion']['throttle_profile']
    if not isinstance(profile,list) or len(profile)<2: raise ConfigError('throttle_profile requires at least two points')
    last=-1
    for point in profile:
        t,v=vector(point,'throttle_profile point',2)
        if t<0 or t<=last or not 0<=v<=1: raise ConfigError('Invalid throttle profile time/order/value')
        last=t
    if profile[0][0]!=0 or profile[-1][0]<duration: raise ConfigError('Throttle profile must cover the full simulation')
    model,params,module=model_config(imu['model'],imu['parameters'])
    if module and abs(duration*imu['rate_hz']-round(duration*imu['rate_hz']))>1e-7: raise ConfigError('FMU period must divide duration')
    resolved={'schema':'simrocket/resolved-v1','runtime':'phased-preset-v1','scenario':str(Path(path).resolve()),
              'components':components,'imu_model':model,'connections':[dict(zip(('from','to','interface','via'),c)) for c in CONNECTIONS],
              'record':rec,'limitations':['Preset composition remains fixed; arbitrary graph assembly is stage 5.',
                                         'Propulsion, environment and mass remain encapsulated inside native dynamics; actuator is ideal feedthrough.']}
    resolved['components']['imu']['parameters']=params
    output={'rocket':rocket,'random_seed':sim['random_seed'],'initial_state':doc['initial_state'],'estimator_prior':doc['estimator_prior'],
            'actuator_initial':act,'component_runtime':resolved,
            'environment':{'simulation_time_s':duration,'real_time_factor':sim['real_time_factor'],'gravity_z':gravity[2],
                           **dict(zip(('wind_velocity_x_m_s','wind_velocity_y_m_s','wind_velocity_z_m_s'),wind))},
            'location':{'latitude':origin['latitude_deg'],'longitude':origin['longitude_deg'],'altitude_m':origin['altitude_m']},
            'sensors':{'imu':{'enabled':module is None,'update_rate_hz':imu['rate_hz'],**params},
                       'gps':{'update_rate_hz':components['gps']['rate_hz'],'delay_ms':1000*gps['delay_s'],'position_noise_std_m':gps['position_noise_std_m']}},
            'control':{'update_rate_hz':components['controller']['rate_hz'],'tvc':{'max_gimbal_deg':math.degrees(tvc['max_gimbal_rad']),'pid_kp':tvc['kp'],'pid_kd':tvc['kd']},
                       'engine_controllers':[{'engine_id':rocket['engines'][0]['engine_id'],'type':'profile','throttle_profile':profile}]},
            'telemetry':{'update_rate_hz':rec['rate_hz'],'log_to_file':False},
            'scheduler':{'base_rate_hz':base,'execution_order':(['imu_fmu'] if module else [])+['sensors','control','propulsion_control','dynamics','telemetry']}}
    if 'visuals' in vehicle: output['visuals']=vehicle['visuals']
    if module:
        module['update_rate_hz']=imu['rate_hz'];output['external_modules']={'imu_fmu':module}
    return output


if __name__ == '__main__':
    try:
        print(json.dumps(compile_scenario(sys.argv[1]),allow_nan=False))
    except (ConfigError, KeyError, TypeError, ValueError, OSError) as error:
        print(f'[configuration] {error}',file=sys.stderr)
        sys.exit(1)
