# Simulation Configuration Guide (`config.json`)

SimRocket is entirely data-driven. The simulation setup, physical parameters of the rocket, environmental conditions, and telemetry configurations are all defined in a single configuration file (typically `config.json` or `test_liquid.json`).

This document provides a comprehensive guide on how to properly define the configuration file and describes all supported parameters.

---

## 1. Top-Level Structure

A valid configuration file is a JSON object with several main categories:

```json
{
    "rocket": { ... },
    "control": { ... },
    "sensors": { ... },
    "environment": { ... },
    "telemetry": { ... },
    "location": { ... },
    "visuals": { ... }
}
```

---

## 2. `rocket` Block

This block defines the physical characteristics of the launch vehicle.

### `engines` (Array)
Defines the propulsive elements of the rocket.

| Parameter | Type | Description |
|-----------|------|-------------|
| `engine_id` | Integer | Unique identifier for the engine. |
| `type` | String | `"solid"` or `"liquid"`. Determines the underlying physics model. |
| **For `type="liquid"`** | | |
| `max_thrust_vac_n` | Float | Maximum thrust in vacuum [N]. |
| `max_thrust_sl_n` | Float | Maximum thrust at sea level [N]. |
| `max_mass_flow_kg_s` | Float | Maximum propellant mass flow rate [kg/s]. |
| **For `type="solid"`** | | |
| `thrust_curve_csv` | String | Path to the CSV file containing the thrust curve (Time, Thrust). |
| `burn_time_s` | Float | Total engine burn time [s]. |
| `initial_propellant_mass_kg` | Float | Propellant mass before ignition [kg]. |

### `actuators` (Array)
Maps engines to physical mount points (used for TVC/thrust moments).

| Parameter | Type | Description |
|-----------|------|-------------|
| `actuator_id` | Integer | Unique identifier for the actuator. |
| `engine_id` | Integer | ID of the engine this actuator controls. |
| `position_m` | Array(3) | `[x, y, z]` vector defining the mounting position relative to the rocket's coordinate frame origin. (e.g., `[0.0, 0.0, -2.5]`) |

### `mass`
Defines the mass properties of the vehicle.

| Parameter | Type | Description |
|-----------|------|-------------|
| `type` | String | `"rigid"` (constant mass) or `"dynamic"` (mass changes over time). |
| `dry_mass_kg` | Float | Empty mass of the rocket without propellant. |
| `dry_cg_z_m` | Float | Z-coordinate of the Center of Gravity (CG) for the empty rocket. |
| `inertia_diagonal_kg_m2` | Array(3) | `[Ixx, Iyy, Izz]` principal moments of inertia. |
| **For `type="dynamic"`** | | |
| `tanks` | Array | List of propellant tanks. |
| `tanks[].tank_id` | Integer | Unique tank ID. |
| `tanks[].z_bottom_m` | Float | Z-coordinate of the tank's bottom. |
| `tanks[].max_height_m` | Float | Maximum height of the tank. |
| `tanks[].radius_m` | Float | Radius of the tank. |
| `tanks[].propellant_density_kg_m3` | Float | Density of the propellant inside. |
| `tanks[].mass_kg` | Float | Initial mass of propellant in the tank. |
| **For `type="rigid"`** | | |
| `initial_propellant_mass_kg` | Float | Total propellant mass (remains constant if `rigid`). |
| `propellant_cg_z_m` | Float | Z-coordinate of the propellant CG. |

### `aerodynamics`
Aerodynamic coefficients.

| Parameter | Type | Description |
|-----------|------|-------------|
| `drag_coefficient` | Float | Coefficient of drag ($C_D$). |
| `normal_force_coefficient` | Float | Normal force coefficient gradient ($C_{N\alpha}$). |
| `reference_area_m2` | Float | Reference area for aero calculations [$m^2$]. |
| `center_of_pressure_z_m` | Float | Z-coordinate of the Center of Pressure (COP). |
| `pitch_yaw_damping_coefficient` | Float | Aerodynamic damping for pitch/yaw rotations. |
| `roll_damping_coefficient` | Float | Aerodynamic damping for roll rotations. |

---

## 3. `control` Block

Configures active control systems.

### `tvc`
Thrust Vector Control settings.

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `max_gimbal_deg` | Float | Maximum gimbal angle in degrees. | 10.0 |
| `pid_kp` | Float | Proportional gain for pitch/yaw control. | 0.5 |
| `pid_kd` | Float | Derivative gain for pitch/yaw control. | 0.1 |

### `engine_controllers` (Array)
Defines how engines are throttled/fired over time.

| Parameter | Type | Description |
|-----------|------|-------------|
| `engine_id` | Integer | Engine to control. |
| `type` | String | Controller type (`"profile"`). |
| `throttle_profile` | Array | For inline definition: A list of `[time_s, throttle_0_to_1]` pairs defining the sequence. |
| `csv_file` | String | Alternative to `throttle_profile`. Path to a CSV file (Time, Throttle) to load the profile from. |

---

## 4. `sensors` Block

Configures noise and update rates for simulated sensors.

| Component | Parameter | Type | Description |
|-----------|-----------|------|-------------|
| `imu` | `gyro_noise_std_rad_s` | Float | Gyroscope white noise standard deviation. |
| `imu` | `gyro_bias_instability_rad_s2`| Float | Gyroscope bias random walk. |
| `imu` | `accel_noise_std_m_s2` | Float | Accelerometer white noise standard deviation. |
| `imu` | `accel_bias_instability_m_s3` | Float | Accelerometer bias random walk. |
| `gps` | `update_rate_hz` | Float | GPS measurement frequency. |
| `gps` | `delay_ms` | Float | GPS processing delay in milliseconds. |
| `gps` | `position_noise_std_m` | Float | GPS position error standard deviation. |

---

## 5. `environment` Block

Global simulation settings and initial state.

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `simulation_time_s` | Float | Duration to run the simulation. | 5.0 |
| `real_time_factor` | Float | Speed up/slow down factor. `0.0` runs as fast as possible. `1.0` is real-time. | 1.0 |
| `gravity_z` | Float | Gravity acceleration vector Z-component. | |
| `wind_velocity_[x/y/z]_m_s` | Float | Constant wind vector. | 0.0 |
| `initial_pitch_y_deg` | Float | Initial pitch off the launchpad. | 0.0 |
| `initial_yaw_x_deg` | Float | Initial yaw off the launchpad. | 0.0 |

---

## 6. `location` Block

Geographic coordinates of the launchpad. Used as the origin `(0, 0, 0)` in the local flat-earth Cartesian frame.

| Parameter | Type | Description |
|-----------|------|-------------|
| `latitude` | Float | Decimal degrees (e.g. 28.5623). |
| `longitude` | Float | Decimal degrees (e.g. -80.5774). |
| `altitude_m` | Float | Altitude above sea level in meters. |

---

## 7. `telemetry` Block

| Parameter | Type | Description |
|-----------|------|-------------|
| `update_rate_hz` | Float | Frequency at which telemetry packets (UDP MsgPack) and CSV rows are generated. |

---

## Example: Complete Liquid Rocket Configuration

```json
{
    "rocket": {
        "engines": [
            {
                "engine_id": 0,
                "type": "liquid",
                "max_thrust_vac_n": 5000.0,
                "max_thrust_sl_n": 4200.0,
                "max_mass_flow_kg_s": 2.0
            }
        ],
        "actuators": [
            {
                "actuator_id": 0,
                "engine_id": 0,
                "position_m": [0.0, 0.0, -2.5]
            }
        ],
        "mass": {
            "type": "dynamic",
            "dry_mass_kg": 20.0,
            "inertia_diagonal_kg_m2": [50.0, 50.0, 1.0],
            "dry_cg_z_m": 0.0,
            "tanks": [
                {
                    "tank_id": 0,
                    "z_bottom_m": 0.5,
                    "radius_m": 0.2,
                    "max_height_m": 1.5,
                    "propellant_density_kg_m3": 1141.0,
                    "mass_kg": 50.0
                }
            ]
        },
        "aerodynamics": {
            "drag_coefficient": 0.35,
            "normal_force_coefficient": 1.5,
            "reference_area_m2": 0.0706,
            "center_of_pressure_z_m": -1.8,
            "pitch_yaw_damping_coefficient": 0.02,
            "roll_damping_coefficient": 0.01
        }
    },
    "control": {
        "tvc": {
            "max_gimbal_deg": 8.0,
            "pid_kp": 0.6,
            "pid_kd": 0.15
        },
        "engine_controllers": [
            {
                "engine_id": 0,
                "type": "profile",
                "throttle_profile": [
                    [0.0, 0.0],
                    [1.0, 1.0],
                    [10.0, 1.0],
                    [15.0, 0.0]
                ]
            }
        ]
    },
    "sensors": {
        "imu": {
            "gyro_noise_std_rad_s": 0.005,
            "gyro_bias_instability_rad_s2": 0.0001,
            "accel_noise_std_m_s2": 0.05,
            "accel_bias_instability_m_s3": 0.001
        },
        "gps": {
            "update_rate_hz": 10.0,
            "delay_ms": 150.0,
            "position_noise_std_m": 2.0
        }
    },
    "environment": {
        "simulation_time_s": 30.0,
        "real_time_factor": 0.0,
        "gravity_z": -9.81,
        "initial_pitch_y_deg": 5.0,
        "initial_yaw_x_deg": 0.0,
        "wind_velocity_x_m_s": 5.0,
        "wind_velocity_y_m_s": 0.0,
        "wind_velocity_z_m_s": 0.0
    },
    "telemetry": {
        "update_rate_hz": 100.0
    },
    "location": {
        "latitude": 28.5623,
        "longitude": -80.5774,
        "altitude_m": 10.0
    }
}
```
