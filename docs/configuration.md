# Configuration (config.json)

The simulation is entirely data-driven through `config.json`.

## Rocket Definition
The `rocket` block contains the physical characteristics of the vehicle.
- `engines`: Array of engine definitions (thrust, burn time, mass).
- `actuators`: Array of actuator definitions mapping an `engine_id` to a physical position (`position_m`).
- `mass`: Dry mass, propellant mass, center of gravity (CG), and inertia tensor.
- `aerodynamics`: Drag, normal force, center of pressure (COP), and damping coefficients.

## Control System
- `control.tvc`: PID gains and maximum gimbal angles for the TVC system.

## Environment & Location
- `environment`: Gravity, wind velocities, and initial Euler orientation.
- `location`: Specifies the geographic starting point (`latitude`, `longitude`, `altitude_m`). The EKF and Telemetry bridge use this to calculate dynamic flat-earth coordinates relative to the launchpad.

## Telemetry
- `telemetry`: Settings for data output, such as `update_rate_hz` (e.g., `100.0` or `1000.0`) which controls how often MsgPack UDP packets and CSV rows are generated.
