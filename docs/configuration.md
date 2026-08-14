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

## Environment
- `environment`: Gravity, wind velocities, and initial Euler orientation.
