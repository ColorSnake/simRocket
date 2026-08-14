# Physics Module

The physics module contains the core components simulating the natural phenomena and physical properties of the rocket.

## Integrators
- **`EulerIntegrator`**: Fast, simple, 1st-order solver. Used for rapid, low-fidelity simulations.
- **`RK4Integrator`**: Accurate, 4th-order Runge-Kutta solver. Recommended for precise flight dynamics.

## Mass & Aerodynamics
- **`RigidBodyMassModel`**: Tracks the depletion of propellant mass over time and updates the position of the Center of Gravity (CG) dynamically.
- **`SimpleAerodynamicsModel`**: Computes aerodynamic drag and normal forces based on the Angle of Attack (AoA), dampening rotational rates, and applies torque at the Center of Pressure (COP).

## Environment
- **`SimpleEnvironmentModel`**: Simulates gravity vectors and 3D wind velocity profiles (crosswinds).

## Engines and Actuators
- **`IEngineModel`**: Base class for thrust generation. Computes scalar thrust and mass flow (e.g. `SolidMotorModel`).
- **`IActuatorModel`**: Base class for mechanical linkages. Maps the generated thrust vector through 3D space (`TvcActuatorModel` uses quaternions to pitch/yaw the thrust around a pivot point).
