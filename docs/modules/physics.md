# Physics Module

The physics module contains the core components simulating the natural phenomena and physical properties of the rocket.

## Integrators
- **`EulerIntegrator`**: Fast, simple, 1st-order solver. Used for rapid, low-fidelity simulations.
- **`RK4Integrator`**: Accurate, 4th-order Runge-Kutta solver. Recommended for precise flight dynamics.
  > [!WARNING]
  > **RK4 Stability Limits:** At extremely high dynamic pressures and very high aerodynamic damping coefficients (e.g., `pitch_yaw_damping_coefficient > 0.1`), RK4 can experience numerical instability (diverging to NaN values). Ensure damping coefficients are tuned properly (e.g. `0.02`) when flying through dense atmosphere at high speeds.

## Mass & Aerodynamics
- **`DynamicMassModel`**: Advanced model for liquid rockets (LRE). Treats tanks as cylinders (with `radius_m` and `height_m`). As propellant mass decreases, it calculates the shrinking height of the liquid cylinder and dynamically updates the full inertia tensor (`Ixx`, `Iyy`, `Izz`) and Center of Gravity (`CG`) using the Parallel Axis Theorem (Steiner's theorem).
- **`RigidBodyMassModel`**: (Legacy/Alternative) Simpler model for mass tracking.
- **`SimpleAerodynamicsModel`**: Computes aerodynamic drag and normal forces based on the Angle of Attack (AoA), dampening rotational rates, and applies torque at the Center of Pressure (COP).

## Environment
- **`SimpleEnvironmentModel`**: Simulates gravity vectors and 3D wind velocity profiles (crosswinds).

## Engines and Actuators
- **`IEngineModel`**: Base class for thrust generation. Computes scalar thrust and mass flow.
- **`SolidMotorModel`**: Simulates a solid rocket motor using `ThrustCurve` for realistic thrust profiles (supports `.eng` RASP format and `.csv`). Calculates dynamic mass depletion based on the integral of the thrust curve (Total Impulse).
- **`LiquidEngineModel`**: Simulates liquid rocket engines with throttle support (0-100% via MessageBus). It calculates variable thrust and mass flow, incorporating ambient pressure adjustments (Sea-Level vs Vacuum thrust limits).
- **`ThrustCurve`**: Utility class that parses motor files, interpolates thrust over time, and computes total impulse.
- **`IActuatorModel`**: Base class for mechanical linkages. Maps the generated thrust vector through 3D space (`TvcActuatorModel` uses quaternions to pitch/yaw the thrust around a pivot point).
