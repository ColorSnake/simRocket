# simRocket Framework

Welcome to the **simRocket** documentation! 

`simRocket` is an advanced Software-in-the-Loop (SIL) simulation framework designed for modeling rocket flight dynamics, multi-engine thrust vectors, and complex aerodynamics.

## Key Features
- **Modular Architecture**: Separate interfaces for engines, actuators, environment, and mass models.
- **Multi-Engine Support**: Easily define N-engines and N-actuators (e.g. Starship 33 Raptor cluster) purely from JSON config.
- **6-DOF Dynamics**: Accurate 6 Degree-of-Freedom physics modeling with RK4 and Euler integrators.
- **Message Bus Architecture**: Event-driven decoupled communication for sensors and actuators.
- **Telemetry & Visualization**: Real-time UDP telemetry bridging to ROS2 and Foxglove Studio for 3D visualization.

## Documentation Structure
- [Architecture & Workflow](architecture.md)
- [Configuration](configuration.md)
- [Modules - Physics & Mechanics](modules/physics.md)
- [Modules - Control](modules/control.md)
- [Modules - Communication (MessageBus)](modules/communication.md)
- [Modules - Telemetry](modules/telemetry.md)
