# System Architecture

The `simRocket` framework is built around a decoupled, modular architecture.

## Core Concepts

1. **Message Bus (`MessageBus`)**
   A thread-safe publish-subscribe system that connects sensors (like IMU), controllers (like TVC), and actuators. It mimics a real hardware CAN/Ethernet bus.

2. **Dynamics Model (`NativeRocketDynamicsModel`)**
   The core physics engine. It collects outputs from various sub-models (Engines, Actuators, Aero, Environment, Mass) and computes the Net Force and Net Torque.

3. **Interface Separation**
   - `IEngineModel`: Responsible only for calculating raw scalar thrust and mass flow (propellant depletion).
   - `IActuatorModel`: Responsible only for spatial transforms (e.g. pivoting a nozzle via a gimbal).

By combining `IEngineModel` and `IActuatorModel`, the dynamics model can simulate highly complex clusters of vectored engines.
