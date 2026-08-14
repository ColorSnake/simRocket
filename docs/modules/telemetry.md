# Telemetry Module

Telemetry is output to external systems (e.g., ROS2 / Foxglove) using a lightweight UDP binary protocol.

## Binary Structure
The payload size is dynamically determined based on the number of configured engines.

1. **Header (`TelemetryPacket`)**: 284 bytes.
   - Timestamp
   - Pose (XYZ), Velocity (XYZ), Accel (XYZ)
   - Quaternion Orientation, Angular Velocity
   - Mass, CG
   - Net Thrust, Aero Forces, Inertia
   - Wind Velocity, TVC Diagnostics
   - `num_engines` (uint32_t)

2. **Payload (`EngineTelemetry`)**: 24 bytes per engine.
   - Thrust Vector X, Y, Z (transformed into the Body frame)

## Python Bridge (`telemetry_bridge.py`)
A standalone ROS2 node that listens on UDP `9876`.
- Unpacks the binary structs dynamically.
- Translates local Body frame kinematics into ROS `Odometry`, `AccelStamped`, and `NavSatFix`.
- Generates `MarkerArray` messages to visualize the rocket body, Center of Pressure, Center of Gravity, aerodynamic forces, and **N individual engine thrust vectors**.
