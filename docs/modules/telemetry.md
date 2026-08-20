# Telemetry Module

Telemetry is output to external systems (e.g., ROS2 / Foxglove) using a lightweight UDP binary protocol.

## MsgPack Structure (UDP)
The simulation utilizes `nlohmann::json` to construct a dynamic, self-describing payload that is serialized into a highly efficient binary format using **MsgPack** before transmission over UDP. This allows the telemetry structure to evolve without breaking the python bridge.

The `msgpack` payload unpacks into the following dictionary:
- `timestamp_us`: Simulation time in microseconds (uint64)
- `true`: Ideal physics state (`pos`, `vel`, `acc`, `quat`, `ang_vel`)
- `dyn`: Rigid body dynamics (`mass_kg`, `cg_z`, `thrust`, `aero`, `inertia`, `wind`)
- `ctrl`: TVC commands and errors (`tvc_cmd`, `tvc_err`)
- `sensors`: Noisy sensor readings (`imu_gyro`, `imu_acc`, `gps`)
- `est`: EKF Estimated state (`pos`, `vel`, `quat`, `bg`, `ba`)
- `engines`: Array of engine diagnostics (e.g., individual `thrust` vectors)

## Python Bridge (`telemetry_bridge.py`)
A standalone ROS2 node that listens on UDP `9876`.
- Unpacks the binary structs dynamically.
- Translates local Body frame kinematics into ROS `Odometry`, `AccelStamped`, and `NavSatFix`.
- Generates `MarkerArray` messages to visualize the rocket body, Center of Pressure, Center of Gravity, aerodynamic forces, and **N individual engine thrust vectors**.
  > [!NOTE]
  > Note on Foxglove Thrust Arrows: The engine thrust vectors are published as a `MarkerArray` on the `/rocket/visuals/thrust_array` topic. Ensure this topic is correctly mapped in the Foxglove 3D panel to visualize engine thrusts.

## CSV Logger (Direct Logging)
For rapid verification without ROS bags, the C++ simulation provides a built-in `CsvLogger` that runs alongside the simulation loop and saves the `TelemetryPacket` directly to disk (e.g. `logs/sim_log.csv`). This is activated by passing the `--log-csv` flag.

### Logged Signals
The following signals are logged in the CSV file (matching the `TelemetryPacket` structure) and can be immediately analyzed in Python or Excel:
- **Time**: `time_s`
- **Position**: `pos_x`, `pos_y`, `pos_z`
- **Velocity**: `vel_x`, `vel_y`, `vel_z`
- **Acceleration**: `acc_x`, `acc_y`, `acc_z`
- **Orientation**: `quat_w`, `quat_x`, `quat_y`, `quat_z`
- **Angular Velocity**: `ang_vel_x`, `ang_vel_y`, `ang_vel_z`
- **Mass Properties**: `mass_kg`, `cg_z`, `inertia_x`, `inertia_y`, `inertia_z`
- **Forces**: `thrust_x`, `thrust_y`, `thrust_z` (Net Thrust), `aero_x`, `aero_y`, `aero_z` (Aero Force)
- **Environment**: `wind_x`, `wind_y`, `wind_z`
- **TVC**: `tvc_cmd_pitch`, `tvc_cmd_yaw`, `tvc_err_pitch`, `tvc_err_yaw`
