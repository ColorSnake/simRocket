# Control Module

The control module is responsible for keeping the rocket stabilized during flight.

## TVC Controller (`TvcController`)
A PID-based control loop that aims to keep the rocket pointing along a desired trajectory (typically vertical).
- Takes the current `ImuStateMessage` as input.
- Computes Pitch and Yaw error.
- Publishes a generalized `TvcLogicalCommand` onto the `MessageBus`.

## TVC Mixer (`TvcMixer`)
The mixer translates logical demands into physical actuation.
- Listens for `TvcLogicalCommand`.
- Given a list of active `actuator_id`s, it maps the generalized pitch/yaw command to specific `ActuatorCommandMessage`s.
- This decoupling allows the system to control complex multi-engine clusters without altering the core flight controller logic.

## Error-State Kalman Filter (`ErrorStateKalmanFilter`)
An ES-EKF is used to estimate the true state of the rocket from noisy IMU and GPS sensors.
- Predicts the state using 1000 Hz IMU measurements (Acceleration + Gyroscope).
- Corrects the state using 10 Hz GPS measurements.
- Dynamically initializes the local Flat-Earth origin (0, 0, 0) from the first received GPS coordinate, ensuring seamless integration of the sensor data regardless of launch location.
- Outputs an `EstimatedStateMessage` which the TVC controller uses for stabilization.
