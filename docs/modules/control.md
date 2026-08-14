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
