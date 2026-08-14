# Communication Architecture (MessageBus)

The internal communication within the `simRocket` framework is modeled as an event-driven Publish/Subscribe architecture. This closely mimics real-world rocket avionics, where discrete hardware modules (e.g., Flight Management Unit (FMU), IMU sensors, TVC controllers, and individual engine servos) communicate over a shared medium like a CAN bus or an Ethernet network.

## The `MessageBus`

At the heart of the system is the `MessageBus` class. It acts as a centralized, thread-safe broker for all internal data exchange between the simulated software components.

### Key Characteristics:
- **Decoupled**: Publishers do not need to know who is listening, and subscribers do not need to know where the data originated. This ensures high modularity.
- **Type-Safe**: Messages are routed based on their runtime type ID (`typeid`), ensuring that subscribers only receive the structs they explicitly requested.
- **Thread-Safe**: Internal mutexes protect the subscriber lists, allowing components running on different threads (if implemented) to publish and consume safely.

## Message Structure

All messages inherit from a base `Message` struct, which enforces a standardized timestamp and type definition:

```cpp
struct Message {
    MessageType type;
    uint64_t timestamp_us;
    
    virtual ~Message() = default;
};
```

### Standard Messages

The framework currently defines several concrete message types to facilitate the Flight Control loop:

1. **`ImuStateMessage`**
   - **Published by**: Dynamics Model (or a dedicated Sensor Model in the future).
   - **Contains**: Current rocket orientation (Quaternion) and angular velocity.
   - **Consumed by**: `TvcController`.

2. **`TvcLogicalCommand`**
   - **Published by**: `TvcController` (the main flight computer logic).
   - **Contains**: Desired generalized pitch and yaw angles for the entire rocket to stabilize.
   - **Consumed by**: `TvcMixer`.

3. **`ActuatorCommandMessage`**
   - **Published by**: `TvcMixer`.
   - **Contains**: Specific target pitch/yaw angles mapped to a unique `actuator_id`.
   - **Consumed by**: `TvcActuatorModel` (the individual physical servo system).

## Workflow Example

1. **Sensing**: The physics integrator advances time by `dt`. The updated state is packed into an `ImuStateMessage` and published.
2. **Computing**: The `TvcController` receives the IMU data, calculates the error using its PID loop, and publishes a general `TvcLogicalCommand`.
3. **Mixing**: The `TvcMixer` receives the logical command, maps it to the 4 available actuators (e.g., for a 4-engine cluster), and publishes 4 individual `ActuatorCommandMessage`s.
4. **Actuation**: Each of the 4 `TvcActuatorModel` instances receives its specific command, updates its mechanical position (pitch/yaw), and makes the new `Transform3D` available to the physics engine for the next integration step.
