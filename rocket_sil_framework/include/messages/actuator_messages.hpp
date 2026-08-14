#pragma once

#include <cstdint>

struct ActuatorCommandMessage {
    uint32_t actuator_id;
    double pitch_angle_rad; // Commanded rotation around Y axis
    double yaw_angle_rad;   // Commanded rotation around X axis
};
