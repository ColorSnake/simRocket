#pragma once

struct TvcCommandMessage {
    double pitch_angle_rad; // Commanded rotation around Y axis
    double yaw_angle_rad;   // Commanded rotation around X axis
};
