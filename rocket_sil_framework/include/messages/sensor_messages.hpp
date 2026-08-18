#pragma once
#include <Eigen/Dense>

struct ImuStateMessage {
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity; // Body frame
    Eigen::Vector3d linear_acceleration; // Body frame (Specific Force)
};

struct GpsStateMessage {
    double latitude;
    double longitude;
    double altitude_m;
    Eigen::Vector3d velocity_ned; // North-East-Down
};
