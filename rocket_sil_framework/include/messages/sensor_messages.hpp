#pragma once
#include <Eigen/Dense>
struct ImuStateMessage {
    double timestamp_s = 0; // Measurement time, simulation seconds
    double available_time_s = 0;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // optional truth/AHRS; not used by EKF
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero(); // Body, rad/s
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero(); // Body specific force, m/s^2
};
struct GpsStateMessage {
    double timestamp_s = 0; // Acquisition time, not delivery time
    double available_time_s = 0;
    double latitude = 0, longitude = 0, altitude_m = 0;
    Eigen::Vector3d velocity_ned = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_local = Eigen::Vector3d::Zero(); // ENU, relative to configured origin
};
