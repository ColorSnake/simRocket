#pragma once
#include <Eigen/Dense>

struct EstimatedStateMessage {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity; // Bias-corrected
    Eigen::Vector3d linear_acceleration; // Bias-corrected
    
    // Estimated Biases (for telemetry/debugging)
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d accel_bias;
};
