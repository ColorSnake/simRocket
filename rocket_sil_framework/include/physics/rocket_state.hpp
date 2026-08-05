#pragma once

#include <Eigen/Dense>

struct RocketState {
    double time = 0.0; // Current simulation time
    
    // Translational state
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

    // Rotational state
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
};
