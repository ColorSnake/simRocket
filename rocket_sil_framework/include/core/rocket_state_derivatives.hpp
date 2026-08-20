#pragma once

#include <Eigen/Dense>

struct RocketStateDerivatives {
    double time_dot = 1.0; // Derivative of time is 1
    
    // Translational derivatives
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();       // Derivative of position
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();   // Derivative of velocity

    // Rotational derivatives
    Eigen::Quaterniond q_dot = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0); // Derivative of orientation
    Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();    // Derivative of angular_velocity
};
