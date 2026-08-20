#pragma once

#include <Eigen/Dense>

struct RocketInputs {
    // Forces and torques in Body frame
    Eigen::Vector3d force_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_body = Eigen::Vector3d::Zero();

    // Mass properties
    double mass = 100.0;
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d inertia_inv = Eigen::Matrix3d::Identity();

    // Environment
    Eigen::Vector3d gravity_inertial = Eigen::Vector3d(0.0, 0.0, -9.81);
};
