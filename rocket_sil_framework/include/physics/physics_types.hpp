#pragma once

#include <eigen3/Eigen/Dense>

struct EngineOutput {
    Eigen::Vector3d thrust_body; // N
    Eigen::Vector3d torque_body; // Nm
    double mass_flow_rate;       // kg/s (positive value meaning mass leaving the vehicle)
};

struct MassProperties {
    double total_mass;           // kg
    Eigen::Vector3d center_of_gravity; // m (relative to vehicle datum)
    Eigen::Matrix3d inertia_tensor;    // kg*m^2
};

struct AeroForces {
    Eigen::Vector3d aerodynamic_force_body;  // N
    Eigen::Vector3d aerodynamic_moment_body; // Nm
};
