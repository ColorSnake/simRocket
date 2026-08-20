#pragma once

#include <Eigen/Dense>

struct RocketDiagnostics {
    double current_mass_kg = 0.0;
    double current_cg_z_m = 0.0;
    Eigen::Vector3d inertia_diagonal_kg_m2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d thrust_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d aero_force_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d wind_velocity_inertial = Eigen::Vector3d::Zero();
};
