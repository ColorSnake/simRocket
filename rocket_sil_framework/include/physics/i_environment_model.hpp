#pragma once

#include <Eigen/Dense>
#include "rocket_state.hpp"

struct EnvironmentState {
    double air_density;                     // kg/m^3
    Eigen::Vector3d wind_velocity_inertial; // m/s (Inertial Frame)
    Eigen::Vector3d gravity_inertial;       // m/s^2 (Inertial Frame)
};

class IEnvironmentModel {
public:
    virtual ~IEnvironmentModel() = default;
    
    // Computes environment state at the current rocket position and time
    virtual EnvironmentState compute(const RocketState& state) = 0;
};
