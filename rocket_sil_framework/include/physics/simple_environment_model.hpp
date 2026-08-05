#pragma once

#include "i_environment_model.hpp"
#include <Eigen/Dense>

class SimpleEnvironmentModel : public IEnvironmentModel {
public:
    SimpleEnvironmentModel(const Eigen::Vector3d& gravity, const Eigen::Vector3d& wind_velocity);

    EnvironmentState compute(const RocketState& state) override;

private:
    Eigen::Vector3d gravity_;
    Eigen::Vector3d constant_wind_;
    
    // Simple ISA (International Standard Atmosphere) for Troposphere
    double calculateISADensity(double altitude_m) const;
};
