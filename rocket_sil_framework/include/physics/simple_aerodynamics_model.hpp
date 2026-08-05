#pragma once

#include "i_aerodynamics_model.hpp"

class SimpleAerodynamicsModel : public IAerodynamicsModel {
public:
    SimpleAerodynamicsModel(double drag_coefficient, double normal_force_coefficient, 
                            double reference_area_m2, double center_of_pressure_z,
                            double pitch_yaw_damping_coeff, double roll_damping_coeff);
    
    AeroForces compute(const RocketState& state, const MassProperties& mass_props, const EnvironmentState& env) override;

private:
    double drag_coefficient_;
    double normal_force_coefficient_;
    double reference_area_;
    double center_of_pressure_z_;
    double pitch_yaw_damping_coeff_;
    double roll_damping_coeff_;
};
