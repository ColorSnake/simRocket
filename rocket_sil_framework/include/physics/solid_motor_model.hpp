#pragma once

#include "i_engine_model.hpp"

class SolidMotorModel : public IEngineModel {
public:
    SolidMotorModel(double burn_time_s, double constant_thrust_n, double total_propellant_mass_kg, double engine_position_z_m);
    
    EngineOutput compute(double time_s, const MassProperties& mass_props) override;

private:
    double burn_time_s_;
    double constant_thrust_n_;
    double mass_flow_rate_;
    double engine_position_z_m_;
};
