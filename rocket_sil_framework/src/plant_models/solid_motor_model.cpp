#include "rocket_sil_framework/include/plant_models/solid_motor_model.hpp"
#include <algorithm>
#include <iostream>

SolidMotorModel::SolidMotorModel(uint32_t engine_id, std::shared_ptr<ThrustCurve> thrust_curve, double total_propellant_mass_kg)
    : engine_id_(engine_id), thrust_curve_(thrust_curve), total_propellant_mass_kg_(total_propellant_mass_kg) {
}

EngineOutput SolidMotorModel::compute(double time_s, const MassProperties& mass_props, double ambient_pressure_pa) {
    EngineOutput out;
    out.thrust_body.setZero();
    out.torque_body.setZero();
    out.mass_flow_rate = 0.0;
    
    if (!thrust_curve_) return out;

    double max_burn_time = thrust_curve_->getMaxBurnTime();
    
    if (time_s >= 0.0 && time_s <= max_burn_time) {
        double current_thrust = thrust_curve_->getThrustAt(time_s);
        
        out.thrust_body = Eigen::Vector3d(0.0, 0.0, current_thrust);
        out.torque_body.setZero();
        
        // Calculate dynamic mass flow: dm/dt = M_prop * F(t) / I_tot
        double i_tot = thrust_curve_->getTotalImpulse();
        if (i_tot > 0.0) {
            out.mass_flow_rate = total_propellant_mass_kg_ * (current_thrust / i_tot);
        }
    }
    
    return out;
}
