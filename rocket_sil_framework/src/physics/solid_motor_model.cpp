#include "rocket_sil_framework/include/physics/solid_motor_model.hpp"
#include <algorithm>

SolidMotorModel::SolidMotorModel(double burn_time_s, double constant_thrust_n, double total_propellant_mass_kg, double engine_position_z_m)
    : burn_time_s_(burn_time_s), constant_thrust_n_(constant_thrust_n), engine_position_z_m_(engine_position_z_m) {
    if (burn_time_s_ > 0.0) {
        mass_flow_rate_ = total_propellant_mass_kg / burn_time_s_;
    } else {
        mass_flow_rate_ = 0.0;
    }
}

EngineOutput SolidMotorModel::compute(double time_s, const MassProperties& mass_props) {
    EngineOutput out;
    out.thrust_body.setZero();
    out.mass_flow_rate = 0.0;
    
    // Silnik odpala się w t=0 i pali się do t=burn_time_s_
    if (time_s >= 0.0 && time_s <= burn_time_s_) {
        // Ciąg pchający od ogona w kierunku nosa (+Z body frame).
        out.thrust_body = Eigen::Vector3d(0.0, 0.0, constant_thrust_n_);
        out.mass_flow_rate = mass_flow_rate_;
    }
    
    // Brak thrust_torque na razie, dodamy jeśli będzie odchylenie silnika.
    // Z uwagi na to, że silnik działa idealnie wzdłuż osi Z, a CG jest też na osi Z,
    // (Pos - CG) krzyżowo z ThrustZ daje (0,0,0).
    return out;
}
