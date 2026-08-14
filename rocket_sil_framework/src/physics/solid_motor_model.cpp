#include "rocket_sil_framework/include/physics/solid_motor_model.hpp"
#include <algorithm>

SolidMotorModel::SolidMotorModel(uint32_t engine_id, double burn_time_s, double constant_thrust_n, double total_propellant_mass_kg)
    : engine_id_(engine_id), burn_time_s_(burn_time_s), constant_thrust_n_(constant_thrust_n) {
    if (burn_time_s_ > 0.0) {
        mass_flow_rate_ = total_propellant_mass_kg / burn_time_s_;
    } else {
        mass_flow_rate_ = 0.0;
    }
}

EngineOutput SolidMotorModel::compute(double time_s, const MassProperties& mass_props) {
    EngineOutput out;
    out.thrust_body.setZero();
    out.torque_body.setZero();
    out.mass_flow_rate = 0.0;
    
    // Engine ignites at t=0 and burns until t=burn_time_s_
    if (time_s >= 0.0 && time_s <= burn_time_s_) {
        // Thrust is output as a local vector. The DynamicsModel will rotate it using the Actuator.
        // For IEngineModel, thrust_body field temporarily stores the local thrust.
        out.thrust_body = Eigen::Vector3d(0.0, 0.0, constant_thrust_n_);
        // torque_body is zero from the engine itself (nozzle offset torque calculated in DynamicsModel)
        out.torque_body.setZero();
        out.mass_flow_rate = mass_flow_rate_;
    }
    
    return out;
}
