#pragma once

#include "i_engine_model.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/actuator_messages.hpp"
#include "rocket_sil_framework/include/math/transform3d.hpp"
#include <memory>

class SolidMotorModel : public IEngineModel {
public:
    SolidMotorModel(double burn_time_s, double constant_thrust_n, double total_propellant_mass_kg, double engine_position_z_m, std::shared_ptr<MessageBus> bus = nullptr);
    
    EngineOutput compute(double time_s, const MassProperties& mass_props) override;

private:
    double burn_time_s_;
    double constant_thrust_n_;
    double mass_flow_rate_;
    double engine_position_z_m_;
    
    // TVC State
    double current_pitch_rad_ = 0.0;
    double current_yaw_rad_ = 0.0;
};
