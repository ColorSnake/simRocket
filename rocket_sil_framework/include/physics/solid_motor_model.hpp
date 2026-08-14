#pragma once

#include "i_engine_model.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/actuator_messages.hpp"
#include "rocket_sil_framework/include/math/transform3d.hpp"
#include <memory>

class SolidMotorModel : public IEngineModel {
public:
    SolidMotorModel(uint32_t engine_id, double burn_time_s, double constant_thrust_n, double total_propellant_mass_kg);
    
    EngineOutput compute(double time_s, const MassProperties& mass_props) override;
    
    uint32_t getEngineId() const override { return engine_id_; }

private:
    uint32_t engine_id_;
    double burn_time_s_;
    double constant_thrust_n_;
    double mass_flow_rate_;
};
