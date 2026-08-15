#pragma once

#include "i_engine_model.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/actuator_messages.hpp"
#include "rocket_sil_framework/include/math/transform3d.hpp"
#include "rocket_sil_framework/include/physics/thrust_curve.hpp"
#include <memory>

class SolidMotorModel : public IEngineModel {
public:
    // Takes a ThrustCurve (which encapsulates impulse and burn time) and total propellant mass
    SolidMotorModel(uint32_t engine_id, std::shared_ptr<ThrustCurve> thrust_curve, double total_propellant_mass_kg);
    
    EngineOutput compute(double time_s, const MassProperties& mass_props) override;
    
    uint32_t getEngineId() const override { return engine_id_; }

private:
    uint32_t engine_id_;
    std::shared_ptr<ThrustCurve> thrust_curve_;
    double total_propellant_mass_kg_;
};
