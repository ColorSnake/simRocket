#pragma once

#include <memory>
#include "rocket_sil_framework/include/core/i_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/core/i_integrator.hpp"
#include "rocket_sil_framework/include/plant_models/i_engine_model.hpp"
#include "rocket_sil_framework/include/vehicle/i_mass_model.hpp"
#include "rocket_sil_framework/include/physics/i_aerodynamics_model.hpp"
#include "rocket_sil_framework/include/physics/i_environment_model.hpp"

#include "rocket_sil_framework/include/plant_models/i_actuator_model.hpp"
#include <vector>
#include <map>

class NativeRocketDynamicsModel : public IRocketDynamicsModel {
public:
    void setExternalActuatorScheduling(bool enabled) { external_actuator_scheduling_ = enabled; }
    void setActuatorTransform(uint32_t id, const Transform3D& transform) { actuator_transforms_[id] = transform; }
    explicit NativeRocketDynamicsModel(
        std::unique_ptr<IIntegrator> integrator,
        std::vector<std::unique_ptr<IEngineModel>> engines,
        std::vector<std::unique_ptr<IActuatorModel>> actuators,
        std::unique_ptr<IMassModel> mass = nullptr,
        std::unique_ptr<IAerodynamicsModel> aero = nullptr,
        std::unique_ptr<IEnvironmentModel> env = nullptr);
    
    void update(double dt, const RocketInputs& inputs, RocketState& state) override;
    void refreshDiagnostics(const RocketState& state, const RocketInputs& inputs) const { calculateDerivatives(state, inputs); }
    RocketDiagnostics getDiagnostics() const override { return diagnostics_; }

private:
    bool external_actuator_scheduling_ = false;
    std::map<uint32_t, Transform3D> actuator_transforms_;
    std::unique_ptr<IIntegrator> integrator_;
    std::vector<std::unique_ptr<IEngineModel>> engines_;
    std::vector<std::unique_ptr<IActuatorModel>> actuators_;
    std::unique_ptr<IMassModel> mass_;
    std::unique_ptr<IAerodynamicsModel> aero_;
    std::unique_ptr<IEnvironmentModel> env_;
    
    // Przechowuje diagnostykę wyliczoną w trakcie ostatniego kroku (k1)
    mutable RocketDiagnostics diagnostics_;
    
    RocketStateDerivatives calculateDerivatives(const RocketState& state, const RocketInputs& inputs) const;
};
