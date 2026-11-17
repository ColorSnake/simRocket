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
#include "rocket_sil_framework/include/physics/i_dynamics_physics.hpp"

class NativeRocketDynamicsModel : public IRocketDynamicsModel {
public:
    NativeRocketDynamicsModel(std::unique_ptr<IIntegrator>, std::unique_ptr<IDynamicsPhysics>);
    // Compatibility constructor for existing native clients and tests.
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
    std::unique_ptr<IIntegrator> integrator_;
    std::unique_ptr<IDynamicsPhysics> physics_;

    // Diagnostics from the latest physics evaluation (including the final state).
    mutable RocketDiagnostics diagnostics_;
    
    RocketStateDerivatives calculateDerivatives(const RocketState& state, const RocketInputs& inputs) const;
};
