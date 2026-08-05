#pragma once

#include <memory>
#include "rocket_sil_framework/include/physics/i_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/physics/i_integrator.hpp"
#include "rocket_sil_framework/include/physics/i_engine_model.hpp"
#include "rocket_sil_framework/include/physics/i_mass_model.hpp"
#include "rocket_sil_framework/include/physics/i_aerodynamics_model.hpp"
#include "rocket_sil_framework/include/physics/i_environment_model.hpp"

class NativeRocketDynamicsModel : public IRocketDynamicsModel {
public:
    explicit NativeRocketDynamicsModel(
        std::unique_ptr<IIntegrator> integrator,
        std::unique_ptr<IEngineModel> engine = nullptr,
        std::unique_ptr<IMassModel> mass = nullptr,
        std::unique_ptr<IAerodynamicsModel> aero = nullptr,
        std::unique_ptr<IEnvironmentModel> env = nullptr);
    
    void update(double dt, const RocketInputs& inputs, RocketState& state) override;
    RocketDiagnostics getDiagnostics() const override { return diagnostics_; }

private:
    std::unique_ptr<IIntegrator> integrator_;
    std::unique_ptr<IEngineModel> engine_;
    std::unique_ptr<IMassModel> mass_;
    std::unique_ptr<IAerodynamicsModel> aero_;
    std::unique_ptr<IEnvironmentModel> env_;
    
    // Przechowuje diagnostykę wyliczoną w trakcie ostatniego kroku (k1)
    mutable RocketDiagnostics diagnostics_;
    
    RocketStateDerivatives calculateDerivatives(const RocketState& state, const RocketInputs& inputs) const;
};
