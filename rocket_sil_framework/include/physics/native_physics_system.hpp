#pragma once
#include "i_dynamics_physics.hpp"
#include "i_aerodynamics_model.hpp"
#include "i_environment_model.hpp"
#include "rocket_sil_framework/include/vehicle/i_mass_model.hpp"
#include "rocket_sil_framework/include/plant_models/propulsion_system.hpp"

// Composes separate physical models. Does not integrate position or orientation.
class NativePhysicsSystem : public IDynamicsPhysics {
    std::unique_ptr<PropulsionSystem> propulsion_;
    std::unique_ptr<IMassModel> mass_;
    std::unique_ptr<IAerodynamicsModel> aero_;
    std::unique_ptr<IEnvironmentModel> environment_;
    MassProperties massProperties(const RocketInputs&) const;
    EnvironmentState environment(const RocketState&, const RocketInputs&) const;
public:
    NativePhysicsSystem(std::unique_ptr<PropulsionSystem>, std::unique_ptr<IMassModel>,
                        std::unique_ptr<IAerodynamicsModel>, std::unique_ptr<IEnvironmentModel>);
    PhysicsStep prepareStep(double, const RocketState&, const RocketInputs&) override;
    void advanceActuators(double dt) override;
    void consumePropellant(double flow, double dt) override;
    DynamicsEvaluation evaluate(const RocketState&, const RocketInputs&) const override;
};
