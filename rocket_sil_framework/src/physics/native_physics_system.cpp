#include "rocket_sil_framework/include/physics/native_physics_system.hpp"
#include <algorithm>
#include <stdexcept>

NativePhysicsSystem::NativePhysicsSystem(std::unique_ptr<PropulsionSystem> propulsion,
        std::unique_ptr<IMassModel> mass, std::unique_ptr<IAerodynamicsModel> aero,
        std::unique_ptr<IEnvironmentModel> environment)
    : propulsion_(std::move(propulsion)), mass_(std::move(mass)), aero_(std::move(aero)),
      environment_(std::move(environment)) {
    if (!propulsion_) throw std::invalid_argument("Physics system requires propulsion composition (may be empty)");
}
MassProperties NativePhysicsSystem::massProperties(const RocketInputs& inputs) const {
    if (mass_) return mass_->getProperties();
    MassProperties result;
    result.total_mass = inputs.mass;
    result.center_of_gravity.setZero();
    result.inertia_tensor = inputs.inertia;
    return result;
}
EnvironmentState NativePhysicsSystem::environment(const RocketState& state, const RocketInputs& inputs) const {
    if (environment_) return environment_->compute(state);
    EnvironmentState result;
    result.gravity_inertial = inputs.gravity_inertial;
    result.wind_velocity_inertial.setZero();
    result.air_density = 1.225;
    result.ambient_pressure_pa = 101325.0;
    return result;
}
PhysicsStep NativePhysicsSystem::prepareStep(double dt, const RocketState& state, const RocketInputs& inputs) {
    PhysicsStep result{dt, 0};
    if (mass_) {
        const auto env = environment(state, inputs);
        const auto mass = mass_->getProperties();
        result.mass_flow_kg_s = propulsion_->massFlow(state.time, mass, env.ambient_pressure_pa);
        if (result.mass_flow_kg_s > 0 && mass.propellant_mass > 0)
            result.powered_duration = std::min(dt, mass.propellant_mass / result.mass_flow_kg_s);
    }
    return result;
}
void NativePhysicsSystem::advanceActuators(double dt) { propulsion_->advanceActuators(dt); }
void NativePhysicsSystem::consumePropellant(double flow, double dt) { if (mass_) mass_->update(flow, dt); }
DynamicsEvaluation NativePhysicsSystem::evaluate(const RocketState& state, const RocketInputs& inputs) const {
    const auto mass = massProperties(inputs);
    const auto env = environment(state, inputs);
    DynamicsEvaluation result;
    result.mass = mass.total_mass;
    result.inertia = mass.inertia_tensor;
    result.inertia_inv = inputs.inertia_inv;
    if (mass_) result.inertia_inv = result.inertia.inverse();
    result.force_body = inputs.force_body;
    result.moment_at_cg_body = inputs.torque_body;
    result.gravity_world = env.gravity_inertial;
    propulsion_->accumulate(state.time, mass, env.ambient_pressure_pa,
        result.force_body, result.moment_at_cg_body, result.diagnostics);
    if (aero_) {
        const auto forces = aero_->compute(state, mass, env);
        result.force_body += forces.aerodynamic_force_body;
        result.moment_at_cg_body += forces.aerodynamic_moment_body;
        result.diagnostics.aero_force_body = forces.aerodynamic_force_body;
    }
    result.diagnostics.current_mass_kg = result.mass;
    result.diagnostics.current_cg_z_m = mass.center_of_gravity.z();
    result.diagnostics.inertia_diagonal_kg_m2 = result.inertia.diagonal();
    result.diagnostics.wind_velocity_inertial = env.wind_velocity_inertial;
    return result;
}
