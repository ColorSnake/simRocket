#include "rocket_sil_framework/include/core/native_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/physics/native_physics_system.hpp"
#include <stdexcept>

NativeRocketDynamicsModel::NativeRocketDynamicsModel(
    std::unique_ptr<IIntegrator> integrator,
    std::vector<std::unique_ptr<IEngineModel>> engines,
    std::vector<std::unique_ptr<IActuatorModel>> actuators,
    std::unique_ptr<IMassModel> mass,
    std::unique_ptr<IAerodynamicsModel> aero,
    std::unique_ptr<IEnvironmentModel> env)
    : NativeRocketDynamicsModel(std::move(integrator), std::make_unique<NativePhysicsSystem>(
        std::make_unique<PropulsionSystem>(std::move(engines), std::move(actuators)),
        std::move(mass), std::move(aero), std::move(env))) {}

NativeRocketDynamicsModel::NativeRocketDynamicsModel(std::unique_ptr<IIntegrator> integrator,
        std::unique_ptr<IDynamicsPhysics> physics)
    : integrator_(std::move(integrator)), physics_(std::move(physics)) {
    if (!physics_) throw std::invalid_argument("Dynamics requires a physics system");
}

void NativeRocketDynamicsModel::update(double dt, const RocketInputs& inputs, RocketState& state) {
    if (!integrator_) {
        return;
    }
    
    const auto step = physics_->prepareStep(dt, state, inputs);
    const double integration_dt = step.powered_duration;
    physics_->advanceActuators(integration_dt);

    auto calc_derivs = [this, &inputs](const RocketState& s) -> RocketStateDerivatives {
        return this->calculateDerivatives(s, inputs);
    };

    state = integrator_->integrate(state, integration_dt, calc_derivs);
    physics_->consumePropellant(step.mass_flow_kg_s, integration_dt);
    const double remaining = dt - integration_dt;
    if (remaining > 1e-12) {
        // Fuel is now exhausted. The remaining interval uses the unpowered model.
        physics_->advanceActuators(remaining);
        state = integrator_->integrate(state, remaining, calc_derivs);
    }

    // Save current accelerations into state for sensors
    RocketStateDerivatives derivs = calculateDerivatives(state, inputs);
    state.acceleration = derivs.acceleration;
    state.angular_acceleration = derivs.angular_acceleration;
}

RocketStateDerivatives NativeRocketDynamicsModel::calculateDerivatives(const RocketState& state, const RocketInputs& inputs) const {
    RocketStateDerivatives derivs;

    const auto physical = physics_->evaluate(state, inputs);
    const auto& current_mass = physical.mass;
    const auto& current_inertia = physical.inertia;
    const auto& current_inertia_inv = physical.inertia_inv;
    const auto& torque_body = physical.moment_at_cg_body;
    Eigen::Vector3d force_inertial = state.orientation * physical.force_body;
    force_inertial += current_mass * physical.gravity_world;
    diagnostics_ = physical.diagnostics;

    derivs.acceleration = force_inertial / current_mass;
    derivs.velocity = state.velocity; // Position derivative is velocity

    // 5. Rotational dynamics (T = I * alpha + omega x (I * omega))
    derivs.angular_acceleration = current_inertia_inv * (torque_body - state.angular_velocity.cross(current_inertia * state.angular_velocity));
    
    // Quaternion derivative
    Eigen::Quaterniond omega_quat(0.0, state.angular_velocity.x(), state.angular_velocity.y(), state.angular_velocity.z());
    
    derivs.q_dot.w() = -0.5 * (state.orientation.x() * omega_quat.x() + state.orientation.y() * omega_quat.y() + state.orientation.z() * omega_quat.z());
    derivs.q_dot.x() =  0.5 * (state.orientation.w() * omega_quat.x() + state.orientation.y() * omega_quat.z() - state.orientation.z() * omega_quat.y());
    derivs.q_dot.y() =  0.5 * (state.orientation.w() * omega_quat.y() - state.orientation.x() * omega_quat.z() + state.orientation.z() * omega_quat.x());
    derivs.q_dot.z() =  0.5 * (state.orientation.w() * omega_quat.z() + state.orientation.x() * omega_quat.y() - state.orientation.y() * omega_quat.x());

    return derivs;
}
