#include "rocket_sil_framework/include/physics/native_rocket_dynamics_model.hpp"

NativeRocketDynamicsModel::NativeRocketDynamicsModel(
    std::unique_ptr<IIntegrator> integrator,
    std::unique_ptr<IEngineModel> engine,
    std::unique_ptr<IMassModel> mass,
    std::unique_ptr<IAerodynamicsModel> aero,
    std::unique_ptr<IEnvironmentModel> env)
    : integrator_(std::move(integrator)),
      engine_(std::move(engine)),
      mass_(std::move(mass)),
      aero_(std::move(aero)),
      env_(std::move(env)) {}

void NativeRocketDynamicsModel::update(double dt, const RocketInputs& inputs, RocketState& state) {
    if (!integrator_) {
        return;
    }
    
    // Update mass properties before the step based on mass flow rate
    if (engine_ && mass_) {
        MassProperties current_props = mass_->getProperties();
        EngineOutput eng_out = engine_->compute(state.time, current_props);
        mass_->update(eng_out.mass_flow_rate, dt);
    }
    
    auto calc_derivs = [this, &inputs](const RocketState& s) -> RocketStateDerivatives {
        return this->calculateDerivatives(s, inputs);
    };

    state = integrator_->integrate(state, dt, calc_derivs);
}

RocketStateDerivatives NativeRocketDynamicsModel::calculateDerivatives(const RocketState& state, const RocketInputs& inputs) const {
    RocketStateDerivatives derivs;

    Eigen::Vector3d force_body = inputs.force_body;
    Eigen::Vector3d torque_body = inputs.torque_body;

    double current_mass = inputs.mass;
    Eigen::Matrix3d current_inertia_inv = inputs.inertia_inv;
    Eigen::Matrix3d current_inertia = inputs.inertia;

    MassProperties current_props;
    // 1. Mass properties
    if (mass_) {
        current_props = mass_->getProperties();
        current_mass = current_props.total_mass;
        current_inertia = current_props.inertia_tensor;
        current_inertia_inv = current_inertia.inverse();
    }

    // 1.5 Environment
    EnvironmentState env_state;
    if (env_) {
        env_state = env_->compute(state);
    } else {
        // Fallback if no environment model is provided
        env_state.gravity_inertial = inputs.gravity_inertial;
        env_state.wind_velocity_inertial = Eigen::Vector3d::Zero();
        env_state.air_density = 1.225;
    }

    // 2. Engine thrust
    if (engine_) {
        EngineOutput eng = engine_->compute(state.time, current_props);
        force_body += eng.thrust_body;
    }

    // 3. Aerodynamics
    if (aero_) {
        AeroForces aero_forces = aero_->compute(state, current_props, env_state);
        force_body += aero_forces.aerodynamic_force_body;
        torque_body += aero_forces.aerodynamic_moment_body;
    }

    // 4. Translational dynamics
    Eigen::Vector3d force_inertial = state.orientation * force_body;
    force_inertial += current_mass * env_state.gravity_inertial;
    
    // Diagnostics update based on state (since this is a const method, diagnostics_ is mutable)
    diagnostics_.current_mass_kg = current_mass;
    diagnostics_.current_cg_z_m = current_props.center_of_gravity.z();
    diagnostics_.inertia_diagonal_kg_m2 = Eigen::Vector3d(current_inertia(0,0), current_inertia(1,1), current_inertia(2,2));
    diagnostics_.wind_velocity_inertial = env_state.wind_velocity_inertial;
    if (engine_) diagnostics_.thrust_body = engine_->compute(state.time, current_props).thrust_body;
    if (aero_) diagnostics_.aero_force_body = aero_->compute(state, current_props, env_state).aerodynamic_force_body;

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
