#include "rocket_sil_framework/include/physics/euler_integrator.hpp"

RocketState EulerIntegrator::integrate(const RocketState& current_state, double dt, const DerivativeFunc& calc_derivatives) {
    RocketStateDerivatives derivs = calc_derivatives(current_state);
    RocketState next_state = current_state;

    // Translation
    next_state.time += derivs.time_dot * dt;
    next_state.position += derivs.velocity * dt;
    next_state.velocity += derivs.acceleration * dt;

    // Rotation
    next_state.orientation.w() += derivs.q_dot.w() * dt;
    next_state.orientation.x() += derivs.q_dot.x() * dt;
    next_state.orientation.y() += derivs.q_dot.y() * dt;
    next_state.orientation.z() += derivs.q_dot.z() * dt;
    next_state.orientation.normalize();

    next_state.angular_velocity += derivs.angular_acceleration * dt;

    return next_state;
}
