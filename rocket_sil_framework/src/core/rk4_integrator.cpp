#include "rocket_sil_framework/include/core/rk4_integrator.hpp"

RocketState RK4Integrator::integrate(const RocketState& current_state, double dt, const DerivativeFunc& calc_derivatives) {
    auto add_deriv = [](const RocketState& s, const RocketStateDerivatives& d, double scalar) -> RocketState {
        RocketState res = s;
        res.time += d.time_dot * scalar;
        res.position += d.velocity * scalar;
        res.velocity += d.acceleration * scalar;
        
        res.orientation.w() += d.q_dot.w() * scalar;
        res.orientation.x() += d.q_dot.x() * scalar;
        res.orientation.y() += d.q_dot.y() * scalar;
        res.orientation.z() += d.q_dot.z() * scalar;
        res.orientation.normalize();
        
        res.angular_velocity += d.angular_acceleration * scalar;
        return res;
    };

    RocketStateDerivatives k1 = calc_derivatives(current_state);
    
    RocketState s2 = add_deriv(current_state, k1, 0.5 * dt);
    RocketStateDerivatives k2 = calc_derivatives(s2);
    
    RocketState s3 = add_deriv(current_state, k2, 0.5 * dt);
    RocketStateDerivatives k3 = calc_derivatives(s3);
    
    RocketState s4 = add_deriv(current_state, k3, dt);
    RocketStateDerivatives k4 = calc_derivatives(s4);

    RocketState next_state = current_state;
    next_state.time += (k1.time_dot + 2.0 * k2.time_dot + 2.0 * k3.time_dot + k4.time_dot) * (dt / 6.0);
    next_state.position += (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity) * (dt / 6.0);
    next_state.velocity += (k1.acceleration + 2.0 * k2.acceleration + 2.0 * k3.acceleration + k4.acceleration) * (dt / 6.0);

    next_state.orientation.w() += (k1.q_dot.w() + 2.0 * k2.q_dot.w() + 2.0 * k3.q_dot.w() + k4.q_dot.w()) * (dt / 6.0);
    next_state.orientation.x() += (k1.q_dot.x() + 2.0 * k2.q_dot.x() + 2.0 * k3.q_dot.x() + k4.q_dot.x()) * (dt / 6.0);
    next_state.orientation.y() += (k1.q_dot.y() + 2.0 * k2.q_dot.y() + 2.0 * k3.q_dot.y() + k4.q_dot.y()) * (dt / 6.0);
    next_state.orientation.z() += (k1.q_dot.z() + 2.0 * k2.q_dot.z() + 2.0 * k3.q_dot.z() + k4.q_dot.z()) * (dt / 6.0);
    next_state.orientation.normalize();

    next_state.angular_velocity += (k1.angular_acceleration + 2.0 * k2.angular_acceleration + 2.0 * k3.angular_acceleration + k4.angular_acceleration) * (dt / 6.0);

    return next_state;
}
