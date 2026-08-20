#pragma once

#include "rocket_sil_framework/include/core/i_integrator.hpp"

class EulerIntegrator : public IIntegrator {
public:
    RocketState integrate(const RocketState& current_state, double dt, const DerivativeFunc& calc_derivatives) override;
};
