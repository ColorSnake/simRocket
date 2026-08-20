#pragma once

#include <functional>
#include "rocket_sil_framework/include/core/rocket_state.hpp"
#include "rocket_sil_framework/include/core/rocket_state_derivatives.hpp"

using DerivativeFunc = std::function<RocketStateDerivatives(const RocketState&)>;

class IIntegrator {
public:
    virtual ~IIntegrator() = default;
    virtual RocketState integrate(const RocketState& current_state, double dt, const DerivativeFunc& calc_derivatives) = 0;
};
