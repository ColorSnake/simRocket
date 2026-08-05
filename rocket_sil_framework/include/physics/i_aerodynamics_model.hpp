#pragma once

#include "physics_types.hpp"
#include "rocket_state.hpp"

class IAerodynamicsModel {
public:
    virtual ~IAerodynamicsModel() = default;
    
    // Wylicza siły aerodynamiczne (opór, siła nośna) na podstawie aktualnego stanu i środowiska
    // (w uproszczeniu: gęstości powietrza)
    virtual AeroForces compute(const RocketState& state, const MassProperties& mass_props, double air_density = 1.225) = 0;
};
